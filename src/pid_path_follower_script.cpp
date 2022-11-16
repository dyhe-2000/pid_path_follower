#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>      // std::pair, std::make_pair
#include <Eigen/LU>
#include <Eigen/QR>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "boat_interfaces/msg/vector_array.hpp"
#include "pid_path_follower/buffer_subscriber.hpp"
#include "pid_path_follower/Timer.hpp"

std::mutex mtx;

using namespace std::chrono_literals;

#define MAP_SIZE 1800 // will +1 for making origin
#define MAP_RESOLUTION 0.05
/*
#define KP 0.25
#define KD 0.1
#define KI 0.0003
#define SPEED 150 // -1000 - 1000
#define DIFFERENTIAL_MAG 100 // one side + mag other side - msg
#define MAX_STEERING_MAG 150
*/

// <x, y>
void bresenham2d(std::pair<std::pair<double, double>, std::pair<double, double>> theData, std::vector<std::pair<int, int>> *theVector) {
	int y1, x1, y2, x2;
	int dx, dy, sx, sy;
	int e2;
	int error;

	x1 = int(round(theData.first.first));
	y1 = int(round(theData.first.second));
	x2 = int(round(theData.second.first));
	y2 = int(round(theData.second.second));

	dx = abs(x1 - x2);
	dy = -abs(y1 - y2);

	sx = x1 < x2 ? 1 : -1;
	sy = y1 < y2 ? 1 : -1;

	error = dx + dy;

	while (1) {
		theVector->push_back(std::pair<int, int>(x1, y1));
		if (x1 == x2 && y1 == y2)
			break;
		e2 = 2 * error;

		if (e2 >= dy) {
			if (x2 == x1) break;
			error = error + dy;
			x1 = x1 + sx;
		}
		if (e2 <= dx) {
			if (y2 == y1) break;
			error = error + dx;
			y1 = y1 + sy;
		}
	}
}

double distance(std::pair<double, double>& p1, std::pair<double, double>& p2) {
    return sqrt(pow((p1.first - p2.first),2) + pow((p1.second - p2.second), 2));
}

bool sortBySecond(const std::pair<int, double>& a, const std::pair<int, double>& b) {
    return (a.second < b.second);
}

Eigen::MatrixXd calc_worldTbody(Eigen::MatrixXd& x){
	// x is 3 by 1
	Eigen::MatrixXd worldTbody(4, 4); //4 by n matrix
	worldTbody = Eigen::MatrixXd::Zero(4,4);
	worldTbody(0,0) = 1;
	worldTbody(1,1) = worldTbody(2,2) = worldTbody(3,3) = worldTbody(0,0);
	worldTbody(0,0) = cos(x(2,0));
	worldTbody(0,1) = -sin(x(2,0));
	worldTbody(1,0) = sin(x(2,0));
	worldTbody(1,1) = cos(x(2,0));
	worldTbody(0,3) = x(0,0);
	worldTbody(1,3) = x(1,0);
	return worldTbody;
}

Eigen::MatrixXd calc_T_inv(Eigen::MatrixXd& T){
    Eigen::MatrixXd T_inv(4, 4); //4 by 4 matrix
	T_inv = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd R = T.block(0, 0, 3, 3);
    Eigen::MatrixXd p = T.block(0, 3, 3, 1);
    Eigen::MatrixXd R_T = R.transpose();
    T_inv(3,3) = 1;
    T_inv.block(0,0,3,3) = R_T;
    T_inv.block(0,3,3,1) = -R_T*p;
    return T_inv;
}

class LineSegment{
public:
    std::pair<double, double> start;
    std::pair<double, double> end;
    LineSegment(double start_x=0.0, double start_y=0.0, double end_x=0.0, double end_y=0.0){
        this->start.first = start_x;
        this->start.second = start_y;
        this->end.first = end_x;
        this->end.second = end_y;
    }
};

class PIDPathFollower : public rclcpp::Node{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_cmd_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cur_global_state_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr cur_map_state_subscription_;
    rclcpp::Subscription<boat_interfaces::msg::VectorArray>::SharedPtr golbal_path_subscription_;
    rclcpp::Subscription<boat_interfaces::msg::VectorArray>::SharedPtr map_path_subscription_;
    
    size_t count_;
    int pwrL;
    int pwrR;
    int posL;
    int posR;
    double pre_cross_track_error;
    double cumulation_cross_track_error;
    LineSegment curSegmentGlobal;

    double KP;
    double KD;
    double KI;
    int SPEED;
    int MAX_STEERING_MAG;

    Eigen::MatrixXd mapTworld = Eigen::MatrixXd::Zero(4,4);
    Eigen::MatrixXd worldTbody = Eigen::MatrixXd::Zero(4,4);

    geometry_msgs::msg::Vector3 cur_boat_global_pos;
    geometry_msgs::msg::Vector3 cur_boat_map_pos;
    boat_interfaces::msg::VectorArray global_path;
    boat_interfaces::msg::VectorArray map_path;
public:
    PIDPathFollower() : Node("pid_path_follower") {
        this->pwrL = 0;
        this->pwrR = 0;
        this->posL = 0;
        this->posR = 0;
        this->count_ = 0;
        this->pre_cross_track_error = 0.0;
        this->cumulation_cross_track_error = 0.0;

        this->cur_boat_global_pos.x = 0;
        this->cur_boat_global_pos.y = 0;
        this->cur_boat_global_pos.z = 0;

        this->motor_cmd_publisher_ = this->create_publisher<std_msgs::msg::String>("/wamv/torqeedo/motor_cmd", 10);
        this->timer_ = this->create_wall_timer(50ms, std::bind(&PIDPathFollower::step, this));
        this->cur_global_state_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/boat_pos_global_frame", 1, std::bind(&PIDPathFollower::receiveGlobalState, this, std::placeholders::_1));
        this->cur_map_state_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>("/boat_pos_map_frame", 1, std::bind(&PIDPathFollower::receiveMapState, this, std::placeholders::_1));
        this->golbal_path_subscription_ = this->create_subscription<boat_interfaces::msg::VectorArray>("/global_path", 1, std::bind(&PIDPathFollower::receiveGlobalPath, this, std::placeholders::_1));
        this->map_path_subscription_ = this->create_subscription<boat_interfaces::msg::VectorArray>("/map_path", 1, std::bind(&PIDPathFollower::receiveMapPath, this, std::placeholders::_1));

        // initialize mapTworld
		mapTworld(0,0) = 1/MAP_RESOLUTION;
		mapTworld(1,1) = 1/MAP_RESOLUTION;
		mapTworld(2,2) = 1/MAP_RESOLUTION;
		mapTworld(3,3) = 1;
		mapTworld(0,3) = MAP_SIZE/2;
		mapTworld(1,3) = MAP_SIZE/2;
		std::cout << "mapTworld: \n" << mapTworld << std::endl;

        // initialize worldTbody
		worldTbody(0,0) = 1;
		worldTbody(1,1) = worldTbody(2,2) = worldTbody(3,3) = worldTbody(0,0);
		std::cout << "worldTbody: \n" << worldTbody << std::endl;

        std::fstream myfile("pid_config.txt", std::ios_base::in);
        double KPa, KDa, KIa, SPEEDa, MAX_STEERING_MAGa;
        std::string temp;
        while (myfile >> temp >> KPa >> temp >> KDa >> temp >> KIa >> temp >> SPEEDa >> temp >> MAX_STEERING_MAGa){
            std::cout << "KP = " << KPa << std::endl;
            std::cout << "KD = " << KDa << std::endl;
            std::cout << "KI = " << KIa << std::endl;
            std::cout << "SPEED = " << SPEEDa << std::endl;
            std::cout << "MAX_STEERING_MAG = " << MAX_STEERING_MAGa << std::endl;
        }
        this->KP = KPa;
        this->KD = KDa;
        this->KI = KIa;
        this->SPEED = SPEEDa;
        this->MAX_STEERING_MAG = MAX_STEERING_MAGa;

    }
    ~PIDPathFollower(){
        //this->publish_motor_cmd(0, 0, 0, 0);
    }
    void publish_motor_cmd(int pwrL, int pwrR, int posL, int posR){
        auto message = std_msgs::msg::String();
        message.data = "{\"lp\":" + std::to_string(pwrL) +  ", \"rp\":" + std::to_string(pwrR) + ", \"la\":" + std::to_string(posL) +", \"ra\":"+ std::to_string(posR)+ "}";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());

        this->motor_cmd_publisher_->publish(message);
    }
    void receiveGlobalState(const geometry_msgs::msg::Vector3 msg) {
        mtx.lock();
        this->cur_boat_global_pos = msg;
        
        //std::cout << "received global pos: " <<  this->cur_boat_global_pos.x << " " << this->cur_boat_global_pos.y << " " << this->cur_boat_global_pos.z << std::endl;
        Eigen::MatrixXd boat_global_pos_3vec = Eigen::MatrixXd::Zero(3,1);
        boat_global_pos_3vec(0,0) = cur_boat_global_pos.x;
        boat_global_pos_3vec(1,0) = cur_boat_global_pos.y;
        boat_global_pos_3vec(2,0) = cur_boat_global_pos.z;
        this->worldTbody = calc_worldTbody(boat_global_pos_3vec);
        /*
        //std::cout << "worldTbody: " << worldTbody << std::endl;

        Eigen::MatrixXd boat_global_pos_4vec = Eigen::MatrixXd::Zero(4,1);
        boat_global_pos_4vec(0,0) = cur_boat_global_pos.x;
        boat_global_pos_4vec(1,0) = cur_boat_global_pos.y;
        boat_global_pos_4vec(2,0) = 0;
        boat_global_pos_4vec(3,0) = 1;

        Eigen::MatrixXd convertedBodyFramePos = calc_T_inv(this->worldTbody)*boat_global_pos_4vec;
        std::cout << "convertedBodyFramePos x: " << convertedBodyFramePos(0,0) << " , y: " <<  convertedBodyFramePos(1,0) << std::endl;
        std::cout << std::endl;
        */
        mtx.unlock();
    }

    // input map pos x, map pos y (in int)
    // output 4 homo vec with global x,y,0,1
    Eigen::MatrixXd MapToGlobal(int map_pos_x, int map_pos_y){
        Eigen::MatrixXd convertedGlobalFramePos = Eigen::MatrixXd::Zero(4,1);
        convertedGlobalFramePos(0,0) = (map_pos_x-MAP_SIZE/2)*MAP_RESOLUTION;
        convertedGlobalFramePos(1,0) = (map_pos_y-MAP_SIZE/2)*MAP_RESOLUTION;
        convertedGlobalFramePos(2,0) = 0;
        convertedGlobalFramePos(3,0) = 1;
        return convertedGlobalFramePos;
    }
    void receiveMapState(const geometry_msgs::msg::Vector3 msg) {
        mtx.lock();
        this->cur_boat_map_pos = msg;
        
        //std::cout << "received map pos: " <<  this->cur_boat_map_pos.x << " " << this->cur_boat_map_pos.y << " " << this->cur_boat_map_pos.z << std::endl;
        /*
        Eigen::MatrixXd convertedGlobalFramePos = MapToGlobal(this->cur_boat_map_pos.x, this->cur_boat_map_pos.y);
        std::cout << "convertedGlobalFramePos x: " << convertedGlobalFramePos(0,0) << " , y: " <<  convertedGlobalFramePos(1,0) << std::endl;

        Eigen::MatrixXd boat_global_pos_3vec = Eigen::MatrixXd::Zero(3,1);
        boat_global_pos_3vec(0,0) = convertedGlobalFramePos(0,0);
        boat_global_pos_3vec(1,0) = convertedGlobalFramePos(1,0);
        boat_global_pos_3vec(2,0) = this->cur_boat_map_pos.z;
        this->worldTbody = calc_worldTbody(boat_global_pos_3vec);

        //std::cout << "worldTbody: " << worldTbody << std::endl;

        Eigen::MatrixXd convertedBodyFramePos = calc_T_inv(this->worldTbody)*convertedGlobalFramePos;
        std::cout << "convertedBodyFramePos x: " << convertedBodyFramePos(0,0) << " , y: " <<  convertedBodyFramePos(1,0) << std::endl;
        std::cout << std::endl;
        */
        mtx.unlock();
    }
    void receiveGlobalPath(const boat_interfaces::msg::VectorArray msg) {
        mtx.lock();
        this->global_path = msg;
        std::cout << "received global path" << std::endl;
        mtx.unlock();
    }
    void receiveMapPath(const boat_interfaces::msg::VectorArray msg) {
        mtx.lock();
        this->map_path = msg;
        std::cout << "received map path" << std::endl;
        if(this->map_path.vec3list.size() != 0){
            this->curSegmentGlobal.start.first = this->cur_boat_global_pos.x;
            this->curSegmentGlobal.start.second = this->cur_boat_global_pos.y;
            Eigen::MatrixXd convertedGlobalFramePoint = MapToGlobal(this->map_path.vec3list[0].x, this->map_path.vec3list[0].y);
            this->curSegmentGlobal.end.first = convertedGlobalFramePoint(0,0);
            this->curSegmentGlobal.end.second = convertedGlobalFramePoint(1,0);
        }
        mtx.unlock();
    }
    void step(){
        mtx.lock();
        if(this->map_path.vec3list.size() == 0){
            std::cout << "map path point list exhausted" << std::endl;
            this->pwrL = 0;
            this->pwrR = 0;
            this->posL = 0;
            this->posR = 0;
            this->publish_motor_cmd(this->pwrL, this->pwrR, this->posL, this->posR);
        }
        else{
            bool deleted = false; // tracking if deteled then make new segment
            // starting from the beginning, delete path point that has reached (in 0.5 meter)
            for(int i = 0; i < this->map_path.vec3list.size(); ++i){
                geometry_msgs::msg::Vector3 thePoint = map_path.vec3list[i]; // point in map frame
                Eigen::MatrixXd convertedGlobalFramePoint = MapToGlobal(thePoint.x, thePoint.y);
                Eigen::MatrixXd convertedBodyFramePoint = calc_T_inv(this->worldTbody)*convertedGlobalFramePoint;

                std::pair<double,double> p1;
                p1.first = convertedBodyFramePoint(0,0);
                p1.second = convertedBodyFramePoint(1,0);
                std::pair<double,double> p2;
                p2.first = 0.0;
                p2.second = 0.0;
                if(distance(p1,p2) < 0.5){ // searched from beginning and reached point near so delete
                    deleted = true;
                    this->map_path.vec3list.erase(this->map_path.vec3list.begin() + i);
                    --i;
                }
                else{ // searched from beginning and reached first point far
                    break;
                }
            }
            // after deleting near points in the beginning portion of the path
            if(this->map_path.vec3list.size() != 0){
                if(deleted){
                    this->curSegmentGlobal.start.first = this->cur_boat_global_pos.x;
                    this->curSegmentGlobal.start.second = this->cur_boat_global_pos.y;
                    Eigen::MatrixXd convertedGlobalFramePoint = MapToGlobal(this->map_path.vec3list[0].x, this->map_path.vec3list[0].y);
                    this->curSegmentGlobal.end.first = convertedGlobalFramePoint(0,0);
                    this->curSegmentGlobal.end.second = convertedGlobalFramePoint(1,0);
                }

                std::cout << std::endl;
                std::cout << "start global x: " << curSegmentGlobal.start.first << " y: " << curSegmentGlobal.start.second << std::endl;
                std::cout << "finish global x: " << curSegmentGlobal.end.first << " y: " << curSegmentGlobal.end.second << std::endl;
                Eigen::MatrixXd startGlobal = Eigen::MatrixXd::Zero(4,1);
                Eigen::MatrixXd endGlobal = Eigen::MatrixXd::Zero(4,1);
                startGlobal(0,0) = curSegmentGlobal.start.first;
                startGlobal(1,0) = curSegmentGlobal.start.second;
                startGlobal(2,0) = 0;
                startGlobal(3,0) = 1;
                endGlobal(0,0) = curSegmentGlobal.end.first;
                endGlobal(1,0) = curSegmentGlobal.end.second;
                endGlobal(2,0) = 0;
                endGlobal(3,0) = 1;
                Eigen::MatrixXd startMap = mapTworld*startGlobal;
                Eigen::MatrixXd endMap = mapTworld*endGlobal;
                std::cout << "start map x: " << startMap(0,0) << " y: " << startMap(1,0) << std::endl;
                std::cout << "finish map x: " << endMap(0,0) << " y: " << endMap(1,0) << std::endl;

                std::cout << "body global x: " << cur_boat_global_pos.x << " y: " << cur_boat_global_pos.y << std::endl;
                std::cout << "body map x: " << cur_boat_map_pos.x << " y: " << cur_boat_map_pos.y << std::endl;

                geometry_msgs::msg::Vector3 thePoint = map_path.vec3list[0]; // point in map frame
                Eigen::MatrixXd convertedGlobalFramePoint = MapToGlobal(thePoint.x, thePoint.y);
                std::cout << "end point in global frame x: " << convertedGlobalFramePoint(0,0) << " , y: " << convertedGlobalFramePoint(1,0) << std::endl;
                Eigen::MatrixXd convertedBodyFramePoint = calc_T_inv(this->worldTbody)*convertedGlobalFramePoint;
                std::cout << "end point in body frame x: " << convertedBodyFramePoint(0,0) << " , y: " << convertedBodyFramePoint(1,0) << std::endl;

                convertedBodyFramePoint(1,0); // pos on left, neg on right
                double cross_track_error; // norm so it's positive
                Eigen::MatrixXd y = Eigen::MatrixXd::Zero(4,1);
                y(2,0) = 0;
                y(3,0) = 1;
                if(this->curSegmentGlobal.start.first == this->cur_boat_global_pos.x && this->curSegmentGlobal.start.second == this->cur_boat_global_pos.y){
                    cross_track_error = 0.0;
                }
                else{
                    Eigen::MatrixXd z = Eigen::MatrixXd::Zero(2,1);
                    Eigen::MatrixXd x1 = Eigen::MatrixXd::Zero(2,1);
                    Eigen::MatrixXd x2 = Eigen::MatrixXd::Zero(2,1);
                    z(0,0) = this->cur_boat_global_pos.x;
                    z(1,0) = this->cur_boat_global_pos.y;
                    x1(0,0) = this->curSegmentGlobal.end.first;
                    x1(1,0) = this->curSegmentGlobal.end.second;
                    x2(0,0) = this->curSegmentGlobal.start.first;
                    x2(1,0) = this->curSegmentGlobal.start.second;
                    Eigen::MatrixXd zmx2 = z-x2;
                    Eigen::MatrixXd x1mx2 = x1-x2;
                    double numerator = zmx2(0,0)*x1mx2(0,0)+zmx2(1,0)*x1mx2(1,0);
                    double denominator = pow(x1mx2(0,0),2) + pow(x1mx2(1,0), 2);
                    y(0,0) = x1mx2(0,0)*numerator/denominator;
                    y(1,0) = x1mx2(1,0)*numerator/denominator;
                    cross_track_error = sqrt(pow((zmx2(0,0)-y(0,0)),2) + pow((zmx2(1,0)-y(1,0)), 2));
                }
                y(0,0) = y(0,0)+this->curSegmentGlobal.start.first;
                y(1,0) = y(1,0)+this->curSegmentGlobal.start.second;
                double cross_track_error_rate = (cross_track_error - this->pre_cross_track_error)/(0.05);
                std::cout << "nearest point global x: " << y(0,0) << " y: " << y(1,0) << std::endl;
                Eigen::MatrixXd convertedBodyFrameNearestPathPoint = calc_T_inv(this->worldTbody)*y;
                std::cout << "nearest point body x: " << convertedBodyFrameNearestPathPoint(0,0) << " y: " << convertedBodyFrameNearestPathPoint(1,0) << std::endl;

                if(convertedBodyFrameNearestPathPoint(1,0) < 0.0){
                    std::cout << "nearest path point on right" << std::endl;
                    cross_track_error = -cross_track_error;
                }
                else if(convertedBodyFrameNearestPathPoint(1,0) == 0.0){
                    std::cout << "on the path" << std::endl;
                    cross_track_error = 0.0;
                }
                else{
                    std::cout << "nearest path point on left" << std::endl;
                    // cross_track_error = cross_track_error;
                }
                std::cout << "cross_track_error: " << cross_track_error << std::endl;

                double steering_mag = (cross_track_error*KP+cross_track_error_rate*KD);

                if(steering_mag < -MAX_STEERING_MAG){
                    steering_mag = -MAX_STEERING_MAG;
                }
                else if(steering_mag > MAX_STEERING_MAG){
                    steering_mag = MAX_STEERING_MAG;
                }

                this->pre_cross_track_error = cross_track_error;
                this->cumulation_cross_track_error += cross_track_error;
                this->pwrL = SPEED-steering_mag;
                this->pwrR = SPEED+steering_mag;
                this->posL = 0;
                this->posR = 0;
                this->publish_motor_cmd(this->pwrL, this->pwrR, this->posL, this->posR);
            }
        }
        mtx.unlock();
    }
};

int main(int argc, char * argv[]){
    // example for using vec3list
    /*
    boat_interfaces::msg::VectorArray path;
    geometry_msgs::msg::Vector3 theVec;
    theVec.x = 0;
    theVec.y = 1;
    theVec.z = 2;
    path.vec3list.push_back(theVec);
    std::cout << path.vec3list.size() << std::endl;
    for(int i = 0; i < path.vec3list.size(); ++i){
        std::cout << path.vec3list[i].x << " " << path.vec3list[i].y << " " << path.vec3list[i].z << std::endl;
    }
    */

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPathFollower>());
    rclcpp::shutdown();
    return 0;
}
