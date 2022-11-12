#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

std::mutex mtx;

using namespace std::chrono_literals;

class PIDPathFollower : public rclcpp::Node{
public:
    PIDPathFollower() : Node("pid_path_follower"), count_(0) {
        this->pwrL = 0;
        this->pwrR = 0;
        this->posL = 0;
        this->posR = 0;
        this->count_ = 0;
        publisher_ = this->create_publisher<std_msgs::msg::String>("/wamv/torqeedo/motor_cmd", 10);
        timer_ = this->create_wall_timer(50ms, std::bind(&PIDPathFollower::timer_callback, this));
    }

private:
    void timer_callback(){
        mtx.lock();
        auto message = std_msgs::msg::String();
        if(this->count_ < 100){
            this->pwrL = 30;
            this->pwrR = 30;
            this->posL = 0;
            this->posR = 0;
        }
        else{
            this->pwrL = 0;
            this->pwrR = 0;
            this->posL = 0;
            this->posR = 0;
        }
        message.data = "{\"lp\":" + std::to_string(pwrL) +  ", \"rp\":" + std::to_string(pwrR) + ", \"la\":" + std::to_string(posL) +", \"ra\":"+ std::to_string(posR)+ "}";
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        count_ += 1;
        mtx.unlock();
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
    int pwrL;
    int pwrR;
    int posL;
    int posR;
};

int main(int argc, char * argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDPathFollower>());
    rclcpp::shutdown();
    return 0;
}