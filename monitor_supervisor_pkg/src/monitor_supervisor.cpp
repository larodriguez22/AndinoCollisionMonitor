#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
#include <iostream>
#include <queue>
#include <functional>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class MonitorSupervisor : public rclcpp::Node
{
public:
    MonitorSupervisor()
        : Node("monitor_supervisor")
    {
        // Create subscribers
        sub_cmd_vel_raw_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_raw", 10, std::bind(&MonitorSupervisor::cmdVelRawCallback, this, std::placeholders::_1));
        
        sub_cmd_vel_smoothed_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel_smoothed", 10, std::bind(&MonitorSupervisor::cmdVelSmoothedCallback, this, std::placeholders::_1));
        
        // Create publisher
        pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        pub_cmd_vel_raw_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel_raw", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MonitorSupervisor::timer_callback, this));
        // Initialize message variables
        cmd_vel_raw_ = geometry_msgs::msg::Twist();
        cmd_vel_smoothed_ = geometry_msgs::msg::Twist();
    }

private:
    void timer_callback()
    {
      if(cmd_vel_smoothed_buffer_.size()>1)
      {
        publishCmdVel();
      }
    }

    // Subscriber callbacks
    void cmdVelRawCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vel_raw_ = *msg;
        checkBuffer(cmd_vel_raw_buffer_);
        if (!flag_sign_ )
        { 
            assignSign();
            // flag_sign_ = false;
        }
        // else
        // {
        //     contador_++;
        // }
        cmd_vel_raw_buffer_.push(cmd_vel_raw_);
        // counter_buffer_raw++;
    }

    void cmdVelSmoothedCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vel_smoothed_ = *msg;
        checkBuffer(cmd_vel_smoothed_buffer_);
        cmd_vel_smoothed_buffer_.push(cmd_vel_raw_);
        // counter_buffer_raw++;
    }

    void checkBuffer(std::queue<geometry_msgs::msg::Twist> &buffer)
    {
        if (buffer.size() == BUFFER_SIZE)
        {
            buffer.pop();
        }
    }

    void assignSign(){

        if (cmd_vel_raw_.linear.x == 0)
        {
            const_vel.linear.x = 0;
        }
        else
        {
            const_vel.linear.x = cmd_vel_raw_.linear.x > 0 ? -0.1 : 0.1;
            const_vel.linear.x = cmd_vel_raw_.linear.x > 0 ? -0.1 : 0.1;
        }

        if (cmd_vel_raw_.linear.y == 0)
        {
            const_vel.linear.y = 0;
        }
        else
        {
            const_vel.linear.y = cmd_vel_raw_.linear.y > 0 ? -0.1 : 0.1;
        }

        if (cmd_vel_raw_.linear.z == 0)
        {
            const_vel.linear.z = 0;
        }
        else
        {
            const_vel.linear.z = cmd_vel_raw_.linear.z > 0 ? -0.1 : 0.1;
        }
    }

   geometry_msgs::msg::Twist sumBuffer(std::queue<geometry_msgs::msg::Twist> buffer)
    {
        geometry_msgs::msg::Twist sum;
        sum.linear.x = 0;
        sum.linear.y = 0;
        sum.angular.z = 0;

        for (size_t i = 0; i < buffer.size(); i++)
        {
            sum.linear.x += buffer.front().linear.x;
            sum.linear.y += buffer.front().linear.y;
            sum.angular.z += buffer.front().angular.z;
            buffer.push(buffer.front());
            buffer.pop();
        }

        return sum;
    }  
   
    // Publish to the /cmd_vel topic
    void publishCmdVel()
    {
        geometry_msgs::msg::Twist output_cmd_vel;

        geometry_msgs::msg::Twist sum_cmd_smoothed = sumBuffer(cmd_vel_smoothed_buffer_);

        // Check if cmd smoothed has been zero 
        if (cmd_vel_smoothed_.linear.x == 0 && cmd_vel_smoothed_.linear.y == 0 && cmd_vel_smoothed_.angular.z == 0)
        {
            // Check if cmd_raw is zero
            if (cmd_vel_raw_.linear.x == 0 && cmd_vel_raw_.linear.y == 0 && cmd_vel_raw_.angular.z == 0)
            {
                // If cmd_raw is zero, then publish cmd_raw
                output_cmd_vel = cmd_vel_smoothed_;
                std::cout << "Publishing cmd_vel_smoothed_" << std::endl;
                flag_sign_ = false;
            }
            else
            {
                // If cmd_raw is not zero, then publish cmd_raw
                pub_cmd_vel_raw_->publish(const_vel);
                output_cmd_vel = const_vel;
                std::cout << "Publishing const_vel" << std::endl;
                
                contador_++;
                flag_sign_ = true;
                
            }
        }
        else
        {
            if (contador_ > 3)
            {
                geometry_msgs::msg::Twist zero_vel;
                zero_vel.linear.x = 0;
                zero_vel.linear.y = 0;
                zero_vel.linear.z = 0;
                zero_vel.angular.x = 0;
                zero_vel.angular.y = 0;
                zero_vel.angular.z = 0;

                pub_cmd_vel_raw_->publish(zero_vel);
                output_cmd_vel = zero_vel;
                std::cout << "Publishing zero_vel" << std::endl;
                contador_ = 0;
            }
            else
            {
                // If cmd_smoothed is not zero, then publish cmd_smoothed
                output_cmd_vel = cmd_vel_smoothed_;
                flag_sign_ = false;
            }

        }

        // Publish the resulting command
        pub_cmd_vel_->publish(output_cmd_vel);
    }

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_raw_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_smoothed_;

    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_raw_;

    // Stored message data
    geometry_msgs::msg::Twist const_vel;
    geometry_msgs::msg::Twist cmd_vel_raw_;
    geometry_msgs::msg::Twist cmd_vel_smoothed_;

    // Flags
    bool flag_sign_{false};
    int contador_{0};

    // Storage arrays
    std::queue<geometry_msgs::msg::Twist> cmd_vel_raw_buffer_;
    std::queue<geometry_msgs::msg::Twist> cmd_vel_smoothed_buffer_;

    // Constants
    const unsigned int BUFFER_SIZE{5};
    size_t counter_buffer_raw{1};
    size_t counter_buffer_{1};
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MonitorSupervisor>());
    rclcpp::shutdown();
    
    return 0;
}