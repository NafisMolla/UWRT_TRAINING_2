#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/twist.hpp"

namespace circular_move_namespace{

    class turtle_move_circular : public rclcpp::Node{


        public:
            explicit turtle_move_circular(const rclcpp::NodeOptions &options);

        private:
            int counter;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
            rclcpp::TimerBase::SharedPtr timer_;


    };
}