#include <software_training/turtle_move_circle.hpp>


namespace circular_move_namespace{


    turtle_move_circular::turtle_move_circular(const rclcpp::NodeOptions &options)
    : Node("cmd_", options){
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);

        auto callback = [this](void) -> void{
            auto msg = geometry_msgs::msg::Twist();

            msg.linear.x = 12;
            msg.linear.y = 12;
            msg.linear.z = 12;

            msg.angular.x = 1.41;
            msg.angular.y = 1.41;
            msg.angular.z = 1.41;


            this->cmd_vel_publisher_->publish(msg);

        };

        timer_ = this->create_wall_timer(std::chrono::seconds(1),callback);

    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(circular_move_namespace::turtle_move_circular)