#include <software_training/distance_publisher.hpp>


namespace turtle_composition{

    turtle_distance_publisher::turtle_distance_publisher(const rclcpp::NodeOptions &options) : Node("turtle_publisher", options) {

        // callback for stationary turtle position
        auto stationary_turt_callback =
            [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
            this->x_stationary_turt = msg->x;
            this->y_stationary_turt = msg->y;
        };

        // callback for moving turtle position
        auto moving_turt_callback =
            [this](const turtlesim::msg::Pose::SharedPtr msg) -> void {
            this->x_moving_turt = msg->x;
            this->y_moving_turt = msg->y;
        };

        // publisher callback
        auto publisher_callback = [this](void) -> void {
            // compute absolute difference in coordinates
            double position_x{abs(this->x_stationary_turt - this->x_moving_turt)};
            double position_y{abs(this->y_stationary_turt - this->y_moving_turt)};

            // create message to publish
            auto msg = training_interfaces::msg::TurtleDistance();
            msg.x_pos = position_x;
            msg.y_pos = position_y;
            // compute distance using trig
            msg.distance = sqrt((position_x * position_x) + (position_y * position_y));

            // publish message
            this->publisher->publish(msg); 
                                
        };


        
        stationary_turt_sub = this->create_subscription<turtlesim::msg::Pose>("/stationary_turtle/pose", 10, stationary_turt_callback);

        moving_turt_sub = this->create_subscription<turtlesim::msg::Pose>("/moving_turtle/pose", 10, moving_turt_callback);

        // instantiate publisher
        publisher = this->create_publisher<training_interfaces::msg::TurtleDistance>("/distance", 10);

        // set timer for publisher
        timer = this->create_wall_timer(std::chrono::seconds(3), publisher_callback);
        
    }

}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(turtle_composition::turtle_distance_publisher)