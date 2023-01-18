#include <training_interfaces/msg/turtle_distance.hpp>
#include <turtlesim/msg/pose.hpp>
#include <rclcpp/rclcpp.hpp>

namespace turtle_composition{

    class turtle_distance_publisher : public rclcpp::Node {

    public:
        explicit turtle_distance_publisher(const rclcpp::NodeOptions &options);

    private:
        //subscribers
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr stationary_turt_sub;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr moving_turt_sub;

        // turtle publisher with custom message
        rclcpp::Publisher<training_interfaces::msg::TurtleDistance>::SharedPtr publisher;

        rclcpp::TimerBase::SharedPtr timer;
        
        float x_stationary_turt;
        float y_stationary_turt;

        float x_moving_turt;
        float y_moving_turt;

        float total_distance;

    };

}