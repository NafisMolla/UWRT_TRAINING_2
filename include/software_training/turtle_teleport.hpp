#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <training_interfaces/srv/turtle_reset.hpp>

namespace turtle_composition
{

    class teleport_turtle : public rclcpp::Node
    {

    public:
        explicit teleport_turtle(const rclcpp::NodeOptions &options);

    private:
        typedef struct turtle_info
        {
            constexpr static float x = 5.44;
            constexpr static float y = .44;
            constexpr static float theta = 0;
        } turtle_info;

        

        void teleport_callback(const std::shared_ptr<training_interfaces::srv::TurtleReset::Request> request , const std::shared_ptr<training_interfaces::srv::TurtleReset::Response> response);

        rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr client_;
        rclcpp::Service<training_interfaces::srv::TurtleReset>::SharedPtr service_;

    
    };

}