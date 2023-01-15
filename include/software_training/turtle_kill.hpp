#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/kill.hpp>

namespace kill_all_composition{


class kill_all_turtle_service_call: public rclcpp::Node{

//Constructor
public: 
    explicit kill_all_turtle_service_call(const rclcpp::NodeOptions &options);

private:
//client pointer
rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client_;

//array with all the possible names for the turtles
std::vector<std::string> turtle_name = {"turlte1","moving_turtle","stationary_turtle"};


void call_kill_service();

};


}