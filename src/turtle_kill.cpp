#include <software_training/turtle_kill.hpp>

namespace kill_all_composition{

    //making the constructor
    kill_all_turtle_service_call::kill_all_turtle_service_call(const rclcpp::NodeOptions &options)
    : Node("kill_all_turtle_service_call", options){
        //this is the service we want to call
        client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        RCLCPP_INFO(this->get_logger(), "THIS IS WORKING LETS GOOOOO");
        
        
    }


    //making the kill()
    void kill_all_turtle_service_call::call_kill_service(){

        //wait for the server to be up
         while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        //for every possible turtle available we will call the kill command on it

        for(std::string name:turtle_name){
            RCLCPP_INFO(this->get_logger(), "%s",name);
        }



    }
    



}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(kill_all_composition::kill_all_turtle_service_call)