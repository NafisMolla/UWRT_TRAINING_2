#include <software_training/turtle_kill.hpp>

namespace kill_all_composition{

    //making the constructor
    kill_all_turtle_service_call::kill_all_turtle_service_call(const rclcpp::NodeOptions &options)
    : Node("kill_all_turtle_service_call", options){
        //this is the service we want to call
        client_ = this->create_client<turtlesim::srv::Kill>("/kill");
        RCLCPP_INFO(this->get_logger(), "THIS IS WORKING LETS GOOOOO");
        this->call_kill_service();
        
        
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

            //make a request object 
            auto request = std::make_shared<turtlesim::srv::Kill::Request>();
            request->name = name;

            auto callback_client_ = [this,name] (rclcpp::Client<turtlesim::srv::Kill>::SharedFuture response) -> void{
                (void) response;
                RCLCPP_INFO(this->get_logger(), "Turtle %s Killed", name.c_str());
                rclcpp::shutdown();

            };
            auto result = client_->async_send_request(request,callback_client_);


        }



    }
    



}

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(kill_all_composition::kill_all_turtle_service_call)