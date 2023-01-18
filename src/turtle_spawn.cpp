#include <software_training/turtle_spawn.hpp>

namespace spawn_composition
{

    spawn_turtle::spawn_turtle(const rclcpp::NodeOptions &options) : rclcpp::Node("spawn_service_call", options)
    {

        client_ = this->create_client<turtlesim::srv::Spawn>("/spawn");
        RCLCPP_INFO(this->get_logger(), "working");

        
        turtle_info stationary;
        turtle_info moving_turtle;
        //set stationary
        stationary.x = 5;
        stationary.y = 5;
        stationary.theta = 0;

        //set moving
        moving_turtle.x = 10;
        moving_turtle.y = 7;
        moving_turtle.theta = 0;
        
        turtle_bio.push_back(stationary);
        turtle_bio.push_back(moving_turtle);

        RCLCPP_INFO(this->get_logger(), "%f",turtle_bio.at(0).x);



        //  fill up map with contents
        for (int i=0; i < 2; ++i)
        {
            spawn_turtle_info_map.insert({turtle_names[i], turtle_bio[i]});
        }

        spawn_turtle_callback();

        
        
    }


    void spawn_turtle::spawn_turtle_callback(){

        //wait for the server to be up
         while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server to be up...");
        }

        //seding the request for each turtle
        for(const auto key: spawn_turtle_info_map){

            RCLCPP_INFO(this->get_logger(), "%s ---- %f",key.first.c_str(),key.second.x);

            auto msg = std::make_shared<turtlesim::srv::Spawn::Request>() ;

            msg->name = key.first.c_str();
            msg->x = key.second.x;
            msg->y = key.second.y;
            msg->theta = key.second.theta;

            //callback for sending the request

            auto callback = [this] (rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture response) -> void {
                
                RCLCPP_INFO(this->get_logger(), "Turtle Created: %s", response.get()->name.c_str());
                rclcpp::shutdown();

            };

            auto result = client_->async_send_request(msg,callback);
        
        
        }

        

    }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(spawn_composition::spawn_turtle)