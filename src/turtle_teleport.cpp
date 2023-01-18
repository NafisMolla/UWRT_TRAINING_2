#include <software_training/turtle_teleport.hpp>

using namespace std::placeholders;
namespace turtle_composition{

    teleport_turtle::teleport_turtle(const rclcpp::NodeOptions &options) : rclcpp::Node("teleport_turtle",options){
        //client for turtlesim teleport service
        client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/moving_turtle/teleport_absolute");
        
        //custom service 
        service_ = this->create_service<training_interfaces::srv::TurtleReset>("/reset_moving_turtle",std::bind(&teleport_turtle::teleport_callback,this,_1,_2));

        RCLCPP_INFO(this->get_logger(),"this is working yayaa");

    }


    void teleport_turtle::teleport_callback(const std::shared_ptr<training_interfaces::srv::TurtleReset::Request> request , const std::shared_ptr<training_interfaces::srv::TurtleReset::Response> response){

        (void) request;
		
		// waiting for the client
		if (!client_->wait_for_service(std::chrono::seconds(1))) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "System Aborted");
				response->success = false;
				return;
			}
			RCLCPP_INFO(this->get_logger(), "Service is not available! Exit!");
			response->success = false;
			return;
		}


		auto client_request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();

		 // fill request data
		client_request->x = teleport_turtle::turtle_info::x;
		client_request->y = teleport_turtle::turtle_info::y;
		client_request->theta = teleport_turtle::turtle_info::theta;


		//callback for service call
		auto response_callback = [this](rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedFuture response)->void{
			RCLCPP_INFO(this->get_logger(), "Turtle Moved");
			(void) response;
		};

		auto result = client_->async_send_request(client_request,response_callback);

		RCLCPP_INFO(this->get_logger(), "Turtle Reseted");

  		response->success = true;


    }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(turtle_composition::teleport_turtle)