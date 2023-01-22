#include <software_training/turtle_action_server.hpp>
#include <cmath>
#include <memory>

using namespace std::placeholders;
namespace turtle_composition{

    using GoalHandleActionServer = rclcpp_action::ServerGoalHandle<training_interfaces::action::Software>;
    using SoftwareAction = training_interfaces::action::Software;

    //constructor
    turtle_action_server::turtle_action_server(const rclcpp::NodeOptions &options) : Node("action_turtle_server",options){
        
        //making the action server
        action_server_ = rclcpp_action::create_server<training_interfaces::action::Software>(this,"software_action_",
                                                                        std::bind(&turtle_action_server::handle_goal,this,_1,_2),
                                                                        std::bind(&turtle_action_server::handle_cancel,this,_1),
                                                                        std::bind(&turtle_action_server::handle_accepted,this,_1));


        //create publisher to cmd_vel
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/moving_turtle/cmd_vel",10);
        RCLCPP_INFO(this->get_logger(),"action is working");



        //create subscriber and subscriber calback
        auto callback = [this](const turtlesim::msg::Pose::SharedPtr msg) -> void{
            
            //get the current corrdinates of the turtle
            this->moving_turtle.x = msg->x;
            this->moving_turtle.y = msg->y;
            this->moving_turtle.theta = msg->theta;
            this->moving_turtle.linear_velocity = msg->linear_velocity;
            this->moving_turtle.angular_velocity = msg->angular_velocity;
            RCLCPP_INFO(this->get_logger(),"action is working");

        };
        
        subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/moving_turtle/pose",10,callback);

        
    }

    //cancel callback
    rclcpp_action::CancelResponse turtle_action_server::handle_cancel(const std::shared_ptr<GoalHandleActionServer> goal_handle){
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }


    //accepted callback
    void turtle_action_server::handle_accepted(const std::shared_ptr<GoalHandleActionServer> goal_handle){
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&turtle_action_server::execute, this, _1), goal_handle}.detach();

    }


    //handle goal callback
    rclcpp_action::GoalResponse turtle_action_server::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SoftwareAction::Goal> goal){
        
        RCLCPP_INFO(this->get_logger(), "Received goal request with order x:%f y:%f", goal->x,goal->y);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

    }



    //execute callback
    void turtle_action_server::execute(const std::shared_ptr<GoalHandleActionServer> goal_handle){
        
        RCLCPP_INFO(this->get_logger(), "running the execute function");
        //getting the start time
        auto start_time = this->now();

        //set the loop rate
        rclcpp::Rate loop_rate(1);

        //getting the goal
        const auto goal = goal_handle->get_goal();

        //creating a result ojb
        auto result = std::make_shared<training_interfaces::action::Software::Result>();

        //creating a feedback obj
        auto feedback = std::make_shared<training_interfaces::action::Software::Feedback>();

        //getting the goal x and goal y
        auto final_x = goal->x;
        auto final_y = goal->y;

        //setting a velocity for the turtle in the x and y direction 
        float vel_x = 0.25;
        float vel_y = 0.25;

        //this is the margin of error
        float MOE = 0.5;

        //main while loop that will keep running untill the node dies or we reach the target
        while(rclcpp::ok() && !((moving_turtle.x >= final_x - MOE) && (moving_turtle.x <= final_x + MOE)) || !((moving_turtle.y >= final_y - MOE) && (moving_turtle.y <= final_y + MOE))){
            
            auto msg = geometry_msgs::msg::Twist();
            
            msg.linear.x = vel_x;
            msg.linear.y = vel_y;
            msg.linear.z = 0.0f;
            
            msg.angular.x = 0.0f;
            msg.angular.y = 0.0f;
            msg.angular.z = 0.0f;

            //get the distance from the target, with good old pythagorean theorem
            feedback->distance = sqrt(pow(final_x - moving_turtle.x,2)+ pow(final_y - moving_turtle.y,2));
            //publish the feedback
            goal_handle->publish_feedback(feedback);

            //publish the new pos to the turtle
            this->publisher_->publish(msg);
            
            loop_rate.sleep();
        }

        //we are at the point now where the turtle has already reached the goal
        if (rclcpp::ok()) {

            //since we have reached the target we will set all the twist values to 0 (stanionary)
            auto msg = geometry_msgs::msg::Twist();

            msg.linear.x = 0;
            msg.linear.y = 0;
            msg.linear.z = 0.0;
            
            msg.angular.x = 0.0f;
            msg.angular.y = 0.0f;
            msg.angular.z = 0.0f;

            this->publisher_->publish(msg);

            rclcpp::Time end = this->now();          
            rclcpp::Duration duration = end - start_time;

            long int final_time = duration.nanoseconds();

            
            result->duration = final_time;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }


    }


}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(turtle_composition::turtle_action_server)