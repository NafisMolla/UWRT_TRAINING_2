#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp> 
#include <training_interfaces/action/software.hpp>
#include <thread>
#include "rclcpp_components/register_node_macro.hpp"

#include <geometry_msgs/msg/twist.hpp> 
#include <turtlesim/msg/pose.hpp> 


namespace turtle_composition{

    class turtle_action_server : public rclcpp::Node{


        public:
            using GoalHandleActionServer = rclcpp_action::ServerGoalHandle<training_interfaces::action::Software>;
            using SoftwareAction = training_interfaces::action::Software;

            explicit turtle_action_server(const rclcpp::NodeOptions &options);

            

        private:

            //subscriber to POSE
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;


            //publisher to /MOVING_TURTLE/CMD_VEL
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

            //action server
            rclcpp_action::Server<SoftwareAction>::SharedPtr action_server_;

            //cancel callback
            rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleActionServer> goal_handle);


            //accepted callback
            void handle_accepted(const std::shared_ptr<GoalHandleActionServer> goal_handle);


            //handle goal callback
            rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const SoftwareAction::Goal> goal);



            //execute callback
            void execute(const std::shared_ptr<GoalHandleActionServer> goal_handle);



            //store the position of the turtle
            typedef struct turtle_pos{
                float x;
                float y;
                float theta;
                float linear_velocity;
                float angular_velocity;

            } moving_turtle_pos;

            moving_turtle_pos moving_turtle;


    };//class
}//namespace 