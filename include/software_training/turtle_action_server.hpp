#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>                 // ros2 time header
#include <rclcpp_action/rclcpp_action.hpp> // ros2 action header
#include <training_interfaces/action/software.hpp>


#include <geometry_msgs/msg/twist.hpp> // cmd_vel publisher message
#include <turtlesim/msg/pose.hpp> // header for message to get moving turt position


namespace turtle_composition{

    class turtle_action_server : public rclcpp::Node{


        public:

            explicit turtle_action_server(const rclcpp::NodeOptions &options);

            using GoalHandleActionServer = rclcpp_action::ServerGoalHandle<training_interfaces::action::Software>;

        private:

            //subscriber to POSE
            rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscriber_;


            //publisher to /MOVING_TURTLE/CMD_VEL
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher;

            //action server
            rclcpp_action::Server<training_interfaces::action::Software>::SharedPtr action_server_;

            //cancel callback


            //accepted callback


            //execute callback


            //store the position of the turtle

            typedef struct turtle_pos{
                float x;
                float y;
                float theta;
                float linear_velocity;
                float angular_velocity;

            } moving_turtle_pos;

            moving_turtle_pos moving_turtle;








    }















































}