#include <rclcpp/rclcpp.hpp>
#include <turtlesim/srv/spawn.hpp>

namespace spawn_composition
{

    class spawn_turtle : public rclcpp::Node
    {

    public:
        explicit spawn_turtle(const rclcpp::NodeOptions &options);

    private:
        typedef struct turtle_info
        {
            float x;
            float y;
            float theta;
        } turtle_info;

        void spawn_turtle_callback();

        turtle_info stationary;
        turtle_info moving_turtle;

        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client_;
        std::vector<std::string> turtle_names{"stationary_turtle", "moving_turtle"};
        //how does the dot thing work??
        std::vector<turtle_info> turtle_bio;
        
        std::map<std::string, turtle_info> spawn_turtle_info_map;
    };

}