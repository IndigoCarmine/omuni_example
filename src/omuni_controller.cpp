#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>


#include <can_plugins2/msg/frame.hpp>
#include <can_utils.hpp>

namespace omuni_controller
{
    //It is a class for moter, especially for shirasu moter driver.
    class Moter{
        private:
            unsigned int can_id; //standard can id
            float x_coefficient;
            float y_coefficient; 
            float max_velocity;
        public:
            Moter(unsigned int can_id,float angle,float max_velocity):can_id(can_id),max_velocity(max_velocity){
                set_angle(angle);
            }
            //the angle is how the moter is mounted on the robot.
            //you may not need to call this function, moter is mounted on the robot in a fixed angle.
            void set_angle(float angle){
                //angle is radian.
                x_coefficient = std::cos(angle);
                y_coefficient = std::sin(angle);
            }

            void set_max_velocity(float max_velocity){
                max_velocity = max_velocity;
            }

            //xy is the vector of velocity. its absolute value should be less than 1.
            std::unique_ptr<can_plugins2::msg::Frame> generate_frame(float x, float y){
                float velocity = (x*x_coefficient + y*y_coefficient)*max_velocity;
                return can_utils::generate_frame(can_id,velocity);
            }
    };

    class OmuniController : public rclcpp::Node
    {
        private:
            //TODO: add a parameter to set the max velocity of the moter.
            Moter moters[4] = {
                Moter(0x201,0,1),
                Moter(0x202,0,1),
                Moter(0x203,0,1),
                Moter(0x204,0,1)
            };
            rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
            rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_pub_;
            void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
        public:
            OmuniController(const rclcpp::NodeOptions & options): Node("omuni_controller",options)
            {

                joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                    "joy", 10, std::bind(&OmuniController::joy_callback, this, std::placeholders::_1));
                can_pub_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);
                RCLCPP_INFO(this->get_logger(), "OmuniController is ready.");
            }

    };

    void OmuniController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        float x = msg->axes[0];
        float y = msg->axes[1];
        for(auto &moter:moters){
            auto frame = moter.generate_frame(x,y);
            can_pub_->publish(std::move(frame));
        }   
    }
    
} // namespace omuni_controller


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(omuni_controller::OmuniController)