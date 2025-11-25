#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>

using namespace std::chrono_literals;

class ForceLandNode : public rclcpp::Node
{
public:
    ForceLandNode()
        : Node("force_land_node"),
          land_request_active_(false),
          landing_in_progress_(false)
    {
        rmw_qos_profile_t sensor_qos = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(
            rclcpp::QoSInitialization(sensor_qos.history, sensor_qos.depth),
            sensor_qos
        );

        pos_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position",
            qos,
            std::bind(&ForceLandNode::positionCallback, this, std::placeholders::_1)
        );

        land_state_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
            "/fmu/out/vehicle_land_detected",
            qos,
            std::bind(&ForceLandNode::landStateCallback, this, std::placeholders::_1)
        );

        command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10
        );

        timer_ = create_wall_timer(
            10ms,
            std::bind(&ForceLandNode::processLandingLogic, this)
        );

        RCLCPP_INFO(get_logger(), "Force Land node initialized.");
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr pos_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_state_sub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    bool land_request_active_;     
    bool landing_in_progress_;     


    void positionCallback(const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg)
    {
        float altitude_enu = -msg->z;

        RCLCPP_INFO(get_logger(), "Altitude: %.2f m", altitude_enu);

        if (altitude_enu > 20.0f && !landing_in_progress_)
        {
            land_request_active_ = true;
        }
    }

 
    void landStateCallback(const px4_msgs::msg::VehicleLandDetected::UniquePtr msg)
    {
        if (msg->landed && landing_in_progress_)
        {
            RCLCPP_INFO(get_logger(), "Landing completed. Resetting state.");
            landing_in_progress_ = false;
            land_request_active_ = false;
        }
    }

    void processLandingLogic()
    {
        if (!land_request_active_)
            return;

        RCLCPP_WARN(get_logger(), "Altitude threshold exceeded! Sending LAND command...");

        px4_msgs::msg::VehicleCommand land_cmd{};
        land_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        land_cmd.target_system = 1;
        land_cmd.target_component = 1;
        land_cmd.source_system = 1;
        land_cmd.source_component = 1;
        land_cmd.from_external = true;
        land_cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;

        command_pub_->publish(land_cmd);

        landing_in_progress_ = true;
        land_request_active_ = false;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ForceLandNode>());
    rclcpp::shutdown();
    return 0;
}
