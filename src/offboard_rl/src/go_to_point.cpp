#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <unsupported/Eigen/Splines>
#include <offboard_rl/utils.h>

using namespace std::chrono_literals;

class SplineFlightNode : public rclcpp::Node
{
public:
    SplineFlightNode()
        : Node("spline_flight_node"),
          offboard_enabled_(false),
          spline_ready_(false),
          landing_phase_(false),
          elapsed_time_(0.0),
          offboard_tick_(0)
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            std::bind(&SplineFlightNode::onPosition, this, std::placeholders::_1));

        attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&SplineFlightNode::onAttitude, this, std::placeholders::_1));

        offboard_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);

        cmd_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        trajectory_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);

        offboard_timer_ = create_wall_timer(
            100ms, std::bind(&SplineFlightNode::updateOffboard, this));

        control_timer_ = create_wall_timer(
            20ms, std::bind(&SplineFlightNode::updateTrajectory, this));
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr position_sub_;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_pub_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_pub_;

    rclcpp::TimerBase::SharedPtr offboard_timer_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    bool offboard_enabled_;
    bool spline_ready_;
    bool landing_phase_;
    double total_time_ = 40.0;
    double elapsed_time_;
    int offboard_tick_;

    px4_msgs::msg::VehicleLocalPosition last_position_;
    px4_msgs::msg::VehicleAttitude last_attitude_;

    typedef Eigen::Spline<double, 4> Spline4D;
    Spline4D spline_;

    void onPosition(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
    {
        last_position_ = *msg;
    }

    void onAttitude(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
    {
        last_attitude_ = *msg;
    }

    void buildSpline()
    {
        Eigen::MatrixXd pts(4, 7);

        Eigen::Vector4d q(last_attitude_.q[0], last_attitude_.q[1],
                          last_attitude_.q[2], last_attitude_.q[3]);
        double yaw0 = utilities::quatToRpy(q)(2);

        pts.col(0) << last_position_.x, last_position_.y, last_position_.z, yaw0;
        pts.col(1) << last_position_.x + 5.0, last_position_.y + 3.0,  -3.0, yaw0 + 0.3;
        pts.col(2) << last_position_.x + 12.0, last_position_.y + 10.0, -6.0, yaw0 + 0.9;
        pts.col(3) << last_position_.x + 6.0, last_position_.y + 18.0, -12.0, yaw0 + 1.4;
        pts.col(4) << last_position_.x - 4.0, last_position_.y + 20.0, -9.0,  yaw0 + 1.9;
        pts.col(5) << last_position_.x - 10.0, last_position_.y + 12.0, -7.0, yaw0 + 2.5;
        pts.col(6) << last_position_.x - 2.0, last_position_.y + 3.0,  -4.0,  yaw0 + 3.14;

        spline_ = Eigen::SplineFitting<Spline4D>::Interpolate(pts, 3);

        spline_ready_ = true;
        elapsed_time_ = 0.0;
    }

    void updateOffboard()
    {
        if (landing_phase_)
            return;

        if (offboard_tick_ == 10)
        {
            if (!spline_ready_)
                buildSpline();

            sendModeCommand();
            sendArmCommand();
            offboard_enabled_ = true;
        }

        publishOffboardFlags();

        if (offboard_tick_ < 11)
            offboard_tick_++;
    }

    void sendModeCommand()
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE;
        cmd.param1 = 1;
        cmd.param2 = 6;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = nowUSec();
        cmd_pub_->publish(cmd);
    }

    void sendArmCommand()
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = nowUSec();
        cmd_pub_->publish(cmd);
    }

    void publishOffboardFlags()
    {
        px4_msgs::msg::OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = true;
        msg.acceleration = true;
        msg.timestamp = nowUSec();
        offboard_pub_->publish(msg);
    }

    void updateTrajectory()
    {
        if (!offboard_enabled_ || !spline_ready_ || landing_phase_)
            return;

        double u = elapsed_time_ / total_time_;

        if (u >= 1.0)
        {
            triggerLanding();
            return;
        }

        auto d = spline_.derivatives(u, 2);
        double du = 1.0 / total_time_;
        double du2 = du * du;

        Eigen::Vector4d pos = d.col(0);
        Eigen::Vector4d vel = d.col(1) * du;
        Eigen::Vector4d acc = d.col(2) * du2;

        px4_msgs::msg::TrajectorySetpoint sp{};
        sp.position = {float(pos[0]), float(pos[1]), float(pos[2])};
        sp.velocity = {float(vel[0]), float(vel[1]), float(vel[2])};
        sp.acceleration = {float(acc[0]), float(acc[1]), float(acc[2])};
        sp.yaw = float(pos[3]);
        sp.timestamp = nowUSec();
        trajectory_pub_->publish(sp);

        elapsed_time_ += 0.02;
    }

    void triggerLanding()
    {
        px4_msgs::msg::VehicleCommand cmd{};
        cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND;
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = nowUSec();
        cmd_pub_->publish(cmd);

        landing_phase_ = true;
    }

    uint64_t nowUSec()
    {
        return this->get_clock()->now().nanoseconds() / 1000;
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SplineFlightNode>());
    rclcpp::shutdown();
    return 0;
}
