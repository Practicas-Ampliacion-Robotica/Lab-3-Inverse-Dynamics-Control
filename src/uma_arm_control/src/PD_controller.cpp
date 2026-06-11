#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>

class PDControllerNode : public rclcpp::Node
{
public:
    PDControllerNode()
        : Node("pd_controller"),
          joint_positions_(Eigen::VectorXd::Zero(2)),
          joint_velocities_(Eigen::VectorXd::Zero(2)),
          topic_send_accelerations(Eigen::VectorXd::Zero(2))
    {
        // Frequency initialization
        this->declare_parameter<double>("frequency", 1000.0);

        // Get frequency [Hz] parameter and compute period [s]
        double frequency = this->get_parameter("frequency").as_double();


        // Set initial joint state
        this->declare_parameter<std::vector<double>>("q0", {0, 0});
        joint_positions_ = Eigen::VectorXd::Map(this->get_parameter("q0").as_double_array().data(), 2);

        // Create subscription to joint_torques
        subscription_joint_states_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 1, std::bind(&PDControllerNode::joint_states_callback, this, std::placeholders::_1));

        // Create publishers for joint torque
        publisher_topic_send_accelerations = this->create_publisher<std_msgs::msg::Float64MultiArray>("desired_joint_accelerations", 1);

        // Set the timer callback at a period (in milliseconds, multiply it by 1000)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000 / frequency)), std::bind(&PDControllerNode::timer_callback, this));
    }

    // Timer callback - when there is a timer callback, computes the new joint acceleration, velocity and position and publishes them
    void timer_callback()
    {
        // Calculate desired acceleration to cancel the dynamic effects
        topic_send_accelerations = PD_control();

        // Publish data
        publish_data();
    }

private:
    // joint_states subscription callback - when a new message arrives, updates the dynamics cancellation and publishes teh joint_torques_
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {

        // Assuming the joint names are "joint_1" and "joint_2"
        auto joint1_index = std::find(msg->name.begin(), msg->name.end(), "joint_1") - msg->name.begin();
        auto joint2_index = std::find(msg->name.begin(), msg->name.end(), "joint_2") - msg->name.begin();

        if (static_cast<std::vector<std::string>::size_type>(joint1_index) < msg->name.size() &&
            static_cast<std::vector<std::string>::size_type>(joint2_index) < msg->name.size())
        {
            joint_positions_(0) = msg->position[joint1_index];
            joint_positions_(1) = msg->position[joint2_index];
            joint_velocities_(0) = msg->velocity[joint1_index];
            joint_velocities_(1) = msg->velocity[joint2_index];
        }
    }

    // Method to publish the joint data
    void publish_data()
    {
        // publish accelerations
        auto topic_send_accelerations_msg = std_msgs::msg::Float64MultiArray();
        topic_send_accelerations_msg.data.assign(topic_send_accelerations.data(), topic_send_accelerations.data() + topic_send_accelerations.size());
        publisher_topic_send_accelerations->publish(topic_send_accelerations_msg);
    }

    // Method to calculate joint acceleration
    Eigen::VectorXd PD_control()
    {
        // Declarar posición deseada: 0.785, -0.785 1.0, 1.0
        Eigen::VectorXd joint_desired_positions_(2);
        joint_desired_positions_ << 1.0, 1.0;

        // Control gains
        Eigen::MatrixXd Kp_;
        Kp_ = Eigen::MatrixXd::Identity(2,2) * 1;
        Eigen::MatrixXd Kd_;
        Kd_ = Eigen::MatrixXd::Identity(2,2) * 1;


        // Calculate desired acceleration using PD control law: q_ddot_desired = Kp * (q_desired - q) + Kd * (q_dot_desired - q_dot)
        Eigen::VectorXd y_;
        y_ = Kp_ * (joint_desired_positions_ - joint_positions_) + Kd_ * (joint_desired_velocities_ - joint_velocities_) + joint_desired_accelerations_;

        return y_;

    }

    // Publishers and subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_joint_states_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_topic_send_accelerations;
    rclcpp::TimerBase::SharedPtr timer_;

    // Definir joints
    Eigen::VectorXd joint_positions_;
    Eigen::VectorXd joint_velocities_;
    Eigen::VectorXd topic_send_accelerations;

    // Declarar velocidad y aceleración
    Eigen::VectorXd joint_desired_accelerations_ = Eigen::VectorXd::Zero(2);
    Eigen::VectorXd joint_desired_velocities_ = Eigen::VectorXd::Zero(2);

};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}