#include "pure_pursuit/pursuit.h"

namespace wbb
{
PurePursuit::PurePursuit()
{
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    std::chrono::duration<double> period = this->declare_parameter<double>("period", 0.02);
    double lookahead = this->declare_parameter<double>("lookahead", 0.1);

    timer_ = this->create_wall_timer(period, std::bind(&PurePursuit::sendControlCommand, this));

    slot_.trajectory = this->create_subscription<wbb_msgs::msg::ImagePath>(
        "/robot/motion", 1, std::bind(&PurePursuit::handleTrajectory, this, _1)
        );

    slot_.bot_pose = this->create_subscription<wbb_msgs::msg::ImagePose>(
        "/board/image/ego", rclcpp::QoS(1).reliability(qos),
        std::bind(&PurePursuit::handleBotPose, this, _1)
        );

    signal_.control = this->create_publisher<wbb_msgs::msg::Control>("/movement", 1);
}

void PurePursuit::handleTrajectory(wbb_msgs::msg::ImagePath::SharedPtr trajectory)
{
    state_.trajectory = std::move(trajectory);
}

void PurePursuit::handleBotPose(wbb_msgs::msg::ImagePose::SharedPtr bot_pose)
{
    state_.bot_pose = std::move(bot_pose);
}

void PurePursuit::sendControlCommand()
{
    // ...
}

} // namespace wbb