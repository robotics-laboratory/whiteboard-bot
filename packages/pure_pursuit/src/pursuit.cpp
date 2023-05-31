#include "pure_pursuit/pursuit.h"

namespace wbb
{

PurePursuit::PurePursuit() : Node("pure_pursuit")
{
    const auto qos = static_cast<rmw_qos_reliability_policy_t>(
        this->declare_parameter<int>("qos", RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT));

    std::chrono::duration<double> period = std::chrono::duration<double>(
        this->declare_parameter<double>("period", 0.02));
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

double calculateDistance(wbb_msgs::msg::ImagePoint::SharedPtr first,
                         wbb_msgs::msg::ImagePose::SharedPtr second)
{
    return std::sqrt(std::pow(first->x - second->x, 2) +
                     std::pow(first->y - second->y, 2));
}

double PurePursuit::calculateCurvature(wbb_msgs::msg::ImagePoint::SharedPtr lookahead,
                                       wbb_msgs::msg::ImagePose::SharedPtr bot_pose)
{
    double chord = calculateDistance(lookahead, bot_pose);

    if (chord == 0)
        return 0;

    double alpha = std::atan2(lookahead->y - bot_pose->y, lookahead->x - bot_pose->x) -
                   M_PI / 2 + bot_pose->theta;

    if (std::cos(alpha) > 0)
        return (2 * std::abs(std::sin(alpha))) / chord;
    return -(2 * std::abs(std::sin(alpha))) / chord;
}

/*wbb_msgs::msg::ImagePoint findClosest(wbb_msgs::msg::ImagePath::SharedPtr trajectory,
                                                 wbb_msgs::msg::ImagePose::SharedPtr bot_pose)
{
    double min_dist = calculateDistance(trajectory->points[0], bot_pose);
    wbb_msgs::msg::ImagePoint min_point = trajectory->points[0];

    for (auto point : trajectory->points)
    {
        double distance = calculateDistance(point, bot_pose);
        if (distance < min_dist)
        {
            min_dist = distance;
            min_point = point;
        }
    }

    return min_point;
}*/

wbb_msgs::msg::ImagePoint::SharedPtr checkSegment(wbb_msgs::msg::ImagePoint start,
                                                  wbb_msgs::msg::ImagePoint end,
                                                  wbb_msgs::msg::ImagePose::SharedPtr bot_pose)
{
    double vector_dot_a = std::pow(end.x - start.x, 2) + std::pow(end.y - start.y, 2);

    double vector_dot_b = 2 * (start.x - bot_pose->x) * (end.x - start.x) +
                          2 * (start.y - bot_pose->y) * (end.y - start.y);

    double vector_dot_c = std::pow(start.x - bot_pose->x, 2) +
                          std::pow(start.y - bot_pose->y, 2) - std::pow(self.lookahead, 2);

    double discr = std::pow(vector_dot_b, 2) - 4 * vector_dot_a * vector_dot_c;

    if (discr < 0 || vector_dot_a == 0)
        return nullptr;

    discr = std::sqrt(discr);

    double t1 = (-vector_dot_b - discr) / (2 * vector_dot_a);
    double t2 = (-vector_dot_b + discr) / (2 * vector_dot_a);

    if (t1 >= 0 && t1 <= 1)
    {
        wbb_msgs::msg::ImagePoint pt;
        pt.x = start.x + t1 * (end.x - start.x);
        pt.y = start.y + t1 * (end.y - start.y);
        return std::make_shared<wbb_msgs::msg::ImagePoint>(pt);
    }
    if (t2 >= 0 && t2 <= 1)
    {
        wbb_msgs::msg::ImagePoint pt;
        pt.x = start.x + t2 * (end.x - start.x);
        pt.y = start.y + t2 * (end.y - start.y);
        return std::make_shared<wbb_msgs::msg::ImagePoint>(pt);
    }

    return nullptr;
}

wbb_msgs::msg::ImagePoint::SharedPtr findLookahead(wbb_msgs::msg::ImagePath::SharedPtr trajectory,
                                                   wbb_msgs::msg::ImagePose::SharedPtr bot_pose)
{
    for (size_t i = 1; i < trajectory->points.size(); i++)
    {
        wbb_msgs::msg::ImagePoint::SharedPtr pt = checkSegment(trajectory->points[i - 1], trajectory->points[i], bot_pose);
        if (!pt)
            continue;

        return pt;
    }

    return nullptr;
}

void PurePursuit::sendControlCommand()
{
    auto stop = [this]()
    {
        wbb_msgs::msg::Control msg;
        msg->curvature = 0;
        msg->velocity = 0;
        signal_.control->publish(msg);
    };

    if (!(state_.trajectory && state_.bot_pose))
    {
        stop();
        return;
    }

    wbb_msgs::msg::ImagePoint::SharedPtr lh = findLookahead(state_.trajectory, state_.bot_pose);

    if (!lh)
    {
        stop();
        return;
    }

    wbb_msgs::msg::Control msg;
    msg->curvature = calculateCurvature(lh, state_.bot_pose);
    msg->velocity = 1.0;
    signal_.control->publish(msg);
}

} // namespace wbb