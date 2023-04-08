//standard c++ library headers
#include <functional>
#include <memory>
#include <string>
#include <chrono>

//ROS and Stats headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/subscription_options.hpp>

class LidarStats : public rclcpp:: Node
{
public:
    LidarStats() : Node("lidar_stats")
    {
        // manually enable topic statistics via options
        auto options = rclcpp::SubscriptionOptions();
        options.topic_stats_options.state = rclcpp::TopicStatisticsState::Enable;

        // configure the collection window and publish period (default 1s)
        options.topic_stats_options.publish_period = std::chrono::seconds(2);

        // configure the topic name (default '/statistics')
        options.topic_stats_options.publish_topic = "/lidar_stats";
        
        //subscriber to the livox point cloud topic
        subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/hitch_angle", 10, std::bind(&LidarStats::topic_callback, this, std::placeholders::_1), options
        );
    }
private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "PointCloud Received");
    }
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;

};

// main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarStats>());
    rclcpp::shutdown();
    return 0;
}