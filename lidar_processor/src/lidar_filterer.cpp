//standard c++ library headers
#include <functional>
#include <memory>
#include <string>

//ROS and PCL headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

//define lidar subscriber nodes
class LidarFilterer : public rclcpp::Node 
{

    //constructor
    public:
        LidarFilterer() : Node("lidar_filterer")
        {
            //add parameter for minimum on the x-direction for the passthrough filter
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Set minimum in the x-direction of what points get filtered out";

            //declare parameter
            this->declare_parameter("x_min", 0.0, param_desc);

            //add parameter for maximum on the x-direction for the passthrough filter
            param_desc.description = "Set maximum in the x-direction of what points get filtered out";

            //declare parameter
            this->declare_parameter("x_max", 2.0, param_desc);


            //subscriber to the livox point cloud topic
            subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "lidar/concatenated", 10, std::bind(&LidarFilterer::filterPCD, this, std::placeholders::_1)
            );

            //publisher of the filtered data
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("concatenated/filtered", 10);

            
            //initialize filter
            pass.setFilterFieldName("x");
            pass.setFilterLimits(this->get_parameter("x_min").get_parameter_value().get<float>(), this->get_parameter("x_max").get_parameter_value().get<float>());

            //initialize point cloud pointer
            pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

            //initialize filtered point cloud pointer
            cloud_filtered =  boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

            //send info to logger
            RCLCPP_INFO(this->get_logger(), "Filtering point clouds from lidar/concatenated and publishing to concatenated/filtered");

        }

    private:
        //function to receive point cloud data and convert to pcl format, display it
        void filterPCD(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {   
            
            //convert from ros message to pcl point cloud
            pcl::fromROSMsg(*msg, *pointCloud);

            //perform filtering
            pass.setInputCloud(pointCloud);
            pass.filter(*cloud_filtered);

            //publish point filtered point cloud data to a new topic
            pcl::toROSMsg(*cloud_filtered, cloud_filtered_ROS);
            publisher_->publish(cloud_filtered_ROS);
            
        }

        

        // field declarations
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        pcl::PassThrough<pcl::PointXYZI> pass;
        sensor_msgs::msg::PointCloud2 cloud_filtered_ROS;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;


};

//main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilterer>());
    rclcpp::shutdown();
    return 0;
}