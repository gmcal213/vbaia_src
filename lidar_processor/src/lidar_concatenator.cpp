//standard c++ library headers
#include <functional>
#include <memory>
#include <string>

//ROS and PCL headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>


//define lidar subscriber nodes
class LidarConcatenator : public rclcpp::Node 
{

    //constructor
    public:
        LidarConcatenator() : Node("lidar_concatenator")
        {
            //add parameter for number of point clouds to concatenate together
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Number of point clouds to concatenate. Each one takes .1 seconds and has 10,000 points in it";

            //declare parameter
            this->declare_parameter("num_point_clouds", 4, param_desc);

            //subscriber to the livox point cloud topic
            subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "livox/lidar", 10, std::bind(&LidarConcatenator::concatPCD, this, std::placeholders::_1)
            );

            //publisher of the filtered data
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar/concatenated", 10);

            //initialize count
            count = this->get_parameter("num_point_clouds").get_parameter_value().get<int>();

            //initialize point cloud 
            pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

        }

    private:
        //function to receive point cloud data and convert to pcl format, display it
        void concatPCD(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {   
            //send info to logger
            RCLCPP_INFO(this->get_logger(), "PointCloud Received");

            //initialize point cloud pointer
            //pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = 
            //    boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            //convert from ros message to pcl point cloud
            pcl::fromROSMsg(*msg, *pointCloud);

            if(count == this->get_parameter("num_point_clouds").get_parameter_value().get<int>())
            {   
                //RCLCPP_INFO(this->get_logger(), "Concatenation initiated");
                //create concatenated point cloud
                cloudConcat = *pointCloud;

                //decrement counter
                count--;
            }
            else if(count > 0 && count < this->get_parameter("num_point_clouds").get_parameter_value().get<int>())
            {
                //RCLCPP_INFO(this->get_logger(), "Concatenating points");

                cloudConcat += *pointCloud;

                //decrement counter
                count--;

            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Publishing point cloud");   
                //publish concatenated point cloud data to a new topic
                pcl::toROSMsg(cloudConcat, cloudConcatROS);

                publisher_->publish(cloudConcatROS);

                //clearing point cloud
                cloudConcat.clear();

                //reset counter
                count = this->get_parameter("num_point_clouds").get_parameter_value().get<int>();
            }
            
            
        }

        

        // field declarations
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        sensor_msgs::msg::PointCloud2 cloudConcatROS;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
        int count;
        pcl::PointCloud<pcl::PointXYZI> cloudConcat;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;


};

//main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarConcatenator>());
    rclcpp::shutdown();
    return 0;
}