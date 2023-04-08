//standard c++ library headers
#include <functional>
#include <memory>
#include <string>
#include <cmath>

//ROS and PCL headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

//define lidar subscriber nodes
class LidarSegmenter : public rclcpp::Node 
{

    //constructor
    public:
        LidarSegmenter() : Node("lidar_segmenter")
        {
            //subscriber to the livox point cloud topic
            subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                "concatenated/filtered", 10, std::bind(&LidarSegmenter::filterPCD, this, std::placeholders::_1)
            );

            //publisher of the segmented point cloud
            publisher1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered/segmented", 10);

            //publisher of the hitch angle
            publisher2_ = this->create_publisher<std_msgs::msg::Float32>("/hitch_angle", 10);

            //setup segmenter object
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);
            seg.setMaxIterations(1000);
            const Eigen::Vector3f axis = Eigen::Vector3f(1.0, 0.0, 0.0);
            seg.setAxis(axis);
            seg.setEpsAngle(20.0 * 3.1459 / 180.0);

            //intialize point cloud pointer
            pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

            //initialize segmented point cloud pointer
            cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

            //initialize segmentation pointers
            coefficients = boost::make_shared<pcl::ModelCoefficients>();
            inliers = boost::make_shared<pcl::PointIndices>();

        }

    private:
        //function to receive point cloud data and convert to pcl format, display it
        void filterPCD(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {   
            //send info to logger
            RCLCPP_INFO(this->get_logger(), "PointCloud Received");

            //initialize point cloud pointer
            //pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud = 
            //    boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            //convert from ros message to pcl point cloud
            pcl::fromROSMsg(*msg, *pointCloud);
            
            //perform segmentation
            //pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            //pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

            //try to segment a plane
            seg.setInputCloud(pointCloud);
            seg.segment(*inliers, *coefficients);

            //if no plane is found
            if(inliers->indices.size() == 0)
            {
                //note no plane was found
                RCLCPP_INFO(this->get_logger(), "No plane found");

            }
            else
            {
            //note the plane found
            RCLCPP_INFO(this->get_logger(), "Size of plane segmented: %d", inliers->indices.size());
            RCLCPP_INFO(this->get_logger(), "Plane equation found: %fx + %fy + %fz + %f = 0", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

            //extract the plane for segmentation
            extract.setInputCloud(pointCloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_filtered);

            //convert ros msg to point cloud
            pcl::toROSMsg(*cloud_filtered, cloudSegROS);

            //publish the segmented plane
            publisher1_->publish(cloudSegROS);

            //calculate hitch angle using the normal vector of the plane (x-z plane)
            theta.data = acos(coefficients->values[0]/sqrt(pow((coefficients->values[0]), 2) + pow((coefficients->values[2]), 2))) * (180/3.14159);

            //RCLCPP_INFO(this->get_logger(), "Hitch Angle Found: %f", theta.data);

            //publish the hitch angle
            publisher2_->publish(theta);
            }
             
        }

        

        // field declarations
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZI> seg;
        pcl::ExtractIndices<pcl::PointXYZI> extract;
        sensor_msgs::msg::PointCloud2 cloudSegROS;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher1_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher2_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::PointIndices::Ptr inliers;
        std_msgs::msg::Float32 theta;

};

//main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSegmenter>());
    rclcpp::shutdown();
    return 0;
}