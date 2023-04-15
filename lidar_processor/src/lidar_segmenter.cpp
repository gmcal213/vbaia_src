//standard c++ library headers
#include <functional>
#include <memory>
#include <string>
#include <cmath>

//ROS and PCL headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
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

            //publisher of the pose oerientation
            publisher3_ = this->create_publisher<geometry_msgs::msg::Pose>("/pose_estimate", 10);

            //setup segmenter object
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);
            seg.setMaxIterations(1000);
            axis = Eigen::Vector3f(1.0, 0.0, 0.0); //must be normal vector here
            seg.setAxis(axis);
            seg.setEpsAngle(20.0 * 3.1459 / 180.0);

            //intialize point cloud pointer
            pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

            //initialize segmented point cloud pointer
            cloud_filtered = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

            //initialize segmentation pointers
            coefficients = boost::make_shared<pcl::ModelCoefficients>();
            inliers = boost::make_shared<pcl::PointIndices>();

            //send info to the logger
            RCLCPP_INFO(this->get_logger(), "Segmenting from concatenated/filtered and publishing to filtered/segmented");
            RCLCPP_INFO(this->get_logger(), "Publishing hitch angle to hitch_angle");
            RCLCPP_INFO(this->get_logger(), "Publishing pose estimate to pose_estimate");

        }

    private:
        //function to receive point cloud data and convert to pcl format, display it
        void filterPCD(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {   
            
            //convert from ros message to pcl point cloud
            pcl::fromROSMsg(*msg, *pointCloud);
            

            //try to segment a plane
            seg.setInputCloud(pointCloud);
            seg.segment(*inliers, *coefficients);

            //if no plane is found
            if(inliers->indices.size() == 0)
            {

            }
            else
            {

            //extract the plane for segmentation
            extract.setInputCloud(pointCloud);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_filtered);

            //convert ros msg to point cloud
            pcl::toROSMsg(*cloud_filtered, cloudSegROS);

            //publish the segmented plane
            publisher1_->publish(cloudSegROS);

            //set the normal vector of plane. coefficient values 0,1,2,3 -> Ax + By + Cz = D of plane equation
            normal = Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
            normal.normalize();

            //calculate hitch angle using the normal vector of the plane (x-z plane) 
            theta.data = acos(coefficients->values[0]/sqrt(pow((coefficients->values[0]), 2) + pow((coefficients->values[2]), 2))) * (180/3.14159);

            //find middle of point cloud
            cloudMiddle = (cloud_filtered->size())/2;

            //set position to be x,y,z point in middle of point cloud
            pose.position.x = cloud_filtered->points[cloudMiddle].x;
            pose.position.y = cloud_filtered->points[cloudMiddle].y;
            pose.position.z = cloud_filtered->points[cloudMiddle].z;

            //dot and cross product
            dotProduct = axis.dot(normal);
            crossProduct = axis.cross(normal);

            //initialize AngleAxis
            aa = Eigen::AngleAxisf(1 + dotProduct, crossProduct);

            //initialize quaternion
            quat = Eigen::Quaternionf(aa);
            quat.normalize();

            //set orientation 
            pose.orientation.x = double(quat.x());
            pose.orientation.y = double(quat.y());
            pose.orientation.z = double(quat.z());
            pose.orientation.w = double(quat.w());

            //publish the hitch angle
            publisher2_->publish(theta);

            //publish the pose estimate
            publisher3_->publish(pose);
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
        rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher3_;
        Eigen::Vector3f normal;
        Eigen::Vector3f axis;
        float dotProduct;
        Eigen::Vector3f crossProduct;
        Eigen::AngleAxisf aa;
        Eigen::Quaternionf quat;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered;
        pcl::ModelCoefficients::Ptr coefficients;
        pcl::PointIndices::Ptr inliers;
        std_msgs::msg::Float32 theta;
        geometry_msgs::msg::Pose pose;
        int cloudMiddle;

};

//main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarSegmenter>());
    rclcpp::shutdown();
    return 0;
}