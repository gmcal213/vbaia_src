//standard c++ library headers
#include <functional>
#include <memory>
#include <string>

//ROS and PCL headers
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

//define lidar subscriber nodes
class LidarVisualizer : public rclcpp::Node 
{

    //constructor
    public:
        LidarVisualizer() : Node("lidar_visualizer")
        {
            //add parameter for the topic name that the visualizer is subscribed to
            auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
            param_desc.description = "Which point cloud topic the visualizer is subscribed to. Change at startup via cli";

            //declare parameter
            this->declare_parameter("topic_name", "livox/lidar", param_desc);


            //subscriber to the parameter defined point cloud topic
            subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                this->get_parameter("topic_name").get_parameter_value().get<std::string>(), 10, std::bind(&LidarVisualizer::processPCD, this, std::placeholders::_1)
            );

            //initialize point cloud
            pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

            //initialize call count
            callCount = 1;

            //send info to logger
            RCLCPP_INFO(this->get_logger(), "Visualizing Point cloud at topic %s", this->get_parameter("topic_name").get_parameter_value().get<std::string>());
            
        }

    private:
        //function to receive point cloud data and convert to pcl format, display it
        void processPCD(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
        {   

            //convert from ros message to pcl point cloud
            pcl::fromROSMsg(*msg, *pointCloud);
            
            
            //initialize point cloud to visualizer or update existing cloud
            if(callCount == 1)
            {
                //send info to logger
                RCLCPP_INFO(this->get_logger(), "Initializing Visualizer");
                
                //initialize point cloud viewer
                viewer = initVis(pointCloud);

                //cloud initialized, just update it now
                callCount--;
            }
            else
            {
                //update exisiting point cloud
                viewer->updatePointCloud<pcl::PointXYZI> (pointCloud, "point cloud");
            }

            //spin lidar viewer
            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
            
        }

        //function to initialize and display point cloud data
        pcl::visualization::PCLVisualizer::Ptr initVis (pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud)
        {
            // --------------------------------------------
            // -----Open 3D viewer and add point cloud-----
            // --------------------------------------------
            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (0, 0, 0);
            viewer->addPointCloud<pcl::PointXYZI> (cloud, "point cloud");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
            viewer->addCoordinateSystem (1.0);
            viewer->initCameraParameters ();
            return (viewer);
        }

        // field declarations
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_;
        pcl::visualization::PCLVisualizer::Ptr viewer;
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        int callCount;
        


};

//main function
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarVisualizer>());
    rclcpp::shutdown();
    return 0;
}