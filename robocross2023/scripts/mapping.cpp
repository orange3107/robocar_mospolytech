#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <stdio.h>
#include <math.h>
#include <filesystem>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.

//#include "grid_map_msgs/GridMap.h"

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 

#include "visualization_msgs/msg/marker.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <vector>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tf2/exceptions.h"


using namespace cv;

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;
 
class LocalMap : public rclcpp::Node
{
    

    double poseX = 0;
    double poseY = 0;
    double poseA = 0;

    int width = 1000;
    int height = 1000;

    Mat map1;
    Mat map;

    std::vector<double> global_origin;   



  public:

    LocalMap()
    : Node("create_local_map")
    {

      auto path = std::filesystem::current_path();

      std::string map_name = "map";

      std::string package_share_directory = ament_index_cpp::get_package_share_directory("robocross2023") + "/maps/";
      
      std::string path_map = package_share_directory + map_name + ".pgm";
      std::string path_yaml = package_share_directory + map_name + ".yaml";

      std::ifstream fin(path_yaml);

      if (!fin.is_open()) {
          std::cerr << "Failed to open file for reading." << std::endl;
          return ;
      }

      YAML::Node config = YAML::Load(fin);
      global_origin = config["origin"].as<std::vector<double>>();

      for (const auto& val : global_origin) {
          std::cout << val << " ";
      }
      
      cout << package_share_directory << endl;

      map1 = cv::imread(path_map, CV_8UC1);
      map = cv::imread(path_map, CV_8UC1);
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&LocalMap::topic_callback, this, _1));

      poseSubscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
      "/poseAuto", 10, std::bind(&LocalMap::pose_topic_callback, this, _1));

// 
      globalMapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", 10, std::bind(&LocalMap::gm_topic_callback, this, _1));

      publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("local_map1", 10);

      publisher1_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("gm_map", 10);

      publisher_image_map_ = this->create_publisher<sensor_msgs::msg::Image>("image_map", 10);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      //cv::cvtColor(map1, map1, cv::COLOR_BGR2RGB);

      cv::flip(map1, map1, 0);

      cv::flip(map, map, 0);

    }
 
  private:


    void gm_topic_callback(const nav_msgs::msg::OccupancyGrid msg)
    {
      //nav_msgs::msg::OccupancyGrid

      //map = msg.data;
    }

    void pose_topic_callback(const visualization_msgs::msg::Marker msg)
    {
      poseX = msg.pose.position.x * 20.0;
      poseY = msg.pose.position.y * 20.0;
      double* euler = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
      poseA = euler[2];
    }

    // double* rotate_direction_by_q(double *a, sensor_msgs::msg::PointCloud b){

    //   double w= b.transform.rotation.w;
    //   double x= b.transform.rotation.x;
    //   double y= b.transform.rotation.y;
    //   double z= b.transform.rotation.z;
    //   static double out[3];

    //   out[0] =  a[0] * (1 - 2*y*y - 2*z*z) + a[1] * (2*x*y - 2*z*w) + a[2] * (2*x*z + 2*y*w) + b.transform.translation.x;
    //   out[1] =  a[0] * (2*x*y + 2*z*w) + a[1] * (1 - 2*x*x - 2*z*z) + a[2] * (2*y*z - 2*x*w) + b.transform.translation.y;
    //   out[2] =  a[0] * (2*x*z - 2*y*w) + a[1] * (2*y*z + 2*x*w) + a[2] * (1 - 2*x*x - 2*y*y) + b.transform.translation.z;

    //   //std::cout  << "NEW" << " " << out[0] << " " << out[1] << " " << out[2]  << " " << a[0] << " " << a[1] << " " << a[2] << std::endl;

    //   return out;
    // }

    double* euler_from_quaternion(double x, double y, double z, double w){

      static double euler[3];
      double t0 = +2.0 * (w * x + y * z);
      double t1 = +1.0 - 2.0 * (x * x + y * y);
      double roll_x = std::atan2(t0, t1);

      double t2 = +2.0 * (w * y - z * x);

      if(t2 > +1.0){
        t2 = +1.0;
      }

      if(t2 < -1.0){
        t2 = -1.0;
      }

      double pitch_y = std::asin(t2);

      double t3 = +2.0 * (w * z + x * y);
      double t4 = +1.0 - 2.0 * (y * y + z * z);
      double yaw_z = std::atan2(t3, t4);

      euler[0] = roll_x;
      euler[1] = pitch_y;
      euler[2] = yaw_z;

      std::cout << euler[0]  << " " << euler[1] << " " << euler[2] << std::endl;

      return euler;
    }

    double* rotate_direction_by_q(double *a, geometry_msgs::msg::TransformStamped b){

      double w= b.transform.rotation.w;
      double x= b.transform.rotation.x;
      double y= b.transform.rotation.y;
      double z= b.transform.rotation.z;
      static double out[3];

      out[0] =  a[0] * (1 - 2*y*y - 2*z*z) + a[1] * (2*x*y - 2*z*w) + a[2] * (2*x*z + 2*y*w) + b.transform.translation.x;
      out[1] =  a[0] * (2*x*y + 2*z*w) + a[1] * (1 - 2*x*x - 2*z*z) + a[2] * (2*y*z - 2*x*w) + b.transform.translation.y;
      out[2] =  a[0] * (2*x*z - 2*y*w) + a[1] * (2*y*z + 2*x*w) + a[2] * (1 - 2*x*x - 2*y*y) + b.transform.translation.z;

      //std::cout  << "NEW" << " " << out[0] << " " << out[1] << " " << out[2]  << " " << a[0] << " " << a[1] << " " << a[2] << std::endl;

      return out;
    }

    // This method executes every 500 milliseconds
    void topic_callback(const sensor_msgs::msg::PointCloud2 msg)
    {
      
      map = map1.clone();
      

      //Point p1(0, 0); 
      //Point p2(width,height); 
   
      //cv::rectangle(map, p1, p2, Scalar(0, 0, 0), -1, LINE_8);

      sensor_msgs::msg::PointCloud out_pointcloud;

      sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
      int x;
      int y;

      //map = map1;
      int x0 = 20;
      int y0 = 20;

      for(int i = 0; i < int(msg.width*msg.height); i ++){
        //std::cout << out_pointcloud.points[i].x << " " << out_pointcloud.points[i].y << " " << out_pointcloud.points[i].z << std::endl;

        // geometry_msgs::msg::TransformStamped angles = tf_buffer_->lookupTransform("velodyne", "map", tf2::TimePointZero);

        // double point[3] {out_pointcloud.points[i].x, out_pointcloud.points[i].y, out_pointcloud.points[i].z};

        // double* newPoint = rotate_direction_by_q(point, angles);

        int filter = 10;
        
        if(std::isnan(out_pointcloud.points[i].z) == false &&   (out_pointcloud.points[i].x > -filter && out_pointcloud.points[i].x < filter)&& (out_pointcloud.points[i].y > -filter && out_pointcloud.points[i].y < filter) && out_pointcloud.points[i].z > -1.2 && out_pointcloud.points[i].z < 10 && ((out_pointcloud.points[i].x > 1.7) || out_pointcloud.points[i].x < -1.7 || out_pointcloud.points[i].y > 3)){

          x = out_pointcloud.points[i].x*20.0;// + global_origin[0]*20;// + global_origin[0];
          y = out_pointcloud.points[i].y*20.0;// + global_origin[1]*20// + global_origin[1];

          double x1 = x*std::cos(poseA) - y*std::sin(poseA) + poseX - global_origin[0]*20;
          double y1 = x*std::sin(poseA) + y*std::cos(poseA) + poseY - global_origin[1]*20;

          Point p1(x1, y1); 
          cv::circle(map, p1, 5, Scalar(100, 100, 100), -1);
        }
      }

        //cv::imwrite("/home/ilya22/ros2_humble/src/robocross2023/maps/local_map.pgm", map); 
        //cv2.imwrite('/home/ilya22/ros2_humble/src/robocross2023/maps/my_map.pgm', image_map)
        float centrX = width/2;
        float centrY = height/2;

        Mat gray_map = map;

        //cv::cvtColor(map, gray_map, cv::COLOR_BGR2GRAY);
        

        nav_msgs::msg::OccupancyGrid grid;
        rclcpp::Time now = this->get_clock()->now();
    
        grid.header.stamp = now;
        grid.info.map_load_time = now;
        grid.info.resolution = 0.05;
        grid.header.frame_id = "map";
        grid.info.height = map.rows;
        grid.info.width = map.cols;
        grid.info.origin.position.x = global_origin[0];
        grid.info.origin.position.y = global_origin[1];
        grid.info.origin.position.z = 0.0;
        

        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(gray_map.data, gray_map.data + gray_map.total());
        //std::cout << "OK" << std::endl;
        publisher_->publish(grid);
    
        grid.header.stamp = now;
        grid.info.map_load_time = now;
        grid.info.resolution = 0.05;
        grid.header.frame_id = "map";
        grid.info.height = map1.rows;
        grid.info.width = map1.cols;
        grid.info.origin.position.x = 0;//-centrX/20;
        grid.info.origin.position.y = 0;//-centrY/20;
        grid.info.origin.position.z = 0.0;

        grid.info.origin.orientation.x = 0.0;
        grid.info.origin.orientation.y = 0.0;
        grid.info.origin.orientation.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.assign(map1.data, map1.data + map1.total());
        //std::cout << "OK" << std::endl;
        publisher1_->publish(grid);


        //imshow("Display window", map1);
        //waitKey(1);

        sensor_msgs::msg::Image::SharedPtr msg_;
        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", gray_map).toImageMsg();
        publisher_image_map_->publish(*msg_.get());

    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr poseSubscription_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_map_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher1_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr globalMapSubscription_;
    
     
};



int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalMap>());
  rclcpp::shutdown();
  return 0;
}
