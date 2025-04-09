#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <stdio.h>
#include <math.h>
#include <filesystem>
#include <fstream>


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
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_srvs/srv/trigger.hpp>
#include <yaml-cpp/yaml.h>


using namespace cv;

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;
 
class Create2DMap : public rclcpp::Node
{
    

    double poseX = 0;
    double poseY = 0;
    double poseA = 0;

    int width = 1000;
    int height = 1000;

    Mat map1;
    Mat map;    

  public:

    Create2DMap()
    : Node("create_2d_map")
    {
      subscription_3d_map_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/map", 10, std::bind(&Create2DMap::topic_callback, this, _1));

      subscription_points_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/velodyne_points", 10, std::bind(&Create2DMap::callback_velodyne_points_, this, _1));

      //subscription_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      //"/current_pose", 10, std::bind(&Create2DMap::callback_pose_, this, _1));

      subscription_pose_ = this->create_subscription<visualization_msgs::msg::Marker>(
      "/poseAuto", 10, std::bind(&Create2DMap::callback_pose_, this, _1));

      save_image_service_ = this->create_service<std_srvs::srv::Trigger>(
      "save_2d_map", std::bind(&Create2DMap::save_image_callback, this, std::placeholders::_1, std::placeholders::_2));

      publisher_merged = this->create_publisher<sensor_msgs::msg::PointCloud>("merged_points", 10);

    }
 
  private:

    sensor_msgs::msg::PointCloud velodyne_points;
    sensor_msgs::msg::PointCloud points_for_map_;

    visualization_msgs::msg::Marker pose_c;

    void callback_pose_(const visualization_msgs::msg::Marker msg){
        pose_c = msg;
    }

    void callback_velodyne_points_(const sensor_msgs::msg::PointCloud2 msg)
    {
        //cout << "fff" << endl;
        sensor_msgs::msg::PointCloud out_pointcloud;
        sensor_msgs::msg::PointCloud filter_point;

        sensor_msgs::convertPointCloud2ToPointCloud(msg, out_pointcloud);
        double filter = 100;

        for(int i = 0; i < int(msg.width*msg.height); i ++){
        
            if(std::isnan(out_pointcloud.points[i].z) == false &&(out_pointcloud.points[i].y > -filter && out_pointcloud.points[i].y < filter) &&(out_pointcloud.points[i].x > -filter && out_pointcloud.points[i].x < filter) && out_pointcloud.points[i].z && out_pointcloud.points[i].z > -1.2 && out_pointcloud.points[i].z < 2 && ((out_pointcloud.points[i].x > 1) || out_pointcloud.points[i].x < -1 || out_pointcloud.points[i].y > 3)){

                geometry_msgs::msg::Point32 point;

                double* euler = euler_from_quaternion(pose_c.pose.orientation.x, pose_c.pose.orientation.y, pose_c.pose.orientation.z, pose_c.pose.orientation.w);
                poseA = euler[2];

                point.x = out_pointcloud.points[i].x*std::cos(poseA) - out_pointcloud.points[i].y*std::sin(poseA) + pose_c.pose.position.x;
                point.y = out_pointcloud.points[i].x*std::sin(poseA) + out_pointcloud.points[i].y*std::cos(poseA) + pose_c.pose.position.y;
                point.z = 0.0;

                filter_point.points.push_back(point);
            }
        }
        
        velodyne_points = filter_point;
    }


    // This method executes every 500 milliseconds
    void topic_callback(const sensor_msgs::msg::PointCloud2 msg)
    {
        points_for_map_.points.insert(points_for_map_.points.end(), velodyne_points.points.begin(), velodyne_points.points.end());

        points_for_map_.header.frame_id = msg.header.frame_id;
        publisher_merged -> publish(points_for_map_);
    }

    bool save_image()
    {
        cout << "СОХРАНЕНИЕ ИЗОБРАЖЕНИЯ" << endl;
        cout << points_for_map_.points.size() << endl;
        if (points_for_map_.points.size() < 1) return false;
        
        int inf = std::numeric_limits<int>::max();
        double maxX = 0;
        double minX = inf;
        double maxY = 0;
        double minY = inf;

        sensor_msgs::msg::PointCloud points_for_map;
        points_for_map = points_for_map_;

        for (size_t i = 0; i < points_for_map.points.size(); ++i) {
            if (true) {
                points_for_map.points[i].x *= 20.0;
                points_for_map.points[i].y *= 20.0;

                if (points_for_map.points[i].x > maxX) maxX = points_for_map.points[i].x;
                if (points_for_map.points[i].y > maxY) maxY = points_for_map.points[i].y;
                if (points_for_map.points[i].x < minX) minX = points_for_map.points[i].x;
                if (points_for_map.points[i].y < minY) minY = points_for_map.points[i].y;

            }
        }

        int height = std::abs(static_cast<int>(maxX - minX)) + 1;
        int width = std::abs(static_cast<int>(maxY - minY)) + 1;

        std::cout << "Height: " << height << std::endl;
        std::cout << "Width: " << width << std::endl;

        // Создание изображения
        cv::Mat image_map = cv::Mat::zeros(width, height, CV_8UC3);
        cv::cvtColor(image_map, image_map, cv::COLOR_BGR2GRAY);

        // Масштабирование точек
        for (size_t i = 0; i < points_for_map.points.size(); ++i) {

          points_for_map.points[i].x = 0 + (points_for_map.points[i].x - minX) * (height - 0) / (maxX - minX);
          points_for_map.points[i].y = 0 + (points_for_map.points[i].y - minY) * (width - 0) / (maxY - minY);
        }

        // Рисование кругов на изображении
        for (size_t i = 0; i < points_for_map.points.size(); ++i) {

            //std::cout << points_for_map_.points[i].x << std::endl;
            //std::cout << points_for_map_.points[i].y << std::endl;
            cv::circle(image_map, cv::Point(static_cast<int>(points_for_map.points[i].x), static_cast<int>(points_for_map.points[i].y)), 40, cv::Scalar(0, 0, 0), -1);
        
        }

        // Рисование меньших кругов
        for (size_t i = 0; i < points_for_map.points.size(); ++i) {
        
            //std::cout << points_for_map_.points[i].x << std::endl;
            //std::cout << points_for_map_.points[i].y << std::endl;
            cv::circle(image_map, cv::Point(static_cast<int>(points_for_map.points[i].x), static_cast<int>(points_for_map.points[i].y)), 2, cv::Scalar(100, 100, 100), -1);
        
        }

        // Поворот и сохранение изображения
        cv::flip(image_map, image_map, 1);
        cv::rotate(image_map, image_map, cv::ROTATE_180);

        double originX = minX/20.0;
        double originY = minY/20.0;

        YAML::Emitter out;
        out << YAML::BeginMap;
        out << YAML::Key << "image" << YAML::Value << "map.pgm";
        out << YAML::Key << "origin" << YAML::Value << YAML::Flow << YAML::BeginSeq << originX << originY << 0 << YAML::EndSeq;
        out << YAML::EndMap;

        std::ofstream fout("/home/polytech/ros2_humble/src/robocross2023/maps/map.yaml");
        fout << out.c_str();
        fout.close();

        cv::imwrite("/home/polytech/ros2_humble/src/robocross2023/maps/map.pgm", image_map);
        return true;

    }

    void save_image_callback(const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
                            std_srvs::srv::Trigger::Response::SharedPtr response)
    {        
        response->success = save_image();
    }

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

      //std::cout << euler[0]  << " " << euler[1] << " " << euler[2] << std::endl;

      return euler;
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_3d_map_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_points_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr subscription_pose_;
    //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_pose_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_image_service_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr publisher_merged;

    
};



int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<Create2DMap>());
  rclcpp::shutdown();
  return 0;
}
