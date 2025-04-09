#include <chrono> // Date and time
#include <functional> // Arithmetic, comparisons, and logical operations
#include <memory> // Dynamic memory management
#include <string> // String functions
#include <stdio.h>
#include <math.h>
#include <filesystem>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "geometry_msgs/msg/twist.hpp" 


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include "sensor_msgs/msg/joint_state.hpp"

#include <nav_msgs/msg/path.hpp>

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.

#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/point_cloud_conversion.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> 
#include <opencv2/highgui/highgui.hpp> 

#include "visualization_msgs/msg/marker.hpp"


using namespace cv;

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;

class myPoint{
public:

  double x, y, a;

};

class myTwist{
public:

  double linear, angular;

};
 
class PurePursuit : public rclcpp::Node
{
    

    myPoint car; 

    int width = 1000;
    int height = 1000;

    geometry_msgs::msg::PoseStamped *pathArr;

    nav_msgs::msg::Path pathIn;

    Mat map1;
    Mat map;

    double pi = 2 * acos(0.0);

    int iAct = 0;
    myPoint pr;

    double wheelAngle = 0;

    bool vels_ch = false;

    bool pedal = true;

    rclcpp::Time timer;
    bool start_timer = false;
    bool stop_timer = false;
    string nav_status = "None";

    //Mat map = cv::imread("/home/ilya22/ros2_humble/src/robocross2023/maps/my_map.pgm");
    
  public:

    PurePursuit()
    : Node("pure_pursuit")
    {
      pathSubscription_ = this->create_subscription<nav_msgs::msg::Path>("/global_path", 10, std::bind(&PurePursuit::global_path_callback, this, _1));

      localPathSubscription_ = this->create_subscription<nav_msgs::msg::Path>("/pathWay", 10, std::bind(&PurePursuit::local_path_callback, this, _1));

      velsSubscription_ = this->create_subscription<sensor_msgs::msg::JointState>("/vels", 10, std::bind(&PurePursuit::join_state_callback, this, _1));

      pedalSubscription_ = this->create_subscription<std_msgs::msg::Bool>("/stop_pedal_status", 10, std::bind(&PurePursuit::pedal_state_callback, this, _1));

      navSubscription_ = this->create_subscription<std_msgs::msg::String>("/nav_status", 10, std::bind(&PurePursuit::nav_status_collback, this, _1));

      
        

      publishPP_ = this->create_publisher<visualization_msgs::msg::Marker>("/p_p", 10);

      publishPP1_ = this->create_publisher<visualization_msgs::msg::Marker>("/p_p1", 10);

      publishPP2_ = this->create_publisher<visualization_msgs::msg::Marker>("/p_p2", 10);

      publishTwist_ = this-> create_publisher <geometry_msgs::msg::Twist> ("pure_cmd_vel", 10);

      path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("pure_pursuit_path", 10);

      poseSubscription_ = this->create_subscription<visualization_msgs::msg::Marker>(
      "/odomAuto", 10, std::bind(&PurePursuit::pose_topic_callback, this, _1));

      timer_ = this->create_wall_timer(5ms, std::bind(&PurePursuit::timer_callback, this));
    }
 
  private:

    void pose_topic_callback(const visualization_msgs::msg::Marker msg){

      car.x = msg.pose.position.x;
      car.y = msg.pose.position.y;
      double* euler = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
      car.a = euler[2];
    }

    void nav_status_collback(const std_msgs::msg::String msg)
    {
      nav_status = msg.data;
    }

    void pedal_state_callback(const std_msgs::msg::Bool msg)
    {
      pedal = msg.data;
    }

    void join_state_callback(const sensor_msgs::msg::JointState msg)
    {

      if(abs(msg.velocity[0]) + abs(msg.velocity[1]) < 0.1) vels_ch = false;

      else vels_ch = true;

    }

    void global_path_callback(const nav_msgs::msg::Path msg)
    {
      
      pathIn = msg;

      //cout << pathIn.poses[0].pose.position.x  << endl;
    }

    void local_path_callback(const nav_msgs::msg::Path msg)
    {

      usleep(1);
      
      iAct = 0;
      //cout << iAct << endl;

      //pr.x = pathIn.poses[0].pose.position.x;
      //pr.y = pathIn.poses[0].pose.position.y;

      

      //cout << pathIn.poses[0].pose.position.x  << endl;
    }

    void timer_callback(){

      if(size(pathIn.poses) < 1) {
        return;
      } 

      double L = 3;
      double seg = 0.01;
      int len = int(size(pathIn.poses));
      double length = 0;
      

      double angleVec = 0;
      myPoint prevVec;
      
      //cout << typeid(pathIn.poses[0].pose.position).name() << endl;


      for(int i = iAct; i < len - 1; i++){


              visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "/map";
      marker.type = marker.SPHERE;
      marker.action = marker.ADD;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.pose.position.x = pathIn.poses[i].pose.position.x;
      marker.pose.position.y = pathIn.poses[i].pose.position.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      publishPP1_ -> publish(marker);

      //myTwist mTwist = getTwist(car, pr);


      marker.header.frame_id = "/map";
      marker.type = marker.SPHERE;
      marker.action = marker.ADD;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.pose.position.x = pathIn.poses[i+1].pose.position.x;
      marker.pose.position.y = pathIn.poses[i+1].pose.position.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      publishPP2_ -> publish(marker);

      //myTwist mTwist = getTwist(car, pr);

        double segment = seg;

        auto p1 = pathIn.poses[i].pose.position;
        auto p2 = pathIn.poses[i + 1].pose.position;

        pr = steerPoint(p1, p2, segment);

        //cout << i << endl;

        myPoint vec;
        vec.x = pathIn.poses[i+1].pose.position.x - pathIn.poses[i].pose.position.x;
        vec.y = pathIn.poses[i+1].pose.position.y - pathIn.poses[i].pose.position.y;

        if(abs(vec.x) < 0.001 && abs(vec.y) < 0.001){
          continue;
          //cout << "fff" << endl;

        }


        if(abs(vecAngle(vec, prevVec)) > pi/2 && IsPointInCircle(car.x, car.y, pr.x, pr.y, 0.5) == false && i > 0){

          //cout << "1f" << " " << abs(vecAngle(vec, prevVec)) << " " << i << endl;
          iAct = i;
          break;

        }

        prevVec = vec;

        if(distance(p1, p2) > L){
          p1.x= pr.x;
          p1.y= pr.y;

          segment = L;

          //cout << "BIG" << " " << endl;
        }

        while(distance(p1, p2) > segment){
        
          pr = steerPoint(p1, p2, segment);
          segment += seg;
          length += seg;

          if(distance(car, pr) > L){
            //cout << "2f" << " " <<  i << " " << iAct << endl;
            iAct = i;
            break;
          }
          //if(length > L){
            //break;
          //}

        }
        //if(length > L){
          //break;
        //}

        if(distance(pr, car) > L){
          iAct = i;
          //cout << "kill" << endl;
          break;

        }
      }

      //cout << "done" << " " << pr.x << " " << pr.y << endl;

      visualization_msgs::msg::Marker marker;

      marker.header.frame_id = "/map";
      marker.type = marker.SPHERE;
      marker.action = marker.ADD;
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.pose.position.x = pr.x;
      marker.pose.position.y = pr.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 0.0;
      publishPP_ -> publish(marker);

      myTwist mTwist = getTwist(car, pr);

      if ((nav_status != "done" && nav_status != "Go") || size(pathIn.poses) == 0 || IsPointInCircle(car.x, car.y, pathIn.poses[size(pathIn.poses)-1].pose.position.x, pathIn.poses[size(pathIn.poses)-1].pose.position.y, 0.5)){
        
        mTwist.linear = 0;
        //cout << nav_status << endl;
      }

      if(!vels_ch && !pedal){

        if (mTwist.linear > 0)mTwist.linear = 2;

        else mTwist.linear = -2;
          
      }


      //cout << mTwist.angular << endl;


      geometry_msgs::msg::Twist twist;

      twist.linear.x = mTwist.linear;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;

      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = mTwist.angular;

      publishTwist_ -> publish(twist);

    }



    double distance(auto p1, auto p2){
      double dist = sqrt(pow(p2.x-p1.x, 2) + pow(p2.y-p1.y, 2));
      return dist;
    }

    

    myPoint steerPoint(auto locStart, auto locEnd, double stp){

      myPoint point;
      double vec[2] {locEnd.x - locStart.x, locEnd.y - locStart.y};
      double offset[2] = {0.0,0.0};

      if(vec[0]+vec[1] == 0){

        offset[0] = 0;
        offset[1] = 0;

      }
      else{

        double* unVec = UnVector(vec);
        offset[0] = stp*unVec[0];
        offset[1] = stp*unVec[1];
        

      }

        point.x = offset[0] + locStart.x, 
        point.y = offset[1] + locStart.y;

        return point;
    }

    double* UnVector(double* vec){

      double dist = sqrt(pow(vec[0], 2) + pow(vec[1], 2));    

      static double u_hat[2];

       u_hat[0] = vec[0]/dist;
       u_hat[1] = vec[1]/dist;

      //cout  << dist << " " << vec[0] << " " << vec[1] << endl;
      return u_hat;
    }

    myTwist getTwist(myPoint cCar, myPoint point){

      myPoint vecAuto;
      myPoint vecAutoPoint;

      vecAuto.x = cos(cCar.a + pi/2);
      vecAuto.y = sin(cCar.a + pi/2);

      cCar.a += pi/2;
      

      if (cCar.a < 0){
        cCar.a = pi/2 - abs(cCar.a)+ 3*pi/2;
      }

      vecAutoPoint.x = point.x - cCar.x;
      vecAutoPoint.y = point.y - cCar.y;

      //cout << cCar.x << " " << cCar.y << endl;

      double engleAutoPoint = atan2(vecAuto.x*vecAutoPoint.y - vecAuto.y*vecAutoPoint.x, vecAuto.x*vecAutoPoint.x + vecAuto.y*vecAutoPoint.y);

      double L = distance(cCar, point);

      double wheelBase = 2.64;

      double angle = atan((2*wheelBase*sin(engleAutoPoint))/L); 

      //cout << angle << endl;

      myTwist twist;

      bool st = true;

      cout << vels_ch << " " << !pedal << endl;


      if (abs(engleAutoPoint) < pi/2){

        twist.linear = 1;
      
        if(false){
          twist.angular = wheelAngle;
        }

        else{
          twist.angular = angle;
          wheelAngle = angle;
        }

      }

      else{

        st = false;
        twist.linear = -1;
        
        if(false){
        twist.angular = wheelAngle;
        }
        
        else{
          twist.angular = -angle;
          wheelAngle = -angle;
        }
        
      }


      //publish_path(angle, L, wheelBase, st);

      return twist;

    } 

    bool IsPointInCircle(double x, double y, double xc, double yc, double r){

      bool b = false;

      if(pow((pow((x-xc),2)+pow((y-yc),2)), 0.5) <= r){
        b = true;
      }

      return b;
    }

    void publish_path(double steering_angle, double lookahead_distance, double wheelbase, bool st)
    {
        // Получение текущего угла поворота колес
        // Расстояние до точки стремления

        // Параметры автомобиля
        // Колесная база автомобиля

        // Создание сообщения Path
        auto path_msg = std::make_shared<nav_msgs::msg::Path>();
        path_msg->header.frame_id = "odometry";
        path_msg->header.stamp = this->now();

        // Создание начальной точки (центра задней оси автомобиля)
        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header.frame_id = "odometry";
        start_pose.header.stamp = this->now();
        start_pose.pose.position.x = 0.0;
        start_pose.pose.position.y = 0.0;
        start_pose.pose.position.z = 0.0;
        start_pose.pose.orientation.w = 1.0;

        path_msg->poses.push_back(start_pose);

        // // Рассчет точек дуги
        // int num_points = 50;
        // for (int i = 1; i <= num_points; ++i)
        // {
        //     double arc_length = lookahead_distance * (static_cast<double>(i) / num_points);
        //     double theta = arc_length * tan(steering_angle) / wheelbase;

        //     double x, y;
        //     if (std::abs(theta) > 1e-5)
        //     {
        //         double radius = arc_length / theta;
        //         x = radius * sin(theta);
        //         y = radius * (1 - cos(theta));
        //     }
        //     else
        //     {
        //         x = arc_length;
        //         y = 0.0;
        //     }

        //     double radians = 90 * M_PI / 180.0;

        //     double x_new = x * cos(radians) - y * sin(radians);
        //     double y_new = x * sin(radians) + y * cos(radians);

        //     if(st == false){
        //       y_new = -y_new;
        //     }

        //     geometry_msgs::msg::PoseStamped pose;
        //     pose.header.frame_id = "odometry";
        //     pose.header.stamp = this->now();
        //     pose.pose.position.x = x_new;
        //     pose.pose.position.y = y_new;
        //     pose.pose.position.z = 0.0;
        //     pose.pose.orientation.w = 1.0;

        //     path_msg->poses.push_back(pose);
        // }

        // path_publisher_->publish(*path_msg);
    }


    double vecAngle(auto v1, auto v2){

      double anglev1v2 = atan2(v1.x*v2.y - v1.y*v2.x, v1.x*v2.x + v1.y*v2.y);
      return anglev1v2;

    }

    int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {

      double err = setpoint - input;
      static double integral = 0, prevErr = 0;
      
      integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);

      double D = (err - prevErr) / dt;
      prevErr = err;
      return constrain(err * kp + integral + D * kd, minOut, maxOut);

}

    double constrain(double x, double a, double b){
      double out = x;

      if(x < a) out = a;
      if(x > b) out = b;

      return out;
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
//84955321300


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

    // This method executes every 500 milliseconds

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr pathSubscription_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr localPathSubscription_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr velsSubscription_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pedalSubscription_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr navSubscription_;



    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publishPP_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publishPP1_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publishPP2_;

    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr poseSubscription_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publishTwist_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;



    //rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher1_;
    //rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr globalMapSubscription_;
    
     
};



int main(int argc, char * argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuit>());
  rclcpp::shutdown();
  return 0;
}
