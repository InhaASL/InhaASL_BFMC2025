#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>

class PurePursuit
{
public:
  PurePursuit()
  {
    ros::NodeHandle nh, pnh("~");
    pnh.param("lookahead",      Ld_, 0.6);   // [m]
    pnh.param("linear_speed",   v_,  1.0);   // [m/s]
    pnh.param("path_topic",     path_topic_, std::string("/global_path"));

    sub_pose_ = nh.subscribe("/global_pose", 1, &PurePursuit::poseCb, this);
    sub_path_ = nh.subscribe(path_topic_,    1, &PurePursuit::pathCb, this);
    pub_cmd_  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    ROS_INFO("Pure Pursuit ready (lookahead=%.2f m, v=%.2f m/s)", Ld_, v_);
  }

private:
  ros::Subscriber sub_pose_, sub_path_;
  ros::Publisher  pub_cmd_;
  nav_msgs::Path  path_;
  double Ld_, v_;
  std::string path_topic_;

  void pathCb(const nav_msgs::Path::ConstPtr& msg) { path_ = *msg; }

  void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    if (path_.poses.empty()) return;

    // 현재 위치·Yaw
    const auto& P = msg->pose.pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // lookahead 목표점 찾기
    geometry_msgs::Point target;
    bool found=false;
    for (const auto& pose : path_.poses)
    {
      double dx = pose.pose.position.x - P.x;
      double dy = pose.pose.position.y - P.y;
      if (std::hypot(dx,dy) >= Ld_) { target=pose.pose.position; found=true; break; }
    }
    if (!found) target = path_.poses.back().pose.position; // 끝까지 왔을 때

    // 차량 좌표계로 변환
    double dx =  cos(yaw)*(target.x-P.x) + sin(yaw)*(target.y-P.y);
    double dy = -sin(yaw)*(target.x-P.x) + cos(yaw)*(target.y-P.y);

    double curvature = (2.0*dy)/(Ld_*Ld_);       // Pure Pursuit 공식
    double omega     = curvature*v_;

    geometry_msgs::Twist cmd;
    cmd.linear.x  = v_;
    cmd.angular.z = omega;
    pub_cmd_.publish(cmd);
  }
};

int main(int argc,char** argv)
{
  ros::init(argc,argv,"pure_pursuit_node");
  PurePursuit pp;
  ros::spin();
  return 0;
}
