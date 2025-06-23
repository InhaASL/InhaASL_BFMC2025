// pure_pursuit_ack.cpp
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <vector>

class PurePursuitAck
{
public:
  PurePursuitAck()
  {
    ros::NodeHandle nh, pnh("~");
    pnh.param("wheelbase",        L_,      0.26);  // [m]
    pnh.param("lookahead",        Ld_,     0.6);   // [m]
    pnh.param("speed",            v_set_,  1.0);   // [m/s]
    pnh.param("min_speed_ratio",  v_min_r_,0.2);
    pnh.param("max_steer_angle",  steer_max_, 0.418); // 24° rad
    pnh.param("search_window",    win_,    30);    // path index window

    sub_pose_ = nh.subscribe("/global_pose1", 1, &PurePursuitAck::poseCb, this);
    sub_path_ = nh.subscribe("/global_path", 1, &PurePursuitAck::pathCb, this);
    pub_cmd_  = nh.advertise<ackermann_msgs::AckermannDriveStamped>(
                   "/ackermann_cmd_mux/input/navigation", 1);

    ROS_INFO("Pure-Pursuit-Ackermann ready (Ld=%.2f m, v=%.2f m/s)", Ld_, v_set_);
  }

private:
  /* parameters */
  double L_, Ld_, v_set_, v_min_r_, steer_max_;
  int    win_;

  /* data */
  nav_msgs::Path path_;
  int  idx_start_ = 0;
  bool path_ok_   = false;

  /* ROS I/O */
  ros::Subscriber sub_pose_, sub_path_;
  ros::Publisher  pub_cmd_;

  /* helpers */
  static double normAngle(double a)
  {
    while (a >  M_PI) a -= 2*M_PI;
    while (a < -M_PI) a += 2*M_PI;
    return a;
  }

  void pathCb(const nav_msgs::Path::ConstPtr& msg)
  {
    path_   = *msg;
    idx_start_ = 0;
    path_ok_ = !path_.poses.empty();
  }

  void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    if (!path_ok_) return;

    const auto& P = msg->pose.pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll,pitch,yaw;
    tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);

    /* --- look-ahead 목표점 찾기 --- */
    int idx_end   = std::min(idx_start_+win_, (int)path_.poses.size()-1);
    int goal_idx  = path_.poses.size()-1;       // fallback = 마지막점
    for (int i=idx_start_; i<=idx_end; ++i)
    {
      double dx = path_.poses[i].pose.position.x - P.x;
      double dy = path_.poses[i].pose.position.y - P.y;
      if (std::hypot(dx,dy) >= Ld_) { goal_idx=i; break; }
    }
    idx_start_ = goal_idx;                      // 다음 루프 검색 시작점

    const auto& G = path_.poses[goal_idx].pose.position;
    double dx = G.x - P.x;
    double dy = G.y - P.y;

    /* 차량좌표계 변환 */
    double x_r =  std::cos(yaw)*dx + std::sin(yaw)*dy;
    double y_r = -std::sin(yaw)*dx + std::cos(yaw)*dy;

    /* Pure-Pursuit 조향 = atan2(2L sinα / Ld)  (여기서 sinα ≈ y_r/Ld) */
    double steer = std::atan2( 2.0 * L_ * y_r , (Ld_*Ld_) );
    if (steer >  steer_max_)  steer =  steer_max_;
    if (steer < -steer_max_)  steer = -steer_max_;

    /* 조향크기에 따라 속도 선형 감소 (Stanley 코드와 동일) */
    double v_dyn = v_set_ * (1.0 - std::abs(steer)/steer_max_ * (1.0 - v_min_r_));
    v_dyn = std::max(0.1, v_dyn);

    /* publish Ackermann */
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer;
    cmd.drive.speed          = v_dyn;
    pub_cmd_.publish(cmd);
  }
};

int main(int argc,char** argv)
{
  ros::init(argc,argv,"pure_pursuit_ack");
  PurePursuitAck pp;
  ros::spin();
  return 0;
}
