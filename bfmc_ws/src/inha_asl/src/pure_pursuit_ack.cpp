// pure_pursuit_ack.cpp  –  원본(스케일 보정 제거)

/* ----------  헤더  ---------- */
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include <vector>

class PurePursuitAck
{
public:
  PurePursuitAck()
  {
    ros::NodeHandle nh, pnh("~");
    pnh.param("wheelbase",       L_,       0.268);
    pnh.param("lookahead",       Ld_,      0.30);
    pnh.param("speed",           v_set_,   1.0);
    pnh.param("min_speed_ratio", v_min_r_, 0.2);
    pnh.param("max_steer_angle", steer_max_, 0.418);
    pnh.param("search_window",   win_,     30);

    sub_pose_ = nh.subscribe("/global_pose1", 1, &PurePursuitAck::poseCb, this);
    sub_path_ = nh.subscribe("/global_path",  1, &PurePursuitAck::pathCb, this);

    pub_cmd_   = nh.advertise<ackermann_msgs::AckermannDriveStamped>(
                   "/ackermann_cmd_mux/input/navigation", 1);
    marker_pub_= nh.advertise<visualization_msgs::Marker>(
                   "/pp_target_marker", 1);

    ROS_INFO("Pure-Pursuit-Ackermann ready (Ld=%.2f m, v=%.2f m/s)",
             Ld_, v_set_);
  }

private:
  /* parameters */
  double L_, Ld_, v_set_, v_min_r_, steer_max_;
  int    win_;

  /* data */
  nav_msgs::Path path_;
  int   idx_start_ = 0;
  bool  path_ok_   = false;

  /* ROS I/O */
  ros::Subscriber sub_pose_, sub_path_;
  ros::Publisher  pub_cmd_, marker_pub_;

  /* callback – Path */
  void pathCb(const nav_msgs::Path::ConstPtr& msg)
  {
    path_     = *msg;          // 그대로 복사(스케일 보정 없음)
    idx_start_= 0;
    path_ok_  = !path_.poses.empty();
  }

  /* callback – Pose */
  void poseCb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
  {
    if (!path_ok_) return;

    /* 현재 위치 & yaw */
    const auto& P = msg->pose.pose.position;
    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);
    double roll,pitch,yaw;
    tf2::Matrix3x3(q).getRPY(roll,pitch,yaw);

    /* look-ahead 점 선택 */
    int idx_end  = std::min(idx_start_+win_,
                            (int)path_.poses.size()-1);
    int goal_idx = path_.poses.size()-1;
    for (int i=idx_start_; i<=idx_end; ++i)
    {
      double dx = path_.poses[i].pose.position.x - P.x;
      double dy = path_.poses[i].pose.position.y - P.y;
      if (std::hypot(dx,dy) >= Ld_) { goal_idx=i; break; }
    }

    // 인덱스 디버깅용 
    double dist = std::hypot(dx, dy);              // dx,dy는 G-P 계산 직후 값
    ROS_WARN_STREAM_THROTTLE(0.5,
    "goal_idx=" << goal_idx << "  dist=" << dist << "  Ld=" << Ld_);
    idx_start_ = goal_idx;

    const auto& G = path_.poses[goal_idx].pose.position;
    double dx = G.x - P.x;
    double dy = G.y - P.y;

    /* 차량 좌표계 변환 */
    double x_r =  std::cos(yaw)*dx + std::sin(yaw)*dy;
    double y_r = -std::sin(yaw)*dx + std::cos(yaw)*dy;

    /* Pure-Pursuit 조향 */
    double steer = std::atan2(2.0*L_*y_r , Ld_*Ld_);
    steer = std::max(-steer_max_, std::min(steer, steer_max_));

    /* 속도 보정 */
    double v_dyn = v_set_ * (1.0 - std::abs(steer)/steer_max_
                             * (1.0 - v_min_r_));
    v_dyn = std::max(0.1, v_dyn);

    /* 퍼블리시 */
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.header.stamp    = ros::Time::now();
    cmd.header.frame_id = "base_link";
    cmd.drive.steering_angle = steer;
    cmd.drive.speed          = v_dyn;
    pub_cmd_.publish(cmd);

    /* RViz Marker (look-ahead) */
    visualization_msgs::Marker mk;
    mk.header.frame_id = "map";
    mk.header.stamp    = ros::Time::now();
    mk.ns   = "pp_target";
    mk.id   = 0;
    mk.type = visualization_msgs::Marker::SPHERE;
    mk.action = visualization_msgs::Marker::ADD;
    mk.pose.position.x = G.x;
    mk.pose.position.y = G.y;
    mk.pose.position.z = 0.05;
    mk.pose.orientation.w = 1.0;
    mk.scale.x = mk.scale.y = mk.scale.z = 0.25;
    mk.color.r = 1.0; mk.color.g = 0.0; mk.color.b = 0.0; mk.color.a = 1.0;
    marker_pub_.publish(mk);
  }
};

int main(int argc,char** argv)
{
  ros::init(argc, argv, "pure_pursuit_ack");
  PurePursuitAck node;
  ros::spin();
  return 0;
}
