#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <Eigen/Dense>

class EKFAckermannOdom
{
private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber drive_sub_;
    ros::Subscriber imu_sub_;
    tf::TransformBroadcaster tf_broadcaster_;

    // EKF 상태 벡터 [x, y, theta, v, omega]
    Eigen::VectorXd state_;           // 상태 벡터 (5x1)
    Eigen::MatrixXd P_;              // 공분산 행렬 (5x5)
    Eigen::MatrixXd Q_;              // 프로세스 노이즈 (5x5)
    Eigen::MatrixXd R_ackermann_;    // Ackermann 관측 노이즈 (2x2)
    Eigen::MatrixXd R_imu_;          // IMU 관측 노이즈 (1x1)

    double wheel_base_;
    double driving_gain_, steering_gain_;
    bool use_tf_;
    ros::Time last_time_;
    bool initialized_;

    // 토픽명들
    std::string odom_topic_;
    std::string drive_topic_;
    std::string imu_topic_;
    std::string odom_frame_;
    std::string base_frame_;

    // 공분산 행렬들을 직접 저장

public:
    EKFAckermannOdom() : initialized_(false)
    {
        // 파라미터 로드
        nh_.param("wheel_base", wheel_base_, 0.26);   
        nh_.param("use_tf", use_tf_, true);          
        nh_.param("drive_gain", driving_gain_, 9.05);
        nh_.param("steering_gain", steering_gain_, 11.0);
        
        // 토픽명 파라미터
        nh_.param<std::string>("odom_topic", odom_topic_, "odom");
        nh_.param<std::string>("drive_topic", drive_topic_, "current_speed");
        nh_.param<std::string>("imu_topic", imu_topic_, "/d455/imu");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        
        // 공분산 행렬들을 config에서 로드
        loadCovarianceMatrices();

        // ROS 통신 설정
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 50);
        drive_sub_ = nh_.subscribe(drive_topic_, 10, &EKFAckermannOdom::driveCallback, this);
        imu_sub_ = nh_.subscribe(imu_topic_, 10, &EKFAckermannOdom::imuCallback, this);

        // EKF 초기화
        initializeEKF();
        
        last_time_ = ros::Time::now();
    }

    void loadCovarianceMatrices()
    {
        // 프로세스 노이즈 행렬 Q (5x5)
        std::vector<double> process_cov;
        if (nh_.getParam("process_covariance", process_cov) && process_cov.size() == 25)
        {
            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    Q_(i, j) = process_cov[i * 5 + j];
                }
            }
        }
        else
        {
            // 기본값 설정
            Q_ = Eigen::MatrixXd::Zero(5, 5);
            Q_(0, 0) = 0.1;   // x 위치
            Q_(1, 1) = 0.1;   // y 위치
            Q_(2, 2) = 0.1;   // theta 각도
            Q_(3, 3) = 0.1;   // 선속도
            Q_(4, 4) = 0.1;   // 각속도
            ROS_WARN("Using default process covariance matrix");
        }

        // Ackermann 관측 노이즈 행렬 R (2x2)
        std::vector<double> ackermann_cov;
        if (nh_.getParam("ackermann_covariance", ackermann_cov) && ackermann_cov.size() == 4)
        {
            for (int i = 0; i < 2; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    R_ackermann_(i, j) = ackermann_cov[i * 2 + j];
                }
            }
        }
        else
        {
            // 기본값 설정
            R_ackermann_ = Eigen::MatrixXd::Zero(2, 2);
            R_ackermann_(0, 0) = 0.2;   // 속도 노이즈
            R_ackermann_(1, 1) = 0.2;   // 각속도 노이즈
            ROS_WARN("Using default Ackermann covariance matrix");
        }

        // IMU 관측 노이즈 행렬 R (1x1)
        std::vector<double> imu_cov;
        if (nh_.getParam("imu_covariance", imu_cov) && imu_cov.size() == 1)
        {
            R_imu_(0, 0) = imu_cov[0];
        }
        else
        {
            // 기본값 설정
            R_imu_(0, 0) = 0.05;
            ROS_WARN("Using default IMU covariance matrix");
        }

        // 초기 공분산 행렬 P (5x5)
        std::vector<double> initial_cov;
        if (nh_.getParam("initial_covariance", initial_cov) && initial_cov.size() == 25)
        {
            for (int i = 0; i < 5; i++)
            {
                for (int j = 0; j < 5; j++)
                {
                    P_(i, j) = initial_cov[i * 5 + j];
                }
            }
        }
        else
        {
            // 기본값 설정
            P_ = Eigen::MatrixXd::Identity(5, 5) * 1.0;
            ROS_WARN("Using default initial covariance matrix");
        }

        // 로드된 행렬들을 출력
        ROS_INFO("Process Covariance Matrix Q:");
        std::cout << Q_ << std::endl;
        ROS_INFO("Ackermann Observation Covariance Matrix R:");
        std::cout << R_ackermann_ << std::endl;
        ROS_INFO("IMU Observation Covariance Matrix R:");
        std::cout << R_imu_ << std::endl;
        ROS_INFO("Initial Covariance Matrix P:");
        std::cout << P_ << std::endl;
    }

    void initializeEKF()
    {
        // 상태 벡터 초기화 [x, y, theta, v, omega]
        state_ = Eigen::VectorXd::Zero(5);
        
        // 공분산 행렬들은 loadCovarianceMatrices()에서 설정됨
    }

    void predict(double dt)
    {
        // 상태 전이 (gain으로 보정된 속도 사용)
        double x = state_(0);
        double y = state_(1);
        double theta = state_(2);
        double v = state_(3);        // 이미 gain으로 보정된 속도
        double omega = state_(4);    // 이미 gain으로 보정된 각속도

        // 예측 단계 - 보정된 속도로 적분
        state_(0) = x + v * cos(theta) * dt;  // x
        state_(1) = y + v * sin(theta) * dt;  // y
        state_(2) = theta + omega * dt;       // theta
        // state_(3) = v;                     // v (일정하다고 가정)
        // state_(4) = omega;                 // omega (일정하다고 가정)

        // 각도 정규화 (-π ~ π)
        while (state_(2) > M_PI) state_(2) -= 2.0 * M_PI;
        while (state_(2) < -M_PI) state_(2) += 2.0 * M_PI;

        // 야코비안 행렬 계산
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
        F(0, 2) = -v * sin(theta) * dt;  // dx/dtheta
        F(0, 3) = cos(theta) * dt;       // dx/dv
        F(1, 2) = v * cos(theta) * dt;   // dy/dtheta
        F(1, 3) = sin(theta) * dt;       // dy/dv
        F(2, 4) = dt;                    // dtheta/domega

        // 공분산 예측
        P_ = F * P_ * F.transpose() + Q_;
    }

    void updateAckermann(double v_measured, double omega_measured)
    {
        // 관측 벡터 [v, omega]
        Eigen::VectorXd z(2);
        z(0) = v_measured;
        z(1) = omega_measured;

        // 예측된 관측값
        Eigen::VectorXd h(2);
        h(0) = state_(3);  // 예측된 속도
        h(1) = state_(4);  // 예측된 각속도

        // 관측 야코비안 (속도와 각속도를 직접 관측)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
        H(0, 3) = 1.0;  // v 관측
        H(1, 4) = 1.0;  // omega 관측

        // 칼만 이득 계산
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_ackermann_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 상태 업데이트
        Eigen::VectorXd innovation = z - h;
        state_ = state_ + K * innovation;

        // 공분산 업데이트
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
        P_ = (I - K * H) * P_;
    }

    void updateIMU(double omega_imu)
    {
        // 관측 벡터 [omega]
        Eigen::VectorXd z(1);
        z(0) = omega_imu;

        // 예측된 관측값
        Eigen::VectorXd h(1);
        h(0) = state_(4);  // 예측된 각속도

        // 관측 야코비안
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 5);
        H(0, 4) = 1.0;  // omega 관측

        // 칼만 이득 계산
        Eigen::MatrixXd S = H * P_ * H.transpose() + R_imu_;
        Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

        // 상태 업데이트
        Eigen::VectorXd innovation = z - h;
        state_ = state_ + K * innovation;

        // 공분산 업데이트
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
        P_ = (I - K * H) * P_;
    }

    void driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        double v = msg->drive.speed;
        double steering_angle = -msg->drive.steering_angle;

        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();
        
        if (dt <= 0) return;
        
        last_time_ = current_time;

        // Ackermann 모델로부터 각속도 계산 (gain 적용)
        double omega_ackermann = (v / wheel_base_) * tan(steering_angle) / steering_gain_;
        double v_corrected = v / driving_gain_;

        if (!initialized_)
        {
            state_(3) = v_corrected;
            state_(4) = omega_ackermann;
            initialized_ = true;
            return;
        }

        // EKF 예측 단계 (보정된 값으로 예측)
        predict(dt);

        // Ackermann 관측으로 업데이트 (보정된 값 사용)
        updateAckermann(v_corrected, omega_ackermann);

        // 오도메트리 발행
        publishOdometry(current_time);
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        if (!initialized_) return;

        // IMU 각속도 (z축)
        double omega_imu = msg->angular_velocity.z;

        // IMU 관측으로 업데이트
        updateIMU(omega_imu);
    }

    void publishOdometry(const ros::Time& current_time)
    {
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(state_(2));

        // TF 발행
        if (use_tf_)
        {
            geometry_msgs::TransformStamped odom_tf;
            odom_tf.header.stamp = current_time;
            odom_tf.header.frame_id = odom_frame_;
            odom_tf.child_frame_id = base_frame_;
            odom_tf.transform.translation.x = state_(0);
            odom_tf.transform.translation.y = state_(1);
            odom_tf.transform.translation.z = 0.0;
            odom_tf.transform.rotation = odom_quat;
            tf_broadcaster_.sendTransform(odom_tf);
        }

        // 오도메트리 메시지 발행
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;

        odom.pose.pose.position.x = state_(0);
        odom.pose.pose.position.y = state_(1);
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // 공분산 설정
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (i < 3 && j < 3)
                {
                    // 위치와 회전에 대한 공분산
                    if (i == 2 && j == 2)
                        odom.pose.covariance[i * 6 + j] = P_(2, 2);  // theta
                    else if (i < 2 && j < 2)
                        odom.pose.covariance[i * 6 + j] = P_(i, j);  // x, y
                    else
                        odom.pose.covariance[i * 6 + j] = 0.0;
                }
                else
                {
                    odom.pose.covariance[i * 6 + j] = 0.0;
                }
            }
        }

        odom.twist.twist.linear.x = state_(3);  // 이미 gain으로 보정된 속도
        odom.twist.twist.angular.z = state_(4);

        // 속도 공분산 설정
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                if (i == 0 && j == 0)
                    odom.twist.covariance[i * 6 + j] = P_(3, 3);  // 선속도
                else if (i == 5 && j == 5)
                    odom.twist.covariance[i * 6 + j] = P_(4, 4);  // 각속도
                else
                    odom.twist.covariance[i * 6 + j] = 0.0;
            }
        }

        odom_pub_.publish(odom);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ekf_ackermann_odom_node");
    EKFAckermannOdom odom_node;
    ros::spin();
    return 0;
}