#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <random>
#include <mutex>
#include <Eigen/Dense>
#include <vector>

class AckermannEKF
{
private:
    ros::NodeHandle nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber drive_sub_;
    ros::Subscriber imu_sub_;
    tf::TransformBroadcaster tf_broadcaster_;
    ros::Timer publish_timer_;

    // EKF 상태 벡터 [x, y, theta, v, omega] (5차원으로 단순화)
    Eigen::VectorXd state_;         // 5x1 상태 벡터
    Eigen::MatrixXd covariance_;    // 5x5 공분산 행렬
    
    // 프로세스 및 관측 노이즈
    Eigen::MatrixXd Q_;             // 5x5 프로세스 노이즈 공분산
    Eigen::MatrixXd R_ackermann_;   // 2x2 Ackermann 관측 노이즈 공분산
    Eigen::MatrixXd R_imu_;         // 1x1 IMU 관측 노이즈 공분산
    
    // 설정 파라미터들
    double wheel_base_;
    double driving_gain_, steering_gain_;
    bool use_tf_;
    double publish_rate_;
    std::string odom_topic_, drive_topic_, imu_topic_;
    std::string odom_frame_, base_frame_;
    
    // Preintegration 설정
    double preint_accel_noise_std_;
    double preint_gyro_noise_std_;
    double preint_max_time_;
    int preint_min_measurements_;
    
    // 시간 관리
    ros::Time last_predict_time_;
    ros::Time last_publish_time_;
    
    // 초기화 관련
    bool ekf_initialized_;
    std::mutex ekf_mutex_;
    
    // 주파수 추정
    int imu_count_, ackermann_count_;
    ros::Time freq_measure_start_;

public:
    AckermannEKF() : state_(5), covariance_(5, 5), Q_(5, 5), 
                     R_ackermann_(2, 2), R_imu_(1, 1),
                     ekf_initialized_(false)
    {
        // Config 파라미터 읽기
        loadConfiguration();
        
        // EKF 초기화
        initializeEKF();
        
        // Publisher & Subscriber 설정
        setupROS();
        
        // 시간 초기화
        last_predict_time_ = ros::Time::now();
        last_publish_time_ = ros::Time::now();
        
        // 주파수 측정 초기화
        imu_count_ = ackermann_count_ = 0;
        freq_measure_start_ = ros::Time::now();
        
        ROS_INFO("AckermannEKF 노드가 시작되었사옵니다.");
        printConfiguration();
    }
    
private:
    void loadConfiguration()
    {
        // 기본 파라미터
        nh_.param("wheel_base", wheel_base_, 0.26);
        nh_.param("use_tf", use_tf_, true);
        
        // 토픽 및 프레임 설정
        nh_.param<std::string>("odom_topic", odom_topic_, "odom");
        nh_.param<std::string>("drive_topic", drive_topic_, "current_speed");
        nh_.param<std::string>("imu_topic", imu_topic_, "/d455/imu");
        nh_.param<std::string>("odom_frame", odom_frame_, "odom");
        nh_.param<std::string>("base_frame", base_frame_, "base_link");
        nh_.param("publish_rate", publish_rate_, 30.0);
        
        // Preintegration 설정
        nh_.param("preint_accel_noise_std", preint_accel_noise_std_, 0.2);
        nh_.param("preint_gyro_noise_std", preint_gyro_noise_std_, 0.1);
        nh_.param("preint_max_time", preint_max_time_, 0.5);
        nh_.param("preint_min_measurements", preint_min_measurements_, 5);
        
        // 센서 캘리브레이션 gain
        nh_.param("drive_gain", driving_gain_, 9.05);
        nh_.param("steering_gain", steering_gain_, 11.0);
        
        // 프로세스 노이즈 공분산 행렬 Q (5x5)
        std::vector<double> process_cov;
        if (nh_.getParam("process_covariance", process_cov) && process_cov.size() == 25) {
            Q_ = Eigen::Map<Eigen::MatrixXd>(process_cov.data(), 5, 5).transpose();
        } else {
            ROS_WARN("process_covariance 파라미터를 찾을 수 없거나 크기가 잘못되었습니다. 기본값을 사용합니다.");
            Q_ = Eigen::MatrixXd::Identity(5, 5) * 0.1;
        }
        
        // Ackermann 관측 노이즈 공분산 행렬 R (2x2)
        std::vector<double> ackermann_cov;
        if (nh_.getParam("ackermann_covariance", ackermann_cov) && ackermann_cov.size() == 4) {
            R_ackermann_ = Eigen::Map<Eigen::MatrixXd>(ackermann_cov.data(), 2, 2).transpose();
        } else {
            ROS_WARN("ackermann_covariance 파라미터를 찾을 수 없거나 크기가 잘못되었습니다. 기본값을 사용합니다.");
            R_ackermann_ = Eigen::MatrixXd::Identity(2, 2) * 0.2;
        }
        
        // IMU 관측 노이즈 공분산 행렬 R (1x1)
        std::vector<double> imu_cov;
        if (nh_.getParam("imu_covariance", imu_cov) && imu_cov.size() == 1) {
            R_imu_(0, 0) = imu_cov[0];
        } else {
            ROS_WARN("imu_covariance 파라미터를 찾을 수 없거나 크기가 잘못되었습니다. 기본값을 사용합니다.");
            R_imu_(0, 0) = 0.05;
        }
        
        // 초기 상태 공분산 행렬 P (5x5)
        std::vector<double> initial_cov;
        if (nh_.getParam("initial_covariance", initial_cov) && initial_cov.size() == 25) {
            covariance_ = Eigen::Map<Eigen::MatrixXd>(initial_cov.data(), 5, 5).transpose();
        } else {
            ROS_WARN("initial_covariance 파라미터를 찾을 수 없거나 크기가 잘못되었습니다. 기본값을 사용합니다.");
            covariance_ = Eigen::MatrixXd::Identity(5, 5) * 1.0;
        }
    }
    
    void printConfiguration()
    {
        ROS_INFO("=== EKF Ackermann Odometry 설정 ===");
        ROS_INFO("wheel_base: %.3f", wheel_base_);
        ROS_INFO("drive_gain: %.2f, steering_gain: %.2f", driving_gain_, steering_gain_);
        ROS_INFO("토픽 - odom: %s, drive: %s, imu: %s", 
                 odom_topic_.c_str(), drive_topic_.c_str(), imu_topic_.c_str());
        ROS_INFO("프레임 - odom: %s, base: %s", odom_frame_.c_str(), base_frame_.c_str());
        ROS_INFO("발행 주파수: %.1f Hz", publish_rate_);
        ROS_INFO("Preintegration - accel_noise: %.3f, gyro_noise: %.3f, max_time: %.3f", 
                 preint_accel_noise_std_, preint_gyro_noise_std_, preint_max_time_);
        ROS_INFO("=====================================");
    }
    
    void setupROS()
    {
        // Publisher
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(odom_topic_, 50);
        
        // Subscribers
        drive_sub_ = nh_.subscribe(drive_topic_, 10, &AckermannEKF::driveCallback, this);
        imu_sub_ = nh_.subscribe(imu_topic_, 100, &AckermannEKF::imuCallback, this);
        
        // Timer (optional)
        if (publish_rate_ > 0.0) {
            publish_timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), 
                                           &AckermannEKF::publishTimerCallback, this);
        }
    }
    
    void initializeEKF()
    {
        // 초기 상태 벡터 [x, y, theta, v, omega]
        state_.setZero();
        
        // 공분산 행렬은 config에서 로드됨
        // Q_, R_ackermann_, R_imu_도 config에서 로드됨
        
        ROS_INFO("EKF 초기화 완료 - 상태 차원: %ld", state_.size());
    }
    
    void predictStepIMU(double dt, const sensor_msgs::Imu::ConstPtr& msg)
    {
        if (dt <= 0.0 || dt > 0.1) return; // IMU 100Hz 기준
        
        // 현재 상태 추출
        double x = state_(0);
        double y = state_(1);
        double theta = state_(2);
        double v = state_(3);
        double omega = state_(4);
        
        // IMU 데이터 (Body frame → World frame)
        double cos_theta = cos(theta);
        double sin_theta = sin(theta);
        
        double ax_body = msg->linear_acceleration.x;
        double ay_body = msg->linear_acceleration.y;
        double omega_z = msg->angular_velocity.z;
        
        double ax_world = cos_theta * ax_body - sin_theta * ay_body;
        double ay_world = sin_theta * ax_body + cos_theta * ay_body;
        
        // IMU 기반 상태 예측 (상태 차원 5)
        Eigen::VectorXd predicted_state(5);
        predicted_state(0) = x + v * cos_theta * dt + 0.5 * ax_world * dt * dt;  // x
        predicted_state(1) = y + v * sin_theta * dt + 0.5 * ay_world * dt * dt;  // y
        predicted_state(2) = theta + omega_z * dt;                               // theta
        predicted_state(3) = sqrt(ax_world*ax_world + ay_world*ay_world) * dt + v; // v (속도 크기)
        predicted_state(4) = omega_z;                                            // omega
        
        // 야코비안 행렬 F (5x5)
        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(5, 5);
        F(0, 2) = -v * sin_theta * dt;     // dx/dtheta
        F(0, 3) = cos_theta * dt;          // dx/dv
        
        F(1, 2) = v * cos_theta * dt;      // dy/dtheta
        F(1, 3) = sin_theta * dt;          // dy/dv
        
        F(2, 4) = dt;                      // dtheta/domega
        
        // 공분산 예측
        covariance_ = F * covariance_ * F.transpose() + Q_ * dt;
        
        // 상태 업데이트
        state_ = predicted_state;
        state_(2) = normalizeAngle(state_(2));
        
        ROS_DEBUG("IMU Process: dt=%.4f, ax=%.3f, ay=%.3f, ω=%.3f", 
                  dt, ax_world, ay_world, omega_z);
    }
    
    void updateAckermann(double v, double steering_angle)
    {
        // Ackermann 관측값 [v, omega]
        double omega_ackermann = (v / wheel_base_) * tan(steering_angle) / steering_gain_;
        double v_corrected = v / driving_gain_;
        
        Eigen::VectorXd z_ackermann(2);
        z_ackermann(0) = v_corrected;
        z_ackermann(1) = omega_ackermann;
        
        // 예측된 관측값
        Eigen::VectorXd h_pred(2);
        h_pred(0) = state_(3);  // v
        h_pred(1) = state_(4);  // omega
        
        // 관측 모델의 야코비안 행렬 H (2x5)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(2, 5);
        H(0, 3) = 1.0;  // dv/dv
        H(1, 4) = 1.0;  // domega/domega
        
        // 칼만 갱신
        kalmanUpdate(z_ackermann, h_pred, H, R_ackermann_);
        
        ROS_DEBUG("Ackermann Correction: v=%.3f, ω=%.3f", v_corrected, omega_ackermann);
    }
    
    void updateIMU(const sensor_msgs::Imu::ConstPtr& msg)
    {
        // IMU 관측값 [omega_z]
        double omega_z = msg->angular_velocity.z;
        
        Eigen::VectorXd z_imu(1);
        z_imu(0) = omega_z;
        
        // 예측된 관측값
        Eigen::VectorXd h_pred(1);
        h_pred(0) = state_(4);  // omega
        
        // 관측 모델의 야코비안 행렬 H (1x5)
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 5);
        H(0, 4) = 1.0;  // domega/domega
        
        // 칼만 갱신
        kalmanUpdate(z_imu, h_pred, H, R_imu_);
        
        ROS_DEBUG("IMU Correction: ω=%.3f", omega_z);
    }
    
    void kalmanUpdate(const Eigen::VectorXd& z, const Eigen::VectorXd& h_pred, 
                     const Eigen::MatrixXd& H, const Eigen::MatrixXd& R)
    {
        // 혁신(innovation) 계산
        Eigen::VectorXd y = z - h_pred;
        
        // 혁신 공분산 계산
        Eigen::MatrixXd S = H * covariance_ * H.transpose() + R;
        
        // 칼만 게인 계산
        Eigen::MatrixXd K = covariance_ * H.transpose() * S.inverse();
        
        // 상태 업데이트
        state_ = state_ + K * y;
        
        // 공분산 업데이트 (Joseph form for numerical stability)
        Eigen::MatrixXd I = Eigen::MatrixXd::Identity(5, 5);
        Eigen::MatrixXd IKH = I - K * H;
        covariance_ = IKH * covariance_ * IKH.transpose() + K * R * K.transpose();
        
        // 각도 정규화
        state_(2) = normalizeAngle(state_(2));
    }
    
    double normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    void updateFrequencyEstimates()
    {
        ros::Time now = ros::Time::now();
        double elapsed = (now - freq_measure_start_).toSec();
        
        if (elapsed > 5.0) { // 5초간 측정
            double imu_freq = imu_count_ / elapsed;
            double ackermann_freq = ackermann_count_ / elapsed;
            
            ROS_INFO("센서 주파수 - IMU: %.1f Hz, Ackermann: %.1f Hz", imu_freq, ackermann_freq);
            
            // 카운터 리셋
            imu_count_ = ackermann_count_ = 0;
            freq_measure_start_ = now;
        }
    }
    
public:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        
        // 주파수 측정
        imu_count_++;
        updateFrequencyEstimates();
        
        ros::Time current_time = msg->header.stamp;
        if (current_time.isZero()) current_time = ros::Time::now();
        
        if (!ekf_initialized_) {
            last_predict_time_ = current_time;
            ekf_initialized_ = true;
            return;
        }
        
        double dt = (current_time - last_predict_time_).toSec();
        
        // IMU를 Process Model로 사용 (100Hz)
        if (dt > 0.0) {
            predictStepIMU(dt, msg);
            last_predict_time_ = current_time;
        }
        
        ROS_DEBUG("IMU Process: x=%.3f, y=%.3f, θ=%.3f", 
                  state_(0), state_(1), state_(2));
    }
    
    void driveCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        
        // 주파수 측정
        ackermann_count_++;
        updateFrequencyEstimates();
        
        double v = msg->drive.speed;
        double steering_angle = -msg->drive.steering_angle;
        
        if (!ekf_initialized_) {
            return; // IMU 초기화 대기
        }
        
        // Ackermann을 Correction으로 사용 (10Hz)
        updateAckermann(v, steering_angle);
        
        ROS_DEBUG("Ackermann Correction: v=%.3f, δ=%.3f", v, steering_angle);
    }
    
    void publishTimerCallback(const ros::TimerEvent& event)
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        publishOdometry();
    }
    
    void publishOdometry()
    {
        if (!ekf_initialized_) return;
        
        ros::Time current_time = ros::Time::now();
        
        // 상태에서 값들 추출
        double x = state_(0);
        double y = state_(1);
        double theta = state_(2);
        double v = state_(3);
        double omega = state_(4);
        
        // 속도를 body frame으로 변환
        double vx = v * cos(theta);
        double vy = v * sin(theta);
        
        // 쿼터니언 생성
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        
        // TF 브로드캐스트
        if (use_tf_) {
            geometry_msgs::TransformStamped odom_tf;
            odom_tf.header.stamp = current_time;
            odom_tf.header.frame_id = odom_frame_;
            odom_tf.child_frame_id = base_frame_;
            odom_tf.transform.translation.x = x;
            odom_tf.transform.translation.y = y;
            odom_tf.transform.translation.z = 0.0;
            odom_tf.transform.rotation = odom_quat;
            tf_broadcaster_.sendTransform(odom_tf);
        }
        
        // 오도메트리 메시지 생성
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        
        // 위치 및 자세
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        
        // 속도 정보
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = omega;
        
        // 공분산 정보 (EKF 공분산 행렬에서 추출)
        for (int i = 0; i < 36; i++) {
            odom.pose.covariance[i] = 0.0;
            odom.twist.covariance[i] = 0.0;
        }
        
        // 위치 공분산 (x, y, yaw)
        odom.pose.covariance[0] = covariance_(0, 0);   // x
        odom.pose.covariance[7] = covariance_(1, 1);   // y
        odom.pose.covariance[35] = covariance_(2, 2);  // yaw
        
        // 속도 공분산 (vx, vy는 v와 theta로부터 계산)
        double cos_th = cos(theta);
        double sin_th = sin(theta);
        odom.twist.covariance[0] = cos_th * cos_th * covariance_(3, 3) + 
                                  v * v * sin_th * sin_th * covariance_(2, 2);  // vx
        odom.twist.covariance[7] = sin_th * sin_th * covariance_(3, 3) + 
                                  v * v * cos_th * cos_th * covariance_(2, 2);  // vy
        odom.twist.covariance[35] = covariance_(4, 4);  // omega
        
        odom_pub_.publish(odom);
        
        ROS_DEBUG("EKF 오도메트리 발행: x=%.3f, y=%.3f, θ=%.3f, v=%.3f, ω=%.3f", 
                  x, y, theta, v, omega);
    }
    
    void resetEKF()
    {
        std::lock_guard<std::mutex> lock(ekf_mutex_);
        initializeEKF();
        ekf_initialized_ = false;
        ROS_INFO("EKF가 리셋되었사옵니다.");
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ackermann_ekf_odom_node");
    
    try {
        AckermannEKF ekf_node;
        ROS_INFO("Ackermann EKF 오도메트리 노드가 실행 중이옵니다...");
        ros::spin();
    }
    catch (const std::exception& e) {
        ROS_ERROR("노드 실행 중 오류가 발생하였사옵니다: %s", e.what());
        return -1;
    }
    
    return 0;
}