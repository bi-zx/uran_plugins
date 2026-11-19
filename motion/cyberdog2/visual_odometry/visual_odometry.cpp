#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <thread>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Geometry>
#include <opencv2/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

// The board ships libvins_lib without its public mivins_api.h.
int MiVins_StartUp(std::string config, std::string calibration);
int MiVins_GetOdomStatus();
int MiVins_GetImuStatus();
int MiVins_ImuInput(double time, const Eigen::Vector3d &acc, const Eigen::Vector3d &gyro);
int MiVins_ImgInput(int64_t stamp, const std::vector<cv::Mat> &images, const std::map<int, cv::Mat> &depths);
int MiVins_GetStage();
std::vector<std::vector<double>> MiVins_GetLastFramesPose(std::string sensor);

using Image = sensor_msgs::msg::Image;
using Imu = sensor_msgs::msg::Imu;
using Odom = nav_msgs::msg::Odometry;
using Pose = Eigen::Isometry3d;

class VisualOdometry : public rclcpp::Node {
public:
  VisualOdometry() : Node("visual_odometry"), sync_(left_, right_, 8) {
    auto config = declare_parameter<std::string>("config", "");
    auto calibration = declare_parameter<std::string>("calibration", "");
    auto mount = declare_parameter<std::vector<double>>("imu_from_body", std::vector<double>{});
    auto plane = declare_parameter<std::vector<double>>("starting_floor", std::vector<double>{});
    if (mount.size() != 16 || plane.size() != 3)
      throw std::runtime_error("Use the device calibration and the starting floor plane from prepare.py");
    imu_from_body_.matrix() = Eigen::Map<const Eigen::Matrix<double,4,4,Eigen::RowMajor>>(mount.data());
    // In the starting body frame the measured floor is z = a*x + b*y + c.
    Eigen::Vector3d up(-plane[0], -plane[1], 1.);
    initial_body_.translation().z() = -plane[2] / up.norm();
    up.normalize();
    Eigen::Vector3d left = up.cross(Eigen::Vector3d::UnitX()).normalized();
    initial_body_.linear().row(0) = left.cross(up).transpose();
    initial_body_.linear().row(1) = left.transpose();
    initial_body_.linear().row(2) = up.transpose();

    if (!MiVins_StartUp(config, calibration) || MiVins_GetOdomStatus() || !MiVins_GetImuStatus())
      throw std::runtime_error("MiVINS requires stereo and IMU with external odometry disabled");
    odom_frame_ = declare_parameter<std::string>("odom_frame", "visual_odom");
    body_frame_ = declare_parameter<std::string>("body_frame", "base_link");
    publisher_ = create_publisher<Odom>("visual_odom", 10);
    auto image_qos = rmw_qos_profile_sensor_data;
    image_qos.depth = 2;
    left_.subscribe(this, "camera/infra1/image_rect_raw", image_qos);
    right_.subscribe(this, "camera/infra2/image_rect_raw", image_qos);
    sync_.registerCallback(std::bind(&VisualOdometry::stereo, this, std::placeholders::_1, std::placeholders::_2));
    imu_ = create_subscription<Imu>("camera/imu", rclcpp::SensorDataQoS().keep_last(400),
      [this](Imu::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        imu_samples_.push_back(msg);
        if (imu_samples_.size() > 1000) imu_samples_.pop_front();
        if (!pending_.empty()) ready_.notify_one();
      });
    clahe_ = cv::createCLAHE(3.0, cv::Size(8,8));
    worker_ = std::thread([this]() { track(); });
  }

  bool failed() const { return failed_.load(); }

  ~VisualOdometry() override {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
    }
    ready_.notify_one();
    worker_.join();
  }

private:
  using Pair = std::pair<Image::ConstSharedPtr, Image::ConstSharedPtr>;
  static int64_t stamp_ns(const std_msgs::msg::Header &header) {
    return rclcpp::Time(header.stamp).nanoseconds();
  }

  void stereo(const Image::ConstSharedPtr &left, const Image::ConstSharedPtr &right) {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.emplace_back(left, right);
    if (pending_.size() > 12) pending_.pop_front();
    ready_.notify_one();
  }

  void track() {
    try {
      for (;;) {
        Pair pair;
        std::deque<Imu::ConstSharedPtr> batch;
        {
          std::unique_lock<std::mutex> lock(mutex_);
          ready_.wait(lock, [this]() {
            return stopping_ || (!pending_.empty() && !imu_samples_.empty() &&
                stamp_ns(imu_samples_.back()->header) >= stamp_ns(pending_.front().first->header));
          });
          if (stopping_) return;
          auto newest = stamp_ns(imu_samples_.back()->header);
          while (!pending_.empty() && stamp_ns(pending_.front().first->header) <= newest) {
            pair = std::move(pending_.front());
            pending_.pop_front();
          }
          auto stamp = stamp_ns(pair.first->header);
          while (!imu_samples_.empty() && (batch.empty() || stamp_ns(batch.back()->header) < stamp)) {
            batch.push_back(std::move(imu_samples_.front()));
            imu_samples_.pop_front();
          }
        }
        auto stamp = stamp_ns(pair.first->header);
        for (const auto &m : batch) {
          auto t = stamp_ns(m->header);
          if (t <= last_imu_ns_) continue;
          const auto &a = m->linear_acceleration;
          const auto &g = m->angular_velocity;
          MiVins_ImuInput(t*1e-9, {a.x,a.y,a.z}, {g.x,g.y,g.z});
          last_imu_ns_ = t;
        }
        if (stamp > last_pose_ns_) process(pair, stamp);
      }
    } catch (const std::exception &e) {
      failed_.store(true);
      RCLCPP_ERROR(get_logger(), "%s", e.what());
      rclcpp::shutdown();
    }
  }

  void process(const Pair &pair, int64_t stamp) {
    std::vector<cv::Mat> images;
    for (const auto &msg : {pair.first,pair.second}) {
      if (msg->encoding != "mono8") throw std::runtime_error("Expected rectified mono8 infrared images");
      cv::Mat image(msg->height, msg->width, CV_8U, const_cast<uint8_t*>(msg->data.data()), msg->step);
      cv::Mat enhanced;
      clahe_->apply(image, enhanced);
      images.push_back(enhanced);
    }
    auto header = pair.first->header;
    MiVins_ImgInput(stamp, images, {});
    int stage = MiVins_GetStage();
    if (stage != stage_) RCLCPP_INFO(get_logger(), "MiVINS tracking stage %d", stage);
    stage_ = stage;
    if (stage != 2) {
      if (last_pose_ns_) throw std::runtime_error("Visual tracking lost; start a new reference before mapping again");
      return;
    }
    auto result = MiVins_GetLastFramesPose("imu");
    if (result.empty() || result.front().size() != 7) return;
    const auto &p = result.front();
    // The SDK can reject a frame when IMU samples are missing and return its previous pose.
    // It exposes no acquisition stamp in this API. Keep that old pose at its original stamp.
    if (p == last_sdk_pose_) return;
    last_sdk_pose_ = p;
    Eigen::Quaterniond q(p[6],p[3],p[4],p[5]);
    Pose world_from_imu = Pose::Identity();
    world_from_imu.linear() = q.normalized().toRotationMatrix();
    world_from_imu.translation() = Eigen::Vector3d(p[0],p[1],p[2]);
    Pose world_from_body = world_from_imu * imu_from_body_;
    if (!world_from_body.matrix().allFinite()) throw std::runtime_error("Invalid visual pose");
    if (!last_pose_ns_) origin_ = initial_body_ * world_from_body.inverse();
    Pose current = origin_ * world_from_body;
    Eigen::Quaterniond orientation(current.linear());
    Odom out;
    out.header = header;
    out.header.frame_id = odom_frame_;
    out.child_frame_id = body_frame_;
    auto &pose = out.pose.pose;
    pose.position.x = current.translation().x();
    pose.position.y = current.translation().y();
    pose.position.z = current.translation().z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    if (last_pose_ns_) {
      double dt = (stamp-last_pose_ns_)*1e-9;
      if (dt <= 0.) return;
      Eigen::Vector3d velocity = current.linear().transpose() * (current.translation()-previous_.translation()) / dt;
      Eigen::AngleAxisd rotation(previous_.linear().transpose()*current.linear());
      Eigen::Vector3d gyro = rotation.axis()*rotation.angle()/dt;
      auto &twist = out.twist.twist;
      twist.linear.x = velocity.x(); twist.linear.y = velocity.y(); twist.linear.z = velocity.z();
      twist.angular.x = gyro.x(); twist.angular.y = gyro.y(); twist.angular.z = gyro.z();
    }
    previous_ = current;
    last_pose_ns_ = stamp;
    publisher_->publish(out);
  }

  message_filters::Subscriber<Image> left_, right_;
  message_filters::TimeSynchronizer<Image,Image> sync_;
  std::deque<Pair> pending_;
  std::deque<Imu::ConstSharedPtr> imu_samples_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::thread worker_;
  bool stopping_ = false;
  std::atomic<bool> failed_{false};
  std::vector<double> last_sdk_pose_;
  rclcpp::Subscription<Imu>::SharedPtr imu_;
  rclcpp::Publisher<Odom>::SharedPtr publisher_;
  cv::Ptr<cv::CLAHE> clahe_;
  int64_t last_imu_ns_ = 0, last_pose_ns_ = 0;
  int stage_ = -1;
  std::string odom_frame_, body_frame_;
  Pose imu_from_body_, origin_, previous_, initial_body_ = Pose::Identity();
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  cv::setNumThreads(1);
  try {
    auto node = std::make_shared<VisualOdometry>();
    rclcpp::spin(node);
    if (node->failed()) return 1;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("visual_odometry"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
