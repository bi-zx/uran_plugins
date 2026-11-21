// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <algorithm>
#include "pose_extrapolator/measurement_collector.h"

namespace cartographer {
namespace pose_extrapolator {
namespace {

// DDS can deliver a delayed sample after a later acquisition.
template <typename Measurement>
void InsertByTime(std::deque<Measurement>& pending, const Measurement& sample) {
  if (pending.empty() || pending.back().time < sample.time) {
    pending.push_back(sample);
    return;
  }
  auto pos = std::lower_bound(pending.begin(), pending.end(), sample.time,
      [](const Measurement& value, common::Time time) { return value.time < time; });
  if (pos == pending.end() || pos->time != sample.time) pending.insert(pos, sample);
}

}  // namespace

void MeasurementCollector::AddImuData(const sensor::ImuData& sample) {
  std::lock_guard<std::mutex> lock(imu_lk_);
  if (!pose_extrapolator_) {
    const auto initial_pose = transform::TimedRigid3d{
        transform::Rigid3d::Rotation(Eigen::Quaterniond::Identity()), sample.time};
    pose_extrapolator_ = PoseExtraPolatorInterface::CreateWithImuData(param_, {sample}, {initial_pose});
    last_extrapolate_time_ = sample.time;
    return;
  }
  if (sample.time >= last_extrapolate_time_) InsertByTime(imu_datas_, sample);
}

bool MeasurementCollector::InitialWithPose(const std::vector<transform::TimedRigid3d>& initial_poses) {
  std::lock_guard<std::mutex> lock(imu_lk_);
  CHECK(!pose_extrapolator_);
  const sensor::ImuData initial_imu{common::Time::min(), Eigen::Vector3d(0., 0., 10.), Eigen::Vector3d::Zero()};
  pose_extrapolator_ = PoseExtraPolatorInterface::CreateWithImuData(param_, {initial_imu}, initial_poses);
  last_extrapolate_time_ = pose_extrapolator_->GetLastPoseTime();
  return true;
}

void MeasurementCollector::AddOdometryData(const sensor::OdometryData& sample) {
  std::lock_guard<std::mutex> lock(imu_lk_);
  if (pose_extrapolator_ && sample.time >= last_extrapolate_time_) InsertByTime(odom_datas_, sample);
}

void MeasurementCollector::AddPose(const transform::TimedRigid3d& pose) {
  std::lock_guard<std::mutex> lock(imu_lk_);
  if (!pose_extrapolator_) return;
  pose_extrapolator_->AddPose(pose);
  last_extrapolate_time_ = std::max(last_extrapolate_time_, pose.timestamp);
}

transform::Rigid3d MeasurementCollector::ExtrapolatePose(const common::Time& time) {
  std::lock_guard<std::mutex> lock(imu_lk_);
  if (!pose_extrapolator_) return transform::Rigid3d::Identity();

  // A sample at this scan's time belongs to the next interval.
  while (!imu_datas_.empty() && imu_datas_.front().time < time) {
    if (imu_datas_.front().time >= last_extrapolate_time_) pose_extrapolator_->AddImuData(imu_datas_.front());
    imu_datas_.pop_front();
  }
  while (!odom_datas_.empty() && odom_datas_.front().time < time) {
    if (odom_datas_.front().time >= last_extrapolate_time_) pose_extrapolator_->AddOdometryData(odom_datas_.front());
    odom_datas_.pop_front();
  }
  const auto pose = pose_extrapolator_->ExtrapolatePose(time);
  last_extrapolate_time_ = time;
  return pose;
}

}  // namespace pose_extrapolator
}  // namespace cartographer
