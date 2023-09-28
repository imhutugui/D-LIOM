/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/urdf_reader.h"
#include <cartographer_ros/sensor_json_parser.h>
#include <string>
#include <vector>

#include "cartographer_ros/msg_conversion.h"
#include "urdf/model.h"

namespace cartographer_ros {

std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromUrdf(
    const std::string& urdf_filename, tf2_ros::Buffer* const tf_buffer) {
  urdf::Model model;
  CHECK(model.initFile(urdf_filename));
#if URDFDOM_HEADERS_HAS_SHARED_PTR_DEFS
  std::vector<urdf::LinkSharedPtr> links;
#else
  std::vector<boost::shared_ptr<urdf::Link> > links;
#endif
  model.getLinks(links);
  std::vector<geometry_msgs::TransformStamped> transforms;
  for (const auto& link : links) {
    if (!link->getParent() || link->parent_joint->type != urdf::Joint::FIXED) {
      continue;
    }

    const urdf::Pose& pose =
        link->parent_joint->parent_to_joint_origin_transform;
    geometry_msgs::TransformStamped transform;
    transform.transform =
        ToGeometryMsgTransform(cartographer::transform::Rigid3d(
            Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z),
            Eigen::Quaterniond(pose.rotation.w, pose.rotation.x,
                               pose.rotation.y, pose.rotation.z)));
    transform.child_frame_id = link->name;
    transform.header.frame_id = link->getParent()->name;
    tf_buffer->setTransform(transform, "urdf", true /* is_static */);
    transforms.push_back(transform);
  }
  return transforms;
}

std::vector<geometry_msgs::TransformStamped> ReadStaticTransformsFromJson(
    const std::string& json_filename,
    tf2_ros::Buffer* const tf_buffer) {
    sensor_model cam0;
    sensor_model cam1;
    sensor_model cam2;
    sensor_model cam3;
    sensor_model lidar_horiz;
    sensor_model lidar_vert;
    sensor_model imu;
    parserIntrinsicJson(json_filename, cam0, cam1, cam2, cam3, lidar_horiz, lidar_vert, imu);

    std::vector<geometry_msgs::TransformStamped> transforms;

    // 添加imu的位姿信息
    geometry_msgs::TransformStamped imuTransform;
    imuTransform.transform =
        ToGeometryMsgTransform(cartographer::transform::Rigid3d(
            Eigen::Vector3d(imu.position[0], imu.position[1], imu.position[2]),
            Eigen::Quaterniond(imu.orientation[3], imu.orientation[0],
                               imu.orientation[1], imu.orientation[2])));

    imuTransform.child_frame_id = imu.name;
    imuTransform.header.frame_id = "base_link";
    tf_buffer->setTransform(imuTransform, "sensor_json", true);
    transforms.push_back(imuTransform);

    // 添加水平lidar的位姿信息
    geometry_msgs::TransformStamped lidarHorizTransform;
    lidarHorizTransform.transform =
        ToGeometryMsgTransform(cartographer::transform::Rigid3d(
            Eigen::Vector3d(lidar_horiz.position[0], lidar_horiz.position[1], lidar_horiz.position[2]),
            Eigen::Quaterniond(lidar_horiz.orientation[3], lidar_horiz.orientation[0],
                               lidar_horiz.orientation[1], lidar_horiz.orientation[2])));

    lidarHorizTransform.child_frame_id = lidar_horiz.name;
    lidarHorizTransform.header.frame_id = imu.name;
    tf_buffer->setTransform(lidarHorizTransform, "sensor_json", true);
    transforms.push_back(lidarHorizTransform);

    // 添加垂直lidar的位姿信息
    geometry_msgs::TransformStamped lidarVertTransfrom;
    lidarVertTransfrom.transform =
        ToGeometryMsgTransform(cartographer::transform::Rigid3d(
            Eigen::Vector3d(lidar_vert.position[0], lidar_vert.position[1], lidar_vert.position[2]),
            Eigen::Quaterniond(lidar_vert.orientation[3], lidar_vert.orientation[0],
                               lidar_vert.orientation[1], lidar_vert.orientation[2])));

    lidarVertTransfrom.child_frame_id = lidar_vert.name;
    lidarVertTransfrom.header.frame_id = lidar_horiz.name;
    tf_buffer->setTransform(lidarVertTransfrom, "sensor_json", true);
    transforms.push_back(lidarVertTransfrom);

    return transforms;
}

}  // namespace cartographer_ros
