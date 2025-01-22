//
// Created by lfc on 2021/2/28.
//

#include "ros2_livox/livox_points_plugin.h"

#include <gazebo/physics/Model.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/transport/Node.hh>
#include <gazebo_ros/node.hpp>

#include "rclcpp/rclcpp.hpp"
#include "ros2_livox/csv_reader.hpp"
#include "ros2_livox/livox_ode_multiray_shape.h"

namespace gazebo {

GZ_REGISTER_SENSOR_PLUGIN(LivoxPointsPlugin)

LivoxPointsPlugin::LivoxPointsPlugin() {}

LivoxPointsPlugin::~LivoxPointsPlugin() {}

void convertDataToRotateInfo(const std::vector<std::vector<double>> &datas,
                             std::vector<AviaRotateInfo> &avia_infos) {
  avia_infos.reserve(datas.size());
  double deg_2_rad = M_PI / 180.0;
  for (auto &data : datas) {
    if (data.size() == 3) {
      avia_infos.emplace_back();
      avia_infos.back().time = data[0];
      avia_infos.back().azimuth = data[1] * deg_2_rad;
      avia_infos.back().zenith =
          data[2] * deg_2_rad - M_PI_2;  // 转化成标准的右手系角度
    }
  }
}

void LivoxPointsPlugin::Load(gazebo::sensors::SensorPtr _parent,
                             sdf::ElementPtr sdf) {
  node_ = gazebo_ros::Node::Get(sdf);

  std::vector<std::vector<double>> datas;
  std::string file_name = sdf->Get<std::string>("csv_file_name");
  if (!CsvReader::ReadCsvFile(file_name, datas)) {
    return;
  }
  sdfPtr = sdf;
  auto rayElem = sdfPtr->GetElement("ray");
  auto scanElem = rayElem->GetElement("scan");
  auto rangeElem = rayElem->GetElement("range");

  raySensor = _parent;
  auto sensor_pose = raySensor->Pose();
  auto curr_scan_topic = sdf->Get<std::string>("topic");
  child_name = raySensor->Name();
  parent_name = raySensor->ParentName();
  size_t delimiter_pos = parent_name.find("::");
  parent_name = parent_name.substr(delimiter_pos + 2);

  node = transport::NodePtr(new transport::Node());
  node->Init(raySensor->WorldName());
  cloud2_pub = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      curr_scan_topic, 10);

  // scanPub = node->Advertise<msgs::LaserScanStamped>(
  //     curr_scan_topic + "laserscan", 50);
  aviaInfos.clear();
  convertDataToRotateInfo(datas, aviaInfos);
  maxPointSize = aviaInfos.size();

  RayPlugin::Load(_parent, sdfPtr);
  laserMsg.mutable_scan()->set_frame(_parent->ParentName());
  // parentEntity = world->GetEntity(_parent->ParentName());
  parentEntity = this->world->EntityByName(_parent->ParentName());
  // SendRosTf(sensor_pose, raySensor->ParentName(), raySensor->Name());
  auto physics = world->Physics();
  laserCollision = physics->CreateCollision("multiray", _parent->ParentName());
  laserCollision->SetName("ray_sensor_collision");
  laserCollision->SetRelativePose(_parent->Pose());
  laserCollision->SetInitialRelativePose(_parent->Pose());
  rayShape.reset(new gazebo::physics::LivoxOdeMultiRayShape(laserCollision));
  laserCollision->SetShape(rayShape);
  samplesStep = sdfPtr->Get<int>("samples");
  downSample = sdfPtr->Get<int>("downsample");
  if (downSample < 1) {
    downSample = 1;
  }

  rayShape->RayShapes().reserve(samplesStep / downSample);
  rayShape->Load(sdfPtr);
  rayShape->Init();
  minDist = rangeElem->Get<double>("min");
  maxDist = rangeElem->Get<double>("max");
  auto offset = laserCollision->RelativePose();
  ignition::math::Vector3d start_point, end_point;
  for (int j = 0; j < samplesStep; j += downSample) {
    int index = j % maxPointSize;
    auto &rotate_info = aviaInfos[index];
    ignition::math::Quaterniond ray;
    ray.Euler(
        ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
    auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
    start_point = 0.1 * axis + offset.Pos();
    end_point = maxDist * axis + offset.Pos();
    rayShape->AddRay(start_point, end_point);
  }
}
void LivoxPointsPlugin::InitPointcloud2MsgHeader(
    sensor_msgs::msg::PointCloud2 &cloud) {
  //cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(7);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = PointField::UINT8;
  cloud.fields[6].offset = 18;
  cloud.fields[6].name = "timestamp";
  cloud.fields[6].count = 1;
  cloud.fields[6].datatype = PointField::FLOAT64;
  cloud.point_step = sizeof(LivoxPointXyzrtlt);
}

void LivoxPointsPlugin::OnNewLaserScans() {
  if (rayShape) {
    std::vector<std::pair<int, AviaRotateInfo>> points_pair;
    InitializeRays(points_pair, rayShape);
    rayShape->Update();

    msgs::Set(laserMsg.mutable_time(), world->SimTime());
    // msgs::LaserScan *scan = laserMsg.mutable_scan();
    // InitializeScan(scan);

    // SendRosTf(parentEntity->WorldPose(), world->Name(),
    // raySensor->ParentName());

    auto rayCount = RayCount();
    auto verticalRayCount = VerticalRayCount();
    auto angle_min = AngleMin().Radian();
    auto angle_incre = AngleResolution();
    auto verticle_min = VerticalAngleMin().Radian();
    auto verticle_incre = VerticalAngleResolution();

    sensor_msgs::msg::PointCloud2 cloud2;
    cloud2.header.stamp = node_->get_clock()->now();
    cloud2.header.frame_id = raySensor->Name();
    std::vector<LivoxPointXyzrtlt> points;

    for (auto &pair : points_pair) {
      int verticle_index =
          roundf((pair.second.zenith - verticle_min) / verticle_incre);
      int horizon_index =
          roundf((pair.second.azimuth - angle_min) / angle_incre);
      if (verticle_index < 0 || horizon_index < 0) {
        continue;
      }
      if (verticle_index < verticalRayCount && horizon_index < rayCount) {
        auto index =
            (verticalRayCount - verticle_index - 1) * rayCount + horizon_index;
        auto range = rayShape->GetRange(pair.first);

        if(minDist>range || maxDist<range){
          continue;
        }

        auto rotate_info = pair.second;
        ignition::math::Quaterniond ray;
        ray.Euler(ignition::math::Vector3d(0.0, rotate_info.zenith,
                                           rotate_info.azimuth));
        auto axis = ray * ignition::math::Vector3d(1.0, 0.0, 0.0);

        auto point = range * axis;

        LivoxPointXyzrtlt livox_point;

        livox_point.x = point.X();
        livox_point.y = point.Y();
        // auto angle = atan2(livox_point.y,livox_point.x);
        // if(angle< (-3.14/2 - 0.77) || angle > (3.14 - 0.8)){
        //   continue;
        // }


        livox_point.z = point.Z();
        livox_point.reflectivity = 1.0f;
        livox_point.tag = 0;
        livox_point.line = 1;
        livox_point.timestamp = static_cast<double>(node_->get_clock()->now().nanoseconds());
        points.push_back(std::move(livox_point));

      } else {
        //                ROS_INFO_STREAM("count is wrong:" << verticle_index <<
        //                "," << verticalRayCount << ","
        //                << horizon_index
        //                          << "," << rayCount << "," <<
        //                          pair.second.zenith << "," <<
        //                          pair.second.azimuth);
      }
    }
    // if (scanPub && scanPub->HasConnections()) scanPub->Publish(laserMsg);

    // scanPub->Publish(laserMsg);
    InitPointcloud2MsgHeader(cloud2);
    cloud2.point_step = sizeof(LivoxPointXyzrtlt);
    cloud2.width = points.size();
    cloud2.row_step = cloud2.width * cloud2.point_step;
    cloud2.is_bigendian = false;
    cloud2.is_dense = true;
    cloud2.data.resize(cloud2.width * sizeof(LivoxPointXyzrtlt));
    memcpy(cloud2.data.data(), points.data(),
           cloud2.width * sizeof(LivoxPointXyzrtlt));
    cloud2_pub->publish(cloud2);
  }
}

void LivoxPointsPlugin::InitializeRays(
    std::vector<std::pair<int, AviaRotateInfo>> &points_pair,
    boost::shared_ptr<physics::LivoxOdeMultiRayShape> &ray_shape) {
  auto &rays = ray_shape->RayShapes();
  ignition::math::Vector3d start_point, end_point;
  ignition::math::Quaterniond ray;
  auto offset = laserCollision->RelativePose();
  int64_t end_index = currStartIndex + samplesStep;
  long unsigned int ray_index = 0;
  auto ray_size = rays.size();
  points_pair.reserve(rays.size());
  for (int k = currStartIndex; k < end_index; k += downSample) {
    auto index = k % maxPointSize;
    auto &rotate_info = aviaInfos[index];
    ray.Euler(
        ignition::math::Vector3d(0.0, rotate_info.zenith, rotate_info.azimuth));
    auto axis = offset.Rot() * ray * ignition::math::Vector3d(1.0, 0.0, 0.0);
    start_point = 0.1 * axis + offset.Pos();
    end_point = maxDist * axis + offset.Pos();
    if (ray_index < ray_size) {
      rays[ray_index]->SetPoints(start_point, end_point);
      points_pair.emplace_back(ray_index, rotate_info);
    }
    ray_index++;
  }
  currStartIndex += samplesStep;
}

void LivoxPointsPlugin::InitializeScan(msgs::LaserScan *&scan) {
  // Store the latest laser scans into laserMsg
  msgs::Set(scan->mutable_world_pose(),
            raySensor->Pose() + parentEntity->WorldPose());
  scan->set_angle_min(AngleMin().Radian());
  scan->set_angle_max(AngleMax().Radian());
  scan->set_angle_step(AngleResolution());
  scan->set_count(RangeCount());

  scan->set_vertical_angle_min(VerticalAngleMin().Radian());
  scan->set_vertical_angle_max(VerticalAngleMax().Radian());
  scan->set_vertical_angle_step(VerticalAngleResolution());
  scan->set_vertical_count(VerticalRangeCount());

  scan->set_range_min(RangeMin());
  scan->set_range_max(RangeMax());

  scan->clear_ranges();
  scan->clear_intensities();

  unsigned int rangeCount = RangeCount();
  unsigned int verticalRangeCount = VerticalRangeCount();

  for (unsigned int j = 0; j < verticalRangeCount; ++j) {
    for (unsigned int i = 0; i < rangeCount; ++i) {
      scan->add_ranges(0);
      scan->add_intensities(0);
    }
  }
}

ignition::math::Angle LivoxPointsPlugin::AngleMin() const {
  if (rayShape)
    return rayShape->MinAngle();
  else
    return -1;
}

ignition::math::Angle LivoxPointsPlugin::AngleMax() const {
  if (rayShape) {
    return ignition::math::Angle(rayShape->MaxAngle().Radian());
  } else
    return -1;
}

double LivoxPointsPlugin::GetRangeMin() const { return RangeMin(); }

double LivoxPointsPlugin::RangeMin() const {
  if (rayShape)
    return rayShape->GetMinRange();
  else
    return -1;
}

double LivoxPointsPlugin::GetRangeMax() const { return RangeMax(); }

double LivoxPointsPlugin::RangeMax() const {
  if (rayShape)
    return rayShape->GetMaxRange();
  else
    return -1;
}

double LivoxPointsPlugin::GetAngleResolution() const {
  return AngleResolution();
}

double LivoxPointsPlugin::AngleResolution() const {
  return (AngleMax() - AngleMin()).Radian() / (RangeCount() - 1);
}

double LivoxPointsPlugin::GetRangeResolution() const {
  return RangeResolution();
}

double LivoxPointsPlugin::RangeResolution() const {
  if (rayShape)
    return rayShape->GetResRange();
  else
    return -1;
}

int LivoxPointsPlugin::GetRayCount() const { return RayCount(); }

int LivoxPointsPlugin::RayCount() const {
  if (rayShape)
    return rayShape->GetSampleCount();
  else
    return -1;
}

int LivoxPointsPlugin::GetRangeCount() const { return RangeCount(); }

int LivoxPointsPlugin::RangeCount() const {
  if (rayShape)
    return rayShape->GetSampleCount() * rayShape->GetScanResolution();
  else
    return -1;
}

int LivoxPointsPlugin::GetVerticalRayCount() const {
  return VerticalRayCount();
}

int LivoxPointsPlugin::VerticalRayCount() const {
  if (rayShape)
    return rayShape->GetVerticalSampleCount();
  else
    return -1;
}

int LivoxPointsPlugin::GetVerticalRangeCount() const {
  return VerticalRangeCount();
}

int LivoxPointsPlugin::VerticalRangeCount() const {
  if (rayShape)
    return rayShape->GetVerticalSampleCount() *
           rayShape->GetVerticalScanResolution();
  else
    return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMin() const {
  if (rayShape) {
    return ignition::math::Angle(rayShape->VerticalMinAngle().Radian());
  } else
    return -1;
}

ignition::math::Angle LivoxPointsPlugin::VerticalAngleMax() const {
  if (rayShape) {
    return ignition::math::Angle(rayShape->VerticalMaxAngle().Radian());
  } else
    return -1;
}

double LivoxPointsPlugin::GetVerticalAngleResolution() const {
  return VerticalAngleResolution();
}

double LivoxPointsPlugin::VerticalAngleResolution() const {
  return (VerticalAngleMax() - VerticalAngleMin()).Radian() /
         (VerticalRangeCount() - 1);
}

}  // namespace gazebo
