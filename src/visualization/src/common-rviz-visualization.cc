#include "visualization/common-rviz-visualization.h"

#include <algorithm>

#include <glog/logging.h>
#include <pcl_ros/point_cloud.h>
#include <tf/LinearMath/Transform.h>
#include <tf/transform_broadcaster.h>

#include "visualization/color.h"
#include "visualization/eigen-visualization.h"

namespace visualization {

void setPoseToIdentity(visualization_msgs::Marker* marker) {
  CHECK_NOTNULL(marker);

  marker->pose.position.x = 0.0;
  marker->pose.position.y = 0.0;
  marker->pose.position.z = 0.0;
  marker->pose.orientation.x = 0.0;
  marker->pose.orientation.y = 0.0;
  marker->pose.orientation.z = 0.0;
  marker->pose.orientation.w = 1.0;
}

std_msgs::ColorRGBA commonColorToRosColor(
    const visualization::Color& color, double alpha) {
  CHECK_GE(alpha, 0.0);
  CHECK_LE(alpha, 1.0);

  std_msgs::ColorRGBA ros_color;
  ros_color.r = (static_cast<double>(color.red) / 255.0);
  ros_color.g = (static_cast<double>(color.green) / 255.0);
  ros_color.b = (static_cast<double>(color.blue) / 255.0);
  ros_color.a = alpha;

  return ros_color;
}

void createEmptySphereListMarker(
    size_t marker_id, const std::string& frame, const std::string& name_space,
    double scale, double alpha, visualization_msgs::Marker* sphere_marker) {
  CHECK_NOTNULL(sphere_marker);
  CHECK_GE(scale, 0.0);
  CHECK_GE(alpha, 0.0);
  CHECK_LE(alpha, 1.0);
  CHECK(ros::isInitialized())
      << "ROS hasn't been initialized. Call "
      << "RVizVisualizationSink::init() in your application code if you intend"
      << " to use RViz visualizations.";

  sphere_marker->points.clear();
  sphere_marker->colors.clear();

  sphere_marker->id = marker_id;
  sphere_marker->type = visualization_msgs::Marker::SPHERE_LIST;
  sphere_marker->scale.x = scale;
  sphere_marker->scale.y = scale;
  sphere_marker->scale.z = scale;

  sphere_marker->pose.position.x = 0.0;
  sphere_marker->pose.position.y = 0.0;
  sphere_marker->pose.position.z = 0.0;

  sphere_marker->color.a = alpha;
  sphere_marker->header.frame_id = frame;
  sphere_marker->header.stamp = ros::Time::now();
  sphere_marker->ns = name_space;
}

void eigen3XdMatrixToSpheres(
    const Eigen::Matrix3Xd& G_points, visualization_msgs::Marker* spheres) {
  CHECK_NOTNULL(spheres);

  spheres->type = visualization_msgs::Marker::SPHERE_LIST;

  setPoseToIdentity(spheres);

  const size_t num_points = G_points.cols();
  VLOG(5) << "Converting " << num_points << " points to spheres.";

  spheres->points.resize(num_points);

  for (size_t idx = 0u; idx < num_points; ++idx) {
    (spheres->points)[idx].x = G_points(0, idx);
    (spheres->points)[idx].y = G_points(1, idx);
    (spheres->points)[idx].z = G_points(2, idx);
  }
}

void eigen3XdMatrixToPointCloud(
    const Eigen::Matrix3Xd& points, const visualization::Color& color,
    unsigned char alpha, sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);

  const size_t num_points = static_cast<size_t>(points.cols());
  VLOG(5) << "Converting " << num_points << " points to point cloud.";

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(num_points);
  for (size_t idx = 0u; idx < num_points; ++idx) {
    pcl::PointXYZRGB point;
    point.x = points(0, idx);
    point.y = points(1, idx);
    point.z = points(2, idx);
    point.r = color.red;
    point.g = color.green;
    point.b = color.blue;
    point.a = alpha;
    cloud.push_back(point);
  }

  pcl::toROSMsg(cloud, *point_cloud);
}

void eigen3XfMatrixWithIntensitiesToPointCloud(
    const Eigen::Matrix3Xf& points, const Eigen::VectorXf& intensities,
    sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);

  const size_t num_points = static_cast<size_t>(points.cols());
  CHECK_EQ(num_points, static_cast<size_t>(intensities.rows()));

  point_cloud->height = 3;
  point_cloud->width = num_points;
  point_cloud->fields.resize(4);

  point_cloud->fields[0].name = "x";
  point_cloud->fields[0].offset = 0;
  point_cloud->fields[0].count = 1;
  point_cloud->fields[0].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[1].name = "y";
  point_cloud->fields[1].offset = 4;
  point_cloud->fields[1].count = 1;
  point_cloud->fields[1].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[2].name = "z";
  point_cloud->fields[2].offset = 8;
  point_cloud->fields[2].count = 1;
  point_cloud->fields[2].datatype = sensor_msgs::PointField::FLOAT32;

  point_cloud->fields[3].name = "rgb";
  point_cloud->fields[3].offset = 12;
  point_cloud->fields[3].count = 1;
  point_cloud->fields[3].datatype = sensor_msgs::PointField::UINT32;

  point_cloud->point_step = 16;
  point_cloud->row_step = point_cloud->point_step * point_cloud->width;
  point_cloud->data.resize(point_cloud->row_step * point_cloud->height);
  point_cloud->is_dense = false;

  int offset = 0;
  for (size_t point_idx = 0u; point_idx < num_points; ++point_idx) {
    const Eigen::Vector3f& point = points.col(point_idx);
    memcpy(&point_cloud->data[offset + 0], &point.x(), sizeof(point.x()));
    memcpy(
        &point_cloud->data[offset + sizeof(point.x())], &point.y(),
        sizeof(point.y()));
    memcpy(
        &point_cloud->data[offset + sizeof(point.x()) + sizeof(point.y())],
        &point.z(), sizeof(point.z()));

    const uint8_t gray = intensities[point_idx];
    const uint32_t rgb = (gray << 16) | (gray << 8) | gray;
    memcpy(&point_cloud->data[offset + 12], &rgb, sizeof(uint32_t));
    offset += point_cloud->point_step;
  }
}

void eigen3XdMatrixToPointCloud(
    const Eigen::Matrix3Xd& points, const visualization::Color& color,
    sensor_msgs::PointCloud2* point_cloud) {
  const unsigned char kFullAlpha = 255u;
  eigen3XdMatrixToPointCloud(points, color, kFullAlpha, point_cloud);
}

void eigen3XdMatrixToPointCloud(
    const Eigen::Matrix3Xd& points, sensor_msgs::PointCloud2* point_cloud) {
  const unsigned char kFullAlpha = 255u;
  eigen3XdMatrixToPointCloud(points, kCommonWhite, kFullAlpha, point_cloud);
}

void spheresToPointCloud(
    const SphereVector& spheres, sensor_msgs::PointCloud2* point_cloud) {
  CHECK_NOTNULL(point_cloud);
  if (spheres.empty()) {
    return;
  }

  const size_t num_spheres = spheres.size();
  VLOG(5) << "Converting " << num_spheres << " points to point cloud.";

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(num_spheres);
  for (size_t idx = 0u; idx < num_spheres; ++idx) {
    const Sphere& sphere = spheres[idx];

    pcl::PointXYZRGB point;
    point.x = sphere.position(0);
    point.y = sphere.position(1);
    point.z = sphere.position(2);
    point.r = sphere.color.red;
    point.g = sphere.color.green;
    point.b = sphere.color.blue;
    point.a = sphere.alpha;
    cloud.push_back(point);
  }

  pcl::toROSMsg(cloud, *point_cloud);
}

void publishLines(
    const visualization::LineSegmentVector& line_segments, size_t marker_id,
    const std::string& frame, const std::string& name_space,
    const std::string& topic) {
  CHECK(!topic.empty());
  CHECK(ros::isInitialized())
      << "ROS hasn't been initialized. Call "
      << "RVizVisualizationSink::init() in your application code if you intend"
      << " to use RViz visualizations.";

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.id = marker_id;

  double alpha = 1.0;
  if (!line_segments.empty()) {
    marker.scale.x = line_segments[0].scale;
    marker.scale.y = line_segments[0].scale;
    marker.scale.z = line_segments[0].scale;
    alpha = line_segments[0].alpha;
  }

  setPoseToIdentity(&marker);

  marker.color.a = alpha;
  marker.header.frame_id = frame;
  marker.header.stamp = ros::Time();
  marker.ns = name_space;

  const size_t num_line_segments = line_segments.size();

  marker.points.resize(2u * num_line_segments);
  marker.colors.resize(2u * num_line_segments);

  for (size_t idx = 0u; idx < num_line_segments; ++idx) {
    geometry_msgs::Point vertex0;
    vertex0.x = line_segments[idx].from[0];
    vertex0.y = line_segments[idx].from[1];
    vertex0.z = line_segments[idx].from[2];
    geometry_msgs::Point vertex1;
    vertex1.x = line_segments[idx].to[0];
    vertex1.y = line_segments[idx].to[1];
    vertex1.z = line_segments[idx].to[2];

    CHECK_EQ(line_segments[idx].alpha, alpha)
        << "All line segments must have "
           "identical alpha. Use a marker array instead if you want individual "
           "alpha values.";
    std_msgs::ColorRGBA color =
        commonColorToRosColor(line_segments[idx].color, alpha);

    marker.colors[2u * idx] = color;
    marker.colors[(2u * idx) + 1u] = color;

    marker.points[2u * idx] = vertex0;
    marker.points[(2u * idx) + 1u] = vertex1;
  }

  RVizVisualizationSink::publish<visualization_msgs::Marker>(topic, marker);
}

void publishVerticesFromPoseVector(
    PoseVector poses, const std::string& frame,
    const std::string& name_space, const std::string& topic) {
  CHECK(!topic.empty());
  const size_t num_poses = poses.size();
  if (num_poses == 0u) {
    return;
  }
  CHECK(ros::isInitialized())
      << "ROS hasn't been initialized. Call "
      << "RVizVisualizationSink::init() in your application code if you intend"
      << " to use RViz visualizations.";

  visualization_msgs::MarkerArray pose_array;

  for (size_t pose_idx = 0u; pose_idx < num_poses; ++pose_idx) {
    visualization_msgs::Marker pose_msg;

    drawAxes(
        poses[pose_idx].G_p_B, poses[pose_idx].G_q_B, poses[pose_idx].scale,
        poses[pose_idx].line_width, poses[pose_idx].alpha,
        poses[pose_idx].color, &pose_msg);

    pose_msg.header.frame_id = frame;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.id = poses[pose_idx].id;
    pose_msg.action = poses[pose_idx].action;
    pose_msg.ns = name_space;
    pose_array.markers.push_back(pose_msg);
  }
  RVizVisualizationSink::publish<visualization_msgs::MarkerArray>(
      topic, pose_array);
}

void publish3DPointsAsPointCloud(
    const Eigen::Matrix3Xd& points_G, const visualization::Color& color,
    double alpha, const std::string& frame, const std::string& topic) {
  CHECK(!topic.empty());
  if (points_G.cols() == 0u) {
    return;
  }
  CHECK_GE(alpha, 0.0);
  CHECK_LE(alpha, 1.0);
  if (alpha == 0.0) {
    LOG(WARNING) << "Alpha is 0.0. The point cloud will be invisible.";
  }
  CHECK(ros::isInitialized())
      << "ROS hasn't been initialized. Call "
      << "RVizVisualizationSink::init() in your application code if you intend"
      << " to use RViz visualizations.";

  sensor_msgs::PointCloud2 point_cloud;
  eigen3XdMatrixToPointCloud(
      points_G, color, std::floor(255.0 * alpha), &point_cloud);

  point_cloud.header.frame_id = frame;
  point_cloud.header.stamp = ros::Time::now();

  RVizVisualizationSink::publish<sensor_msgs::PointCloud2>(topic, point_cloud);
}

void publish3DPointsAsPointCloud(
    const Eigen::Matrix3Xd& points_G, std::vector<visualization::Color>& colors,
    double alpha, const std::string& frame, const std::string& topic) {
  CHECK(!topic.empty());
  if (points_G.cols() == 0u) {
    return;
  }

  CHECK(ros::isInitialized())
      << "ROS hasn't been initialized. Call "
      << "RVizVisualizationSink::init() in your application code if you intend"
      << " to use RViz visualizations.";

  const size_t num_points = static_cast<size_t>(points_G.cols());
  VLOG(5) << "Converting " << num_points << " points to point cloud.";

  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.reserve(num_points);
  for (size_t idx = 0u; idx < num_points; ++idx) {
    pcl::PointXYZRGB point;
    point.x = points_G(0, idx);
    point.y = points_G(1, idx);
    point.z = points_G(2, idx);
    point.r = colors[idx].red;
    point.g = colors[idx].green;
    point.b = colors[idx].blue;
    point.a = 1.0;
    cloud.push_back(point);
  }
  sensor_msgs::PointCloud2 point_cloud;
  pcl::toROSMsg(cloud, point_cloud);

  point_cloud.header.frame_id = frame;
  point_cloud.header.stamp = ros::Time::now();

  RVizVisualizationSink::publish<sensor_msgs::PointCloud2>(topic, point_cloud);
}

void publish3DPointsAsPointCloud(
    const Eigen::Matrix3Xf& points, const Eigen::VectorXf& intensities,
    const std::string& frame, const std::string& topic) {
  CHECK(!topic.empty());
  const size_t num_points = static_cast<size_t>(points.cols());
  if (num_points == 0u) {
    return;
  }
  CHECK_EQ(num_points, static_cast<size_t>(intensities.rows()));
  CHECK(ros::isInitialized())
      << "ROS hasn't been initialized. Call "
      << "RVizVisualizationSink::init() in your application code if you intend"
      << " to use RViz visualizations.";

  sensor_msgs::PointCloud2 point_cloud;
  eigen3XfMatrixWithIntensitiesToPointCloud(points, intensities, &point_cloud);

  point_cloud.header.frame_id = frame;
  point_cloud.header.stamp = ros::Time::now();

  RVizVisualizationSink::publish<sensor_msgs::PointCloud2>(topic, point_cloud);
}

void publishLines(
    const Eigen::Matrix3Xd& points_from, const Eigen::Matrix3Xd& points_to,
    const std::vector<visualization::Color>& color_list, double alpha,
    double scale, size_t marker_id, const std::string& frame,
    const std::string& name_space, const std::string& topic) {
  CHECK(!topic.empty());
  const size_t num_lines = points_from.cols();
  CHECK_GE(alpha, 0.0);
  CHECK_LE(alpha, 1.0);
  if (alpha == 0.0) {
    LOG(WARNING) << "Alpha is 0.0. The lines will be invisible.";
  }
  if (scale == 0.0) {
    LOG(WARNING) << "Scale is 0.0. The lines will be invisible.";
  }

  CHECK_EQ(num_lines, static_cast<size_t>(points_to.cols()));
  CHECK_EQ(num_lines, color_list.size());

  LineSegmentVector line_segments(num_lines);

  for (size_t line_idx = 0u; line_idx < num_lines; ++line_idx) {
    LineSegment& line_segment = line_segments[line_idx];

    line_segment.from = points_from.col(line_idx);
    line_segment.to = points_to.col(line_idx);

    line_segment.alpha = alpha;
    line_segment.color = color_list[line_idx];
    line_segment.scale = scale;
  }

  publishLines(line_segments, marker_id, frame, name_space, topic);
}

void publishLines(
    const Eigen::Vector3d& common_line_start_point,
    const Vector3dList& line_end_points,
    const std::vector<visualization::Color>& colors, double alpha, double scale,
    size_t marker_id, const std::string& frame, const std::string& name_space,
    const std::string& topic) {
  CHECK(!topic.empty());
  const size_t num_lines = line_end_points.size();
  if (num_lines == 0u) {
    return;
  }
  CHECK_GE(alpha, 0.0);
  CHECK_LE(alpha, 1.0);
  if (alpha == 0.0) {
    LOG(WARNING) << "Alpha is 0.0. The lines will be invisible.";
  }

  Eigen::Matrix3Xd points_from = Eigen::Matrix3Xd::Zero(3, num_lines);
  Eigen::Matrix3Xd points_to = Eigen::Matrix3Xd::Zero(3, num_lines);
  for (size_t line_idx = 0u; line_idx < num_lines; ++line_idx) {
    points_from.col(line_idx) = common_line_start_point;
    points_to.col(line_idx) = line_end_points[line_idx];
  }

  publishLines(
      points_from, points_to, colors, alpha, scale, marker_id, frame,
      name_space, topic);
}


void deleteMarker(const std::string& topic, size_t marker_id) {
  CHECK(!topic.empty());

  visualization_msgs::Marker marker;
  marker.id = marker_id;
  marker.action = visualization_msgs::Marker::DELETE;

  RVizVisualizationSink::publish<visualization_msgs::Marker>(topic, marker);
}


}  // namespace visualization
