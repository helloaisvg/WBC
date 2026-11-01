#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Eigen>
#include <random>
#include <std_msgs/Int32.h>


using namespace std; 

pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_z;

ros::Publisher _all_map_pub;
ros::Publisher _visual_map_pub;
ros::Publisher _dynamic_obs_pub;

int _obs_num, _float_obs_num;
int _dynamic_obs_num;
int _map_type;
double _x_size, _y_size, _z_size;
double _x_l, _x_h, _y_l, _y_h;
double _resolution, _pub_rate;
double _min_dist;
double _protected_radius;
double _dynamic_radius, _dynamic_speed, _dynamic_cross_x, _dynamic_cross_y, _dynamic_z;
double _dynamic_cross_offset, _dynamic_spawn_delay;
double _dynamic_y_amplitude;
bool _dynamic_loop;
std::vector<Eigen::Vector2d> _protected_positions;

bool _map_ok = false;
bool _has_odom = false;

uniform_real_distribution<double> rand_z_;
uniform_real_distribution<double> rand_box_x_;
uniform_real_distribution<double> rand_box_y_;
uniform_real_distribution<double> rand_box_z_;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 visualMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

bool nearProtectedPosition(double x, double y, double extra_radius){
  for(const auto &position : _protected_positions){
    if((Eigen::Vector2d(x, y) - position).norm() <
       _protected_radius + extra_radius){
      return true;
    }
  }
  return false;
}

void GenerateWall(double x_l, double x_h, 
                  double y_l, double y_h, 
                  double z_l, double z_h, 
                  pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  int x_num, y_num, z_num;
  double resolution = _resolution / 2.0;
  x_num = ceil((x_h - x_l)/resolution);
  y_num = ceil((y_h - y_l)/resolution);
  z_num = ceil((z_h - z_l)/resolution);
  pcl::PointXYZ pt;
  for (int i=0; i<x_num; i++)
    for (int j=0; j<y_num; j++)
      for (int k=0; k<z_num; k++){
        pt.x = x_l + i * resolution;
        pt.y = y_l + j * resolution;
        pt.z = z_l + k * resolution;
        cloudMap.points.push_back(pt);
      }
}

void GenerateBox(Eigen::Vector3d pos, Eigen::Vector3d box_size, pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  GenerateWall(pos(0) - box_size(0) / 2, pos(0) + box_size(0) / 2, 
               pos(1) - box_size(1) / 2, pos(1) + box_size(1) / 2,
               pos(2) - box_size(2) / 2, pos(2) + box_size(2) / 2, 
               cloudMap);
}

void GenerateWall(Eigen::Vector3d pos, double theta, Eigen::Vector3d wall_size, pcl::PointCloud<pcl::PointXYZ>& cloudMap){
  Eigen::Vector2d dirx(cos(theta), sin(theta));
  Eigen::Vector2d diry(cos(theta + M_PI_2), sin(theta + M_PI_2));
  double resolution = _resolution / 3.0;
  int x_num = ceil(wall_size(0) / resolution);
  int y_num = ceil(wall_size(1) / resolution);
  int z_num = ceil(wall_size(2) / resolution);
  Eigen::Vector2d posx;
  Eigen::Vector2d posy;
  pcl::PointXYZ pt;
  for (int i=0; i<x_num; i++){
    posx = pos.head(2) + (double)i * resolution * dirx;
    pt.x = posx(0);
    for (int j=0; j<y_num; j++){
      posy = posx + (double)j * resolution * diry;
      pt.y = posy(1);
      for (int k=0; k<z_num; k++){
        pt.z = pos(2) + (double)k * resolution;
        cloudMap.points.push_back(pt);
      }
    }
  }
}

void GenerateBridge(){
  pcl::PointXYZ pt_random;
  vector<Eigen::Vector2d> obs_position, obs_size;
  double x, y;
  Eigen::Vector3d bridge_size, box_size;
  Eigen::Vector2d bridge_pos;
  bridge_size << 0.8, 1.8, 0.4;
  box_size << 0.2, 0.2, 0.2;
  int bridge_num = 1;

  rand_x = uniform_real_distribution<double>(_x_l * 0.75, _x_h * 0.75);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_z_ = uniform_real_distribution<double>(-0.1, 0.1);

  rand_box_x_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_y_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_z_ = uniform_real_distribution<double>(0.8, 3.0);

  // generate wall
  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  GenerateWall(-_resolution, _resolution, _y_l, -0.75, 0.0, 0.2, cloudMap);
  GenerateWall(-_resolution, _resolution, 0.75, _y_h, 0.0, 0.2, cloudMap);

  std::vector<Eigen::Vector2d> bridge_pos_list;
  std::vector<Eigen::Vector3d> bridge_size_list;
  for(int i = 0; i < bridge_num; ++i){
    switch(i){
      case 0 :{
        bridge_pos << 0.0, 0.0;
        bridge_size << 0.6, 1.5, 0.7;
        break;
      }
    }
    bridge_pos_list.push_back(bridge_pos);
    bridge_size_list.push_back(bridge_size);
  }

  for (int i = 0; i < bridge_num && ros::ok(); i++){
    x = bridge_pos_list[i](0);
    y = bridge_pos_list[i](1);
    bridge_size = bridge_size_list[i];

    obs_position.push_back(Eigen::Vector2d(x, y));

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;

    auto pos = Eigen::Vector2d(x, y);
      // board
    GenerateWall(pos(0) - bridge_size(0) / 2.0, pos(0) + bridge_size(0) / 2.0,
                pos(1) - bridge_size(1) / 2.0, pos(1) + bridge_size(1) / 2.0,
                bridge_size(2), bridge_size(2) + 1e-3 + 2 * _resolution,
                cloudMap);
  // feet
    std::vector<Eigen::Vector2d> corner_list;
    Eigen::Vector2d corner;
    corner << pos(0) - bridge_size(0) / 2, pos(1) - bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) - bridge_size(0) / 2, pos(1) + bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) + bridge_size(0) / 2, pos(1) - bridge_size(1) / 2;
    corner_list.push_back(corner);
    corner << pos(0) + bridge_size(0) / 2, pos(1) + bridge_size(1) / 2;
    corner_list.push_back(corner);

    GenerateWall(pos(0) - bridge_size(0) / 2, pos(0) + bridge_size(0) / 2, 
                pos(1) - bridge_size(1) / 2, pos(1) - bridge_size(1) / 2 + 2 * _resolution,
                0.0, bridge_size(2), cloudMap);
    GenerateWall(pos(0) - bridge_size(0) / 2, pos(0) + bridge_size(0) / 2 , 
                pos(1) + bridge_size(1) / 2 - 2 * _resolution, pos(1) + bridge_size(1) / 2,
                0.0, bridge_size(2), cloudMap);
  }

  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;


  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
  ROS_WARN("Finished generate Bridge Map ");
}

void GenerateCuboids(){
    pcl::PointXYZ pt_random;
  vector<Eigen::Vector2d> obs_position, obs_size;
  double x, y, z;
  Eigen::Vector3d bridge_size, box_size;
  Eigen::Vector2d bridge_pos;
  bridge_size << 0.8, 1.8, 0.4;
  box_size << 0.2, 0.2, 0.2;
  int bridge_num = 0;

  std::vector<Eigen::Vector2d> bridge_pos_list;
  std::vector<Eigen::Vector3d> bridge_size_list;
  for(int i = 0; i < bridge_num; ++i){
    switch(i){
      case 0 :{
        bridge_pos << 4.7, 2.2;
        bridge_size << 0.8, 1.8, 0.4;
        break;
      }
    }
    bridge_pos_list.push_back(bridge_pos);
    bridge_size_list.push_back(bridge_size);
  }


  rand_x = uniform_real_distribution<double>(_x_l * 0.75, _x_h * 0.75);
  rand_y = uniform_real_distribution<double>(_y_l, _y_h);
  rand_z_ = uniform_real_distribution<double>(-0.1, 0.1);

  auto rand_float_x = uniform_real_distribution<double>(_x_l * 0.6, _x_h * 0.6);
  auto rand_float_y = uniform_real_distribution<double>(_y_l, _y_h);
  auto rand_float_z = uniform_real_distribution<double>(0.6, 1.1);


  rand_box_x_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_y_ = uniform_real_distribution<double>(0.1, 0.8);
  rand_box_z_ = uniform_real_distribution<double>(0.8, 3.0);

  auto rand_float_box_x = uniform_real_distribution<double>(0.3, 0.6);
  auto rand_float_box_y = uniform_real_distribution<double>(0.3, 0.6);
  auto rand_float_box_z = uniform_real_distribution<double>(0.3, 0.6);

  // generate wall
  GenerateWall(_x_l, _x_h, _y_l, _y_l + _resolution, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_h, _y_h - _resolution, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_l, _x_l + _resolution, _y_l, _y_h, 0.0, 0.2, cloudMap);
  GenerateWall(_x_h - _resolution, _x_h, _y_l, _y_h, 0.0, 0.2, cloudMap);

  // generate random box
  obs_position.clear();
  obs_size.clear();
  int box_attempts = 0;
  const int max_box_attempts = std::max(100, _obs_num * 200);
  while ((int)obs_position.size() < _obs_num && ros::ok() &&
         box_attempts++ < max_box_attempts){
    x = rand_x(eng);
    y = rand_y(eng);
    z = rand_z_(eng);
    double size_x = rand_box_x_(eng);
    double size_y = rand_box_y_(eng);
    double size_z = rand_box_z_(eng);

    box_size << size_x, size_y, size_z;

    if(nearProtectedPosition(x, y, 0.5 * std::max(size_x, size_y))){
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < _min_dist /*metres*/)
      {
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));
    obs_size.push_back(Eigen::Vector2d(size_x, size_y));

    GenerateBox(Eigen::Vector3d(x, y, z), box_size, cloudMap);
  }
  const int ground_obs_count = obs_position.size();

  obs_position.clear();
  obs_size.clear();
  int float_attempts = 0;
  const int max_float_attempts = std::max(100, _float_obs_num * 200);
  while ((int)obs_position.size() < _float_obs_num && ros::ok() &&
         float_attempts++ < max_float_attempts){
    x = rand_float_x(eng);
    y = rand_float_y(eng);
    z = rand_float_z(eng);
    double size_x = rand_float_box_x(eng);
    double size_y = rand_float_box_y(eng);
    double size_z = rand_float_box_z(eng);

    box_size << size_x, size_y, size_z;

    if(nearProtectedPosition(x, y, 0.5 * std::max(size_x, size_y))){
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    z = floor(z / _resolution) * _resolution + _resolution / 2.0;

    bool flag_continue = false;
    for (auto p : obs_position)
      if ((Eigen::Vector2d(x, y) - p).norm() < 1.0 /*metres*/)
      {
        flag_continue = true;
        break;
      }
    if (flag_continue)
      continue;

    obs_position.push_back(Eigen::Vector2d(x, y));
    obs_size.push_back(Eigen::Vector2d(size_x, size_y));

    GenerateBox(Eigen::Vector3d(x, y, z), box_size, cloudMap);
  }

  ROS_INFO("Generated %d ground and %zu floating random obstacles.",
           ground_obs_count, obs_position.size());

  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_l + _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_l + _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);
  GenerateBox(Eigen::Vector3d(_x_h - _resolution, _y_h - _resolution, 0.9), Eigen::Vector3d(0.1, 0.1, 1.8), cloudMap);

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;


  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());
  _map_ok = true;
  ROS_WARN("Finished generate Cuboids Map ");
}

void pubPoints(){
  while (ros::ok())
  {
    ros::spinOnce();
    if (_map_ok)
      break;
  }
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);

  static bool visual_map_ready = false;
  if(!visual_map_ready){
    pcl::PointCloud<pcl::PointXYZ> visual_cloud;
    const size_t target_points = 200000;
    const size_t stride = std::max<size_t>(
        1, cloudMap.points.size() / target_points);
    visual_cloud.reserve(std::min(cloudMap.points.size(), target_points));
    for(size_t i = 0; i < cloudMap.points.size(); i += stride){
      visual_cloud.push_back(cloudMap.points[i]);
    }
    visual_cloud.width = visual_cloud.points.size();
    visual_cloud.height = 1;
    visual_cloud.is_dense = true;
    pcl::toROSMsg(visual_cloud, visualMap_pcd);
    visualMap_pcd.header.frame_id = "world";
    visual_map_ready = true;
  }
  _visual_map_pub.publish(visualMap_pcd);
}

void publishDynamicObstacles(){
  if(_dynamic_obs_num <= 0)
    return;

  static double scene_start = -1.0;
  const ros::Time now = ros::Time::now();
  if(scene_start < 0.0)
    scene_start = now.toSec();
  const double elapsed = std::max(0.0, now.toSec() - scene_start);

  visualization_msgs::MarkerArray marker_array;
  for(int i = 0; i < _dynamic_obs_num; ++i){
    if(_dynamic_spawn_delay > 0.0 && i % 2 == 1 &&
       elapsed < _dynamic_spawn_delay){
      continue;
    }
    const double direction = (i % 2 == 0) ? 1.0 : -1.0;
    const double lane = i - 0.5 * (_dynamic_obs_num - 1);
    const double obstacle_elapsed =
        (i % 2 == 1) ? elapsed - _dynamic_spawn_delay : elapsed;
    const double cross_time =
        (i % 2 == 1 ? 2.0 : _dynamic_cross_offset) +
        0.8 * (i % 3) + 0.35 * std::abs(lane);
    const double initial_y =
        _dynamic_cross_y - direction * _dynamic_speed * cross_time;
    double obstacle_y =
        initial_y + direction * _dynamic_speed * obstacle_elapsed;
    double obstacle_vy = direction * _dynamic_speed;
    if(_dynamic_loop){
      const double amplitude = std::max(0.5, _dynamic_y_amplitude);
      const double period = 2.0 * amplitude / _dynamic_speed;
      const double phase = std::fmod(
          obstacle_elapsed / period + 0.5 * (i % 2), 1.0);
      if(phase < 0.5){
        obstacle_y = _dynamic_cross_y + amplitude * (-1.0 + 4.0 * phase);
        obstacle_vy = _dynamic_speed;
      }else{
        obstacle_y = _dynamic_cross_y + amplitude * (3.0 - 4.0 * phase);
        obstacle_vy = -_dynamic_speed;
      }
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = now;
    marker.ns = "dynamic_obstacles";
    marker.id = i;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = _dynamic_cross_x + 0.9 * lane;
    marker.pose.position.y = obstacle_y;
    marker.pose.position.z = _dynamic_z + 0.08 * (i % 2);
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * _dynamic_radius;
    marker.scale.y = 2.0 * _dynamic_radius;
    marker.scale.z = 2.0 * _dynamic_radius;
    marker.color.r = 1.0;
    marker.color.g = 0.25;
    marker.color.b = 0.05;
    marker.color.a = 0.85;
    // Marker.points is unused by a SPHERE, so store the velocity vector there.
    geometry_msgs::Point velocity;
    velocity.x = 0.0;
    velocity.y = obstacle_vy;
    velocity.z = 0.0;
    marker.points.push_back(velocity);
    marker_array.markers.push_back(marker);
  }
  _dynamic_obs_pub.publish(marker_array);
}

int main(int argc, char **argv){
  
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _all_map_pub =
      n.advertise<sensor_msgs::PointCloud2>(
          "/map_generator/global_cloud", 1, true);
  _visual_map_pub =
      n.advertise<sensor_msgs::PointCloud2>(
          "/map_generator/global_cloud_vis", 1, true);
  _dynamic_obs_pub =
      n.advertise<visualization_msgs::MarkerArray>(
          "/map_generator/dynamic_obstacles", 10);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/float_obs_num", _float_obs_num, 30);
  n.param("dynamic_obs/num", _dynamic_obs_num, 0);
  n.param("map/map_type", _map_type, 0);
  n.param("map/resolution", _resolution, 0.1);
  n.param("dynamic_obs/radius", _dynamic_radius, 0.45);
  n.param("dynamic_obs/speed", _dynamic_speed, 0.9);
  n.param("dynamic_obs/cross_x", _dynamic_cross_x, 0.4);
  n.param("dynamic_obs/cross_y", _dynamic_cross_y, 0.0);
  n.param("dynamic_obs/z", _dynamic_z, 0.62);
  n.param("dynamic_obs/cross_offset", _dynamic_cross_offset, 3.5);
  n.param("dynamic_obs/spawn_delay", _dynamic_spawn_delay, 0.0);
  n.param("dynamic_obs/y_amplitude", _dynamic_y_amplitude, 4.0);
  n.param("dynamic_obs/loop", _dynamic_loop, false);

  int seed = 0;
  n.param("map/seed", seed, 0);
  if(seed == 0){seed = rd();} 
  eng.seed(seed);

  n.param("pub_rate", _pub_rate, 10.0);
  n.param("min_distance", _min_dist, 1.0);
  n.param("map/protected_radius", _protected_radius, 1.2);

  _protected_positions.clear();
  std::vector<double> init_state;
  if(n.getParam("fsm/init_state", init_state) && init_state.size() >= 2){
    _protected_positions.emplace_back(init_state[0], init_state[1]);
  }
  int waypoint_num = 0;
  n.param("fsm/waypoint_num", waypoint_num, 0);
  for(int i = 0; i < waypoint_num; ++i){
    std::vector<double> waypoint;
    if(n.getParam("fsm/waypoint" + std::to_string(i), waypoint) &&
       waypoint.size() >= 2){
      _protected_positions.emplace_back(waypoint[0], waypoint[1]);
    }
  }

  _x_l = -_x_size / 2.0;
  _x_h = +_x_size / 2.0;

  _y_l = -_y_size / 2.0;
  _y_h = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  
  if(_map_type == 0){
    GenerateCuboids();
  }else if(_map_type == 1){
    GenerateBridge();
  }

  ros::Rate loop_rate(_pub_rate);
  ros::Time last_static_publish(0.0);
  int static_publish_count = 0;
  while (ros::ok()){
    const ros::Time now = ros::Time::now();
    if(static_publish_count < 5 &&
       (last_static_publish.isZero() ||
        (now - last_static_publish).toSec() >= 1.0)){
      if(_all_map_pub.getNumSubscribers() > 0){
        pubPoints();
        static_publish_count++;
      }
      last_static_publish = now;
    }
    publishDynamicObstacles();
    ros::spinOnce();
    loop_rate.sleep();
  }

}
