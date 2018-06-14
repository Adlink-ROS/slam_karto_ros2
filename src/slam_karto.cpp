/*
 * slam_karto
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * THE WORK (AS DEFINED BELOW) IS PROVIDED UNDER THE TERMS OF THIS CREATIVE
 * COMMONS PUBLIC LICENSE ("CCPL" OR "LICENSE"). THE WORK IS PROTECTED BY
 * COPYRIGHT AND/OR OTHER APPLICABLE LAW. ANY USE OF THE WORK OTHER THAN AS
 * AUTHORIZED UNDER THIS LICENSE OR COPYRIGHT LAW IS PROHIBITED.
 *
 * BY EXERCISING ANY RIGHTS TO THE WORK PROVIDED HERE, YOU ACCEPT AND AGREE TO
 * BE BOUND BY THE TERMS OF THIS LICENSE. THE LICENSOR GRANTS YOU THE RIGHTS
 * CONTAINED HERE IN CONSIDERATION OF YOUR ACCEPTANCE OF SUCH TERMS AND
 * CONDITIONS.
 *
 */

/* Author: Brian Gerkey */

/**

@mainpage karto_gmapping

@htmlinclude manifest.html

*/


#define ROS_INFO RCUTILS_LOG_INFO
#define ROS_ERROR RCUTILS_LOG_ERROR
#define ROS_FATAL RCUTILS_LOG_FATAL
#define ROS_WARN RCUTILS_LOG_WARN
//#define ROS_DEBUG RCUTILS_LOG_DEBUG
#define ROS_DEBUG RCUTILS_LOG_INFO

#include <string>
#include <map>
#include <vector>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/time_source.hpp" 

#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/srv/get_map.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "open_karto/Mapper.h"
#include "spa_solver.h"

// compute linear index for given map coords
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

class SlamKarto
{
  public:
    SlamKarto(std::shared_ptr<rclcpp::Node> node);
    ~SlamKarto();

    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    bool mapCallback(const nav_msgs::srv::GetMap::Request::SharedPtr req,
                           nav_msgs::srv::GetMap::Response::SharedPtr res);

  private:
    bool getOdomPose(karto::Pose2& karto_pose, const rclcpp::Time& t);
    karto::LaserRangeFinder* getLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    bool addScan(karto::LaserRangeFinder* laser,
                 const sensor_msgs::msg::LaserScan::SharedPtr scan,
                 karto::Pose2& karto_pose);
    bool updateMap();
    void publishLoop();
    void publishGraphVisualization();
    double getYaw(tf2::Transform& t);

    // ROS 2.0 Objects (Subscription, Publisher, tf2, etc)
    std::shared_ptr<rclcpp::Node> node;    
    tf2_ros::Buffer tf2_buffer_;    
    tf2_ros::TransformListener tf2_;
    tf2_ros::TransformBroadcaster* tf2B_;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_filter_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr sst_;
    rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr sstm_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    rclcpp::ServiceBase::SharedPtr ss_;
    rclcpp::TimerBase::SharedPtr timer_;


    // The map that will be published / send to service callers
    nav_msgs::srv::GetMap::Response map_;

    // Storage for ROS parameters
    std::string odom_frame_;
    std::string map_frame_;
    std::string base_frame_;
    int throttle_scans_;
    double resolution_;
    double map_update_interval_, transform_tolerance_;

    // mutex lock
    std::mutex map_mutex_;
    std::mutex map_to_odom_mutex_;

    // Karto bookkeeping
    karto::Mapper* mapper_;
    karto::Dataset* dataset_;
    SpaSolver* solver_;
    std::map<std::string, karto::LaserRangeFinder*> lasers_;
    std::map<std::string, bool> lasers_inverted_;

    // Internal state
    bool got_map_;
    int laser_count_;
    tf2::Transform map_to_odom_;
    unsigned marker_count_;
    bool inverted_laser_;
}; // end of class declaration


SlamKarto::SlamKarto(std::shared_ptr<rclcpp::Node> node_) :
        tf2_(tf2_buffer_),
        got_map_(false),
        laser_count_(0),
        marker_count_(0)
{
    node = node_;
    
    map_to_odom_.setIdentity();
  
    // Get parameters
    node->get_parameter_or("odom_frame", odom_frame_, std::string("odom"));  
    node->get_parameter_or("map_frame", map_frame_, std::string("map"));  
    node->get_parameter_or("base_frame", base_frame_, std::string("base_link"));
    node->get_parameter_or("throttle_scans", throttle_scans_, 1);
    node->get_parameter_or("resolution", resolution_, 0.05);
    node->get_parameter_or("delta", resolution_, 0.05);
    node->get_parameter_or("transform_tolerance", transform_tolerance_, 0.05); //sec

    double tmp_sec;
    node->get_parameter_or("map_update_interval", tmp_sec, 5.0); //sec
    map_update_interval_ = tmp_sec*10e9; //to nanosec
    
    double transform_publish_period;
    node->get_parameter_or("transform_publish_period", transform_publish_period, 0.05); //sec

    // Set up advertisements and subscriptions
    tf2B_ = new tf2_ros::TransformBroadcaster(node);
    sst_  = node->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rmw_qos_profile_default);
    sstm_ = node->create_publisher<nav_msgs::msg::MapMetaData>("map_metadata", rmw_qos_profile_default);
    marker_publisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>(
                        "visualization_marker_array", rmw_qos_profile_sensor_data);    

    scan_filter_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
                      "scan", std::bind(&SlamKarto::laserCallback, this, std::placeholders::_1), rmw_qos_profile_sensor_data);

    ss_ = node->create_service<nav_msgs::srv::GetMap>(
          "dynamic_map", std::bind(&SlamKarto::mapCallback, this, std::placeholders::_1, std::placeholders::_2));

    auto loop_interval = std::chrono::milliseconds(int(transform_publish_period*1000.0)); // to milliseconds
    timer_ = node->create_wall_timer(loop_interval, std::bind(&SlamKarto::publishLoop, this));

    // Initialize Karto structures
    mapper_ = new karto::Mapper();
    dataset_ = new karto::Dataset();

    // Setting General Parameters from the Parameter Server
    bool use_scan_matching;
    if(node->get_parameter("use_scan_matching", use_scan_matching))
        mapper_->setParamUseScanMatching(use_scan_matching);

    bool use_scan_barycenter;
    if(node->get_parameter("use_scan_barycenter", use_scan_barycenter))
        mapper_->setParamUseScanBarycenter(use_scan_barycenter);

    double minimum_travel_distance;
    if(node->get_parameter("minimum_travel_distance", minimum_travel_distance))
        mapper_->setParamMinimumTravelDistance(minimum_travel_distance);

    double minimum_travel_heading;
    if(node->get_parameter("minimum_travel_heading", minimum_travel_heading))
        mapper_->setParamMinimumTravelHeading(minimum_travel_heading);

    int scan_buffer_size;
    if(node->get_parameter("scan_buffer_size", scan_buffer_size))
        mapper_->setParamScanBufferSize(scan_buffer_size);

    double scan_buffer_maximum_scan_distance;
    if(node->get_parameter("scan_buffer_maximum_scan_distance", scan_buffer_maximum_scan_distance))
        mapper_->setParamScanBufferMaximumScanDistance(scan_buffer_maximum_scan_distance);

    double link_match_minimum_response_fine;
    if(node->get_parameter("link_match_minimum_response_fine", link_match_minimum_response_fine))
        mapper_->setParamLinkMatchMinimumResponseFine(link_match_minimum_response_fine);

    double link_scan_maximum_distance;
    if(node->get_parameter("link_scan_maximum_distance", link_scan_maximum_distance))
        mapper_->setParamLinkScanMaximumDistance(link_scan_maximum_distance);

    double loop_search_maximum_distance;
    if(node->get_parameter("loop_search_maximum_distance", loop_search_maximum_distance))
        mapper_->setParamLoopSearchMaximumDistance(loop_search_maximum_distance);

    bool do_loop_closing;
    if(node->get_parameter("do_loop_closing", do_loop_closing))
        mapper_->setParamDoLoopClosing(do_loop_closing);

    int loop_match_minimum_chain_size;
    if(node->get_parameter("loop_match_minimum_chain_size", loop_match_minimum_chain_size))
        mapper_->setParamLoopMatchMinimumChainSize(loop_match_minimum_chain_size);

    double loop_match_maximum_variance_coarse;
    if(node->get_parameter("loop_match_maximum_variance_coarse", loop_match_maximum_variance_coarse))
        mapper_->setParamLoopMatchMaximumVarianceCoarse(loop_match_maximum_variance_coarse);

    double loop_match_minimum_response_coarse;
    if(node->get_parameter("loop_match_minimum_response_coarse", loop_match_minimum_response_coarse))
        mapper_->setParamLoopMatchMinimumResponseCoarse(loop_match_minimum_response_coarse);

    double loop_match_minimum_response_fine;
    if(node->get_parameter("loop_match_minimum_response_fine", loop_match_minimum_response_fine))
        mapper_->setParamLoopMatchMinimumResponseFine(loop_match_minimum_response_fine);

    // Setting Correlation Parameters from the Parameter Server

    double correlation_search_space_dimension;
    if(node->get_parameter("correlation_search_space_dimension", correlation_search_space_dimension))
        mapper_->setParamCorrelationSearchSpaceDimension(correlation_search_space_dimension);

    double correlation_search_space_resolution;
    if(node->get_parameter("correlation_search_space_resolution", correlation_search_space_resolution))
        mapper_->setParamCorrelationSearchSpaceResolution(correlation_search_space_resolution);

    double correlation_search_space_smear_deviation;
    if(node->get_parameter("correlation_search_space_smear_deviation", correlation_search_space_smear_deviation))
        mapper_->setParamCorrelationSearchSpaceSmearDeviation(correlation_search_space_smear_deviation);

    // Setting Correlation Parameters, Loop Closure Parameters from the Parameter Server

    double loop_search_space_dimension;
    if(node->get_parameter("loop_search_space_dimension", loop_search_space_dimension))
        mapper_->setParamLoopSearchSpaceDimension(loop_search_space_dimension);

    double loop_search_space_resolution;
    if(node->get_parameter("loop_search_space_resolution", loop_search_space_resolution))
        mapper_->setParamLoopSearchSpaceResolution(loop_search_space_resolution);

    double loop_search_space_smear_deviation;
    if(node->get_parameter("loop_search_space_smear_deviation", loop_search_space_smear_deviation))
        mapper_->setParamLoopSearchSpaceSmearDeviation(loop_search_space_smear_deviation);

    // Setting Scan Matcher Parameters from the Parameter Server

    double distance_variance_penalty;
    if(node->get_parameter("distance_variance_penalty", distance_variance_penalty))
        mapper_->setParamDistanceVariancePenalty(distance_variance_penalty);

    double angle_variance_penalty;
    if(node->get_parameter("angle_variance_penalty", angle_variance_penalty))
        mapper_->setParamAngleVariancePenalty(angle_variance_penalty);

    double fine_search_angle_offset;
    if(node->get_parameter("fine_search_angle_offset", fine_search_angle_offset))
        mapper_->setParamFineSearchAngleOffset(fine_search_angle_offset);

    double coarse_search_angle_offset;
    if(node->get_parameter("coarse_search_angle_offset", coarse_search_angle_offset))
        mapper_->setParamCoarseSearchAngleOffset(coarse_search_angle_offset);

    double coarse_angle_resolution;
    if(node->get_parameter("coarse_angle_resolution", coarse_angle_resolution))
        mapper_->setParamCoarseAngleResolution(coarse_angle_resolution);

    double minimum_angle_penalty;
    if(node->get_parameter("minimum_angle_penalty", minimum_angle_penalty))
        mapper_->setParamMinimumAnglePenalty(minimum_angle_penalty);

    double minimum_distance_penalty;
    if(node->get_parameter("minimum_distance_penalty", minimum_distance_penalty))
        mapper_->setParamMinimumDistancePenalty(minimum_distance_penalty);

    bool use_response_expansion;
    if(node->get_parameter("use_response_expansion", use_response_expansion))
        mapper_->setParamUseResponseExpansion(use_response_expansion);

    // Set solver to be used in loop closure
    solver_ = new SpaSolver();
    mapper_->SetScanSolver(solver_);
}

SlamKarto::~SlamKarto()
{
    if (solver_)
        delete solver_;
    if (mapper_)
        delete mapper_;
    if (dataset_)
        delete dataset_;
    if (tf2B_)
        delete tf2B_;
    // TODO: delete the pointers in the lasers_ map; not sure whether or not
    // I'm supposed to do that.
}


double
SlamKarto::getYaw(tf2::Transform& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}



void 
SlamKarto::publishLoop()
{
    std::unique_lock<std::mutex> lock(map_to_odom_mutex_);

    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = map_frame_;
    tmp_tf_stamped.child_frame_id = odom_frame_;
    tmp_tf_stamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now() + rclcpp::Duration( std::chrono::nanoseconds(int(transform_tolerance_*10e9)) );
    tmp_tf_stamped.transform = tf2::toMsg(map_to_odom_);
    tf2B_->sendTransform(tmp_tf_stamped);
}


karto::LaserRangeFinder* 
SlamKarto::getLaser(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  // Check whether we know about this laser yet
  if(lasers_.find(scan->header.frame_id) == lasers_.end())
  {
    // New laser; need to create a Karto device for it.

    // Get the laser's pose, relative to base.
    tf2::Stamped<tf2::Transform> ident (tf2::Transform(tf2::Quaternion::getIdentity(),
                                           tf2::Vector3(0,0,0)), tf2_ros::fromMsg(scan->header.stamp), scan->header.frame_id);
    tf2::Stamped<tf2::Transform> laser_pose;

    try
    {
        geometry_msgs::msg::TransformStamped laser_pose_msg;
        tf2_buffer_.transform( tf2::toMsg<tf2::Stamped<tf2::Transform>, geometry_msgs::msg::TransformStamped>(ident),
                               laser_pose_msg, base_frame_);
        tf2::fromMsg(laser_pose_msg, laser_pose);          
        //tf2_.transformPose(base_frame_, ident, laser_pose); // target_frame, in, out,       
    }
    catch(tf2::TransformException e)
    {
      ROS_WARN("Failed to compute laser pose, aborting initialization (%s)", e.what());
      return NULL;
    }

    double yaw = getYaw(laser_pose);

    ROS_INFO("laser %s's pose wrt base: %.3f %.3f %.3f",
	     scan->header.frame_id.c_str(),
	     laser_pose.getOrigin().x(),
	     laser_pose.getOrigin().y(),
	     yaw);
    
    // To account for lasers that are mounted upside-down,
    // we create a point 1m above the laser and transform it into the laser frame
    // if the point's z-value is <=0, it is upside-down
    geometry_msgs::msg::Vector3Stamped up_in, up_out;
    up_in.header.stamp = scan->header.stamp;
    up_in.header.frame_id = base_frame_;
    up_in.vector.z = 1.0 + laser_pose.getOrigin().z();

    try
    {
        tf2_buffer_.transform( up_in, up_out, scan->header.frame_id);

        ROS_DEBUG("Z-Axis in sensor frame: %.3f", up_out.vector.z);
    }
    catch (tf2::TransformException& e)
    {
      ROS_WARN("Unable to determine orientation of laser: %s", e.what());
      return NULL;
    }

    bool inverse = lasers_inverted_[scan->header.frame_id] = up_out.vector.z <= 0;
    if (inverse)
      ROS_INFO("laser is mounted upside-down");


    // Create a laser range finder device and copy in data from the first
    // scan
    std::string name = scan->header.frame_id;
    karto::LaserRangeFinder* laser = 
      karto::LaserRangeFinder::CreateLaserRangeFinder(karto::LaserRangeFinder_Custom, karto::Name(name));
    laser->SetOffsetPose(karto::Pose2(laser_pose.getOrigin().x(),
				      laser_pose.getOrigin().y(),
				      yaw));
    laser->SetMinimumRange(scan->range_min);
    laser->SetMaximumRange(scan->range_max);
    laser->SetMinimumAngle(scan->angle_min);
    laser->SetMaximumAngle(scan->angle_max);
    laser->SetAngularResolution(scan->angle_increment);
    // TODO: expose this, and many other parameters
    //laser_->SetRangeThreshold(12.0);

    // Store this laser device for later
    lasers_[scan->header.frame_id] = laser;

    // Add it to the dataset, which seems to be necessary
    dataset_->Add(laser);
  }

  return lasers_[scan->header.frame_id];
}

bool
SlamKarto::getOdomPose(karto::Pose2& karto_pose, const rclcpp::Time& t)
{
    // Get the robot's pose
    tf2::Stamped<tf2::Transform> ident (tf2::Transform(tf2::Quaternion::getIdentity(),
                                        tf2::Vector3(0,0,0)), tf2_ros::fromMsg(t), base_frame_);
    tf2::Stamped<tf2::Transform> odom_pose;

    try
    {
        geometry_msgs::msg::TransformStamped odom_pose_msg;
        tf2_buffer_.transform( tf2::toMsg<tf2::Stamped<tf2::Transform>, geometry_msgs::msg::TransformStamped>(ident),
                               odom_pose_msg, odom_frame_);
        tf2::fromMsg(odom_pose_msg, odom_pose);   
    }
    catch(tf2::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = getYaw(odom_pose);

    karto_pose = karto::Pose2(odom_pose.getOrigin().x(),
                   odom_pose.getOrigin().y(),
                   yaw);
    return true;
}

void
SlamKarto::publishGraphVisualization()
{
  std::vector<float> graph;
  solver_->getGraph(graph);

  visualization_msgs::msg::MarkerArray marray;

  visualization_msgs::msg::Marker m;
  m.header.frame_id = "map";
  m.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  m.id = 0;
  m.ns = "karto";
  m.type = 2; //visualization_msgs::msg::Marker::SPHERE;
  m.pose.position.x = 0.0;
  m.pose.position.y = 0.0;
  m.pose.position.z = 0.0;
  m.scale.x = 0.1;
  m.scale.y = 0.1;
  m.scale.z = 0.1;
  m.color.r = 1.0;
  m.color.g = 0;
  m.color.b = 0.0;
  m.color.a = 1.0;
  m.lifetime = rclcpp::Duration(0,0);

  visualization_msgs::msg::Marker edge;
  edge.header.frame_id = "map";
  edge.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  edge.action = 0; //visualization_msgs::Marker::ADD;
  edge.ns = "karto";
  edge.id = 0;
  edge.type = 4; //visualization_msgs::Marker::LINE_STRIP;
  edge.scale.x = 0.1;
  edge.scale.y = 0.1;
  edge.scale.z = 0.1;
  edge.color.a = 1.0;
  edge.color.r = 0.0;
  edge.color.g = 0.0;
  edge.color.b = 1.0;
  
  m.action = 0; //visualization_msgs::Marker::ADD;
  uint id = 0;
  for (uint i=0; i<graph.size()/2; i++) 
  {
    m.id = id;
    m.pose.position.x = graph[2*i];
    m.pose.position.y = graph[2*i+1];
    marray.markers.push_back(visualization_msgs::msg::Marker(m));
    id++;

    if(i>0)
    {
      edge.points.clear();

      geometry_msgs::msg::Point p;
      p.x = graph[2*(i-1)];
      p.y = graph[2*(i-1)+1];
      edge.points.push_back(p);
      p.x = graph[2*i];
      p.y = graph[2*i+1];
      edge.points.push_back(p);
      edge.id = id;

      marray.markers.push_back(visualization_msgs::msg::Marker(edge));
      id++;
    }
  }

  m.action = 2;//visualization_msgs::Marker::DELETE;
  for (; id < marker_count_; id++) 
  {
    m.id = id;
    marray.markers.push_back(visualization_msgs::msg::Marker(m));
  }

  marker_count_ = marray.markers.size();

  marker_publisher_->publish(marray);
}

void
SlamKarto::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
{
  laser_count_++;
  if ((laser_count_ % throttle_scans_) != 0)
    return;

  static rclcpp::Time last_map_update(0,0); //TODO: replace static variable

  // Check whether we know about this laser yet
  karto::LaserRangeFinder* laser = getLaser(scan);

  if(!laser)
  {
    ROS_WARN("Failed to create laser device for %s; discarding scan",
	     scan->header.frame_id.c_str());
    return;
  }

  karto::Pose2 odom_pose;
  if(addScan(laser, scan, odom_pose))
  {
    ROS_DEBUG("added scan at pose: %.3f %.3f %.3f", 
              odom_pose.GetX(),
              odom_pose.GetY(),
              odom_pose.GetHeading());

    publishGraphVisualization();

    if(!got_map_ || 
       (rclcpp::Time(scan->header.stamp) - last_map_update).nanoseconds() > map_update_interval_)
    {
      if(updateMap())
      {
        last_map_update = rclcpp::Time(scan->header.stamp);
        got_map_ = true;
        ROS_DEBUG("Updated the map");
      }
    }
  }
}

bool
SlamKarto::updateMap()
{
  std::unique_lock<std::mutex> lock(map_mutex_);

  karto::OccupancyGrid* occ_grid = 
          karto::OccupancyGrid::CreateFromScans(mapper_->GetAllProcessedScans(), resolution_);

  if(!occ_grid)
    return false;

  if(!got_map_) {
    map_.map.info.resolution = resolution_;
    map_.map.info.origin.position.x = 0.0;
    map_.map.info.origin.position.y = 0.0;
    map_.map.info.origin.position.z = 0.0;
    map_.map.info.origin.orientation.x = 0.0;
    map_.map.info.origin.orientation.y = 0.0;
    map_.map.info.origin.orientation.z = 0.0;
    map_.map.info.origin.orientation.w = 1.0;
  } 

  // Translate to ROS format
  kt_int32s width = occ_grid->GetWidth();
  kt_int32s height = occ_grid->GetHeight();
  karto::Vector2<kt_double> offset = occ_grid->GetCoordinateConverter()->GetOffset();

  if(map_.map.info.width != (unsigned int) width || 
     map_.map.info.height != (unsigned int) height ||
     map_.map.info.origin.position.x != offset.GetX() ||
     map_.map.info.origin.position.y != offset.GetY())
  {
    map_.map.info.origin.position.x = offset.GetX();
    map_.map.info.origin.position.y = offset.GetY();
    map_.map.info.width = width;
    map_.map.info.height = height;
    map_.map.data.resize(map_.map.info.width * map_.map.info.height);
  }

  for (kt_int32s y=0; y<height; y++)
  {
    for (kt_int32s x=0; x<width; x++) 
    {
      // Getting the value at position x,y
      kt_int8u value = occ_grid->GetValue(karto::Vector2<kt_int32s>(x, y));

      switch (value)
      {
        case karto::GridStates_Unknown:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = -1;
          break;
        case karto::GridStates_Occupied:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 100;
          break;
        case karto::GridStates_Free:
          map_.map.data[MAP_IDX(map_.map.info.width, x, y)] = 0;
          break;
        default:
          ROS_WARN("Encountered unknown cell value at %d, %d", x, y);
          break;
      }
    }
  }
  
  // Set the header information on the map
  map_.map.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  map_.map.header.frame_id = map_frame_;

  sst_->publish(map_.map);
  sstm_->publish(map_.map.info);

  delete occ_grid;

  return true;
}

bool
SlamKarto::addScan(karto::LaserRangeFinder* laser,
		           const sensor_msgs::msg::LaserScan::SharedPtr scan, 
                   karto::Pose2& karto_pose)
{
    if(!getOdomPose(karto_pose, scan->header.stamp))
        return false;

    // Create a vector of doubles for karto
    std::vector<kt_double> readings;

    if (lasers_inverted_[scan->header.frame_id]) 
    {
        for(std::vector<float>::const_reverse_iterator it = scan->ranges.rbegin();
            it != scan->ranges.rend();
            ++it)
        {
            readings.push_back(*it);
        }
    } 
    else
    {
        for(std::vector<float>::const_iterator it = scan->ranges.begin();
            it != scan->ranges.end();
            ++it)
        {
            readings.push_back(*it);
        }
    }

    // create localized range scan
    karto::LocalizedRangeScan* range_scan = new karto::LocalizedRangeScan(laser->GetName(), readings);
    range_scan->SetOdometricPose(karto_pose);
    range_scan->SetCorrectedPose(karto_pose);

    // Add the localized range scan to the mapper
    bool processed;
    if((processed = mapper_->Process(range_scan)))
    {
        //std::cout << "Pose: " << range_scan->GetOdometricPose() << " Corrected Pose: " << range_scan->GetCorrectedPose() << std::endl;

        karto::Pose2 corrected_pose = range_scan->GetCorrectedPose();

        // Compute the map->odom transform
        //tf2::Stamped<tf2::Transform> input (tf2::Transform(tf2::Quaternion::setRPY(0.0, 0.0, corrected_pose.GetHeading()),
        //                                    tf2::Vector3(corrected_pose.GetX(), corrected_pose.GetY(), 0.0)).inverse(), 
        //                                    tf2_ros::fromMsg(scan->header.stamp), base_frame_);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, corrected_pose.GetHeading());
        tf2::Transform tmp_tf(q, tf2::Vector3(corrected_pose.GetX(),
                                              corrected_pose.GetY(),
                                              0.0));
        tf2::Stamped<tf2::Transform> input (tmp_tf.inverse(), tf2_ros::fromMsg(scan->header.stamp), base_frame_);
        tf2::Stamped<tf2::Transform> odom_to_map;

        try
        {
            geometry_msgs::msg::TransformStamped odom_to_map_msg;
            tf2_buffer_.transform( tf2::toMsg<tf2::Stamped<tf2::Transform>, geometry_msgs::msg::TransformStamped>(input),
                                   odom_to_map_msg, odom_frame_);
            tf2::fromMsg(odom_to_map_msg, odom_to_map);   
        }
        catch(tf2::TransformException e)
        {
            ROS_ERROR("Transform from base_link to odom failed\n");
            odom_to_map.setIdentity();
        }
        
        map_to_odom_mutex_.lock();
        map_to_odom_ = tf2::Transform(tf2::Quaternion( odom_to_map.getRotation() ),
                                      tf2::Vector3(    odom_to_map.getOrigin() ) ).inverse();
        map_to_odom_mutex_.unlock();

        // Add the localized range scan to the dataset (for memory management)
        dataset_->Add(range_scan);
    }
    else
        delete range_scan;

    return processed;
}

bool 
SlamKarto::mapCallback(const nav_msgs::srv::GetMap::Request::SharedPtr req,
                             nav_msgs::srv::GetMap::Response::SharedPtr res)

{
    std::unique_lock<std::mutex> lock(map_mutex_);
    if(got_map_ && map_.map.info.width && map_.map.info.height)
    {
        *res = map_;
        return true;
    }
    else
        return false;
}

int
main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("slam_karto");
    std::shared_ptr<SlamKarto> node_ptr;
    node_ptr.reset(new SlamKarto(node));
    
    rclcpp::spin(node);
    //rclcpp::shutdown();
    node_ptr.reset();
    
    return 0;
}
