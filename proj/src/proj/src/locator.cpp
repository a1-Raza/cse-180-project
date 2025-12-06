#include <rclcpp/rclcpp.hpp> 
#include <proj/navigation.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/impl/utils.h>
#include <cmath>
#include <unordered_map>

geometry_msgs::msg::Pose::SharedPtr setPose(float x, float y, float yaw_rad);

class Locator : public Navigator {
public:
  Locator(bool debug, bool verbose) : Navigator(debug, verbose) {
    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", map_qos, std::bind(&Locator::map_callback, this, std::placeholders::_1));
    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 10, std::bind(&Locator::pose_callback, this, std::placeholders::_1));
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Locator::scan_callback, this, std::placeholders::_1));

    map_received_ = false;
    pose_received_ = false;
    should_scan = false;
    
    // hardcoding expected locations of humans (since this stays constant)
    h1.x = 1.0; h1.y = -1.0;
    h1_found = false;
    h2.x = -12.0; h2.y = 15.0;
    h2_found = false; // for testing, assume already found
  }

  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    map_ = *msg;
    map_received_ = true;
  }

  void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    current_pose_ = msg->pose.pose;
    pose_received_ = true;
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!should_scan) return;

    if (!map_received_ || !pose_received_) {
      if (!map_received_) RCLCPP_WARN(this->get_logger(), "no map :((((");
      if (!pose_received_) RCLCPP_WARN(this->get_logger(), "no pose :((((");
      should_scan = false;
      return;
    }

    geometry_msgs::msg::Pose currPose = current_pose_;
    sensor_msgs::msg::LaserScan laserScan = *msg;
    tf2::Quaternion q(currPose.orientation.x, currPose.orientation.y, currPose.orientation.z, currPose.orientation.w);
    float robot_yaw = tf2::impl::getYaw(q); // may want to convert this from [-pi,pi] to [0, 2pi]

    //RCLCPP_WARN(this->get_logger(), "scanning :)))) (min range: %.2f, max range: %.2f)", laserScan.range_min, laserScan.range_max);

    
    // iterate through laser scan
    bool found_something_new = false;
    std::map<std::pair<int, int>, int> temp_found_hash; // temp hash for this scan only
    for (size_t i = 0; i < laserScan.ranges.size(); i ++) { // Check every ray
      float range = laserScan.ranges[i];
      float angle = laserScan.angle_min + i * laserScan.angle_increment + M_PI/2; // ADJUST FOR SENSOR 90 DEG OFFSET FROM ROBOT FRONT
      
      // checking scan validity
      if (!(range > laserScan.range_min && range < RANGE_THRESH)) {continue;}

      // local coords of laser hit
      float local_hit_x = range * cos(angle);
      float local_hit_y = range * sin(angle);
      // local to map coords
      float hit_x = (currPose.position.x + local_hit_x * cos(robot_yaw) - local_hit_y * sin(robot_yaw));
      float hit_y = (currPose.position.y + local_hit_x * sin(robot_yaw) + local_hit_y * cos(robot_yaw));
      // rounded map coords
      int round_hit_x = hit_x >= 0.0 ? (int)(hit_x + 0.5) : (int)(hit_x - 0.5);
      int round_hit_y =  hit_y >= 0.0 ? (int)(hit_y + 0.5) : (int)(hit_y - 0.5);
      // hit's corresponding map values
      //int map_val_round = get_map_value(round_hit_x, round_hit_y);
      //int map_val = get_map_value(hit_x, hit_y);

      // checking if scan hits human at an expected location
      if (looking_for_expected) {
        float hit1DistToExpected = std::hypot(hit_x - h1.x, hit_y - h1.y);
        if (!h1_found && hit1DistToExpected < HIT_THRESH) {
        //if (!h1_found && hit1DistToExpected < HIT_THRESH && map_val_round == 100) {
        //if (round_hit_x == h1.x && round_hit_y == h1.y && !h1_found) {
          h1_hits++;
          if (h1_hits >= EXPECTED_HIT_THRESH) { 
            h1_found = true; 
            RCLCPP_WARN(this->get_logger(), "\tFound P1 at EXPECTED location: (%d, %d)", round_hit_x, round_hit_y);
          }
        }
        else {
          float hit2DistToExpected = std::hypot(hit_x - h2.x, hit_y - h2.y);
          if (!h2_found && hit2DistToExpected < HIT_THRESH) {
          //if (!h2_found && hit2DistToExpected < HIT_THRESH && map_val_round == 100) {
          //if (round_hit_x == h2.x && round_hit_y == h2.y && !h2_found) {
            h2_hits++;
            if (h2_hits >= EXPECTED_HIT_THRESH) {
              h2_found = true;
              RCLCPP_WARN(this->get_logger(), "\tFound P2 at EXPECTED location: (%d, %d)", round_hit_x, round_hit_y);
            }
          }
        }
      }
      else {
        //int map_val_range_round = get_map_value_by_range(round_hit_x, round_hit_y);
        int map_val_range = get_map_value_by_range(hit_x, hit_y);
        if (map_val_range == 0) {
          found_something_new = true;
          //RCLCPP_WARN(this->get_logger(), "\tNEW location; angle %zu: %.2f, range: %.2f, rounded pos: (%d, %d), map val range: %d", i, angle*180.0/M_PI, range, round_hit_x, round_hit_y, map_val_range);
          
          // // add point to hash
          // if (laser_at_free_hash.find(std::make_pair(round_hit_x, round_hit_y)) == laser_at_free_hash.end()) laser_at_free_hash[std::make_pair(round_hit_x, round_hit_y)] = 1;
          // else laser_at_free_hash[std::make_pair(round_hit_x, round_hit_y)] += 1;

          if (temp_found_hash.find(std::make_pair(round_hit_x, round_hit_y)) == temp_found_hash.end()) temp_found_hash[std::make_pair(round_hit_x, round_hit_y)] = 1;
          else {
            if (temp_found_hash[std::make_pair(round_hit_x, round_hit_y)] < MAX_HIT_PER_SCAN) temp_found_hash[std::make_pair(round_hit_x, round_hit_y)] += 1;
          }
        }
      }
    }

    if (found_something_new) {
      // check temp hash and update main hash
      RCLCPP_WARN(this->get_logger(), "Potential location(s) scanned:");
      for (const auto& entry : temp_found_hash) {
        RCLCPP_WARN(this->get_logger(), " - (%d, %d), hit %d times", entry.first.first, entry.first.second, entry.second);
        if (laser_at_free_hash.find(entry.first) == laser_at_free_hash.end()) laser_at_free_hash[entry.first] = entry.second;
        else laser_at_free_hash[entry.first] += entry.second;
      }
    }

    // update main hash based on temp hash
    for (auto it = laser_at_free_hash.begin(); it != laser_at_free_hash.end(); /* No ++it here */) {
      // if found enough times, confirm new location
      if (it->second >= HITS_TO_BE_NEW_LOC && confirmed_locations.find(it->first) == confirmed_locations.end()) {
        confirmed_locations.insert(it->first);
        RCLCPP_WARN(this->get_logger(), ">>> CONFIRMED NEW LOCATION at (%d, %d) <<<", it->first.first, it->first.second);
        
      }
      // if not found in this scan
      if (temp_found_hash.find(it->first) == temp_found_hash.end()) {
          // if not yet confirmed location
          if (it->second < HITS_TO_BE_NEW_LOC) it->second -= NOT_HIT_PENALTY; // retain officially detected locations
          if (it->second <= 0) it = laser_at_free_hash.erase(it);
          else ++it;
      } 
      else ++it;
    }

    should_scan = false; // Only scan once per command
  }

  // checks only the map cell corresponding to (x,y)
  int get_map_value(float x, float y) {
    if (!map_received_) return -2;
    float res = map_.info.resolution;
    int grid_x = (int)(((x - map_.info.origin.position.x) / res)+0.5);
    int grid_y = (int)(((y - map_.info.origin.position.y) / res)+0.5);

    if (grid_x < 0 || grid_x >= (int)map_.info.width || grid_y < 0 || grid_y >= (int)map_.info.height) return -1;
      
    return map_.data[grid_y * map_.info.width + grid_x];
  }

  // checkes a range around the map cell corresponding to (x,y)
  int get_map_value_by_range(float x, float y) {
    if (!map_received_) return -2;
    float res = map_.info.resolution;
    int grid_x = (int)(((x - map_.info.origin.position.x) / res)+0.5);
    int grid_y = (int)(((y - map_.info.origin.position.y) / res)+0.5);

    // threshold parameters
    const int threshold = 50;
    const int max_count = 3;
    const int max_negative_count = 10;

    int count_occupied = 0;
    int count_negative = 0;
    for (int dx = -threshold/2; dx <= threshold/2; dx++) {
      for (int dy = -threshold/2; dy <= threshold/2; dy++) {
        int nx = grid_x + dx;
        int ny = grid_y + dy;
        if (nx < 0 || nx >= (int)map_.info.width || ny < 0 || ny >= (int)map_.info.height) continue;

        int val = map_.data[ny * map_.info.width + nx];
        if (val != 0 && val != -1) count_occupied++;
        else if (val < 0) {
          count_negative++;
          if (count_negative >= max_negative_count) return -1; // unknown
        }
        if (count_occupied >= max_count) {
          return 100; // occupied
        }
      }
    }
    return 0;
  }

  void InspectRoom() {
    if ((int)confirmed_locations.size() >= NumPeopleNotAtExpected()) CancelTask(); // terminate if both expected locations found
    for (int i = 0; i < 1; i++) { // check N times
        should_scan = true; // will become false after one scan
        while (rclcpp::ok() && should_scan) {
          Idle(1.0); // allow time for callbacks to process
      }
    }
  }

  void Idle(float seconds) {
    // RCLCPP_WARN(this->get_logger(), "Idling for %.1f seconds to let sensors settle...", seconds);
    
    // We use the ROS clock (handles simulation time correctly)
    auto start_time = this->now();
    rclcpp::Rate rate(10); // Check 100 times per second

    while (rclcpp::ok() && (this->now() - start_time).seconds() < seconds) {
        // CRITICAL: process callbacks (clear the laser buffer)
        rclcpp::spin_some(this->get_node_base_interface());
        rate.sleep();
    }

    // RCLCPP_WARN(this->get_logger(), "Done idling.");
  }

  void PrintScanHash() {
    RCLCPP_WARN(this->get_logger(), "CURRENT TOTAL SCAN COUNTS:");
    for (const auto& entry : laser_at_free_hash) {
      RCLCPP_WARN(this->get_logger(), " - (%d, %d): %d times", entry.first.first, entry.first.second, entry.second);
    }
  }

  void CheckExpectedLocations() {
    printf("Starting num ppl not seen at expected: %d\n", NumPeopleNotAtExpected());
    geometry_msgs::msg::Pose::SharedPtr goal = setPose(2, 0, 90*M_PI/180.0);
    GoToPose(goal);
    while ( ! IsTaskComplete() ) {}
    InspectRoom();
    Backup(0.1, 0.033);
    while ( ! IsTaskComplete() ) { InspectRoom(); }

    if (NumPeopleNotAtExpected() == 0) {
      RCLCPP_WARN(this->get_logger(), "Both people found at expected locations! Ending search.");
      return;
    }

    printf("Num ppl not seen at expected after first check: %d\n", NumPeopleNotAtExpected());

    goal = setPose(-13, 13, 90*M_PI/180.0);
    GoToPose(goal);
    while ( ! IsTaskComplete() ) {}
    InspectRoom(); 
    Backup(0.1, 0.033);
    while ( ! IsTaskComplete() ) { InspectRoom(); }
    if (NumPeopleNotAtExpected() == 0) {
      RCLCPP_WARN(this->get_logger(), "Both people found at expected locations! Ending search.");
      return;

      printf("Num ppl not seen at expected after second check: %d\n", NumPeopleNotAtExpected());
    }
  }

  int NumPeopleNotAtExpected() {
    return 2 - ((int)h1_found + (int)h2_found);
  }

  void setSearchingForExpected(bool val) {
    looking_for_expected = val;
  }

  std::set<std::pair<int, int>> GetConfirmedLocations() {
    return confirmed_locations;
  }

  void PrintFoundExpectedLocations() {
    if (NumPeopleNotAtExpected() == 2) {
      RCLCPP_WARN(this->get_logger(), "NOBODY FOUND AT EXPECTED LOCATIONS.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "FOUND EXPECTED LOCATIONS:");
    if (h1_found) {
      RCLCPP_WARN(this->get_logger(), " - H1: (%d, %d)", (int)h1.x, (int)h1.y);
    }
    if (h2_found) {
      RCLCPP_WARN(this->get_logger(), " - H2: (%d, %d)", (int)h2.x, (int)h2.y);
    }
  }

  void PrintConfirmedLocations() {
    if (confirmed_locations.size() == 0) {
      RCLCPP_WARN(this->get_logger(), "NO NEW LOCATIONS DETERMINED.");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "CONFIRMED NEW LOCATIONS:");
    for (const auto& loc : confirmed_locations) {
      RCLCPP_WARN(this->get_logger(), " - (%d, %d)", loc.first, loc.second);
    }
  }

private:
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  bool map_received_;
  bool pose_received_;
  bool should_scan;
  bool looking_for_expected = false;
  nav_msgs::msg::OccupancyGrid map_;
  geometry_msgs::msg::Pose current_pose_;

  const float HIT_THRESH = 0.5; // meters
  const float RANGE_THRESH = 5; // meters
  const int MAX_HIT_PER_SCAN = 7; // max hits to consider on a single position per scan

  // new location detection
  std::map<std::pair<int, int>, int> laser_at_free_hash;
  const int HITS_TO_BE_NEW_LOC = 20; // in hash
  const int NOT_HIT_PENALTY = 2; // per scan
  std::set<std::pair<int, int>> confirmed_locations;

  // expected location detection
  geometry_msgs::msg::Point h1;
  int h1_hits = 0;
  bool h1_found;
  geometry_msgs::msg::Point h2;
  int h2_hits = 0;
  bool h2_found;
  const int EXPECTED_HIT_THRESH = 3; // times to detect at expected location

};


geometry_msgs::msg::Pose::SharedPtr setPose(float x, float y, float yaw_rad) {
  geometry_msgs::msg::Pose::SharedPtr pose = std::make_shared<geometry_msgs::msg::Pose>();
  pose->position.x = x;
  pose->position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw_rad);
  pose->orientation.x = q.x();
  pose->orientation.y = q.y();
  pose->orientation.z = q.z();
  pose->orientation.w = q.w();
  return pose;
}


int main(int argc, char ** argv)
{
  ////////// STARTUP //////////

  rclcpp::init(argc,argv);
  Locator locator(true,false); // create node with debug info but not verbose
  geometry_msgs::msg::Pose::SharedPtr init = setPose(2.12, -21.3, M_PI/2);
  locator.SetInitialPose(init);
  locator.WaitUntilNav2Active();

  ////////// MAIN OPERATIONS //////////

  // warmup spin
  locator.Spin(M_PI/2);
  while ( ! locator.IsTaskComplete() ) {}
  locator.Spin(-M_PI/2);
  while ( ! locator.IsTaskComplete() ) {}

  // check expected locations (set mode as well)
  locator.setSearchingForExpected(true);
  locator.CheckExpectedLocations();

  // if both found, return to starting point and end run
  if (locator.NumPeopleNotAtExpected() == 0) {
    locator.GoToPose(init);
    while ( ! locator.IsTaskComplete() ) {}
    rclcpp::shutdown();
    return 0;
  }

  // otherwise, search the environment
  locator.setSearchingForExpected(false);
  std::vector<geometry_msgs::msg::Pose> poses = { // all poses we want to visit
    *setPose(-13,24,90*M_PI/180.0),
    *setPose(-8,24,0*M_PI/180.0),
    *setPose(-8,21,270*M_PI/180.0),
    *setPose(-5,21,0*M_PI/180.0),
    *setPose(-5,24,90*M_PI/180.0),
    *setPose(-2,24,0*M_PI/180.0),
    *setPose(0,22,0*M_PI/180.0),
    *setPose(10,22,0*M_PI/180.0),
    *setPose(10,14,270*M_PI/180.0),
    *setPose(-8,13,180*M_PI/180.0),
    *setPose(-8,6,90*M_PI/180.0),
    *setPose(14,6,0*M_PI/180.0),
    *setPose(6,6,180*M_PI/180.0),
    *setPose(6,1,270*M_PI/180.0),
    *setPose(11,1,0*M_PI/180.0),
    *setPose(11,-12,270*M_PI/180.0),
    *setPose(9,-12,180*M_PI/180.0),
    *setPose(11,-18,270*M_PI/180.0),
    *setPose(11,-23.5,0*M_PI/180.0),
    *setPose(2,-23.5,180*M_PI/180.0),
    *setPose(2,-4,90*M_PI/180.0),
    *setPose(5,-2,90*M_PI/180.0),
    *setPose(0,2,180*M_PI/180.0),
    *setPose(-6,-1,270*M_PI/180.0),
    *setPose(-6,-23.5,270*M_PI/180.0),
    *setPose(-13,-23.5,180*M_PI/180.0),
    *setPose(-13,0,90*M_PI/180.0),
    *setPose(-10,2,0*M_PI/180.0)
  };

  // visit all poses until all ppl found or poses exhausted
  for ( auto pose : poses ) {
    locator.GoToPose(std::make_shared<geometry_msgs::msg::Pose>(pose));
    while ( ! locator.IsTaskComplete() ) {locator.InspectRoom(); }
    locator.PrintScanHash();
    if ((int)locator.GetConfirmedLocations().size() >= locator.NumPeopleNotAtExpected()) {
      printf("Found all people\n");
      locator.CancelTask();
      locator.PrintFoundExpectedLocations();
      locator.PrintConfirmedLocations();
      break;
    }
  }

  ////////// RETURN AND SHUTDOWN //////////

  if ((int)locator.GetConfirmedLocations().size() < locator.NumPeopleNotAtExpected()) {
    printf("Did not find all people :(\n");
    locator.CancelTask();
    locator.PrintFoundExpectedLocations();
    locator.PrintConfirmedLocations();
  }

  locator.GoToPose(init);
  while ( ! locator.IsTaskComplete() ) {}

  rclcpp::shutdown();
  return 0;
}
