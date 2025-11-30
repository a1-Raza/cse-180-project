#include <rclcpp/rclcpp.hpp> 
#include <proj/navigation.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char ** argv)
{

  rclcpp::init(argc,argv); // initialize ROS 
  Navigator navigator(true,false); // create node with debug info but not verbose

  // first: it is mandatory to initialize the pose of the robot

  tf2::Quaternion q;
  q.setRPY(0,0,M_PI/2); // yaw of 90 degrees
  geometry_msgs::msg::Pose::SharedPtr init = std::make_shared<geometry_msgs::msg::Pose>();
  init->position.x = 2.12;
  init->position.y = -21.3;
  init->orientation.x = q.x();
  init->orientation.y = q.y();
  init->orientation.z = q.z();
  init->orientation.w = q.w();
  navigator.SetInitialPose(init);
  // wait for navigation stack to become operationale
  navigator.WaitUntilNav2Active();
  // spin in place of 90 degrees (default parameter)
  /*navigator.Spin();
  while ( ! navigator.IsTaskComplete() ) {
    // busy waiting for task to be completed
  }*/

  // move to new pose
  q.setRPY(0,0,0);
  geometry_msgs::msg::Pose::SharedPtr goal_pos = std::make_shared<geometry_msgs::msg::Pose>();
  goal_pos->position.x = 3.5;
  goal_pos->position.y = -18;
  goal_pos->orientation.x = q.x();
  goal_pos->orientation.y = q.y();
  goal_pos->orientation.z = q.z();
  goal_pos->orientation.w = q.w();
  navigator.GoToPose(goal_pos);
  while ( ! navigator.IsTaskComplete() ) {
    
  }

  // move back to original pose
  navigator.GoToPose(init);
  while ( ! navigator.IsTaskComplete() ) {
    
  }



  // complete here....
  rclcpp::shutdown(); // shutdown ROS
  return 0;

  /*(void) argc;
  (void) argv;

  printf("hello world proj package\n");
  return 0;*/
}
