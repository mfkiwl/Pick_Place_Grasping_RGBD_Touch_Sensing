/* Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021-, Dimitrios Kanoulas
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <cw3.h>

CW3::CW3 (ros::NodeHandle &nh):
  debug_ (false)
{
  nh_ = nh;
  initParams ();
  updateParams (nh);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::initParams ()
{
  // Frames
  this->world_frame_ = "/world_frame";
  
  // Topics
  this->robot_frame_ = "/robot_frame";
  

  // CW3-Q1 Filter
  // Activate filterEnvironment and fastFilterEnvironment
  this->activateFilter_ = false;
  this->activateFastFilter_ = false;

  // CW3-Q2 bumper location
  // Define the bumper 1 on the table 1
  this->bumper1.y = 0.5;
  this->bumper1.z = 0.4;

  // Define the bumper 2 on the table 2
  this->bumper2.x = -0.5;
  this->bumper2.z = 0.4;

  // Define the bumper 3 on the table 3
  this->bumper3.y = -0.5;
  this->bumper3.z = 0.4;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::updateParams (ros::NodeHandle &nh)
{
  // Frames
  nh.getParam("/frame_params/world_frame", this->world_frame_);
  nh.getParam("/frame_params/robot_frame", this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getWorldFrame ()
{
  return (this->world_frame_);
}

////////////////////////////////////////////////////////////////////////////////
std::string
CW3::getRobotFrame ()
{
  return (this->robot_frame_);
}

////////////////////////////////////////////////////////////////////////////////
int
CW3::kbhit()
{
  struct termios oldt, newt;
  int ch;
  int oldf;
  
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  
  ch = getchar();
  
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1CreateFrames ()
{
  // generate a robot frame attached to the world frame (0,0,0)
  transf_.setOrigin (tf::Vector3(0.0, 0.0, 0.0));
  transf_.setRotation (tf::Quaternion(0.0, 0.0, 0.0, 1.0)); //quaternion
  
  // Note that the rotation can be given in various forms (rotation matrix,
  // axis-angle, etc), but the function setRotation gets a tf::Quaternion,
  // thus when a different type rotation is available, it should be converted
  // to a tf::Quaternion.
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::Lab1PublishFrames ()
{
  // publish world->robot
  tranf_br_.sendTransform(tf::StampedTransform(transf_,
                                               ros::Time::now(), 
                                               world_frame_,
                                               robot_frame_));
}

////////////////////////////////////////////////////////////////////////////////
moveit_msgs::CollisionObject
CW3::cw1Q3MakeBox(std::string id, std::string frame_id,
					float dim_x, float dim_y, float dim_z,
					float pos_x, float pos_y, float pos_z)
{
  // Makes a Box collision object at given location with given dimensions. 

  moveit_msgs::CollisionObject collision_object;
  
  // Add the first table where the cube will originally be kept.
  collision_object.id = id;
  collision_object.header.frame_id = frame_id;
  
  /* Define the primitive and its dimensions. */
  collision_object.primitives.resize(1);
  collision_object.primitives[0].type = collision_object.primitives[0].BOX;
  collision_object.primitives[0].dimensions.resize(3);
  collision_object.primitives[0].dimensions[0] = dim_x;
  collision_object.primitives[0].dimensions[1] = dim_y;
  collision_object.primitives[0].dimensions[2] = dim_z;

  /* Define the pose of the table: center of the cube. */
  collision_object.primitive_poses.resize(1);
  collision_object.primitive_poses[0].position.x =  pos_x;
  collision_object.primitive_poses[0].position.y =  pos_y;
  collision_object.primitive_poses[0].position.z =  pos_z;

  collision_object.operation = collision_object.ADD;
  return collision_object;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& 
                       planning_scene_interface)
{
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 4 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);
  
  // Add the first table where the cube will originally be kept.
  collision_objects[0] = CW3::cw1Q3MakeBox("table1", "panda_link0",
                                            0.4, 0.2, 0.4,
                                            0.0, 0.5, 0.2);
  
  // Add the second table where we will be placing the cube.
  collision_objects[1] = CW3::cw1Q3MakeBox("table2", "panda_link0",
                                            0.2, 0.4, 0.4,
					                                 -0.5, 0.0, 0.2);
  
  // Add the second table where we will be placing the cube.
  collision_objects[2] = CW3::cw1Q3MakeBox("table3", "panda_link0",
                                            0.4, 0.2, 0.4,
                                            0.0, -0.5, 0.2);

  // Define the object that we will be manipulating
  collision_objects[3] = CW3::cw1Q3MakeBox("object", "panda_link0",
                                            0.02, 0.02, 0.2,
                                            -0.5, 0.0, 0.5);

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

////////////////////////////////////////////////////////////////////////////////
geometry_msgs::PoseStamped
CW3::trans2Pose(geometry_msgs::TransformStamped &transStamped)
{
  // Define the computation for the transformation matrix (rotation and translation)
  geometry_msgs::PoseStamped poseStamped;
  geometry_msgs::Vector3 vec = transStamped.transform.translation;
  poseStamped.header = transStamped.header;
  poseStamped.pose.orientation = transStamped.transform.rotation;
  poseStamped.pose.position.x = vec.x; // from the orientation
  poseStamped.pose.position.y = vec.y; // from the orientation
  poseStamped.pose.position.z = vec.z; // from the orientation
  return poseStamped;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3Q1Filter()
{
  if( this->activateFilter_ == false)
  {
    // Run the activateFilter, which is Voxel_Grid filterEnvironment
    this->activateFilter_ = true;
  }
  else
  {
    this->activateFilter_ = false;
  }
  std_msgs::Bool msg;
  msg.data = this->activateFilter_;
  this->active_.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
void 
CW3::cw3Q1FastFilter()
{
  if (this->activateFastFilter_ == false)
  {
    // Run the activateFastFilter, which includes 
    // passTroughFilter and Voexl_Grid Environment
    this->activateFastFilter_ = true;
  }
  else
  {
    this->activateFastFilter_ = false;
  }
  std_msgs::Bool msg;
  msg.data = this->activateFastFilter_;
  this->activeFastF_.publish(msg);
  
}

////////////////////////////////////////////////////////////////////////////
void
CW3::cylinderPosePublish(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  std::map< std::string, geometry_msgs::Pose > obj_list, obj_list2 ;

  // Find the cylinder pose to update when it moves
  obj_list = planning_scene_interface.getObjectPoses({"cylinder"});
  geometry_msgs::Pose cylinder = obj_list.find("cylinder")->second;
  this->cyliner_pose_ = cylinder;
  // Find the box-object pose to update when it moves
  obj_list2 = planning_scene_interface.getObjectPoses({"object"});
  geometry_msgs::Pose object = obj_list2.find("object")->second;
  this->object_pose_ = object;

  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "panda_link0";
  msg.header.stamp = ros::Time::now();
  // Publish the updated version (current location) of the cylinder pose
  msg.pose = this->cyliner_pose_;
  this->cylinerPose_pub_.publish(msg);

  // Publish the updated version (current location) of the object pose
  msg.pose = this->object_pose_;
  this->object_pub_.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////
void 
CW3::openGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

////////////////////////////////////////////////////////////////////////////////
void 
CW3::closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::setupGraspLocation(double grasp_RPY[], double grasp_pose[], double pre_grasp[], double post_grasp[])
{
  ROS_INFO("Setting up pickup posture");
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);
  tf2::Quaternion orientation;
  grasps[0].grasp_pose.header.frame_id = "panda_link0";
  orientation.setRPY(grasp_RPY[0], grasp_RPY[1], grasp_RPY[2]);
  grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
  grasps[0].grasp_pose.pose.position.x = grasp_pose[0];
  grasps[0].grasp_pose.pose.position.y = grasp_pose[1];
  grasps[0].grasp_pose.pose.position.z = grasp_pose[2];

  // Pre-grasp
  grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
  grasps[0].pre_grasp_approach.direction.vector.x = pre_grasp[0];
  grasps[0].pre_grasp_approach.min_distance = pre_grasp[1];
  grasps[0].pre_grasp_approach.desired_distance = pre_grasp[2];

  // Pos-grasp
  grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
  grasps[0].post_grasp_retreat.direction.vector.z = post_grasp[0];
  grasps[0].post_grasp_retreat.min_distance = post_grasp[1];
  grasps[0].post_grasp_retreat.desired_distance = post_grasp[2];  

  this->grapsPosture_ = grasps;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::setupPlaceLocation(double place_RPY[], double place_pose[], double pre_place[], double post_place[])
{
  ROS_INFO("Setting up place location");
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  place_location[0].place_pose.header.frame_id = "panda_link0";
  tf2::Quaternion orientation;
  orientation.setRPY(place_RPY[0],place_RPY[1],place_RPY[2]);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = place_pose[0];
  place_location[0].place_pose.pose.position.y = place_pose[1];
  place_location[0].place_pose.pose.position.z = place_pose[2];

  place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = pre_place[0];
  place_location[0].pre_place_approach.min_distance = pre_place[1];
  place_location[0].pre_place_approach.desired_distance = pre_place[2];

  place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = post_place[0];
  place_location[0].post_place_retreat.min_distance = post_place[1];
  place_location[0].post_place_retreat.desired_distance = post_place[2];

  this->placePosture_ = place_location;
}

////////////////////////////////////////////////////////////////////////////////
float
CW3::distanceBetweenPoints(geometry_msgs::Point &p1, geometry_msgs::Point &p2)
{
  // Calculate the distance between the object (box-object or cylinder) and bumper
  // We want to know that on the table is empty or not
  float distance;
  distance = std::pow((p1.x-p2.x),2) + std::pow((p1.y-p2.y),2) + std::pow((p1.z-p2.z),2);
  distance = std::sqrt(distance);
  return distance;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3Q2BumperPub(moveit::planning_interface::MoveGroupInterface& move_group)
{ 
  // Define the distance between the object (box-object or cylinder) and each bumper
  float distance1, distance2, distance3, distanceC1, distanceC2, distanceC3, threshold;
  threshold = 0.2;

  // Find the distance between the box-object and each bumper
  distance1 = distanceBetweenPoints(this->bumper1, this->object_pose_.position);
  distance2 = distanceBetweenPoints(this->bumper2, this->object_pose_.position);
  distance3 = distanceBetweenPoints(this->bumper3, this->object_pose_.position);
  
  // Find the distance between the cylinder and each bumper
  distanceC1 = distanceBetweenPoints(this->bumper1, this->cyliner_pose_.position);
  distanceC2 = distanceBetweenPoints(this->bumper2, this->cyliner_pose_.position);
  distanceC3 = distanceBetweenPoints(this->bumper3, this->cyliner_pose_.position);
  
  // If the distance is less than the threshold, the bumper is updated that means the box-object/cylinder is on the table
  std_msgs::Bool touch;
  this->bump1_ = (distance1 < threshold || distanceC1 < threshold);
  this->bump2_ = (distance2 < threshold || distanceC2 < threshold);
  this->bump3_ = (distance3 < threshold || distanceC3 < threshold);

  // If you want to check the bumper is activated or not in the real-time, please uncomment the code below 
  // ROS_INFO_STREAM("bump1:"<<this->bump1_<<" bump2:"<<this->bump2_<<" bump3:"<<this->bump3_);
  
  // Publish the bumper 1 (table 1)
  touch.data = this->bump1_;
  Touch_1_pub_.publish(touch);
  // Publish the bumper 2 (table 2)
  touch.data = this->bump2_;
  Touch_2_pub_.publish(touch);
  // Publish the bumper 3 (table 3)
  touch.data = this->bump3_;
  Touch_3_pub_.publish(touch);
  return;
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3Q2Grasp1(moveit::planning_interface::MoveGroupInterface& move_group)
{
  //Setup values for graps and place posture
  geometry_msgs::Point object = this->cyliner_pose_.position;
  double grasp_RPY[3] = {M_PI, 0.0, 0.0};
  double grasp_pose[3] = {object.x, object.y, object.z + 0.14}; 
  double pre_grasp[3] = {1.0, 0.095, 0.115}; 
  double post_grasp[3] = {1.0, 0.1, 0.25};
  double place_RPY[3] = {M_PI, 0.0, -M_PI};
  double place_pose[3] = {0.0, 0.5, 0.46}; 
  double pre_place[3] = {-1.0, 0.095, 0.115}; 
  double post_place[3] = {-1.0, 0.1, 0.25};

  setupGraspLocation(grasp_RPY, grasp_pose, pre_grasp, post_grasp);
  setupPlaceLocation(place_RPY, place_pose, pre_place, post_place);

  //Perform pick and place
  openGripper(grapsPosture_[0].pre_grasp_posture);
  closedGripper(grapsPosture_[0].grasp_posture);
  move_group.pick("cylinder", grapsPosture_);

  ros::WallDuration(1.0).sleep();
  openGripper(placePosture_[0].post_place_posture);
  move_group.place("cylinder", placePosture_);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3Q3GraspObject(moveit::planning_interface::MoveGroupInterface& move_group, int table)
{
  //Setup values for graps and place posture
  geometry_msgs::Point object = this->object_pose_.position;
  double grasp_RPY[3] = {M_PI, 0.0, -M_PI/4};
  double grasp_pose[3] = {object.x, object.y, object.z + 0.2}; 
  double pre_grasp[3] = {1.0, 0.095, 0.115}; 
  double post_grasp[3] = {1.0, 0.1, 0.15};
  double pre_place[3] = {-1.0, 0.095, 0.115}; 
  double post_place[3] = {-1.0, 0.1, 0.25};

  if(table == 1)
  {
    double place_RPY[3] = {0.0, 0.0, M_PI/2};
    double place_pose[3] = {0.0, 0.5, 0.5}; 

    setupPlaceLocation(place_RPY, place_pose, pre_place, post_place);
  }
  else if(table == 2)
  {
    double place_RPY[3] = {0.0, 0.0, M_PI/2};
    double place_pose[3] = {-0.5, 0.0, 0.5}; 
    setupPlaceLocation(place_RPY, place_pose, pre_place, post_place);
  }
  else
  {
    double place_RPY[3] = {0.0, 0.0, M_PI/2};
    double place_pose[3] = {0.0, -0.5, 0.5}; 
    setupPlaceLocation(place_RPY, place_pose, pre_place, post_place);
  }

  setupGraspLocation(grasp_RPY, grasp_pose, pre_grasp, post_grasp);

  //Perform pick and place
  openGripper(grapsPosture_[0].pre_grasp_posture);
  closedGripper(grapsPosture_[0].grasp_posture);
  move_group.pick("object", grapsPosture_);

  ros::WallDuration(1.0).sleep();
  openGripper(placePosture_[0].post_place_posture);
  move_group.place("object", placePosture_);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3Q3Grasp2(moveit::planning_interface::MoveGroupInterface& move_group)
{
  //Setup values for graps and place posture
  geometry_msgs::Point object = this->cyliner_pose_.position;
  double grasp_RPY[3] = {M_PI, 0.0, 0.0};
  double grasp_pose[3] = {object.x, object.y, object.z + 0.14}; 
  double pre_grasp[3] = {1.0, 0.095, 0.115}; 
  double post_grasp[3] = {1.0, 0.1, 0.25};
  double place_RPY[3] = {M_PI, 0.0, -M_PI};
  double place_pose[3] = {-0.5, 0.0, 0.46}; 
  double pre_place[3] = {-1.0, 0.095, 0.115}; 
  double post_place[3] = {-1.0, 0.1, 0.25};

  setupGraspLocation(grasp_RPY, grasp_pose, pre_grasp, post_grasp);
  setupPlaceLocation(place_RPY, place_pose, pre_place, post_place);

  //Perform pick and place
  openGripper(grapsPosture_[0].pre_grasp_posture);
  closedGripper(grapsPosture_[0].grasp_posture);
  move_group.pick("cylinder", grapsPosture_);

  ros::WallDuration(1.0).sleep();
  openGripper(placePosture_[0].post_place_posture);
  move_group.place("cylinder", placePosture_);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::cw3Q3Grasp3(moveit::planning_interface::MoveGroupInterface& move_group)
{
  //Setup values for graps and place posture
  geometry_msgs::Point object = this->cyliner_pose_.position;
  double grasp_RPY[3] = {M_PI, 0.0, 0.0};
  double grasp_pose[3] = {object.x, object.y, object.z + 0.14}; 
  double pre_grasp[3] = {1.0, 0.095, 0.115}; 
  double post_grasp[3] = {1.0, 0.1, 0.25};
  double place_RPY[3] = {M_PI, 0.0, -M_PI/2};
  double place_pose[3] = {0.0, -0.5, 0.46}; 
  double pre_place[3] = {-1.0, 0.095, 0.115}; 
  double post_place[3] = {-1.0, 0.1, 0.25};

  setupGraspLocation(grasp_RPY, grasp_pose, pre_grasp, post_grasp);
  setupPlaceLocation(place_RPY, place_pose, pre_place, post_place);

  //Perform pick and place
  openGripper(grapsPosture_[0].pre_grasp_posture);
  closedGripper(grapsPosture_[0].grasp_posture);
  move_group.pick("cylinder", grapsPosture_);

  ros::WallDuration(1.0).sleep();
  openGripper(placePosture_[0].post_place_posture);
  move_group.place("cylinder", placePosture_);
}

////////////////////////////////////////////////////////////////////////////////
void
CW3::checkAvailable(moveit::planning_interface::MoveGroupInterface& move_group, int table)
{
  switch(table)
  {
    case 1:
      //If the specified table is not empty, check which table is 
      //empty and place the box-object from the specified table to
      //empty table
      if(this->bump1_)
      {
        if(this->bump2_)
        {
          cw3Q3GraspObject(move_group, 3);
        }
        else
        {
          cw3Q3GraspObject(move_group, 2);
        }
      }
      break;
    case 2:
      if(this->bump2_)
      {
        if(this->bump1_)
        {
          cw3Q3GraspObject(move_group, 3);
        }
        else
        {
          cw3Q3GraspObject(move_group, 1);
        }
      }
      break;
    case 3:
      if(this->bump3_)
      {
        if(this->bump2_)
        {
          cw3Q3GraspObject(move_group, 1);
        }
        else
        {
          cw3Q3GraspObject(move_group, 2);
        }
      }
      break;
    default:
      break;
  }
}
