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
 *   * Redistributions in binary form must reproduce the above/* Software License Agreement (BSD License)
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

#ifndef CW3_H_
#define CW3_H_

#include <ros/ros.h>

#include <iostream>
#include <vector>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <Eigen/Core>
#include <iostream>
#include <thread>
#include <boost/thread/thread.hpp>

// HW1 Includes
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <tf2/convert.h>
#include <tf/transform_datatypes.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

// cw3 Includes
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// PCL includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/time.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace Eigen;
using namespace geometry_msgs;

/** \brief CW 3.
  *
  * \author Dimitrios Kanoulas
  */
class CW3
{
  public:
    /** \brief Empty constructor.
      *
      * \input[in] nh the ROS node
      */
    CW3 (ros::NodeHandle &nh);
    
    /** \brief Lab 1: initialize the parameters. */
    void
    initParams ();
    
    /** \brief Load the parameters. */
    void
    updateParams (ros::NodeHandle &nh);
    
    /** \brief Lab 1: get the world frame.
      *
      * \return the world frame string
      */
    std::string
    getWorldFrame ();
    
    /** \brief Lab 1: get the robot frame.
      *
      * \return the robot frame string
      */
    std::string
    getRobotFrame ();
    
    /** \brief Lab 1: create the transformation between robot & world
     *  frames. 
     */
    void
    Lab1CreateFrames ();
    
    /** \brief Lab 1: publish the transformations between robot & world
      * frames.
      */
    void
    Lab1PublishFrames ();
    
    /** \brief CW1: helper function to implement a non-blocking key getter.
      *
      * \return 1 for key pressed, 0 for no pressed
      */
    int
    kbhit ();
    
    /** \brief CW1 Q2: makes collision object box
      *
      * \input id name of identifier
      * \input frame name of frame_id
      * \input dim_x box dimensions along x
      * \input dim_y box dimensions along y
      * \input dim_z box dimensions along z
      * \input pos_x centre of box along x
      * \input pos_y centre of box along y
      * \input pos_z centre of box along z
      */
    moveit_msgs::CollisionObject
    cw1Q3MakeBox(std::string id, std::string frame_id,
                        float dim_x, float dim_y, float dim_z,
                        float pos_x, float pos_y, float pos_z);

    /** \brief CW1 Q2: add collision objects: table1, table2, table3, object
      *
      * \input planning_scene_interface the MoveIt! PlanningSceneInterface
      */
    void
    cw1Q3AddColObj (moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);

    geometry_msgs::PoseStamped
    trans2Pose(geometry_msgs::TransformStamped &transStamped);
    
    /** \brief CW3 Q1: active or deactive filter*/
    void
    cw3Q1Filter();

    /** \brief CW3 Q1: Activate or deactivate filter*/
    void 
    cw3Q1FastFilter();

    /** \brief CW3 Q1: Get cylinder and box-object pose and publish them*/
    void
    cylinderPosePublish(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface);
    
    /** \brief CW3 Q2: Publish bumper signal*/
    void
    cw3Q2BumperPub (moveit::planning_interface::MoveGroupInterface& move_group);

    /** \brief Open gripper*/
    void 
    openGripper(trajectory_msgs::JointTrajectory& posture);

    /** \brief Close gripper*/
    void 
    closedGripper(trajectory_msgs::JointTrajectory& posture);

    /** \brief Helper function to setup the grasp posture*/
    void
    setupGraspLocation(double grasp_RPY[], double grasp_pose[], double pre_grasp[], double post_grasp[]);
    
    /** \brief Helper function to setup the place posture*/
    void
    setupPlaceLocation(double place_RPY[], double place_pose[], double pre_place[], double post_place[]);
    
    /** \brief CW3 Q2: Grasp cylinder and place cylinder object on table 1*/
    void
    cw3Q2Grasp1(moveit::planning_interface::MoveGroupInterface& move_group);
    
    /** \brief CW3 Q3: Grasp box object and place it on table (given number) */
    void
    cw3Q3GraspObject(moveit::planning_interface::MoveGroupInterface& move_group, int table);

    /** \brief CW3 Q3: Grasp cylinder and place cylinder object on table 2*/
    void
    cw3Q3Grasp2(moveit::planning_interface::MoveGroupInterface& move_group);

    /** \brief CW3 Q3: Grasp cylinder and place cylinder object on table 3*/
    void
    cw3Q3Grasp3(moveit::planning_interface::MoveGroupInterface& move_group);

    /** 
     * \brief Check if the specified table is available, if not, move the 
     * box object to the empty table
    */
    void
    checkAvailable(moveit::planning_interface::MoveGroupInterface& move_group, int table);
    
    /** \brief Calculate distance between two points*/
    float
    distanceBetweenPoints(geometry_msgs::Point &p1, geometry_msgs::Point &p2);


  public:
    /** \brief Node handle. */
    ros::NodeHandle nh_;
    
    /** \brief World and robot frames. */
    std::string world_frame_, robot_frame_;
    
    /** \brief Lab 1: TF transforms definition. */
    tf::Transform transf_;
    
    /** \brief Lab 1: TF transform broadcaster definitions. */
    tf::TransformBroadcaster tranf_br_; 

    /** \brief Q1-1 Publisher to activate/deactivate filter. */
    ros::Publisher active_ = nh_.advertise<std_msgs::Bool> ("/activateFilter", 1, true);
    /** \brief Q1-2 Publisher to activate/deactivate fast filter. */
    ros::Publisher activeFastF_ = nh_.advertise<std_msgs::Bool> ("/activateFastFilter", 1, true);
    /** \brief Q1-2 Publish the rough area to the cylinder_segment. */
    ros::Publisher areaPos_ = nh_.advertise<geometry_msgs::Vector3> ("/areaPos", 1, true);

    /** \brief Q1-3 Publisher for cylinder-object pose. */
    ros::Publisher cylinerPose_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("/cylinder_pose", 1, true);
    /** \brief Q2-1 Publisher for box-object pose. */
    ros::Publisher object_pub_ = nh_.advertise<geometry_msgs::PoseStamped> ("object_pose", 1, true);
    
    /** \brief Q2-1 Publisher for bumper on each table. */
    ros::Publisher Touch_1_pub_ = nh_.advertise <std_msgs::Bool> ("/bumper1", 1, true);
    ros::Publisher Touch_2_pub_ = nh_.advertise <std_msgs::Bool> ("/bumper2", 1, true);
    ros::Publisher Touch_3_pub_ = nh_.advertise <std_msgs::Bool> ("/bumper3", 1, true);
    
    /** \brief Location of bumper*/
    geometry_msgs::Point bumper1, bumper2, bumper3;
    /** \brief Pose of cyliner and box-object*/
    geometry_msgs::Pose cyliner_pose_, object_pose_;

    /** \brief Posture for graps and place*/
    std::vector<moveit_msgs::Grasp> grapsPosture_;
    std::vector<moveit_msgs::PlaceLocation> placePosture_;
    
    /** \brief Filter acitvation status*/
    bool activateFilter_, activateFastFilter_;
    /** \brief Bumper activation status*/
    bool bump1_, bump2_, bump3_;


 
  protected:
    /** \brief Debug mode. */
    bool debug_;
};
#endif
