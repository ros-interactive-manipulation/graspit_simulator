/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

#ifndef _ROS_GRASPIT_INTERFACE_H_
#define _ROS_GRASPIT_INTERFACE_H_

#include <map>

//GraspIt! includes
#include <include/plugin.h>
namespace db_planner
{
class DatabaseManager;
}
class GraspitDBModel;
class Pr2Gripper2010;
class Body;
class transf;
class GraspableBody;

#include <ros/ros.h>

#include <manipulation_msgs/Grasp.h>
#include <manipulation_msgs/GraspPlanning.h>

#include "graspit_ros_planning_msgs/LoadDatabaseModel.h"
#include "graspit_ros_planning_msgs/LoadObstacle.h"
#include "graspit_ros_planning_msgs/ClearBodies.h"
#include "graspit_ros_planning_msgs/SimulateScan.h"
#include "graspit_ros_planning_msgs/TestGrasp.h"
#include "graspit_ros_planning_msgs/GenerateGrasp.h"
#include "graspit_ros_planning_msgs/VerifyGrasp.h"

namespace graspit_ros_planning
{

//! Main class, combining a ROS node with a GraspIt! interface
/*! Note that this class inherits from GraspIt's Plugin class and implements the necessary functions to serve
  as a GraspIt plugin. See include/plugin.h in the GraspIt code for the base class.

  Provides a number of ROS services that directly operate on the GraspIt world, such as loading objects or
  obstacles, simulating 3D scans of objects, etc.

  In particular, note that this class uses the mainLoop() function to perform the ROS even management calls.
*/
class RosGraspitInterface : public Plugin
{
private:
  //! Node handle in the root namespace
  ros::NodeHandle *root_nh_;

  //! Node handle in the private namespace
  ros::NodeHandle *priv_nh_;

  //! A GraspIt database manager used for talking to the database
  /*! An interesting point is that here we also have to option of using a ROS
   interface to the database; pros and cons should be considered in the future. */
  db_planner::DatabaseManager *db_mgr_;

  //! Server for the load model service
  ros::ServiceServer load_model_srv_;

  //! Server for the load obstacle service
  ros::ServiceServer load_obstacle_srv_;

  //! Server for the clear bodies service
  ros::ServiceServer clear_bodies_srv_;

  //! Server for the scan simulation service
  ros::ServiceServer simulate_scan_srv_;

  //! Server for the test grasp service
  ros::ServiceServer test_grasp_srv_;

  //! Server for the grasp planning service
  ros::ServiceServer grasp_planning_srv_;

  //! Server for the generate grasp service
  ros::ServiceServer generate_grasp_srv_;

  //! Server for the grasp collision checking service
  ros::ServiceServer verify_grasp_srv_;

  //! Publisher for simulated scans
  ros::Publisher scan_publisher_;

  //! Keeps track of all models already retrieved from the database, mapped by their scaled model id
  std::map<int, GraspitDBModel*> models_;

  //! A GraspIt instance of the PR2 gripper
  Pr2Gripper2010 *gripper_;

  //! Used for general GrapsPlanning calls; should be a value from graspit_ros_planning_msgs::TestGrasp
  int default_grasp_test_type_;

  //! Used for converting energy values fo probabilities; soon to be replaced
  double default_energy_threshold_;

  //!-------------------------- populating the graspit world ------------------------------

  //! Will load a model, or retrieve it from the map of previously loaded models
  GraspitDBModel* getModel(int model_id);

  //! Loads the gripper info from its file, if not already loaded
  bool loadGripper();

  // ------------------------- helper functions for grasp tests ---------------------------

  //! Checks collisions between the gripper and the environment
  void gripperCollisionCheck(const Body *object, graspit_ros_planning_msgs::TestGrasp::Response &response);

  //! Performs the grasp test using the grasp energy function 
  void computeEnergy(Body *object, graspit_ros_planning_msgs::TestGrasp::Response &response);

  // ---------------------------------- grasp tests ----------------------------------------

  //! Tests a grasp using the direct method
  void testGraspDirect(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                       graspit_ros_planning_msgs::TestGrasp::Response &response);

  //! Tests a grasp using the compliant method
  void testGraspCompliant(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                          graspit_ros_planning_msgs::TestGrasp::Response &response);

  //! Tests a grasp using the direct method
  void testGraspReactive(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                         graspit_ros_planning_msgs::TestGrasp::Response &response);

  //! Tests a grasp using the direct method
  void testGraspRobustReactive(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                               graspit_ros_planning_msgs::TestGrasp::Response &response);

  // ---------------------------------- callbacks ----------------------------------------

  //! Callback for the load model service
  bool loadModelCB(graspit_ros_planning_msgs::LoadDatabaseModel::Request &request,
                   graspit_ros_planning_msgs::LoadDatabaseModel::Response &response);

  //! Callback for the load obstacle service
  bool loadObstacleCB(graspit_ros_planning_msgs::LoadObstacle::Request &request,
                      graspit_ros_planning_msgs::LoadObstacle::Response &response);

  //! Callback for the clear bodies service
  bool clearBodiesCB(graspit_ros_planning_msgs::ClearBodies::Request &request,
                     graspit_ros_planning_msgs::ClearBodies::Response &response);

  //! Callback for the clear bodies service
  bool simulateScanCB(graspit_ros_planning_msgs::SimulateScan::Request &request,
                      graspit_ros_planning_msgs::SimulateScan::Response &response);

  //! Callback for the detailed, graspit-specific test grasp service
  bool testGraspCB(graspit_ros_planning_msgs::TestGrasp::Request &request,
                   graspit_ros_planning_msgs::TestGrasp::Response &response);

  //! Callback for the general grasp planning service
  bool graspPlanningCB(manipulation_msgs::GraspPlanning::Request &request,
                       manipulation_msgs::GraspPlanning::Response &response);

  //! Callback for the grasp generation service, for PR2 gripper
  bool generateGraspCB(graspit_ros_planning_msgs::GenerateGrasp::Request &request,
                       graspit_ros_planning_msgs::GenerateGrasp::Response &response);

  //! Callback for the grasp verifying service, for PR2 gripper
  bool verifyGraspCB(graspit_ros_planning_msgs::VerifyGrasp::Request &request,
                     graspit_ros_planning_msgs::VerifyGrasp::Response &response);


public:
  //! Inits ROS, but (for now) without passing any arguments
  RosGraspitInterface();
  //! Deletes the node handle and the db manager
  ~RosGraspitInterface();
  //! Creates the node handles, advertises services, connects to the database
  virtual int init(int argc, char **argv);
  //! Simply calls ros::spinOnce() to process the ROS event loop
  virtual int mainLoop();
};

}

#endif
