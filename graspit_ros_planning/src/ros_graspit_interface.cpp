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

#include "graspit_ros_planning/ros_graspit_interface.h"

#include <boost/foreach.hpp>
#include <cmath>
#include <Inventor/actions/SoGetBoundingBoxAction.h>

#include <src/DBase/DBPlanner/ros_database_manager.h>
#include <src/DBase/graspit_db_model.h>
#include <src/Collision/collisionStructures.h>
#include <include/EGPlanner/searchEnergy.h>
#include <include/mytools.h>
#include <include/world.h>
#include <include/body.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>
#include <include/scanSimulator.h>
#include <include/pr2Gripper.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

//------------------------- Convenience functions -------------------------------

template<typename T>
  inline T mean(const std::vector<T> &input)
  {
    T total = 0.0;
    BOOST_FOREACH(const T &val, input)
          {
            total += val;
          }
    return total / input.size();
  }
;

template<typename T>
  inline T stddev(const std::vector<T> &input)
  {
    T m = mean(input);
    T total_err = 0.0;
    BOOST_FOREACH(const T &val, input)
          {
            total_err += std::pow(m - val, 2);
          }

    total_err /= input.size();

    return std::sqrt(total_err);
  }

namespace graspit_ros_planning
{

transf poseToTransf(const geometry_msgs::Pose &pose)
{
  return transf(Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z),
                vec3(1000 * pose.position.x, 1000 * pose.position.y, 1000 * pose.position.z));
}

transf poseStampedToTransf(const geometry_msgs::PoseStamped &pose)
{
  return transf(Quaternion(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z),
                vec3(1000 * pose.pose.position.x, 1000 * pose.pose.position.y, 1000 * pose.pose.position.z));
}

geometry_msgs::Pose transfToPose(const transf &tr)
{
  geometry_msgs::Pose pose;
  return pose;
}

inline void getSbBoxDimension(SbBox3f& bbx, vec3& dimension)
{
  // SbBox3f is axis aligned, so the dimension of the box can be easily computed
  float xmin, ymin, zmin, xmax, ymax, zmax;
  bbx.getBounds(xmin, ymin, zmin, xmax, ymax, zmax);
  dimension.set(xmax - xmin, ymax - ymin, zmax - zmin);
  ROS_INFO("Boundind box: (%.3lf, %.3lf, %.3lf), (%.3lf, %.3lf, %.3lf) ", xmin, ymin, zmin, xmax, ymax, zmax);
}

inline bool smallerThanSbVec3f(const SbVec3f& b1, const SbVec3f& b2)
{
  float x1, y1, z1, x2, y2, z2;
  b1.getValue(x1, y1, z1);
  b2.getValue(x2, y2, z2);

  if (x1 <= x2 && y1 <= y2 && z1 <= z2)
    return true;
  else
    return false;
}

inline bool biggerThanSbVec3f(const SbVec3f& b1, const SbVec3f& b2)
{
  float x1, y1, z1, x2, y2, z2;
  b1.getValue(x1, y1, z1);
  b2.getValue(x2, y2, z2);

  if (x1 >= x2 && y1 >= y2 && z1 >= z2)
    return true;
  else
    return false;
}

inline double randomUniformReal(double min, double max)
{
  return ((1.0 * rand() / RAND_MAX) * (max - min) + min);
}

void randomPoseGenerate(SbBox3f &bb_space, geometry_msgs::Pose& grasp_random_pose)
{

  float xmin, ymin, zmin, xmax, ymax, zmax;
  bb_space.getBounds(xmin, ymin, zmin, xmax, ymax, zmax);

  // Generate a random position within the bounding space first
  grasp_random_pose.position.x = randomUniformReal(xmin, xmax);
  grasp_random_pose.position.y = randomUniformReal(ymin, ymax);
  grasp_random_pose.position.z = randomUniformReal(zmin, zmax);

  // Generate a random orientation then
  double angle = randomUniformReal(0, 2.0 * M_PI);
  vec3 axis(randomUniformReal(0 + 0.0001, 1), randomUniformReal(0, 1), randomUniformReal(0, 1)); //make sure axis valid
  Quaternion orientation(angle, axis);

  grasp_random_pose.orientation.w = orientation.w;
  grasp_random_pose.orientation.x = orientation.x;
  grasp_random_pose.orientation.y = orientation.y;
  grasp_random_pose.orientation.z = orientation.z;
}

RosGraspitInterface::RosGraspitInterface() :
  root_nh_(NULL), priv_nh_(NULL), db_mgr_(NULL)
{
  default_energy_threshold_ = 68.6;
}

RosGraspitInterface::~RosGraspitInterface()
{
  ROS_INFO("ROS GraspIt node stopping");
  ros::shutdown();
  delete root_nh_;
  delete priv_nh_;
  delete db_mgr_;
}

//------------------------- Main class  -------------------------------

int RosGraspitInterface::init(int argc, char **argv)
{
  //copy the arguments somewhere else so we can pass them to ROS
  int ros_argc = argc;
  char** ros_argv = new char*[argc];
  for (int i = 0; i < argc; i++)
  {
    ros_argv[i] = new char[strlen(argv[i])];
    strcpy(ros_argv[i], argv[i]);
  }
  //see if a node name was requested
  std::string node_name("ros_graspit_interface");
  for (int i = 0; i < argc - 1; i++)
  {
    //std::cerr << argv[i] << "\n";
    if (!strcmp(argv[i], "_name"))
    {
      node_name = argv[i + 1];
    }
  }
  //init ros
  ros::init(ros_argc, ros_argv, node_name.c_str());
  //ROS_INFO("Using node name %s", node_name.c_str());
  //clean up ros arguments
  for (int i = 0; i < argc; i++)
  {
    delete ros_argv[i];
  }
  delete ros_argv;
  //init node handles
  root_nh_ = new ros::NodeHandle("");
  priv_nh_ = new ros::NodeHandle("~");
  //advertise service in private namespace
  load_model_srv_ = priv_nh_->advertiseService("load_database_model", &RosGraspitInterface::loadModelCB, this);
  load_obstacle_srv_ = priv_nh_->advertiseService("load_obstacle", &RosGraspitInterface::loadObstacleCB, this);
  clear_bodies_srv_ = priv_nh_->advertiseService("clear_bodies", &RosGraspitInterface::clearBodiesCB, this);
  simulate_scan_srv_ = priv_nh_->advertiseService("simulate_scan", &RosGraspitInterface::simulateScanCB, this);
  test_grasp_srv_ = priv_nh_->advertiseService("test_grasp", &RosGraspitInterface::testGraspCB, this);
  grasp_planning_srv_ = priv_nh_->advertiseService("grasp_planning", &RosGraspitInterface::graspPlanningCB, this);

  generate_grasp_srv_ = priv_nh_->advertiseService("generate_grasp", &RosGraspitInterface::generateGraspCB, this);

  scan_publisher_ = priv_nh_->advertise<sensor_msgs::PointCloud2> ("simulated_scans", 5);

  verify_grasp_srv_ = priv_nh_->advertiseService("verify_grasp", &RosGraspitInterface::verifyGraspCB, this);

  srand(1);
  //initialize database connection
  //careful: we pass a null for the grasp allocator as we don't know yet which hand we'll be using
  db_mgr_ = new db_planner::RosDatabaseManager("wgs36", "5432", "willow", "willow", "household_objects", NULL, NULL);
  //use the special allocator for models that get geometry directly from the database
  GeomGraspitDBModelAllocator* allocator = new GeomGraspitDBModelAllocator(db_mgr_);
  db_mgr_->SetModelAllocator(allocator);
  gripper_ = NULL;
  if (!db_mgr_->isConnected())
  {
    ROS_ERROR("Failed to connect to database");
    return -1;
  }
  ROS_INFO("ROS GraspIt node ready");
  return 0;
}

int RosGraspitInterface::mainLoop()
{
  ros::spinOnce();
  return 0;
}

/*! If the model has laready been loaded, it will retrieve it from the list of models.
 If not, it will load it, add it to the list, and return it. Returns NULL only if the model
 could not be loaded from the database.
 */
GraspitDBModel* RosGraspitInterface::getModel(int model_id)
{
  std::map<int, GraspitDBModel*>::iterator it = models_.find(model_id);
  if (it != models_.end())
    return it->second;

  //retrieve the model from the database
  db_planner::Model* db_model;
  if (!db_mgr_->ScaledModel(db_model, model_id))
  {
    ROS_ERROR("Failed to load database model with id %d", model_id);
    return NULL;
  }
  GraspitDBModel *model = static_cast<GraspitDBModel*> (db_model);
  models_.insert(std::pair<int, GraspitDBModel*>(model_id, model));
  return model;
}

bool RosGraspitInterface::loadGripper()
{
  World *world = graspItGUI->getIVmgr()->getWorld();
  if (gripper_)
  {
    ROS_WARN("Gripper load requested, but gripper already present. Re-loading.");
    world->removeRobot(gripper_);
  }
  std::string hand_path("/models/robots/pr2_gripper/pr2_gripper_2010.xml");
  hand_path = getenv("GRASPIT") + hand_path;
  gripper_ = static_cast<Pr2Gripper2010*> (world->importRobot(QString(hand_path.c_str())));
  if (!gripper_)
  {
    ROS_ERROR("Failed to load PR2 gripper from file %s", hand_path.c_str());
    return false;
  }
  return true;
}

bool RosGraspitInterface::loadModelCB(graspit_ros_planning_msgs::LoadDatabaseModel::Request &request,
                                      graspit_ros_planning_msgs::LoadDatabaseModel::Response &response)
{
  //retrieve model from database, if not already retrieved
  GraspitDBModel *model = getModel(request.model_id);
  if (!model)
  {
    response.result = response.LOAD_FAILURE;
    return true;
  }

  //load the geometry, if not already loaded
  World *world = graspItGUI->getIVmgr()->getWorld();
  if (!model->geometryLoaded())
  {
    if (model->load(world) != SUCCESS)
    {
      ROS_ERROR("Failed to load geometry for database model with id %d", request.model_id);
      response.result = response.LOAD_FAILURE;
      return true;
    }
  }

  //if requested, if there are any models in the world that are not this one, remove them
  if (request.clear_other_models)
  {
    bool done = false;
    while (!done)
    {
      done = true;
      for (int i = 0; i < world->getNumGB(); i++)
      {
        if (world->getGB(i) != model->getGraspableBody())
        {
          world->destroyElement(world->getGB(i), false);
          done = false;
          break;
        }
      }
    }
  }

  //add the model to the world, if not already there
  if (world->getIVRoot()->findChild(model->getGraspableBody()->getIVRoot()) == -1)
  {
    model->getGraspableBody()->addToIvc();
    world->addBody(model->getGraspableBody());
  }

  //set the correct transform
  model->getGraspableBody()->setTran(poseToTransf(request.model_pose));
  model->getGraspableBody()->showAxes(false);
  response.result = response.LOAD_SUCCESS;
  return true;
}

bool RosGraspitInterface::clearBodiesCB(graspit_ros_planning_msgs::ClearBodies::Request &request,
                                        graspit_ros_planning_msgs::ClearBodies::Response &response)
{
  World *world = graspItGUI->getIVmgr()->getWorld();
  if (request.which_bodies == request.GRASPABLE_BODIES || request.which_bodies == request.ALL_BODIES)
  {
    while (world->getNumGB() > 0)
    {
      //careful, we remove it from the world here but do not delete it
      //all geometry stays in place if we need it in the future, it just gets disconnected 
      //from the scene graph and the collision engine
      world->destroyElement(world->getGB(0), false);
    }
    ROS_INFO("Cleared graspable bodies");
  }
  if (request.which_bodies == request.OBSTACLES || request.which_bodies == request.ALL_BODIES)
  {
    bool removal = true;
    while (removal)
    {
      removal = false;
      for (int i = 0; i < world->getNumBodies(); i++)
      {
        if (!world->getBody(i)->inherits("DynamicBody"))
        {
          //obstacles get deleted for good as we don't keep track of them here
          world->destroyElement(world->getBody(i), true);
          removal = true;
          break;
        }
      }
    }
    ROS_INFO("Cleared obstacles bodies");
  }
  return true;
}

bool RosGraspitInterface::loadObstacleCB(graspit_ros_planning_msgs::LoadObstacle::Request &request,
                                         graspit_ros_planning_msgs::LoadObstacle::Response &response)
{
  std::string filename = getenv("GRASPIT") + std::string("/") + request.file_name;
  World *world = graspItGUI->getIVmgr()->getWorld();
  Body *body = world->importBody("Body", QString(filename.c_str()));
  if (!body)
  {
    ROS_ERROR("Failed to import obstacle from file %s", filename.c_str());
    response.result = response.LOAD_FAILURE;
    return true;
  }

  //set the correct transform
  body->setTran(poseToTransf(request.obstacle_pose));
  response.result = response.LOAD_SUCCESS;
  return true;
}

bool RosGraspitInterface::simulateScanCB(graspit_ros_planning_msgs::SimulateScan::Request &request,
                                         graspit_ros_planning_msgs::SimulateScan::Response &response)
{
  ScanSimulator scan_sim;
  scan_sim.setType(ScanSimulator::SCANNER_COORDINATES);
  scan_sim.setPosition(poseToTransf(request.scanner_pose), ScanSimulator::STEREO_CAMERA);
  //for now, we hard-code this in for the stereo camera, but they could be in the message
  double horiz_fov = 45;
  //half resolution to speed things up
  double horiz_res = 640;
  double vertical_res = 480;
  double vertical_fov = horiz_fov * vertical_res / horiz_res;
  scan_sim.setOptics(-horiz_fov / 2.0, horiz_fov / 2.0, horiz_res, -vertical_fov / 2.0, vertical_fov / 2.0,
                     vertical_res);

  //if the gripper is there, get it out of the scene graph so it does not show up in the scan
  World *world = graspItGUI->getIVmgr()->getWorld();
  if (gripper_)
  {
    world->removeElementFromSceneGraph(gripper_);
  }
  //get the scan (without the raw data)
  std::vector<position> cloud;
  ROS_INFO("Simulating scan...");

  if (request.request_ray_directions)
  {
    std::vector<RawScanPoint> rawData;
    scan_sim.scan(&cloud, &rawData);
    ROS_INFO("Simulated scan has %d points, %d rays", (int)cloud.size(), (int)rawData.size());
    // Also publish the direction of the rays that doesn't hit any object
    response.missing_ray_directions.reserve(rawData.size());
    for (size_t j = 0; j < rawData.size(); j++)
    {
      if (rawData[j].distance < -0.9) // If the ray doesn't hit anything, the distance is set to -1
      {
        geometry_msgs::Point32 dir;
        dir.x = rawData[j].dx;
        dir.y = rawData[j].dy;
        dir.z = rawData[j].dz;
        response.missing_ray_directions.push_back(dir);
      }
    }
  }
  else
  {
    scan_sim.scan(&cloud);
    ROS_INFO("Simulated scan has %d points", (int)cloud.size());
  }
  if (gripper_)
  {
    world->addElementToSceneGraph(gripper_);
  }
  //world->getIVRoot()->addChild(scan_sim.getVisualIndicator());

  //convert it to a PointCloud2
  //convert to a PointCloud first
  ROS_INFO("Converting scan to ROS format");
  sensor_msgs::PointCloud point_cloud;
  for (size_t i = 0; i < cloud.size(); i++)
  {
    geometry_msgs::Point32 point;
    point.x = 1.0e-3 * cloud[i].x();
    point.y = 1.0e-3 * cloud[i].y();
    point.z = 1.0e-3 * cloud[i].z();
    point_cloud.points.push_back(point);
  }
  point_cloud.header.frame_id = "graspit_scanner_frame";
  point_cloud.header.stamp = ros::Time::now();
  //convert to a PointCloud2
  ROS_INFO("Converting scan to point cloud 2");
  sensor_msgs::convertPointCloudToPointCloud2(point_cloud, response.scan);
  response.scan.header.stamp = ros::Time::now();

  //publish it on our internal topic
  ROS_INFO("Publishing scan");
  scan_publisher_.publish(response.scan);

  ROS_INFO("Done");
  return true;
}

void RosGraspitInterface::gripperCollisionCheck(const Body *object,
                                                graspit_ros_planning_msgs::TestGrasp::Response &response)
{
  response.hand_environment_collision = false;
  response.hand_object_collision = false;
  std::vector<DynamicBody*> gripper_link_list;
  gripper_->getAllLinks(gripper_link_list);
  std::vector<Body*> interest_list;
  for (size_t i = 0; i < gripper_link_list.size(); i++)
  {
    interest_list.push_back(gripper_link_list[i]);
  }
  CollisionReport collision_report;
  World *world = graspItGUI->getIVmgr()->getWorld();
  world->getCollisionReport(&collision_report, &interest_list);
  for (size_t i = 0; i < collision_report.size(); i++)
  {
    //figure out what the other body we are colliding with is
    Body *collision_body = collision_report[i].first;
    if (collision_body->getOwner() == gripper_)
    {
      collision_body = collision_report[i].second;
    }
    if (collision_body == object)
      response.hand_object_collision = true;
    else
      response.hand_environment_collision = true;
  }
}

void RosGraspitInterface::computeEnergy(Body *object, graspit_ros_planning_msgs::TestGrasp::Response &response)
{
  SearchEnergy energy_calculator;
  energy_calculator.setType(ENERGY_CONTACT);
  energy_calculator.setContactType(CONTACT_PRESET);
  bool legal;
  double energy;
  energy_calculator.analyzeCurrentPosture(gripper_, object, legal, energy, false);
  if (!legal)
  {
    //this should not really happen, as we've checked for collisions already
    ROS_WARN("Energy calculator reports illegal state");
    response.test_result = response.HAND_COLLISION;
    response.energy_value = -1.0;
    return;
  }
  response.energy_value = energy;
  //hard-coded threshold for now, will make it into a parameter
  if (energy < 10.0)
    response.test_result = response.GRASP_SUCCESS;
  else
    response.test_result = response.GRASP_FAILURE;
}

bool RosGraspitInterface::verifyGraspCB(graspit_ros_planning_msgs::VerifyGrasp::Request &request,
                                        graspit_ros_planning_msgs::VerifyGrasp::Response &response)
{
  response.hand_environment_collision = false;
  response.hand_object_collision = false;

  //  static int last_object_id = 0;

  graspit_ros_planning_msgs::LoadDatabaseModel::Request req_load_object;
  graspit_ros_planning_msgs::LoadDatabaseModel::Response res_load_object;

  World *world_p = graspItGUI->getIVmgr()->getWorld();
  GraspableBody *object = NULL;
  Body *obstacle = NULL;

  std::string filename = getenv("GRASPIT") + std::string("/") + request.table_file_name;

  req_load_object.model_id = request.scaled_model_id;
  req_load_object.model_pose = geometry_msgs::Pose();
  req_load_object.model_pose.orientation.w = 1.0;
  req_load_object.clear_other_models = true;

  loadModelCB(req_load_object, res_load_object);

  if (res_load_object.result == graspit_ros_planning_msgs::LoadDatabaseModel::Response::LOAD_FAILURE)
  {
    response.result = graspit_ros_planning_msgs::VerifyGrasp::Response::LOAD_OBJECT_FAILURE;
    ROS_WARN_STREAM("Object load failed!");
    return true;
  }

  object = world_p->getGB(0);

  //Get the bounding box of the object
  SoGetBoundingBoxAction bboxAction(graspItGUI->getIVmgr()->getViewer()->getViewportRegion());
  bboxAction.apply(object->getIVRoot());
  SbBox3f object_bbx = bboxAction.getBoundingBox();
  vec3 obj_bbx_dim;
  SbVec3f object_bbx_min, object_bbx_max;
  object_bbx.getBounds(object_bbx_min, object_bbx_max);
  getSbBoxDimension(object_bbx, obj_bbx_dim);

  // Load the tabletop
  // ROS_INFO_STREAM("Number of objects in the world " << world_p->getNumBodies());
  for (int i = 0; i < world_p->getNumBodies(); i++)
  {
    if (world_p->getBody(i)->getFilename().compare(QString(request.table_file_name.c_str())) == 0)
    {
      obstacle = world_p->getBody(i);
      break;
    }
  }

  graspit_ros_planning_msgs::LoadObstacle::Request req_load_obstacle;
  req_load_obstacle.file_name = request.table_file_name;
  req_load_obstacle.obstacle_pose = geometry_msgs::Pose();
  req_load_obstacle.obstacle_pose.orientation.w = 1.0;
  if (obstacle == NULL)
  {
    graspit_ros_planning_msgs::LoadObstacle::Response res_load_obstacle;

    loadObstacleCB(req_load_obstacle, res_load_obstacle);

    if (res_load_obstacle.result == graspit_ros_planning_msgs::LoadObstacle::Response::LOAD_FAILURE)
    {
      response.result = graspit_ros_planning_msgs::VerifyGrasp::Response::LOAD_OBSTACLE_FAILURE;
      ROS_WARN_STREAM("Tabletop load failed!");
      return true;
    }

    for (int i = 0; i < world_p->getNumBodies(); i++)
    {
      if (world_p->getBody(i)->getFilename().compare(QString(request.table_file_name.c_str())) == 0)
      {
        obstacle = world_p->getBody(i);
        break;
      }
    }
  }

  //Get the bounding box of the obstacle
  bboxAction.apply(obstacle->getIVRoot());
  SbBox3f obstacle_bbx = bboxAction.getBoundingBox();
  vec3 obstacle_bbx_dim;
  SbVec3f obstacle_bbx_min, obstacle_bbx_max;
  obstacle_bbx.getBounds(obstacle_bbx_min, obstacle_bbx_max);
  ROS_INFO("Tabletop");
  getSbBoxDimension(obstacle_bbx, obstacle_bbx_dim);

  float obstacle_current_z = obstacle->getTran().translation().z();
  float obstacle_top_z = obstacle_bbx_max[2];
  float object_bottom_z = object_bbx_min[2];
  float new_obstacle_z = obstacle_current_z + object_bottom_z - obstacle_top_z;
  req_load_obstacle.obstacle_pose.position.z = new_obstacle_z * 0.001; //
  obstacle->setTran(poseToTransf(req_load_obstacle.obstacle_pose));

  bboxAction.apply(obstacle->getIVRoot());
  SbBox3f obstacle_bbx_new = bboxAction.getBoundingBox();
  obstacle_bbx_new.getBounds(obstacle_bbx_min, obstacle_bbx_max);
  getSbBoxDimension(obstacle_bbx_new, obstacle_bbx_dim);

  // Load gripper to the scene
  if (!gripper_ && !loadGripper())
  {
    response.result = graspit_ros_planning_msgs::VerifyGrasp::Response::LOAD_GRIPPER_FAILURE;
    ROS_WARN_STREAM("Gripper load failed!");
    return true;
  }

  // Result valid
  response.result = graspit_ros_planning_msgs::VerifyGrasp::Response::CHECK_SUCCESS;
  //place the hand in the right pose
  gripper_->setTran(poseToTransf(request.grasp_pose));
  //set the gripper dof
  gripper_->forceDOFVal(0, request.grasp_joint_angle);

  graspit_ros_planning_msgs::TestGrasp::Response test_collision_response;

  response.gripper_tabletop_clearance = world_p->getDist(gripper_, obstacle);
  response.gripper_object_distance = world_p->getDist(gripper_, object);
  gripperCollisionCheck(object, test_collision_response);

  bool any_collision = false;

  if (test_collision_response.hand_environment_collision)
  {
    response.hand_environment_collision = true;
    ROS_WARN_STREAM("Collision with environment detected!");
    any_collision = true;
  }
  if (test_collision_response.hand_object_collision)
  {
    response.hand_object_collision = true;
    ROS_WARN_STREAM("Collision with object detected!");
    any_collision = true;
  }

  if (request.grasp_pose.position.z < 0.0 && !test_collision_response.hand_environment_collision)
  {
    ROS_WARN_STREAM("Gripper under desktop, hack the collision result");
    response.hand_environment_collision = true;
    response.gripper_tabletop_clearance = -1.0;
    any_collision = true;
  }

  if (!any_collision)
    ROS_WARN_STREAM("No Collision detected!");
  return true;
}

bool RosGraspitInterface::generateGraspCB(graspit_ros_planning_msgs::GenerateGrasp::Request &request,
                                          graspit_ros_planning_msgs::GenerateGrasp::Response &response)
{
  // The resulted grasp pose doesn't guarantee that the object will be reasonably close to the gripper. (Generating voxelgrid using this grasp pose might get a completedly free voxelgrid)

  response.result = response.GENERATE_FAILURE;

  // Load the requested object to graspit and clear other objects
  graspit_ros_planning_msgs::LoadDatabaseModel::Request req_load_object;
  graspit_ros_planning_msgs::LoadDatabaseModel::Response res_load_object;

  static int last_model_id = 0;
  static int acc_count = 1;

  // object's pose will coincides with world frame, to simplify everything
  req_load_object.model_id = request.model_id;
  req_load_object.model_pose = geometry_msgs::Pose();
  req_load_object.model_pose.orientation.w = 1.0;
  req_load_object.clear_other_models = true;

  loadModelCB(req_load_object, res_load_object);

  if (res_load_object.result == res_load_object.LOAD_FAILURE)
  {
    response.result = response.GENERATE_FAILURE;
    ROS_INFO_STREAM("Object loaded failed!");
    return true;
  }

  //ROS_INFO_STREAM("Object loaded successful!");


  // Get a pointer to the object
  GraspitDBModel *model = getModel(request.model_id);
  if (!model)
  {
    response.result = response.GENERATE_FAILURE;
    return true;
  }

  // Load gripper to the scene
  if (!gripper_ && !loadGripper())
  {
    response.result = response.GENERATE_FAILURE;
    return true;
  }

  //  ROS_INFO_STREAM("Gripper loaded successful!");

  // Calculate bounding box sizes of both the object and gripper
  SoGetBoundingBoxAction bboxAction(graspItGUI->getIVmgr()->getViewer()->getViewportRegion());
  bboxAction.apply(model->getGraspableBody()->getIVRoot());
  SbBox3f object_bbx = bboxAction.getBoundingBox();

  gripper_->setTran(transf());
  //set the gripper dof (for the moment all the way open)
  gripper_->forceDOFVal(0, 0.26);
  bboxAction.apply(gripper_->getIVRoot());
  SbBox3f gripper_bbx = bboxAction.getBoundingBox();

  // Define the bounding space for randomly selecting the gripper
  vec3 gripper_bbx_dim, obj_bbx_dim, bbs_dim;
  //ROS_INFO("Gripper");
  getSbBoxDimension(gripper_bbx, gripper_bbx_dim);
  // double gripper_bbx_diag = sqrt(gripper_bbx_dim.len_sq());
  double gripper_box_xdim = gripper_bbx_dim.x();

  SbVec3f object_bbx_min, object_bbx_max;
  object_bbx.getBounds(object_bbx_min, object_bbx_max);
  // ROS_INFO("Object");
  getSbBoxDimension(object_bbx, obj_bbx_dim);

  SbBox3f bb_space;
  SbVec3f offset(30, 30, 0);
  SbVec3f offset2(30, 30, 30);
  if (last_model_id == request.model_id)
  {
    acc_count = (acc_count+1)%2; // acc_count != 0 and acc_count will accumulate
  }
  else
  {
    acc_count = 0;
  }
  last_model_id = request.model_id;
  SbVec3f bb_min, bb_max;
  bb_min = object_bbx_min - SbVec3f(gripper_box_xdim, gripper_box_xdim, 0) + acc_count * offset;
  bb_max = object_bbx_max + SbVec3f(gripper_box_xdim, gripper_box_xdim, gripper_box_xdim) - acc_count * offset2;

  bb_space.setBounds(bb_min, bb_max);
  //ROS_INFO("Bounding space");
  getSbBoxDimension(bb_space, bbs_dim);

  // PressEnterToContinue();
  ROS_ASSERT(gripper_->getNumChains() == 2); // Pr2 gripper should have two kinematic chains

  // Randomly generate grasp pose within the bounding space
  geometry_msgs::Pose grasp_random_pose;
  graspit_ros_planning_msgs::TestGrasp::Response test_grasp_res;
  std::vector<Body*> interest_list;
  interest_list.push_back(gripper_->getBase()); // palm
  interest_list.push_back(gripper_->getChain(0)->getLink(0)); //upper finger
  interest_list.push_back(gripper_->getChain(1)->getLink(0)); //another upper finger
  if (request.reject_fingertip_collision)
  {
    interest_list.push_back(gripper_->getChain(0)->getLink(1)); //fingertip
    interest_list.push_back(gripper_->getChain(1)->getLink(1)); //fingertip
  }

  World *world = graspItGUI->getIVmgr()->getWorld();
  Body *obstacle = NULL;

  if (request.request_tabletop)
  {
    std::string filename = getenv("GRASPIT") + std::string("/") + request.tabletop_file_name;

    for (int i = 0; i < world->getNumBodies(); i++)
    {
      if (world->getBody(i)->getFilename().compare(QString(request.tabletop_file_name.c_str())) == 0)
      {
        obstacle = world->getBody(i);
        break;
      }
    }

    graspit_ros_planning_msgs::LoadObstacle::Request req_load_obstacle;
    req_load_obstacle.file_name = request.tabletop_file_name;
    req_load_obstacle.obstacle_pose = geometry_msgs::Pose();
    req_load_obstacle.obstacle_pose.orientation.w = 1.0;

    if (obstacle == NULL)
    {
      graspit_ros_planning_msgs::LoadObstacle::Response res_load_obstacle;
      loadObstacleCB(req_load_obstacle, res_load_obstacle);

      if (res_load_obstacle.result == graspit_ros_planning_msgs::LoadObstacle::Response::LOAD_FAILURE)
      {
        response.result = response.GENERATE_FAILURE;
        ROS_WARN_STREAM("Tabletop load failed!");
        return true;
      }
      for (int i = 0; i < world->getNumBodies(); i++)
      {
        if (world->getBody(i)->getFilename().compare(QString(request.tabletop_file_name.c_str())) == 0)
        {
          obstacle = world->getBody(i);
          break;
        }
      }
    }

    //Get the bounding box of the obstacle
    bboxAction.apply(obstacle->getIVRoot());
    SbBox3f obstacle_bbx = bboxAction.getBoundingBox();
    vec3 obstacle_bbx_dim;
    SbVec3f obstacle_bbx_min, obstacle_bbx_max;
    obstacle_bbx.getBounds(obstacle_bbx_min, obstacle_bbx_max);
    getSbBoxDimension(obstacle_bbx, obstacle_bbx_dim);

    float obstacle_current_z = obstacle->getTran().translation().z();
    float obstacle_top_height = obstacle_bbx_max[2];
    float object_bottom_height = object_bbx_min[2];
    float new_obstacle_height = obstacle_current_z + object_bottom_height - obstacle_top_height;
    req_load_obstacle.obstacle_pose.position.z = new_obstacle_height * 0.001; //
    obstacle->setTran(poseToTransf(req_load_obstacle.obstacle_pose));

    bboxAction.apply(obstacle->getIVRoot());
    SbBox3f obstacle_bbx_new = bboxAction.getBoundingBox();
    obstacle_bbx_new.getBounds(obstacle_bbx_min, obstacle_bbx_max);
    getSbBoxDimension(obstacle_bbx_new, obstacle_bbx_dim);
  }

  int test_count = 1;
  double gripper_tabletop_clearance = -1;
  double gripper_object_distance = -1;
  while (true/*test_count > 0*/)
  {
    // ROS_INFO_STREAM("Trial " << test_count);
    test_count++;

    randomPoseGenerate(bb_space, grasp_random_pose); // Generate a random gripper pose in the bounding space
    //   ROS_INFO_STREAM( " Random grasp pose:" << grasp_random_pose);
    //   PressEnterToContinue();
    grasp_random_pose.position.x = grasp_random_pose.position.x * 1.0e-3; //Convert to meters
    grasp_random_pose.position.y = grasp_random_pose.position.y * 1.0e-3;
    grasp_random_pose.position.z = grasp_random_pose.position.z * 1.0e-3;

    //place the hand in the right pose
    gripper_->setTran(poseToTransf(grasp_random_pose));
    //set the gripper dof (for the moment all the way open)
    gripper_->forceDOFVal(0, 0.523);
    // Let the gripper automatically hold to the object
    gripper_->autoGrasp(false, 0.5, true);

    //ROS_INFO_STREAM( " Random grasp pose:" << grasp_random_pose << " grasp joint angle " << gripper_->getDOF(0)->getVal() );

    gripper_object_distance = world->getDist(gripper_, model->getGraspableBody());
    if (request.request_tabletop)
    {
      gripper_tabletop_clearance = world->getDist(gripper_, obstacle);
      graspit_ros_planning_msgs::TestGrasp::Response test_collision_response;

      gripperCollisionCheck(model->getGraspableBody(), test_collision_response);

      if (test_collision_response.hand_environment_collision)
      {
        //      ROS_WARN_STREAM("Collision with environment detected!, reject it!");
        continue;
      }

      if (grasp_random_pose.position.z < 0.0)
      {
        gripper_tabletop_clearance = -1.0;
        continue;
      }
    }

    // For every grasp pose, detect collision between gripper palm, two gripper upper fingers (i.e. all gripper parts except fingertips) and the object
    // If collision exists, reject this grasp pose
    CollisionReport collision_report;
    world->getCollisionReport(&collision_report, &interest_list);
    if (collision_report.size() > 0)
    {
      //      if (request.reject_fingertip_collision)
      //       ROS_INFO_STREAM("Collision exists between the gripper and the object, reject it");
      //     else
      //       ROS_INFO_STREAM("Collision exists between the gripper bottom part and the object, reject it");

      continue; //if collision exists, then continue to the next grasp pose
    }

    //  ROS_INFO_STREAM("No collision exists between the gripper bottom part and the object");

    // Compute grasp energy
    computeEnergy(model->getGraspableBody(), test_grasp_res);

    if (test_grasp_res.test_result == test_grasp_res.HAND_COLLISION) // collision exists, which should be with the fingertip, we found a bad grasp

    {
      if (request.reject_fingertip_collision)
      {
        ROS_INFO_STREAM("Collision exists between fingertip and the object, reject it and this should not happen");
        continue;
      }
      else
      {
        response.hand_object_collision = true;
        ROS_INFO_STREAM("Collision exists between fingertip and the object, accept it");
        break;
      }
    }

    if (test_grasp_res.energy_value > request.energy_high_threshold || test_grasp_res.energy_value
        < request.energy_low_threshold)
    {

      //   if (test_grasp_res.energy_value > request.energy_high_threshold)
      //     ROS_INFO_STREAM("Too high grasp energy, reject it");
      //   if (test_grasp_res.energy_value < request.energy_low_threshold)
      //     ROS_INFO_STREAM("Too low grasp energy , reject it");
      continue; // if energy too high or too low, reject
      // too high could be because object too far away from the gripper, too low is not bad grasp as we want
    }
    if (request.reject_fingertip_collision) // Only accept grasp with a grasp energy between two thresholds

    {
      ROS_INFO_STREAM("Bad grasp we found, accept it");
      response.hand_object_collision = false;
      break;
    }
    else // Only accept bad grasp that involves fingertip collision with the object

    {
      continue;
    }
  }

  ROS_INFO_STREAM( "Random grasp pose:" << grasp_random_pose << " grasp joint angle " << gripper_->getDOF(0)->getVal() <<" energy " << test_grasp_res.energy_value );
  ROS_INFO_STREAM("Number of objects "<<world->getNumBodies());
  response.grasp_pose = grasp_random_pose;
  response.grasp_energy = test_grasp_res.energy_value;
  response.grasp_joint_angle = gripper_->getDOF(0)->getVal();
  response.gripper_tabletop_clearance = gripper_tabletop_clearance;
  response.gripper_object_distance = gripper_object_distance;
  response.result = response.GENERATE_SUCCESS;
  return true;

}

void RosGraspitInterface::testGraspDirect(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                                          graspit_ros_planning_msgs::TestGrasp::Response &response)
{
  //place the hand in the right pose
  gripper_->setTran(poseStampedToTransf(grasp.grasp_pose));
  //set the gripper dof (for the moment all the way open)
  gripper_->forceDOFVal(0, 0.523);//request.grasp.grasp_posture.position[0]);
  gripper_->autoGrasp(false, 0.5, true);
  //check for collisions
  gripperCollisionCheck(object, response);
  //if we find a collision, we are done
  if (response.hand_object_collision || response.hand_environment_collision)
  {
    response.test_result = response.HAND_COLLISION;
    return;
  }
  //compute the energy value
  computeEnergy(object, response);
}

void RosGraspitInterface::testGraspCompliant(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                                             graspit_ros_planning_msgs::TestGrasp::Response &response)
{
  //place the hand in the right pose
  gripper_->setTran(poseStampedToTransf(grasp.grasp_pose));
  //set the gripper dof in the pre-grasp
  gripper_->forceDOFVal(0, grasp.pre_grasp_posture.position[0]);
  //check for collisions
  gripperCollisionCheck(object, response);
  //if we find a collision, we are done
  if (response.hand_object_collision || response.hand_environment_collision)
  {
    response.test_result = response.HAND_COLLISION;
    return;
  }
  //perform the compliant close
  gripper_->compliantClose();
  //compute the energy value
  computeEnergy(object, response);
}

void RosGraspitInterface::testGraspReactive(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                                            graspit_ros_planning_msgs::TestGrasp::Response &response)
{
  // put hand in pre-grasp pose
  transf pre_grasp = poseStampedToTransf(grasp.grasp_pose);
  double moveDist = 100;
  pre_grasp = translate_transf(vec3(0, 0, -moveDist) * gripper_->getApproachTran()) * pre_grasp;
  gripper_->setTran(pre_grasp);
  // Open hand all the way up
  gripper_->forceDOFVal(0, 0.523);
  // check for collisions
  gripperCollisionCheck(object, response);
  //if we find a collision, we are done
  if (response.hand_object_collision || response.hand_environment_collision)
  {
    response.test_result = response.HAND_COLLISION;
    return;
  }
  // move hand forward until it contacts
  gripper_->approachToContact(moveDist, false);
  // Execute a compliant close
  gripper_->compliantClose();
  //Compute the score for the resulting grasp
  computeEnergy(object, response);
  response.energy_value_list.push_back(response.energy_value);
}

void RosGraspitInterface::testGraspRobustReactive(const manipulation_msgs::Grasp &grasp, GraspableBody *object,
                                                  graspit_ros_planning_msgs::TestGrasp::Response &response)
{
  // put hand in pre-grasp pose
  transf orig_pre_grasp = poseStampedToTransf(grasp.grasp_pose);
  double moveDist = 100;
  const int n_tsteps = 3;
  //double t_steps[n_tsteps] = {-15,-10,-5,0,5,10,15};
  double t_steps[n_tsteps] = {-10, 0, 10};
  for (int x_step = 0; x_step < n_tsteps; x_step++)
  {
    transf pre_grasp = translate_transf(vec3(t_steps[x_step], 0, -moveDist) * gripper_->getApproachTran())
        * orig_pre_grasp;
    gripper_->setTran(pre_grasp);
    // Open hand all the way up
    gripper_->forceDOFVal(0, 0.523);
    // check for collisions
    gripperCollisionCheck(object, response);
    //if we find a collision, we are done
    if (response.hand_object_collision || response.hand_environment_collision)
    {
      response.energy_value_list.push_back(100);
    }
    else
    {
      // move hand forward until it contacts
      gripper_->approachToContact(moveDist, false);
      // Execute a compliant close
      gripper_->compliantClose();
      //Compute the score for the resulting grasp
      computeEnergy(object, response);
      response.energy_value_list.push_back(response.energy_value);
    }
  }

  for (int y_step = 0; y_step < n_tsteps; y_step++)
  {
    transf pre_grasp = translate_transf(vec3(0, t_steps[y_step], -moveDist) * gripper_->getApproachTran())
        * orig_pre_grasp;
    gripper_->setTran(pre_grasp);
    // Open hand all the way up
    gripper_->forceDOFVal(0, 0.523);
    // check for collisions
    gripperCollisionCheck(object, response);
    //if we find a collision, we are done
    if (response.hand_object_collision || response.hand_environment_collision)
    {
      response.energy_value_list.push_back(100);
    }
    else
    {
      // move hand forward until it contacts
      gripper_->approachToContact(moveDist, false);
      // Execute a compliant close
      gripper_->compliantClose();

      //Compute the score for the resulting grasp
      computeEnergy(object, response);
      response.energy_value_list.push_back(response.energy_value);
    }
  }

  for (int z_step = 0; z_step < n_tsteps; z_step++)
  {
    transf pre_grasp = translate_transf(vec3(0, 0, t_steps[z_step] - moveDist) * gripper_->getApproachTran())
        * orig_pre_grasp;
    gripper_->setTran(pre_grasp);
    // Open hand all the way up
    gripper_->forceDOFVal(0, 0.523);
    // check for collisions
    gripperCollisionCheck(object, response);
    //if we find a collision, we are done
    if (response.hand_object_collision || response.hand_environment_collision)
    {
      response.energy_value_list.push_back(100);
    }
    else
    {
      // move hand forward until it contacts
      gripper_->approachToContact(moveDist, false);
      // Execute a compliant close
      gripper_->compliantClose();

      //Compute the score for the resulting grasp
      computeEnergy(object, response);
      response.energy_value_list.push_back(response.energy_value);
    }
  }

  double m_energy = mean(response.energy_value_list);
  double stddev_energy = stddev(response.energy_value_list);
  response.energy_value = m_energy + stddev_energy;
}

bool RosGraspitInterface::testGraspCB(graspit_ros_planning_msgs::TestGrasp::Request &request,
                                      graspit_ros_planning_msgs::TestGrasp::Response &response)
{
  response.test_performed = response.TEST_WAS_NOT_PERFORMED;
  //check if we have a gripper
  if (!gripper_ && !loadGripper())
    return true;
  //check if we have an object
  //by default (for now) we only test against the first graspable object in the world
  World *world = graspItGUI->getIVmgr()->getWorld();
  if (!world->getNumGB())
  {
    ROS_ERROR("No graspable object loaded");
    return true;
  }
  GraspableBody *object = world->getGB(0);
  //check is grasp is correctly specified  
  if (request.grasp.grasp_posture.position.empty() || request.grasp.pre_grasp_posture.position.empty())
  {
    ROS_ERROR("Gripper DOF not specified in grasp or pre-grasp");
    return true;
  }

  response.test_performed = response.TEST_WAS_PERFORMED;
  if (request.test_type == request.DIRECT)
  {
    testGraspDirect(request.grasp, object, response);
  }
  else if (request.test_type == request.COMPLIANT_CLOSE)
  {
    testGraspCompliant(request.grasp, object, response);
  }
  else if (request.test_type == request.REACTIVE_GRASP)
  {
    testGraspReactive(request.grasp, object, response);
  }
  else if (request.test_type == request.ROBUST_REACTIVE_GRASP)
  {
    testGraspReactive(request.grasp, object, response);
  }
  else
  {
    ROS_ERROR("Unknown test type requested");
    response.test_performed = response.TEST_WAS_NOT_PERFORMED;
  }
  return true;
}

bool RosGraspitInterface::graspPlanningCB(manipulation_msgs::GraspPlanning::Request &request,
                                          manipulation_msgs::GraspPlanning::Response &response)
{
  response.error_code.value = response.error_code.OTHER_ERROR;
  if (!gripper_ && !loadGripper())
  {
    ROS_ERROR("Failed to load gripper for grasp planning");
    return true;
  }
  if (request.target.potential_models.empty())
  {
    ROS_ERROR("Graspit can only evaluate grasps on database objects");
    return true;
  }
  if (request.target.potential_models.size() != 1)
  {
    ROS_WARN("Graspit can only evaluate grasps on the first potential model in the list");
  }
  if (request.grasps_to_evaluate.empty())
  {
    ROS_ERROR("For now, graspit will test grasps on-line, but not plan new grasps");
    return true;
  }
  graspit_ros_planning_msgs::LoadDatabaseModel load_srv;
  load_srv.request.model_id = request.target.potential_models[0].model_id;
  load_srv.request.clear_other_models = true;
  loadModelCB(load_srv.request, load_srv.response);
  if (load_srv.response.result != load_srv.response.LOAD_SUCCESS)
  {
    ROS_ERROR("Failed to load database model for graspit grasp testing");
    return true;
  }
  GraspableBody *object = graspItGUI->getIVmgr()->getWorld()->getGB(0);

  //have to hardcode SUCCESS as graspit defines over it...
  response.error_code.value = 0;

  BOOST_FOREACH(manipulation_msgs::Grasp grasp, request.grasps_to_evaluate)
        {
          if (grasp.grasp_posture.position.empty() || grasp.pre_grasp_posture.position.empty())
          {
            ROS_ERROR("Gripper DOF not specified in grasp or pre-grasp");
            response.error_code.value = response.error_code.OTHER_ERROR;
            return true;
          }

          graspit_ros_planning_msgs::TestGrasp test_srv;
          if (default_grasp_test_type_ == test_srv.request.DIRECT)
          {
            testGraspDirect(grasp, object, test_srv.response);
          }
          else if (default_grasp_test_type_ == test_srv.request.COMPLIANT_CLOSE)
          {
            testGraspCompliant(grasp, object, test_srv.response);
          }
          else if (default_grasp_test_type_ == test_srv.request.REACTIVE_GRASP)
          {
            testGraspReactive(grasp, object, test_srv.response);
          }
          else if (default_grasp_test_type_ == test_srv.request.ROBUST_REACTIVE_GRASP)
          {
            testGraspReactive(grasp, object, test_srv.response);
          }
          if (test_srv.response.test_performed != test_srv.response.TEST_WAS_PERFORMED)
          {
            ROS_ERROR("Failed to test grasp");
            response.error_code.value = response.error_code.OTHER_ERROR;
            return true;
          }

          //come up with some probability value based on the test result
          double probability;
          //0 probability for grasps that result in collision
          if (test_srv.response.test_result == test_srv.response.HAND_COLLISION)
            probability = 0.0;
          //otherwise, return non-zero linear probability if the energy is below the threshold
          if (default_grasp_test_type_ == test_srv.request.ROBUST_REACTIVE_GRASP)
          {
            // Compute the mean and stddev of the values
          }
          probability = std::max(0.0, 1 - (test_srv.response.energy_value / default_energy_threshold_));
          grasp.grasp_quality = probability;

          response.grasps.push_back(grasp);
        }

  return true;
}

}
