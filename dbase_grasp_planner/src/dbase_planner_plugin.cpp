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

#include "graspit_dbase_tasks/dbTaskDispatcher.h"

#include "dbase_grasp_planner/grasp_planning_task.h"
#include "dbase_grasp_planner/guided_grasp_planning_task.h"
#include "dbase_grasp_planner/grasp_clustering_task.h"

class GraspPlanningTaskCreator : public graspit_dbase_tasks::DBTaskCreator
{
public:
  virtual graspit_dbase_tasks::DBTask* operator()(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                                  db_planner::DatabaseManager *mgr, 
                                                  db_planner::TaskRecord rec)
  {
    return new dbase_grasp_planner::GraspPlanningTask(disp, mgr, rec);
  }
};

class GuidedGraspPlanningTaskCreator : public graspit_dbase_tasks::DBTaskCreator
{
public:
  virtual graspit_dbase_tasks::DBTask* operator()(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                                  db_planner::DatabaseManager *mgr, 
                                                  db_planner::TaskRecord rec)
  {
    return new dbase_grasp_planner::GuidedGraspPlanningTask(disp, mgr, rec);
  }
};

class GraspClusteringTaskCreator : public graspit_dbase_tasks::DBTaskCreator
{
public:
  virtual graspit_dbase_tasks::DBTask* operator()(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                                  db_planner::DatabaseManager *mgr, 
                                                  db_planner::TaskRecord rec)
  {
    return new dbase_grasp_planner::GraspClusteringTask(disp, mgr, rec);
  }
};

extern "C" Plugin* createPlugin() {
  graspit_dbase_tasks::DBTaskDispatcher* dispatcher = new graspit_dbase_tasks::DBTaskDispatcher;

  GraspPlanningTaskCreator *creator = new GraspPlanningTaskCreator;
  dispatcher->registerTaskCreator("GRASP_PLANNING", creator);

  GuidedGraspPlanningTaskCreator *guided_creator = new GuidedGraspPlanningTaskCreator;
  dispatcher->registerTaskCreator("GUIDED_GRASP_PLANNING", guided_creator);

  GraspClusteringTaskCreator *clustering_creator = new GraspClusteringTaskCreator;
  dispatcher->registerTaskCreator("GRASP_CLUSTERING", clustering_creator);

  return dispatcher;
}

extern "C" std::string getType() {
  return "dbase_grasp_planning";
}

