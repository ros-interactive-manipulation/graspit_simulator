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

#ifndef _GUIDEDGRASPPLANNINGTASK_H_
#define _GUIDEDGRASPPLANNINGTASK_H_

#include <QObject>

#include <graspit_dbase_tasks/dbTask.h>

class Hand;
class GraspableBody;
class GuidedPlanner;
class GraspPlanningState;

namespace dbase_grasp_planner {

//! Uses a more advanced GuidedPlanner for grasp planning on dexterous hands; saves the results in a database
/*! This Task is similar to the GraspPlanningTask in this same package. See
  that class documentation to get started.

  The key difference is that this task uses a different type of GraspIt! planner,
  namely the GuidedPlanner, which is better suited for more dexterous hands.

  This task is better suited for planning with the Schunk SDH or the Barrett hands.
 */
class GuidedGraspPlanningTask : public QObject, public graspit_dbase_tasks::DBTask {
  Q_OBJECT
 private:
  //! The object we are planning on
  GraspableBody *mObject;
  //! The planner that we are using
  GuidedPlanner *mPlanner;
  //! The hand we are planning with
  Hand *mHand;
  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;
  //! The index of the last solution saved
  size_t mLastSolution;

  //! Saves a solution grasp to the database
  bool saveGrasp(const GraspPlanningState *pre_gps, const GraspPlanningState *final_gps);
  
public:
  //! Just a stub for now
  GuidedGraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                          db_planner::DatabaseManager *mgr, 
                          db_planner::TaskRecord rec);
  
  //! Removes the object that has been used from the sim world, but not the hand
  ~GuidedGraspPlanningTask();

  //! Loads the hand and the object, initializes and starts a guided planner
  virtual void start();

public slots:
  //! Connected to the update() signal of the planner
  void plannerUpdate();
  //! Connected to the complete() signal of the planner
  void plannerComplete();
};


}

#endif
