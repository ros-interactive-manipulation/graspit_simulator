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

#ifndef _GRASPPLANNINGTASK_H_
#define _GRASPPLANNINGTASK_H_

#include <QObject>

#include <graspit_dbase_tasks/dbTask.h>

class Hand;
class GraspableBody;
class EGPlanner;
class GraspPlanningState;

namespace dbase_grasp_planner {

//! Uses a simple Simulated Annealing loop to plans grasps for the hand and object, and stores them in the database
/*! This task perform grasp planning for simpe robot hands. It uses the 
  Simulated Annealing planner - see the Eigengrasp Planning chapter in the
  GraspIt! User's Manual for more details. This planner is well suited for
  relatively simple hands such as parallel grippers where all that is needed 
  is to bring the fingerpads in contact with the object. 

  It is well suited for planning with the PR2 Gripper.

  For more complex hands, see the GuidedGraspPlanner inside this same package.
  
  For now, it does its own cleanup in the sense that it will remove from 
  the world and delete the object that was used for planning. However, it 
  will leave the hand in, as it might be used by subsequent tasks.
  
  On startup, it will load the hand it needs, unless that hand is already
  the currently selected hand in the world. It will also load the object
  it needs. It will not delete anything from the world ar startup.
  
  The init and cleanup so that the world is used by subsequent tasks is
  not well-defined yet, needs more work. Exactly what initialization and 
  cleanup in the GraspIt world such a planner should do is unclear, might 
  change in the future.
    
 */
class GraspPlanningTask : public QObject, public graspit_dbase_tasks::DBTask {
  Q_OBJECT
 protected:
  //! The object we are planning on
  GraspableBody *mObject;
  //! The planner that we are using
  EGPlanner *mPlanner;
  //! The hand we are planning with
  Hand *mHand;
  //! The index of the last solution that was already saved in the database
  int mLastSolution;
  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;
  
  //! Saves a solution grasp to the database
  bool saveGrasp(const GraspPlanningState *pre_gps, const GraspPlanningState *final_gps);
  
  //! Computes a pre-grasp for the given grasp
  bool computePreGrasp(const GraspPlanningState *final_gps, GraspPlanningState **pre_gps);
  
  //! Sets the hand in the pre-grasp pose given the grasp
  virtual bool setPreGrasp(const GraspPlanningState *graspState);
public:
  //! Just a stub for now
  GraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                    db_planner::DatabaseManager *mgr, 
                    db_planner::TaskRecord rec);
  //! Removes the object that has been used from the sim world, but not the hand
  virtual ~GraspPlanningTask();
  //! Loads the hand and the object, initializes and starts a loop planner
  virtual void start();
public slots:
  //! Connected to the loopUpdate() signal of the planner
  void plannerLoopUpdate();
  //! Connected to the complete() signal of the planner
  void plannerComplete();
};

}

#endif
