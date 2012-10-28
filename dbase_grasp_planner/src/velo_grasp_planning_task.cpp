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

#include <dbase_grasp_planner/velo_grasp_planning_task.h>

#include "robot.h"
#include "EGPlanner/searchState.h"
#include "EGPlanner/loopPlanner.h"
#include "debug.h"

namespace dbase_grasp_planner {

VeloGraspPlanningTask::VeloGraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                           db_planner::DatabaseManager *mgr, 
                                           db_planner::TaskRecord rec) : GraspPlanningTask(disp, mgr, rec) {}

void VeloGraspPlanningTask::start()
{
  GraspPlanningTask::start();
  if (mStatus == RUNNING) {
    static_cast<LoopPlanner*>(mPlanner)->setSaveThreshold(6.0);
  }
}

bool VeloGraspPlanningTask::setPreGrasp(const GraspPlanningState *graspState)
{
  //go back 10cm 
  double RETREAT_BY = -100;

  //place the mHand in position
  graspState->execute();  

  //go to Velo open pose
  std::vector<double> dof(mHand->getNumDOF(),0.0);
  dof[0] = dof[2] =  20.0 * M_PI / 180.0;
  dof[1] = dof[3] = -20.0 * M_PI / 180.0;
  std::vector<double> stepSize(mHand->getNumDOF(), M_PI/36.0);
  mHand->moveDOFToContacts(&dof[0], &stepSize[0], true, false);

  //check if move has succeeded
  for (int d=0; d<mHand->getNumDOF(); d++) {
    if ( fabs( dof[d] - mHand->getDOF(d)->getVal() ) > 1.0e-5) {
      DBGP("  Velo trying to open to " << dof[d] << "; only made it to " << mHand->getDOF(d)->getVal());
      DBGA("  open Velo fails");
      return false;
    }
  }

  //retreat along approach direction
  if (mHand->approachToContact(RETREAT_BY, false)) {
    //we have hit something
    DBGA("  retreat fails");
    return false;
  }

  return true;
}

}


