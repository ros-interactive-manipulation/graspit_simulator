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

#include "dbase_grasp_planner/grasp_clustering_task.h"

#include <algorithm>

#include "world.h"
#include "robot.h"
#include "graspitGUI.h"
#include "matvec3D.h"
#include "EGPlanner/searchState.h"
#include "DBPlanner/db_manager.h"
#include "graspit_db_grasp.h"
#include "debug.h"

namespace dbase_grasp_planner {

GraspClusteringTask::GraspClusteringTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                         db_planner::DatabaseManager *mgr, 
                                         db_planner::TaskRecord rec) : DBTask (disp, mgr, rec)
{
  //nothing so far
}

/*! Will also load the hand, even though the hand is not explicitly used. It is needed for 
  the grasp allocator, plus it might be needed for retrieving DOF values, for example if 
  the grasp is stored in the database as eigengrasp values.
*/
void GraspClusteringTask::start()
{
  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mRecord.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task");
    mStatus = ERROR;
    return;
  }

  World *world = graspItGUI->getIVmgr()->getWorld();
  Hand *hand;
 
  //check if the currently selected hand is the same as the one we need
  //if not, load the hand
  if (world->getCurrentHand() && world->getCurrentHand()->getDBName() == QString(mPlanningTask.handName.c_str())) {
    DBGA("Grasp Clustering Task: using currently loaded hand");
    hand = world->getCurrentHand();
  } else {
    QString handPath =  mDBMgr->getHandGraspitPath(QString(mPlanningTask.handName.c_str()));
    handPath = QString(getenv("GRASPIT")) + handPath;
    DBGA("Grasp Clustering Task: loading hand from " << handPath.latin1());	      
    hand = static_cast<Hand*>(world->importRobot(handPath));
    if ( !hand ) {
      DBGA("Failed to load hand");
      mStatus = ERROR;
      return;
    }
  }
  mDBMgr->SetGraspAllocator(new GraspitDBGraspAllocator(hand));

  //load all the grasps
  std::vector<db_planner::Grasp*> graspList;
  if(!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList)){
    DBGA("Load grasps failed");
    mStatus = ERROR;
    while (!graspList.empty()) {
      delete graspList.back();
      graspList.pop_back();
    }
    return;
  }

  //sort grasps by energy (hard-coded in, maybe later we'll allow other sorting)
  std::sort(graspList.begin(), graspList.end(), db_planner::Grasp::CompareEnergy);

  //if all goes well, we are done
  mStatus = DONE;
  int clusters = 0;
  DBGA("Clustering " << graspList.size() << " grasps");

  while (!graspList.empty()) {
    //pop the front (best grasp)
    db_planner::Grasp* repGrasp = graspList.front();
    graspList.erase(graspList.begin());

    //compliant_copy grasps are ignored by clustering tasks
    if (repGrasp->CompliantCopy()) {
      delete repGrasp;
      continue;
    }

    //mark it as cluster center in the database
    if (!mDBMgr->SetGraspClusterRep(repGrasp, true)) {
      DBGA("Failed to mark cluster rep in database");
      mStatus = ERROR;
      delete repGrasp;
      break;
    }
    clusters++;

    //find other grasps in its cluster
    int cloud=0;
    std::vector<db_planner::Grasp*>::iterator it = graspList.begin();
    while(it!=graspList.end()) {

      //compliant_copy grasps are ignored by clustering tasks
      if ( !(*it)->CompliantCopy() &&  
           clusterGrasps(static_cast<GraspitDBGrasp*>(repGrasp), static_cast<GraspitDBGrasp*>(*it)) ) {
	(*it)->SetClusterRep(false);
	//mark it as non-center in the database
	if (!mDBMgr->SetGraspClusterRep(*it, false)) {
	  DBGA("Failed to mark non-cluster rep in database");
	  mStatus = ERROR;
	  break;
	}
	cloud++;
	delete *it;
	it = graspList.erase(it);
      } else {
	it++;
      }

    }
    DBGA("  Marked cluster of size " << cloud);
    delete repGrasp;
    if (mStatus == ERROR) break;
  }  
  while (!graspList.empty()) {
    delete graspList.back();
    graspList.pop_back();
  }
  DBGA("Successfully marked " << clusters << " clusters");
}

/*! Only looks at the relative distance between gripper positions for the final grasps.
  Returns true if both of the follosing are true:
  - translation between gripper locations is less than DISTANCE_THRESHOLD
  - rotation angles between gripper locations is less than ANGULAR_THRESHOLD.

  Both thresholds are hard-coded in here.
 */
bool GraspClusteringTask::clusterGrasps(const GraspitDBGrasp *g1, const GraspitDBGrasp *g2)
{
  //2 cm distance threshold
  double DISTANCE_THRESHOLD = 20;
  //30 degrees angular threshold
  double ANGULAR_THRESHOLD = 0.52;
  
  transf t1 = g1->getHand()->getApproachTran() * g1->getFinalGraspPlanningState()->getTotalTran();
  transf t2 = g2->getHand()->getApproachTran() * g2->getFinalGraspPlanningState()->getTotalTran();

  vec3 dvec = t1.translation() - t2.translation();
  double d = dvec.len();
  if (d > DISTANCE_THRESHOLD) return false;
  
  Quaternion qvec = t1.rotation() * t2.rotation().inverse();
  vec3 axis; double angle;
  qvec.ToAngleAxis(angle,axis);
  if (angle >  M_PI) angle -= 2*M_PI;
  if (angle < -M_PI) angle += 2*M_PI;
  if (fabs(angle) > ANGULAR_THRESHOLD) return false;
  
  return true;
}

}
