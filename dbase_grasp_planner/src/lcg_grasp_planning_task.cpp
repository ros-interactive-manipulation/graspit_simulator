#include <dbase_grasp_planner/lcg_grasp_planning_task.h>

#include "robot.h"
#include "EGPlanner/searchState.h"
#include "EGPlanner/loopPlanner.h"
#include "debug.h"

namespace dbase_grasp_planner {

LCGGraspPlanningTask::LCGGraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                           db_planner::DatabaseManager *mgr, 
                                           db_planner::TaskRecord rec) : GraspPlanningTask(disp, mgr, rec) {}

void LCGGraspPlanningTask::start()
{
  GraspPlanningTask::start();
  if (mStatus == RUNNING) {
    static_cast<LoopPlanner*>(mPlanner)->setSaveThreshold(6.0);
  }
}

bool LCGGraspPlanningTask::setPreGrasp(const GraspPlanningState *graspState)
{
  //go back 10cm 
  double RETREAT_BY = -100;

  //place the mHand in position
  graspState->execute();  

  //go to LCG open pose
  std::vector<double> dof(mHand->getNumDOF(),0.0);
  dof[0] = dof[2] =  20.0 * M_PI / 180.0;
  dof[1] = dof[3] = -20.0 * M_PI / 180.0;
  std::vector<double> stepSize(mHand->getNumDOF(), M_PI/36.0);
  mHand->moveDOFToContacts(&dof[0], &stepSize[0], true, false);

  //check if move has succeeded
  for (int d=0; d<mHand->getNumDOF(); d++) {
    if ( fabs( dof[d] - mHand->getDOF(d)->getVal() ) > 1.0e-5) {
      DBGP("  LCG trying to open to " << dof[d] << "; only made it to " << mHand->getDOF(d)->getVal());
      DBGA("  open LCG fails");
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


