//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2009  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Matei T. Ciocarlie
//
// $Id: graspPlanningTask.cpp,v 1.3 2010/02/11 18:38:25 cmatei Exp $
//
//######################################################################

#include "dbase_grasp_planner/guided_grasp_planning_task.h"

#include <QString>

#include "mytools.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "body.h"
#include "EGPlanner/searchState.h"
#include "EGPlanner/guidedPlanner.h"
#include "DBPlanner/db_manager.h"

#include "graspit_db_grasp.h"
#include "graspit_db_model.h"

#include "debug.h"

namespace dbase_grasp_planner {

GuidedGraspPlanningTask::GuidedGraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                                                 db_planner::DatabaseManager *mgr, 
                                                 db_planner::TaskRecord rec) : 
  graspit_dbase_tasks::DBTask(disp, mgr, rec),
  mObject(NULL),
  mPlanner(NULL)
{
}

GuidedGraspPlanningTask::~GuidedGraspPlanningTask()
{
  //remove the planning object from the world, but do not delete it
  if (mObject) {
    mObject->getWorld()->destroyElement(mObject, false);
    //clean up the loaded geometry
    //the model itself is left around. we don't have a good solution for that yet
    static_cast<GraspitDBModel*>(mPlanningTask.model)->unload();
  }
  delete mPlanner;
}

void GuidedGraspPlanningTask::start()
{
  //get the details of the planning task itself
  if (!mDBMgr->GetPlanningTaskRecord(mRecord.taskId, &mPlanningTask)) {
    DBGA("Failed to get planning record for task");
    mStatus = ERROR;
    return;
  }

  World *world = graspItGUI->getIVmgr()->getWorld();

  //check if the currently selected hand is the same as the one we need
  //if not, load the hand we need
  if (world->getCurrentHand() && 
      GraspitDBGrasp::getHandDBName(world->getCurrentHand()) == QString(mPlanningTask.handName.c_str())) {
    DBGA("Grasp Planning Task: using currently loaded hand");
    mHand = world->getCurrentHand();
  } else {
    QString handPath = GraspitDBGrasp::getHandGraspitPath(QString(mPlanningTask.handName.c_str()));
    handPath = QString(getenv("GRASPIT")) + handPath;
    DBGA("Grasp Planning Task: loading hand from " << handPath.latin1());	      
    mHand = static_cast<Hand*>(world->importRobot(handPath));
    if ( !mHand ) {
      DBGA("Failed to load hand");
      mStatus = ERROR;
      return;
    }
  }
  //check for virtual contacts
  if (mHand->getNumVirtualContacts()==0) {
    DBGA("Specified hand does not have virtual contacts defined");
    mStatus = ERROR;
    return;
  }
  
  //load the object
  GraspitDBModel *model = static_cast<GraspitDBModel*>(mPlanningTask.model);
  if (model->load(world) != SUCCESS) {
    DBGA("Grasp Planning Task: failed to load model");
    mStatus = ERROR;
    return;
  }
  mObject = model->getGraspableBody();
  mObject->addToIvc();
  world->addBody(mObject);
  
  //initialize the planner
  GraspPlanningState seed(mHand);
  seed.setObject(mObject);
  seed.setPositionType(SPACE_AXIS_ANGLE);
  seed.setPostureType(POSE_EIGEN);
  seed.setRefTran(mObject->getTran());
  seed.reset();
  
  mPlanner = new GuidedPlanner(mHand);
  mPlanner->setModelState(&seed);	
  mPlanner->setContactType(CONTACT_PRESET);

  if (mPlanningTask.taskTime >= 0) mPlanner->setMaxTime(mPlanningTask.taskTime);
  else mPlanner->setMaxTime(-1);
  
  QObject::connect(mPlanner, SIGNAL(update()), this, SLOT(plannerUpdate()));
  QObject::connect(mPlanner, SIGNAL(complete()), this, SLOT(plannerComplete()));

  if (!mPlanner->resetPlanner()) {
    DBGA("Grasp Planning Task: failed to reset planner");
    mStatus = ERROR;
    return;
  }
  mLastSolution = 0;
  mPlanner->startPlanner();
  mStatus = RUNNING;
}

void GuidedGraspPlanningTask::plannerComplete()
{
  mStatus = DONE;
}

void GuidedGraspPlanningTask::plannerUpdate()
{
  while (mLastSolution +1 < mPlanner->getListSize() )
  {
    DBGA("New solution!");
    if (mLastSolution + 2 > mPlanner->getListSize())
    {
      DBGA("Error, expected even number of solutions");
      mStatus = ERROR;
      return;
    }
    if ( !saveGrasp(mPlanner->getGrasp(mLastSolution), mPlanner->getGrasp(mLastSolution+1)) )
    {
      DBGA("Error saving grasp");
      mStatus = ERROR;
      return;
    }
    mLastSolution += 2;
  }
}

bool GuidedGraspPlanningTask::saveGrasp(const GraspPlanningState *pre_gps, const GraspPlanningState *final_gps)
{
  GraspitDBModel* dbModel= mObject->getDBModel();
  assert(dbModel);
  
  db_planner::Grasp* grasp = new db_planner::Grasp;
  
  std::vector<double> contacts = grasp->GetContacts();
  
  grasp->SetSourceModel( *(static_cast<db_planner::Model*>(dbModel)) );
  grasp->SetHandName(GraspitDBGrasp::getHandDBName(mHand).toStdString());
  grasp->SetEpsilonQuality(final_gps->getEpsilonQuality());
  grasp->SetVolumeQuality(final_gps->getVolume());
  grasp->SetEnergy(final_gps->getEnergy());
  grasp->SetClearance(0.0);
  grasp->SetClusterRep(false);
  grasp->SetCompliantCopy(false);
  grasp->SetSource("EIGENGRASPS");
  
  std::vector<double> tempArray;

  //the final grasp
  GraspPlanningState *sol = new GraspPlanningState(final_gps);
  //make sure you pass it sticky=true
  sol->setPositionType(SPACE_COMPLETE,true);
  sol->setPostureType(POSE_DOF,true);
  tempArray.clear();
  for(int i = 0; i < sol->readPosture()->getNumVariables(); ++i){
    tempArray.push_back(sol->readPosture()->readVariable(i));
  }
  grasp->SetFinalgraspJoints(tempArray);
  tempArray.clear();
  for(int i = 0; i < sol->readPosition()->getNumVariables(); ++i){
    tempArray.push_back(sol->readPosition()->readVariable(i));
  }
  grasp->SetFinalgraspPosition(tempArray);
  delete sol;

  //the pre-grasp
  sol = new GraspPlanningState(pre_gps);
  sol->setPositionType(SPACE_COMPLETE,true);
  sol->setPostureType(POSE_DOF,true);
  tempArray.clear();
  for(int i = 0; i < sol->readPosture()->getNumVariables(); ++i){
    tempArray.push_back(sol->readPosture()->readVariable(i));
  }
  grasp->SetPregraspJoints(tempArray);
  tempArray.clear();
  for(int i = 0; i < sol->readPosition()->getNumVariables(); ++i){
    tempArray.push_back(sol->readPosition()->readVariable(i));
  }
  grasp->SetPregraspPosition(tempArray);
  delete sol;
  
  //contacts
  //for some reason, the grasp's contact vector gets initialized to a mess!
  tempArray.clear();
  grasp->SetContacts(tempArray);
  
  std::vector<db_planner::Grasp*> graspList;
  graspList.push_back(grasp);
  
  bool result = mDBMgr->SaveGrasps(graspList);
  delete grasp;
  return result;
}

}
