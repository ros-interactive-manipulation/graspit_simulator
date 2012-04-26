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

#include "dbase_grasp_planner/grasp_planning_task.h"

#include <QString>

#include "mytools.h"
#include "graspitGUI.h"
#include "ivmgr.h"
#include "world.h"
#include "robot.h"
#include "body.h"
#include "EGPlanner/searchState.h"
#include "EGPlanner/loopPlanner.h"
#include "DBPlanner/db_manager.h"

#include "graspit_db_grasp.h"
#include "graspit_db_model.h"

#include "debug.h"

namespace dbase_grasp_planner {

GraspPlanningTask::GraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
				     db_planner::DatabaseManager *mgr, 
				     db_planner::TaskRecord rec) : 
  graspit_dbase_tasks::DBTask(disp, mgr, rec),
  mObject(NULL),
  mPlanner(NULL)
{
}

GraspPlanningTask::~GraspPlanningTask()
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

void GraspPlanningTask::start()
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
      world->getCurrentHand()->getDBName() == QString(mPlanningTask.handName.c_str())) {
    DBGA("Grasp Planning Task: using currently loaded hand");
    mHand = world->getCurrentHand();
  } else {
    QString handPath = mDBMgr->getHandGraspitPath(QString(mPlanningTask.handName.c_str()));
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
  
  mPlanner = new LoopPlanner(mHand);
  QObject::connect(mPlanner, SIGNAL(loopUpdate()), this, SLOT(plannerLoopUpdate()));
  QObject::connect(mPlanner, SIGNAL(complete()), this, SLOT(plannerComplete()));
	
  mPlanner->setEnergyType(ENERGY_CONTACT);
  mPlanner->setContactType(CONTACT_PRESET);
  mPlanner->setMaxSteps(65000);
  mPlanner->setRepeat(true);
  //max time set from database record
  if (mPlanningTask.taskTime >= 0){
    mPlanner->setMaxTime(mPlanningTask.taskTime);
  } else {
    mPlanner->setMaxTime(-1);
  }
  static_cast<SimAnnPlanner*>(mPlanner)->setModelState(&seed);
  
  if (!mPlanner->resetPlanner()) {
    DBGA("Grasp Planning Task: failed to reset planner");
    mStatus = ERROR;
    return ;
  }
  
  //load all already known grasps so that we avoid them in current searches
  mDBMgr->SetGraspAllocator(new GraspitDBGraspAllocator(mHand));
  std::vector<db_planner::Grasp*> graspList;
  if(!mDBMgr->GetGrasps(*(mPlanningTask.model), mPlanningTask.handName, &graspList)){
    //for now, we don't know if this means "no grasps found" or "error" so we assume the first
    DBGA("No grasps found in database for model " << mPlanningTask.model->ModelName());
  } 
  //and pass them on to the planner
  for (size_t i=0; i<graspList.size(); i++) {
    GraspPlanningState *state = new GraspPlanningState(static_cast<GraspitDBGrasp*>(graspList[i])			
						       ->getFinalGraspPlanningState() );
    state->setObject(mObject);
    state->setPositionType(SPACE_AXIS_ANGLE, true);
    //careful here - is it the same posture in both spaces?
    state->setPostureType(POSE_EIGEN, true);
    static_cast<LoopPlanner*>(mPlanner)->addToAvoidList(state);
  }
  while (!graspList.empty()) {
    delete graspList.back();
    graspList.pop_back();
  }
  mLastSolution = mPlanner->getListSize();
  DBGA("Planner starting off with " << mLastSolution << " solutions");
  mPlanner->startPlanner();
  mStatus = RUNNING;
}

void GraspPlanningTask::plannerComplete()
{
  //save solutions that have accumulated in last loop
  plannerLoopUpdate();
  //finish
  mStatus = DONE;
}

void GraspPlanningTask::plannerLoopUpdate()
{
  if (mStatus != RUNNING) return;
  //save all new solutions to database
  for(int i=mLastSolution; i<mPlanner->getListSize(); i++) {
    //copy the solution so we can change it
    GraspPlanningState *final_gps = new GraspPlanningState(mPlanner->getGrasp(i));
    GraspPlanningState *pre_gps;
    if (!computePreGrasp(final_gps, &pre_gps)) continue;
    if (!saveGrasp(pre_gps, final_gps)) {
      DBGA("Error saving grasp");
      mStatus = ERROR;
      break;
    }
  }
  if (mStatus == ERROR) {
    // this is a bit of a hack, but ensures that the planner will stop 
    // as soon as it attempts to take another step. If we specifically start
    // the planner from in here, it causes problem, as this is called from inside
    // the planner callback
    mPlanner->setMaxSteps(0);
  } else {
    DBGA(mPlanner->getListSize() - mLastSolution << " solutions saved to database");
  }
  mLastSolution = mPlanner->getListSize();
}

bool GraspPlanningTask::computePreGrasp(const GraspPlanningState *final_gps,
                                        GraspPlanningState **pre_gps)
{
  if (!setPreGrasp(final_gps)) {
    DBGA("Pre-grasp creation fails");
    return false;
  }
  
  //sanity check
  if (!mHand->getWorld()->noCollision()) {
    DBGA("Collision detected for pre-grasp!");
    return false;
  }

  //create pre-grasp
  *pre_gps = new GraspPlanningState(mHand);
  (*pre_gps)->setPostureType(POSE_DOF, false);
  (*pre_gps)->setPositionType(SPACE_COMPLETE, false);
  (*pre_gps)->setRefTran(mObject->getTran(), false);
  (*pre_gps)->saveCurrentHandState();

  return true;				
}

bool GraspPlanningTask::setPreGrasp(const GraspPlanningState *graspState)
{
  //place the mHand in position
  graspState->execute();  

  //the entire range of motion for the pr2_gripper_2010
  //also, positive change means open gripper for pr2_gripper
  double OPEN_BY = 0.323;
  //go back 10cm 
  double RETREAT_BY = -100;

  //open fingers
  std::vector<double> dof(mHand->getNumDOF(),0.0);
  std::vector<double> stepSize(mHand->getNumDOF(), 0.0);
  mHand->getDOFVals(&dof[0]);
  for (int d=0; d<mHand->getNumDOF(); d++) {
    dof[d] += OPEN_BY;
    if (dof[d] < mHand->getDOF(d)->getMin()) {
      dof[d] = mHand->getDOF(d)->getMin();
    }
    if (dof[d] > mHand->getDOF(d)->getMax()) {
      dof[d] = mHand->getDOF(d)->getMax();
    }
    stepSize[d] = M_PI/36.0;
  }
  mHand->moveDOFToContacts(&dof[0], &stepSize[0], true, false);

  //check if move has succeeded
  for (int d=0; d<mHand->getNumDOF(); d++) {
    if ( fabs( dof[d] - mHand->getDOF(d)->getVal() ) > 1.0e-5) {
      DBGP("  trying to open to " << dof[d] << "; only made it to " << mHand->getDOF(d)->getVal());
      DBGA("  open gripper fails");
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

bool GraspPlanningTask::saveGrasp(const GraspPlanningState *pre_gps, 
                                  const GraspPlanningState *final_gps)
{
  GraspitDBModel* dbModel= mObject->getDBModel();
  assert(dbModel);
  
  db_planner::Grasp* grasp = new db_planner::Grasp;
  
  std::vector<double> contacts = grasp->GetContacts();
  
  grasp->SetSourceModel( *(static_cast<db_planner::Model*>(dbModel)) );
  grasp->SetHandName(mHand->getDBName().toStdString());
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
