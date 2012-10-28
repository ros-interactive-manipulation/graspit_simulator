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
  static_cast<LoopPlanner*>(mPlanner)->setSaveThreshold(10.0);
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
  //go back 10cm 
  double RETREAT_BY = -100;

  //place the mHand in position
  graspState->execute();  

  mHand->autoGrasp(false, -1.0);
  //check if all DOF's are at the min or max of their respective range of travel
  //recall that we are opening, i.e. going against the default velocity
  for (int d=0; d<mHand->getNumDOF(); d++) {
    if (mHand->getDOF(d)->getDefaultVelocity() > 0 && 
        fabs( mHand->getDOF(d)->getVal() - mHand->getDOF(d)->getMin() ) > 1.0e-5 )
    {
      DBGP("  dof " << d << " not at min");
      return false;
    }
    else if (mHand->getDOF(d)->getDefaultVelocity() < 0 && 
             fabs( mHand->getDOF(d)->getVal() - mHand->getDOF(d)->getMax() ) > 1.0e-5 )
    {
      DBGP("  dof " << d << " not at max");
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
  grasp->SetTableClearance(0.0);
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
