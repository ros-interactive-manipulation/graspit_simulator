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

#include <sstream>
#include <cassert>

#include "DBPlanner/ros_database_manager.h"
#include "graspitGUI.h"
#include "graspit_db_model.h"
#include "debug.h"

#include "graspit_dbase_tasks/dbTask.h"

namespace graspit_dbase_tasks {

DBTaskDispatcher::DBTaskDispatcher() : mDBMgr(NULL) , mCurrentTask(NULL), mStatus(READY)
{
  //hard-coded for now
  mMaxTasks = -1;
  mCompletedTasks = 0;
}

DBTaskDispatcher::~DBTaskDispatcher()
{
  if (mCurrentTask) {
    //this should not really happen
    DBGA("Dispatcher: deleting current task on cleanup");
    delete mCurrentTask;
  }
  delete mDBMgr;
}

int DBTaskDispatcher::connect(std::string host, int port, std::string username, 
                              std::string password, std::string database) 
{
  delete mDBMgr;
  std::ostringstream port_str;
  port_str << port;

  //careful: we pass a null for the grasp allocator as we don't know yet which hand we'll be using
  mDBMgr = new db_planner::RosDatabaseManager(host, port_str.str(), username, password, database, NULL, NULL);
  //use the special allocator for models that get geometry directly from the database  
  GeomGraspitDBModelAllocator* allocator = new GeomGraspitDBModelAllocator(mDBMgr);
  mDBMgr->SetModelAllocator(allocator);
  
  if (!mDBMgr->isConnected()) {
    DBGA("DBase operator: Connection failed");
    delete mDBMgr; mDBMgr = NULL;
    return -1;
  }
  return 0;
}

int DBTaskDispatcher::init(int argc, char **argv)
{
  //in the future, we'll get the connection params from arguments
  return connect("10.0.0.81",5432,"willow","willow","household_objects");
}

/*! Gets a new task from the database and starts it. Possible outcomes:
  - no more tasks in database; sets status to NO_TASK
  - max number of tasks exceeded; sets status to DONE
  - error in reading the task; sets status to ERROR
  - error in starting the task; sets status to READY
  - task has started and needs us to surrender control; sets status to RUNNING
  - task is finished in one shot; sets status to READY
*/
void DBTaskDispatcher::startNewTask()
{
  assert(!mCurrentTask);
  // check if we have completed the max number of tasks
  if (mMaxTasks >= 0 && mCompletedTasks >= mMaxTasks) {
    mStatus = DONE;
    return;
  }
  db_planner::TaskRecord rec;
  if (!mDBMgr->AcquireNextTask(&rec, mAcceptedTaskTypes)) {
    DBGA("Dispatcher: error reading next task");
    mStatus = ERROR;
    return;
  }
  //task type 0 is reserved for no task to do
  if (rec.taskType.empty()) {
    DBGA("Dispatcher: no tasks to be executed");
    mStatus = NO_TASK;
    return;
  }
  //check if we have a creator for tasks of this type
  std::map<std::string,DBTaskCreator*>::iterator it = mTaskCreators.find(rec.taskType);
  if (it==mTaskCreators.end()) 
  {
    DBGA("Dispatcher: no creator available for task type: " << rec.taskType);
    mStatus = ERROR;
    return;
  }
  //create the new task
  mCurrentTask = (*(it->second))(this, mDBMgr,rec);
  if (!mCurrentTask) {
    DBGA("Dispatcher: could not start task of type: " << rec.taskType);
    mStatus = ERROR;
    return;
  }
  //start the next task
  mCurrentTask->start();
}

/*! Checks on the current task; if it is finished, cleans up after it and marks the 
  result in the database.

  If task is finished, sets status to READY, unless there is an error marking the 
  finished task in the database, in which case status is set to ERROR.

  Nore that even if the task finishes with an error, the dispatcher will be READY
  for the next task (not abort altogether). The task that had an error is marked
  as such in the database. However, if there is an error in communicating with 
  the database, the dispatcher will abort altogether.
*/
void DBTaskDispatcher::checkCurrentTask()
{
  assert(mCurrentTask);
  switch (mCurrentTask->getStatus()) {
  case DBTask::RUNNING:
    DBGP("Task running");
    mStatus = RUNNING;
    mCurrentTask->mainLoop();
    break;
  case DBTask::ERROR:
    DBGP("Task error");
    mStatus = READY;
    if (!mDBMgr->SetTaskStatus(mCurrentTask->getRecord().taskId,"ERROR")) {
      DBGA("Dispatcher: error marking completed task");
      mStatus = ERROR;
    }
    delete mCurrentTask; mCurrentTask = NULL;
    break;
  case DBTask::DONE:
    DBGP("Task done");
    mStatus = READY;
    mCompletedTasks++;
    if (!mDBMgr->SetTaskStatus(mCurrentTask->getRecord().taskId,"COMPLETED")) {
      DBGA("Dispatcher: error marking completed task");
      mStatus = ERROR;
    }
    delete mCurrentTask; mCurrentTask = NULL;
    break;
  default:
    DBGA("Dispatcher: Unknown task state");
    mStatus = ERROR;
  }
}

/*! Will start tasks as long as there are tasks to be run. If the tasks are of the one-shot type,
  it just loops in here as long as it has tasks. If the task is event-based and needs us to 
  surrender control, it will surrender control but schedule the timer to come back here and 
  check on the task later.
*/
int DBTaskDispatcher::mainLoop()
{
  if(mCurrentTask) {
    checkCurrentTask();
  }
  if (mStatus == READY) {
    startNewTask();
  }
  switch(mStatus) {
  case DONE:
    graspItGUI->exitMainLoop();		
    return 1;
  case ERROR:
    graspItGUI->exitMainLoop();		
    return 1;
  case NO_TASK:
    graspItGUI->exitMainLoop();		
    return 0;
  case RUNNING:
    break;
  case READY:
    break;
  }
  return 0;
}

} //namespace
