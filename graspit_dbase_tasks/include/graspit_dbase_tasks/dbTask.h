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

#ifndef _DB_TASK_H_
#define _DB_TASK_H_

#include <iostream>

#include "DBPlanner/task.h"

namespace db_planner {
  class DatabaseManager;
}

namespace graspit_dbase_tasks {

class DBTaskDispatcher;

//! A task is a database record of some experiment that we want to run
/*! Usually, a task will involve an object, a hand, and some sort of
    experiment, such as planning grasps and recording them in the database.
    This class encapsulates a task, plus it offers some connections so that
    the task can be run automatically by a DBTaskDispatcher.

    A DBTask is responsible for its own event management. It can use a callback
    system (i.e. surrender control and then get it back later via its own timers
    or callbacks). For example, this can be achieved by using one of the EGPlanners,
    which do that natively.

    A DBTask must implement the following interface:
    
    - at the beginning, the DBTaskDispatcher will call the start() fctn. The 
    DBTask must start its own event management from there. 
    
    - if the DBTask is one-shot, it can do all the work in start() and set its 
    status to either DONE or ERROR at the end. The Dispatcher will see this, 
    clean up and go to the next Task.

    - if the DBTask is event based, start() must set up the DBTask event loop, then 
    leave mStatus = RUNNING, and then surrender control. The Dispatcher will in
    turn surrender control and let the DBTask event loop do its thing.

    - when the DBTask is done, it is responsible for setting its own mStatus to
    DONE. The DBTaskDispatcher wakes up periodically and checks on this. When it
    sees that the status of the DBTask is DONE it will clean up and continue to
    another task.
*/
class DBTask {
 public:
  enum Status{RUNNING, ERROR, DONE};
 protected:
  //! The current status of this task
  Status mStatus;

  //! The Dispatcher that started this task
  DBTaskDispatcher* mDispatcher;

  //! A database mgr that can be used to access the dbase
  /*! Acquired from the DBTaskDispatcher */
  db_planner::DatabaseManager *mDBMgr;

  //! The dbase record of the task being executed
  db_planner::TaskRecord mRecord;
  
public:
  DBTask(DBTaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) 
    : mDispatcher(disp), mDBMgr(mgr), mRecord(rec) {};

  virtual ~DBTask(){}

  virtual void start() = 0;

  virtual void mainLoop() {}

  //! Returns the current status of this task
  Status getStatus(){return mStatus;}

  //! Returns a copy of the dbase record of this task
  db_planner::TaskRecord getRecord(){return mRecord;}
};

//! An empty task used to test and debug the dispatcher
/*! Simply gets started in start() and immediately succeeds as soon as mainLoop() is called.
*/
class DBEmptyTask : public DBTask
{
 public:
  DBEmptyTask(DBTaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) :
    DBTask(disp, mgr, rec) {}
  ~DBEmptyTask(){}
  void start(){std::cout << "Started simple task\n"; mStatus = RUNNING;}
  void mainLoop(){std::cout << "Finished simple task\n"; mStatus = DONE;}
};

//! An empty one-shot task used to test and debug the dispatcher
/*! Simply finishes immediately with success.
*/
class DBEmptyOneShotTask : public DBTask
{
 public:
  DBEmptyOneShotTask(DBTaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) :
    DBTask(disp, mgr, rec) {}
  ~DBEmptyOneShotTask(){}
  void start(){std::cout << "Started and finished one-shot simple task\n"; mStatus = DONE;}
};

}

#endif
