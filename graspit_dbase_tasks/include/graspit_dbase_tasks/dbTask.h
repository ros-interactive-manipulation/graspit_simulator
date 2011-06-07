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
// $Id: taskDispatcher.h,v 1.3 2010/04/12 20:15:30 cmatei Exp $
//
//######################################################################

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
/*! Simply calls a timer and completes a few seconds after starting.
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
/*! Simply finishes immediately with success
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
