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

#ifndef _DB_TASK_DISPATCHER_H_
#define _DB_TASK_DISPATCHER_H_

#include <string>
#include <map>

#include "plugin.h"

namespace db_planner {
  class DatabaseManager;
  class TaskRecord;
}

namespace graspit_dbase_tasks {

class DBTask;
class DBTaskDispatcher;

//! Functor interface for creating tasks
class DBTaskCreator
{
public:
  virtual DBTask* operator()(DBTaskDispatcher *disp, db_planner::DatabaseManager *mgr, db_planner::TaskRecord rec) = 0;
};

//! A high-level executive that dispatches tasks based on the contents of the database
/*! The DBTaskDispatcher is in charge of reading the list of tasks to be
    executed from the database, creating and startinng the appropriate
    instances of the DBTask class and then monitoring them.

    It can run either one-shot tasks (that do something and they're done) or event-based
    tasks which have their own callback system.

    On exit, the Dispatcher will exit GraspIt's main loop, thus terminating the
    application. Before doing that, it will set its own status to inform the
    main app of the outcome of the tasks. It can exit because:

    - it has performed a pre-set maximum number of tasks

    - no more tasks to be executed are listed in the database

    - there was an error communicating with the database

    Note that if a DBTask itself finishes with an error, the Dispatcher will mark that in the
    database, then proceed to the next task.
*/
class DBTaskDispatcher : public Plugin
{
 public:
  enum Status {READY, NO_TASK, ERROR, RUNNING, DONE};
 private:
  //! Map from task name in database to task creators
  std::map<std::string,DBTaskCreator*> mTaskCreators;

  //! The db mgr used to connect to the dbase
  db_planner::DatabaseManager *mDBMgr;

  //! The task currently being executed
  DBTask *mCurrentTask;
  
  //! The status of the Dispatcher
  Status mStatus;

  //! The number of tasks completed so far
  int mCompletedTasks;

  //! Max number of tasks to be completed. -1 means no max limit
  int mMaxTasks;

  //! Connects to the database. Returns 0 on success
  int connect(std::string host, int port, std::string username, 
              std::string password, std::string database);
  
public:
  DBTaskDispatcher();
  ~DBTaskDispatcher();

  //! Calls the database connection function
  int init(int argc, char **argv);
  
  //! Checks the status of the current task; cleans up after it if done
  void checkCurrentTask();

  //! Attempts to read a task from the dbase and start it
  void startNewTask();

  //! Register a new task creator for a task with the given name
  void registerTaskCreator(std::string task_name, DBTaskCreator* creator)
  {
    mTaskCreators[task_name] = creator;
  }

  //! Main operation loop, called periodically
  int mainLoop();
  
  //! Returns the status of the Dispatcher
  Status getStatus() const {return mStatus;}
};

} //namespace dbase_tasks

#endif
