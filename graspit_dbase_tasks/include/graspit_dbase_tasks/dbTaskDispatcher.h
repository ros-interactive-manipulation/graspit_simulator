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

#ifndef _DB_TASK_DISPATCHER_H_
#define _DB_TASK_DISPATCHER_H_

#include <vector>
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

    The DBTaskDispatcher is a GraspIt! Plugin.
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

  //! A list of accepted task types. Empty if all task types are accepted. 
  std::vector<std::string> mAcceptedTaskTypes;

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
    mAcceptedTaskTypes.push_back(task_name);
  }

  //! Main operation loop, called periodically
  int mainLoop();
  
  //! Returns the status of the Dispatcher
  Status getStatus() const {return mStatus;}
};

} //namespace dbase_tasks

#endif
