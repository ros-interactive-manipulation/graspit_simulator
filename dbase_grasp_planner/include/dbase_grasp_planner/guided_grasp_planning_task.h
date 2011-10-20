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
// $Id: graspPlanningTask.h,v 1.1 2009/10/08 16:13:11 cmatei Exp $
//
//######################################################################

#ifndef _GUIDEDGRASPPLANNINGTASK_H_
#define _GUIDEDGRASPPLANNINGTASK_H_

#include <QObject>

#include <graspit_dbase_tasks/dbTask.h>

class Hand;
class GraspableBody;
class GuidedPlanner;
class GraspPlanningState;

namespace dbase_grasp_planner {

class GuidedGraspPlanningTask : public QObject, public graspit_dbase_tasks::DBTask {
  Q_OBJECT
 private:
  //! The object we are planning on
  GraspableBody *mObject;
  //! The planner that we are using
  GuidedPlanner *mPlanner;
  //! The hand we are planning with
  Hand *mHand;
  //! The record of the actual planning task
  db_planner::PlanningTaskRecord mPlanningTask;
  //! The index of the last solution saved
  size_t mLastSolution;

  //! Saves a solution grasp to the database
  bool saveGrasp(const GraspPlanningState *pre_gps, const GraspPlanningState *final_gps);
  
public:
  //! Just a stub for now
  GuidedGraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                    db_planner::DatabaseManager *mgr, 
                    db_planner::TaskRecord rec);

  //! Removes the object that has been used from the sim world, but not the hand
  ~GuidedGraspPlanningTask();

  //! Loads the hand and the object, initializes and starts a guided planner
  virtual void start();

public slots:
  //! Connected to the update() signal of the planner
  void plannerUpdate();
  //! Connected to the complete() signal of the planner
  void plannerComplete();
};


}

#endif
