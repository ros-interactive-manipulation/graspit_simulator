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

#ifndef _VELOGRASPPLANNINGTASK_H_
#define _VELOGRASPPLANNINGTASK_H_

#include <QObject>

#include <dbase_grasp_planner/grasp_planning_task.h>

namespace dbase_grasp_planner {

//! A minor specialization of the GraspPlanningTask for the Velo gripper.
/*! For now, the only change is in the way a pre-grasp is defined.   
 */
class VeloGraspPlanningTask : public GraspPlanningTask {
  Q_OBJECT
 private:
  
  virtual bool setPreGrasp(const GraspPlanningState *graspState);

public:
  //! Just a stub 
  VeloGraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                        db_planner::DatabaseManager *mgr, 
                        db_planner::TaskRecord rec);

  //! Just a stub
  virtual ~VeloGraspPlanningTask() {}

  //! Sets the appropriate save threshold for the planner
  virtual void start();
};

}

#endif
