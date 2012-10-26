#ifndef _LCGGRASPPLANNINGTASK_H_
#define _LCGGRASPPLANNINGTASK_H_

#include <QObject>

#include <dbase_grasp_planner/grasp_planning_task.h>

namespace dbase_grasp_planner {

//! Plans grasps for the LCG
/*! For now, the only change is in the way a pre-grasp is defined.   
 */
class LCGGraspPlanningTask : public GraspPlanningTask {
  Q_OBJECT
 private:
  
  virtual bool setPreGrasp(const GraspPlanningState *graspState);

public:
  //! Just a stub 
  LCGGraspPlanningTask(graspit_dbase_tasks::DBTaskDispatcher *disp, 
                    db_planner::DatabaseManager *mgr, 
                    db_planner::TaskRecord rec);

  //! Just a stub
  virtual ~LCGGraspPlanningTask() {}

  //! Sets the appropriate save threshold for the planner
  virtual void start();
};

}

#endif
