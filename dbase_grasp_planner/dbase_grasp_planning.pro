TEMPLATE	= lib
LANGUAGE	= C++
CONFIG += qt plugin 
QT += qt3support
DESTDIR = lib
MOC_DIR = build
OBJECTS_DIR = build

DEFINES += CGDB_ENABLED

INCLUDEPATH += include 
DEPENDPATH += src include include/dbase_grasp_planner

HEADERS += include/dbase_grasp_planner/grasp_clustering_task.h \
           include/dbase_grasp_planner/grasp_planning_task.h \
           include/dbase_grasp_planner/guided_grasp_planning_task.h \
           include/dbase_grasp_planner/lcg_grasp_planning_task.h

SOURCES += src/dbase_planner_plugin.cpp \
           src/grasp_clustering_task.cpp \
           src/grasp_planning_task.cpp \
           src/guided_grasp_planning_task.cpp \
           src/lcg_grasp_planning_task.cpp

GRASPIT_CFLAGS = $$system(rospack export --lang=cpp --attrib=cflags graspit)
QMAKE_CXXFLAGS += $$GRASPIT_CFLAGS

DBASE_TASK_CFLAGS = $$system(rospack export --lang=cpp --attrib=cflags graspit_dbase_tasks)
QMAKE_CXXFLAGS += $$DBASE_TASK_CFLAGS

DBASE_TASK_LFLAGS = $$system(rospack export --lang=cpp --attrib=lflags graspit_dbase_tasks)
QMAKE_LFLAGS += $$DBASE_TASK_LFLAGS
