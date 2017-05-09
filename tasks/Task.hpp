#ifndef SIMULATION_VREP_TASK_TASK_HPP
#define SIMULATION_VREP_TASK_TASK_HPP

#include "simulation_vrep/TaskBase.hpp"
#include <vrep/vrep.hpp>

namespace simulation_vrep
{
    enum MOTOR_NAMES
    {
      DFL,
      DFR,
      DCL,
      DCR,
      DBL,
      DBR,
      SFL,
      SFR,
      SBL,
      SBR,
      WFL,
      WFR,
      WCL,
      WCR,
      WBL,
      WBR,
      GDR,
      GST,
      GWW
    };

    class Task : public TaskBase
    {
  	friend class TaskBase;
    protected:
        vrep::VREP *vrep;

        // Handlers for motors
        static const int joints_number = 19;
        std::vector<int> joints_handles;
        std::vector<std::string> joints_names;
        base::samples::RigidBodyState pose;

        base::Waypoint goalWaypoint;

        base::commands::Motion2D motion_command;
        base::samples::Joints joints_commands;
        base::samples::Joints joints_readings;

        std::vector<base::Waypoint> trajectory;
        base::Waypoint currentWaypoint;

    public:
        Task(std::string const& name = "simulation_vrep::Task");
        Task(std::string const& name, RTT::ExecutionEngine* engine);
	       ~Task();
        bool configureHook();
        bool startHook();
        void updateHook();
        void errorHook();
        void stopHook();
        void cleanupHook();
    };
}

#endif
