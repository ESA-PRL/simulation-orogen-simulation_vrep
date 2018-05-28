#ifndef SIMULATION_VREP_TASK_TASK_HPP
#define SIMULATION_VREP_TASK_TASK_HPP

#include "simulation_vrep/TaskBase.hpp"
#include <base/commands/Motion2D.hpp>
#include <vrep/vrep.hpp>
#include <time.h>

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

        int num_motors;
        int num_joints;
        std::vector<int> joints_handles;
        std::vector<int> ptu_handles;
        std::vector<std::string> joint_vrep_names;
        std::vector<std::string> joint_readings_names;
        std::vector<std::string> ptu_vrep_names;
        std::vector<std::string> ptu_readings_names;
        base::samples::RigidBodyState pose;
        std::vector<float> trajectoryData;
        float joints_position, joints_speed;
        int roverPoseHandle, goalMarkerHandle;
        float position[3] = {0};
        float orientation[3] = {0};
        float q[3] = {0};
        int time;
        base::Time t0;
        base::Time t1;
        clock_t elapsed_time;
        clock_t total_time;

        base::Waypoint goalWaypoint;

        base::commands::Motion2D motion_command;
        base::samples::Joints joints_commands;
        base::samples::Joints ptu_commands;
        base::samples::Joints old_joints_commands;
        base::samples::Joints joints_readings;
        base::samples::Joints motors_readings;
        base::samples::Joints ptu_readings;

        std::vector<base::Waypoint> trajectory;
        base::Waypoint currentWaypoint;
        bool first_iteration;

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
