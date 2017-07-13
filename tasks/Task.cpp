#include "Task.hpp"
#include <math.h>


using namespace simulation_vrep;

Task::Task(std::string const& name):
    TaskBase(name)
{
}

Task::Task(std::string const& name, RTT::ExecutionEngine* engine):
    TaskBase(name, engine)
{
}

Task::~Task()
{
}

bool Task::configureHook()
{
    if(!TaskBase::configureHook())
        return false;

    vrep = new vrep::VREP();

    if(vrep->getClientId() == -1)
    {
        printf("Could not reach VREP simulation, make sure the simulation is running in VREP\n");
        return false;
    }

    joints_names.push_back("joint_drive_fl");
    joints_names.push_back("joint_drive_fr");
    joints_names.push_back("joint_drive_ml");
    joints_names.push_back("joint_drive_mr");
    joints_names.push_back("joint_drive_rl");
    joints_names.push_back("joint_drive_rr");
    joints_names.push_back("joint_steer_fl");
    joints_names.push_back("joint_steer_fr");
    joints_names.push_back("joint_steer_rl");
    joints_names.push_back("joint_steer_rr");
    joints_names.push_back("joint_walk_fl");
    joints_names.push_back("joint_walk_fr");
    joints_names.push_back("joint_walk_ml");
    joints_names.push_back("joint_walk_mr");
    joints_names.push_back("joint_walk_rl");
    joints_names.push_back("joint_walk_rr");
    joints_names.push_back("bogie_lateral_l");
    joints_names.push_back("bogie_lateral_r");
    joints_names.push_back("bogie_rear1");

    joints_readings.resize(joints_number);
    motors_readings.resize(motors_number);

    joints_readings.names[0] = "left_passive";
    joints_readings.names[1] = "right_passive";
    joints_readings.names[2] = "rear_passive";
    joints_readings.names[3] = "fl_walking";
    joints_readings.names[4] = "fr_walking";
    joints_readings.names[5] = "ml_walking";
    joints_readings.names[6] = "mr_walking";
    joints_readings.names[7] = "rl_walking";
    joints_readings.names[8] = "rr_walking";
    joints_readings.names[9] = "fl_steer";
    joints_readings.names[10] = "fr_steer";
    joints_readings.names[11] = "rl_steer";
    joints_readings.names[12] = "rr_steer";
    joints_readings.names[13] = "fl_drive";
    joints_readings.names[14] = "fr_drive";
    joints_readings.names[15] = "ml_drive";
    joints_readings.names[16] = "mr_drive";
    joints_readings.names[17] = "rl_drive";
    joints_readings.names[18] = "rr_drive";

    int handle;
    for(int i = 0; i < joints_number; i++)
    {
        vrep->getObjectHandle(joints_names[i], &handle);
        joints_handles.push_back(handle);
    }

    vrep->getObjectHandle("Pose", &roverPoseHandle);
    vrep->getObjectHandle("GOAL_marker", &goalMarkerHandle);

    std::string const message("Enabling Synchronization");
    vrep->sendStatusMessage(message.c_str());
    t0 = base::Time::now();

    for(int i = 0; i < joints_number; i++)
    {
        vrep->initJointPositionStreaming(joints_handles[i], &joints_position);
        vrep->initJointVelocityStreaming(joints_handles[i], &joints_speed);
    }
    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
        return false;
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(_trajectory.read(trajectory) == RTT::NewData)
    {
        trajectoryData.clear();
        for (unsigned int i = 0; i < trajectory.size() ; i++)
        {
            trajectoryData.push_back(trajectory[i].position(0));
            trajectoryData.push_back(trajectory[i].position(1));
            trajectoryData.push_back(trajectory[i].position(2));
            trajectoryData.push_back(trajectory[i].heading);
        }
        std::string signalName = "trajectory";
        const char *signal = signalName.c_str();
        vrep->appendStringSignal(signal, trajectoryData);
    }
    if(_joints_commands.read(joints_commands) == RTT::NewData)
    {
        for(int i = 0; i < 6; i++) //Driving joints
            {
                if(!isnan(joints_commands.elements[i].speed))
                    vrep->setJointVelocity(joints_handles[i], joints_commands.elements[i].speed);
                if(!isnan(joints_commands.elements[GDR].speed))
                    vrep->setJointVelocity(joints_handles[i], joints_commands.elements[GDR].speed);
            }
	// Due to having the frame of each steering joints oriented with Z pointing down, each
	// command is affected by a minus sign '-'

        for(int i = 6; i < 10; i++) //Steering joints
        {
            if(!isnan(joints_commands.elements[i].position))
                vrep->setJointPosition(joints_handles[i], - joints_commands.elements[i].position);
            if(!isnan(joints_commands.elements[GST].position))
                vrep->setJointPosition(joints_handles[i], - joints_commands.elements[GST].position);
        }

        for(int i = 10; i < 16; i++) //Walking joints
        {
            if(!isnan(joints_commands.elements[i].speed))
	    {
		vrep->disableControlLoop(joints_handles[i]);
                vrep->setJointVelocity(joints_handles[i], joints_commands.elements[i].speed);
	    }
	    if(!isnan(joints_commands.elements[i].position))
	    {
		vrep->enableControlLoop(joints_handles[i]);
                vrep->setJointPosition(joints_handles[i], joints_commands.elements[i].position);
	    }
            if(!isnan(joints_commands.elements[GWW].speed))
                vrep->setJointVelocity(joints_handles[i], joints_commands.elements[GWW].speed);
	    if(!isnan(joints_commands.elements[GWW].position))
                vrep->setJointPosition(joints_handles[i], joints_commands.elements[GWW].position);
        }

    }


  // Get the readings to publish them to ROCK as joint_command_dispatcher works
    for(int i = 0; i < joints_number; i++)
    {

        vrep->getJointPosition(joints_handles[i], &joints_position);
        vrep->getJointVelocity(joints_handles[i], &joints_speed);

      // Driving Joints/Motors
        if (i < 6)
        {
            joints_readings.elements[i+13].position = (double)joints_position;
            joints_readings.elements[i+13].speed = (double)joints_speed;
            motors_readings.elements[i].position = (double)joints_position;
	    motors_readings.elements[i].speed = (double)joints_speed;
        }

      // Steering Joints/Motors
        if ((i > 5)&&(i < 10))
        {
            joints_readings.elements[i+3].position = - (double)joints_position;
            joints_readings.elements[i+3].speed = - (double)joints_speed;
            motors_readings.elements[i].position = - (double)joints_position;
	    motors_readings.elements[i].speed = - (double)joints_speed;
        }

      // Walking Joints/Motors
        if ((i > 9)&&(i < 16))
        {
            joints_readings.elements[i-7].position = (double)joints_position;
            joints_readings.elements[i-7].speed = (double)joints_speed;
            motors_readings.elements[i].position = (double)joints_position;
	    motors_readings.elements[i].speed = (double)joints_speed;
        }

      // Bogie Joints
        if (i > 15)
        {
            joints_readings.elements[i-16].position = (double)joints_position;
            joints_readings.elements[i-16].speed = (double)joints_speed;
        }
    }

    joints_readings.time = base::Time::now();
    motors_readings.time = base::Time::now();
    _joints_readings.write(joints_readings);
    _motors_readings.write(motors_readings);

    vrep->getPosition(roverPoseHandle, -1, position);
    pose.position.x() = position[0];
    pose.position.y() = position[1];
    pose.position.z() = position[2];

    vrep->getQuaternion(pose.orientation.w(),pose.orientation.x(),
                        pose.orientation.y(),pose.orientation.z());

    _pose.write(pose);

    vrep->getPosition(goalMarkerHandle, -1, position);
    vrep->getOrientation(goalMarkerHandle, -1, orientation);
    goalWaypoint.position[0] = position[0];
    goalWaypoint.position[1] = position[1];
    goalWaypoint.heading = orientation[2];
    _goalWaypoint.write(goalWaypoint);    
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();

    // Stop all the jointss
    for(int i = 0; i < 6; i++)
        vrep->setJointVelocity(joints_handles[i], 0.0f);
    for(int i = 6; i < 10; i++)
        vrep->setJointPosition(joints_handles[i], 0.0f);
    for(int i = 10; i < 16; i++)
        vrep->setJointVelocity(joints_handles[i], 0.0f);
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
