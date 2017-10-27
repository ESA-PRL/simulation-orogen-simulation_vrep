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

    joints_names = _joint_vrep_names.get();
    joints_readings_names = _joint_readings_names.get();
    ptu_vrep_names = _ptu_vrep_names.get();
    ptu_readings_names = _ptu_readings_names.get();

    joints_readings.resize(joints_readings_names.size());
    motors_readings.resize(motors_number);
    ptu_readings.resize(ptu_readings_names.size());

    for (uint i = 0; i < joints_readings.size(); i++)
        joints_readings.names[i] = joints_readings_names[i];

    for (uint i = 0; i < ptu_readings.size(); i++)
        ptu_readings.names[i] = ptu_readings_names[i];
    /*ptu_readings.names[0] = "pan_joint";
    ptu_readings.names[1] = "tilt_joint";*/

    joints_handles.resize(joints_names.size());
    ptu_handles.resize(ptu_vrep_names.size());

    for(int i = 0; i < joints_names.size(); i++)
        vrep->getObjectHandle(joints_names[i], &joints_handles[i]);

    for(int i = 0; i < ptu_vrep_names.size(); i++)
        vrep->getObjectHandle(ptu_vrep_names[i], &ptu_handles[i]);

    /*vrep->getObjectHandle("pan", &ptu_handles[0]);
    vrep->getObjectHandle("tilt", &ptu_handles[1]);*/
    vrep->getObjectHandle("Pose", &roverPoseHandle);
    vrep->getObjectHandle("GOAL_marker", &goalMarkerHandle);

    std::string const message("Enabling Synchronization");
    vrep->sendStatusMessage(message.c_str());
    t0.microseconds = 1000*vrep->getSimulationTime();

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

    if(_ptu_commands.read(ptu_commands) == RTT::NewData)
    {
        for(int i = 0; i < 2; i++)
        {
            if(!isnan(ptu_commands.elements[i].speed))
            {
                vrep->disableControlLoop(ptu_handles[i]);
                vrep->setJointVelocity(ptu_handles[i], ptu_commands.elements[i].speed);
            }
            if(!isnan(ptu_commands.elements[i].position))
      	    {
      		      vrep->enableControlLoop(ptu_handles[i]);
                vrep->setJointPosition(ptu_handles[i], ptu_commands.elements[i].position);
      	    }
        }
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
    for(int i = 0; i < 2; i++)
    {
        vrep->getJointPosition(ptu_handles[i], &joints_position);
        vrep->getJointVelocity(ptu_handles[i], &joints_speed);
        ptu_readings.elements[i].position = (double)joints_position;
        ptu_readings.elements[i].speed = (double)joints_speed;
    }

    int current_time = 1000 * vrep->getSimulationTime();

    joints_readings.time.microseconds = current_time;
    motors_readings.time.microseconds = current_time;
    ptu_readings.time.microseconds = current_time;

    vrep->getPosition(roverPoseHandle, -1, position);
    pose.position.x() = position[0];
    pose.position.y() = position[1];
    pose.position.z() = position[2];

    vrep->getQuaternion(pose.orientation.w(),pose.orientation.x(),
                        pose.orientation.y(),pose.orientation.z());

    pose.time.microseconds = current_time;

    vrep->getPosition(goalMarkerHandle, -1, position);
    vrep->getOrientation(goalMarkerHandle, -1, orientation);
    goalWaypoint.position[0] = position[0];
    goalWaypoint.position[1] = position[1];
    goalWaypoint.heading = orientation[2];

    _pose.write(pose);
    _goalWaypoint.write(goalWaypoint);
    _joints_readings.write(joints_readings);
    _motors_readings.write(motors_readings);
    _ptu_readings.write(ptu_readings);
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
