#include "Task.hpp"

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
    {
        return false;
    }

    vrep = new vrep::VREP();

    if(vrep->getClientId() == -1)
    {
        printf("Could not reach VREP simulation, make sure the simulation is running in VREP\n");
        return false;
    }

    motor_names.push_back("joint_drive_fl");
    motor_names.push_back("joint_drive_fr");
    motor_names.push_back("joint_drive_ml");
    motor_names.push_back("joint_drive_mr");
    motor_names.push_back("joint_drive_rl");
    motor_names.push_back("joint_drive_rr");
    motor_names.push_back("joint_steer_fl");
    motor_names.push_back("joint_steer_fr");
    motor_names.push_back("joint_steer_rl");
    motor_names.push_back("joint_steer_rr");
    motor_names.push_back("joint_walk_fl");
    motor_names.push_back("joint_walk_fr");
    motor_names.push_back("joint_walk_ml");
    motor_names.push_back("joint_walk_mr");
    motor_names.push_back("joint_walk_rl");
    motor_names.push_back("joint_walk_rr");

    joints_readings.resize(motor_number);

    int handle;
    for(int i = 0; i < motor_number; i++)
    {
        vrep->getObjectHandle(motor_names[i], &handle);
        motor_handles.push_back(handle);
    }

    std::string const message("Controller configured");
    vrep->sendStatusMessage(message.c_str());

    return true;
}

bool Task::startHook()
{
    if(!TaskBase::startHook())
    {
        return false;
    }
    return true;
}

void Task::updateHook()
{
    TaskBase::updateHook();

    if(_joints_commands.read(joints_commands) == RTT::NewData)
    {
        for(int i = 0; i < 6; i++) //Driving Motors
        {
            if(!isnan(joints_commands.elements[i].speed))
                vrep->setJointVelocity(motor_handles[i], joints_commands.elements[i].speed);

            if(!isnan(joints_commands.elements[GDR].speed))
                vrep->setJointVelocity(motor_handles[i], joints_commands.elements[GDR].speed);
        }

	// Due to having the frame of each steering motor oriented with Z pointing down, each
	// command is affected by a minus sign '-'

        for(int i = 6; i < 10; i++) //Steering Motors
        {
            if(!isnan(joints_commands.elements[i].position))
                vrep->setJointPosition(motor_handles[i], - joints_commands.elements[i].position);

            if(!isnan(joints_commands.elements[GST].position))
                vrep->setJointPosition(motor_handles[i], - joints_commands.elements[GST].position);
        }

        for(int i = 10; i < 16; i++) //Walking Motors
        {
            if(!isnan(joints_commands.elements[i].speed))
                vrep->setJointVelocity(motor_handles[i], joints_commands.elements[i].speed);

            if(!isnan(joints_commands.elements[GWW].speed))
                vrep->setJointVelocity(motor_handles[i], joints_commands.elements[GWW].speed);
        }
    }

    // Get the joint readings to publish them to ROCK
    float motor_position, motor_speed;
    for(int i = 0; i < motor_number; i++)
    {
        vrep->getJointPosition(motor_handles[i], &motor_position);
        vrep->getJointVelocity(motor_handles[i], &motor_speed);
	if ((i>5)&&(i<10))
		joints_readings.elements[i].position = - (double)motor_position;
	else
		joints_readings.elements[i].position = (double)motor_position;
        joints_readings.elements[i].speed = (double)motor_speed;
    }

    joints_readings.time = base::Time::now();
    _joints_readings.write(joints_readings);

}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();

    // Stop all the motors
    for(int i = 0; i < 6; i++)
        vrep->setJointVelocity(motor_handles[i], 0.0f);
    for(int i = 6; i < 10; i++)
        vrep->setJointPosition(motor_handles[i], 0.0f);
    for(int i = 10; i < 16; i++)
        vrep->setJointVelocity(motor_handles[i], 0.0f);
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
