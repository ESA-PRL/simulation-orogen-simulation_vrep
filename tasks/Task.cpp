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

    printf("VREP Simulation reached \n");

    joint_vrep_names = _joint_vrep_names.get();
    joint_readings_names = _joint_readings_names.get();

    joints_readings.resize(joint_readings_names.size());
    num_motors = _num_motors.get();
    num_joints = _num_joints.get();

    

    for (int i = 0; i < num_joints; i++)
        joints_readings.names[i] = joint_readings_names[i];
    

    joints_handles.resize(num_joints);

    for(int i = 0; i < num_joints; i++)
        vrep->getObjectHandle(joint_vrep_names[i], &joints_handles[i]);
    vrep->getObjectHandle("Pose", &roverPoseHandle);
    vrep->getObjectHandle("GOAL_marker", &goalMarkerHandle);

    std::string const message("Enabling Synchronization");
    vrep->sendStatusMessage(message.c_str());
    t0.microseconds = 1000*vrep->getSimulationTime();

    for(int i = 0; i < num_joints; i++)
    {
        vrep->initJointPositionStreaming(joints_handles[i], &joints_position);
        vrep->initJointVelocityStreaming(joints_handles[i], &joints_speed);
    }

    vrep->getObjectHandle("anaglyphStereoSensor", &cameraHandle); //TODO make config parameter

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


    /******************************/
    /** TRAJECTORY VISUALIZATION **/
    /******************************/

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


    /***********************/
    /** COMMANDING JOINTS **/
    /***********************/

    if(_joints_commands.read(joints_commands) == RTT::NewData)
    {
        for(int i = 0; i < num_motors; i++)
        {
            if (joints_commands[i].isPosition())
            {
                //vrep->enableControlLoop(joints_handles[i]);
                if(!isnan(joints_commands.elements[i].position))
                    vrep->setJointPosition(joints_handles[i], joints_commands.elements[i].position);
            }
            else if (joints_commands[i].isSpeed())
            {
                //vrep->disableControlLoop(joints_handles[i]);
                if(!isnan(joints_commands.elements[i].speed))
                    vrep->setJointVelocity(joints_handles[i], joints_commands.elements[i].speed);
                if(!isnan(joints_commands.elements[num_motors].speed)) //Driving Group
                    vrep->setJointVelocity(joints_handles[i], joints_commands.elements[num_motors].speed);
            }
            else
            {
                //For some reason, isPosition() and isSpeed() can be both false, though driving group speed is set to zero
                if(!isnan(joints_commands.elements[num_motors].speed)) //Driving Group
                    vrep->setJointVelocity(joints_handles[i], joints_commands.elements[num_motors].speed);
            }
        }
    }


    /********************/
    /** READING JOINTS **/
    /********************/

  // Get the readings to publish them to ROCK as joint_command_dispatcher works
    for(int i = 0; i < num_joints; i++)
    {

        vrep->getJointPosition(joints_handles[i], &joints_position);
        vrep->getJointVelocity(joints_handles[i], &joints_speed);
        joints_readings.elements[i].position = (double)joints_position;
        joints_readings.elements[i].speed = (double)joints_speed;
    }

    int current_time = 1000 * vrep->getSimulationTime();

    joints_readings.time.microseconds = current_time;

    vrep->getPosition(roverPoseHandle, -1, position);
    pose.position.x() = position[0];
    pose.position.y() = position[1];
    pose.position.z() = position[2];

    vrep->getQuaternion(pose.orientation.w(),pose.orientation.x(),
                        pose.orientation.y(),pose.orientation.z());

    pose.time.microseconds = current_time;


    /******************/
    /** LOCALIZATION **/
    /******************/

    vrep->getPosition(goalMarkerHandle, -1, position);
    vrep->getOrientation(goalMarkerHandle, -1, orientation);
    goalWaypoint.position[0] = position[0];
    goalWaypoint.position[1] = position[1];
    goalWaypoint.heading = orientation[2];

    _pose.write(pose);
    _goalWaypoint.write(goalWaypoint);
    _joints_readings.write(joints_readings);

    // request distance and image
    int resolution_img[2];
    std::vector<uint8_t> img;
    bool black_and_white = true; // TODO make color images work
    if (vrep->getStereoSensorImage(cameraHandle, resolution_img, img, black_and_white))
    {
        base::samples::frame::Frame camera_image(resolution_img[0], resolution_img[1], base::samples::frame::MODE_GRAYSCALE);
        camera_image.setImage(img);
        _camera_image.write(camera_image);
    }
}

void Task::errorHook()
{
    TaskBase::errorHook();
}

void Task::stopHook()
{
    TaskBase::stopHook();
/*
    // Stop all the joints
    for(int i = 0; i < 6; i++)
        vrep->setJointVelocity(joints_handles[i], 0.0f);
    for(int i = 6; i < 10; i++)
        vrep->setJointPosition(joints_handles[i], 0.0f);
    for(int i = 10; i < 16; i++)
        vrep->setJointVelocity(joints_handles[i], 0.0f);*/
}

void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}
