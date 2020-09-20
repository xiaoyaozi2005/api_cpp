#include <iostream>
#include "KortexRobot.h"

namespace k_api = Kinova::Api;

int main(int argc, char **argv)
{
    KortexRobot robot = KortexRobot("192.168.1.10");

    if(robot.IsConnected())
    {
        std::cout << "connection build successfully" << std::endl;
        robot.subscribeToNotification();
        
        //Go to home position;
        auto action_type = k_api::Base::RequestedActionType();
        action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);        
        robot.ExecuteExistingAction("Home", action_type);
        robot.WaitWhileRobotIsMoving(10000);

        //Go to a position in the cartisian frame
        
        tCartesianVector position = tCartesianVector(0.35f, 0.10f, 0.40f);
        tCartesianVector orientation = tCartesianVector(0.0f, 0.0f, 0.0f);
        k_api::Base::CartesianTrajectoryConstraint constraint = k_api::Base::CartesianTrajectoryConstraint();
        constraint.mutable_speed()->set_translation(0.25f);
        constraint.mutable_speed()->set_orientation(30.0f);
        robot.MoveTo(position, orientation, constraint);
        robot.WaitWhileRobotIsMoving(10000);

        
        //angular action
        std::vector <float> jointAngles {340.0f, 25.5f, 205.0f, 240.0f, 345.0f, 320.0f, 100.0f};
        k_api::Base::JointTrajectoryConstraint jointConstraint = k_api::Base::JointTrajectoryConstraint();
        robot.SetJointAngles(jointAngles, jointConstraint);
        robot.WaitWhileRobotIsMoving(10000);

        //twist command
        
        robot.SetTwistReferenceFrame(k_api::Common::CartesianReferenceFrame::CARTESIAN_REFERENCE_FRAME_TOOL);
        tCartesianVector translation = tCartesianVector(0.0f, 0.0f, -0.1f);
        tCartesianVector rotation = tCartesianVector(0.0f, 0.0f, 0.0f);
        robot.SendTwistCommand(translation, rotation);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        robot.Stop();

        //jointspeeds
        std::vector<float> JointSpeeds (6, 0.0f);    //firstly, set six actuators velocity of 0.
        JointSpeeds.push_back(30.0f);                 //then, set the last actuator velocity to be 30
        robot.SetJointSpeeds(JointSpeeds);
        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
        robot.Stop();


        //robot.unsubscribeToNotification();
        //robot.~KortexRobot();
    }
    
    std::cout << "this is the end" << std::endl;
    //no need to call the distructor, because it will be called automatically as the main function finished.

}