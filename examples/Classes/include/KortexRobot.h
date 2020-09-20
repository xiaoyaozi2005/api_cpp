#ifndef KORTEXAPICPPEXAMPLE_KORTEXROBOT_H
#define KORTEXAPICPPEXAMPLE_KORTEXROBOT_H

#include <KDetailedException.h>
#include <KError.h>

#include <SessionManager.h>
#include <DeviceConfigClientRpc.h>
#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ControlConfigClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

namespace k_api = Kinova::Api;


//send cartesian coordinates to robot
struct tCartesianVector
{
    float x, y, z;
    tCartesianVector(const float &X, const float &Y, const float &Z): x(X), y(Y), z(Z) {};
    tCartesianVector operator+ (const tCartesianVector v) const
    {
        return tCartesianVector(x+v.x, y+v.y, z+v.z);
    }

    float Norm() const
    {
        return sqrt(x*x + y*y + z*z);
    }
    
    tCartesianVector Normalized() const
    {
        float norm = Norm();
        if (norm > 0.0f)
            return tCartesianVector(x/norm, y/norm, z/norm);
        else
            return tCartesianVector(0.0f, 0.0f, 0.0f);
    }
};

class KortexRobot
{
public:

    KortexRobot(const std::string &IP);
    ~KortexRobot();

    bool Init();
    void Disconnected();  //disconnected from the server and allocate the memory without destroy the kortexRobot object
    bool IsConnected() {return m_bIsConnected;}

    void subscribeToNotification();
    void unsubscribeToNotification();

    //movement
    bool ExecuteExistingAction(const std::string &actionName, k_api::Base::RequestedActionType &actionType);
    bool MoveTo(const tCartesianVector &position, const tCartesianVector &orientation, const k_api::Base::CartesianTrajectoryConstraint &constraint);
    bool SetJointAngles(const std::vector<float> &angles, const k_api::Base::JointTrajectoryConstraint &J_constraint);
    bool SendTwistCommand(const tCartesianVector &translation, const tCartesianVector &rotation);
    bool SetTwistReferenceFrame(const k_api::Common::CartesianReferenceFrame &frame);
    bool SetJointSpeeds(std::vector<float> &JointSpeeds);

    k_api::Base::SequenceHandle CreateSquence(const k_api::Base::Sequence &sequence)
    {
        return m_pBase->CreateSequence(sequence);
    }

    bool PlaySequence(const k_api::Base::SequenceHandle &sequenceHandle);
    bool ExecuteAction(const k_api::Base::Action &action);

    //nessesary for proerly using the above functions
    bool WaitWhileRobotIsMoving(const int timeout);
    bool Stop();
    

    Kinova::Api::Base::BaseClient *m_pBase;
    Kinova::Api::BaseCyclic::BaseCyclicClient *m_pBaseCyclic;
    //k_api::ControlConfig::ControlConfigClient *m_pControlConfigClient;

    //void OnActionNotificationCallback(Kinova::Api::Base::ActionNotification &notif);

protected:
    void OnError(Kinova::Api::KDetailedException &ex);
    void OnActionNotificationCallback(Kinova::Api::Base::ActionNotification &notif);

protected:
    int m_NbDOF;
    bool m_bIsBusy;   //whether the robot is avaliable to receive an action
    k_api::Base::Action m_Action;  //internal action that being modified to send all the defined actions
    k_api::ControlConfig::ControlConfigClient *m_pControlConfigClient; //contains the information about how the robot behaves in various control modes


    std::string m_sIP;  //IP address of the robot, m_ means the member of a class, s means string
    bool m_bIsConnected;  //confirm if the robot is connected to the API, b means boolean
   
    std::vector<Kinova::Api::Common::NotificationHandle> m_NotificationHandleList; //the subcribed lists

    //p means pointer
    Kinova::Api::TransportClientTcp *m_pTcpClient;
    Kinova::Api::RouterClient *m_pRouterClient;
    Kinova::Api::SessionManager *m_pSessionManager;  //sss
    Kinova::Api::DeviceConfig::DeviceConfigClient *m_pDeviceConfigClient;

};

#endif