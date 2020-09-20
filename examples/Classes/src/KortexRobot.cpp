#include "Classes/include/KortexRobot.h"

#include<stdio.h>

#include<stdlib.h>

namespace k_api = Kinova::Api;
constexpr auto TIMEOUT_DURATION = std::chrono::seconds{20};

KortexRobot::KortexRobot(const std::string &IP)
{
    m_sIP = IP;
    m_bIsConnected = false;
    m_bIsBusy = false;
    Init(); //initialize the robot as soon as create the object, just make the main function clean
}

KortexRobot:: ~KortexRobot()
{
    //to tear down the API if the robot is connnected to API
    if (m_bIsConnected)
    { 
       // Close API session
        m_pSessionManager->CloseSession();
        // Deactivate the router and cleanly disconnect from the transport object
        m_pRouterClient->SetActivationStatus(false);
        m_pTcpClient->disconnect();

        m_bIsConnected = false;
        m_bIsBusy = false;

        // Destroy the API, all are pointers
        if (m_pControlConfigClient != nullptr)
        {
            delete m_pControlConfigClient;     
        }
        //delete m_pBaseCyclic;
        delete m_pBase;
        delete m_pDeviceConfigClient;
        delete m_pSessionManager;
        delete m_pRouterClient;
        delete m_pTcpClient;
    }
}

bool KortexRobot::Init()
{
    // How to create an API with the SessionManager, DeviceConfigClient and BaseClient services
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    m_pTcpClient = new k_api::TransportClientTcp();  //be able to send high level comments, TransportClient is the 
                                                       //struct responsable for assembling the messages of API
    m_pRouterClient = new k_api::RouterClient(m_pTcpClient, error_callback);  //router is the struct for sending the message of API
    
    if(!m_pTcpClient->connect(m_sIP, 10000)){
        std::cout << "fail to connceted !!!!!" << std::endl;
        return false;
    }

    try{
        // Set session data connection information
        m_pSessionManager = new k_api::SessionManager(m_pRouterClient);
        auto create_session_info = k_api::Session::CreateSessionInfo();  //create a session infos
        create_session_info.set_username("admin");
        create_session_info.set_password("admin");
        create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
        create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)
        m_pSessionManager->CreateSession(create_session_info);

        std::cout << "Session created" << std::endl;

        m_bIsConnected = true;

        // Access devices: Create DeviceConfigClient and BaseClient for connecting the base
        m_pDeviceConfigClient = new k_api::DeviceConfig::DeviceConfigClient(m_pRouterClient);
        m_pBase = new k_api::Base::BaseClient(m_pRouterClient);
        m_pControlConfigClient = new k_api::ControlConfig::ControlConfigClient(m_pRouterClient);

        m_NbDOF = m_pBase->GetActuatorCount().count();

        auto servoingMode = k_api::Base::ServoingModeInformation();
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
        m_pBase->SetServoingMode(servoingMode);
    }

    catch(k_api::KDetailedException ex)
    {
        OnError(ex);
    }

    return true;
}


void KortexRobot::OnActionNotificationCallback(k_api::Base::ActionNotification &notif)
{
    switch (notif.action_event())
    {
        case k_api::Base::ACTION_START:
        {
            m_bIsBusy = true;
            std::cout << "The action: " << notif.handle().identifier() << "has started" << std::endl;
            break;
        }

        case k_api::Base::ACTION_END:
        {
            m_bIsBusy = false;
            std::cout << "The action: " << notif.handle().identifier()	 << "has ended" << std::endl;
            break;
        }

        case k_api::Base::ACTION_ABORT:
        {
            m_bIsBusy = false;
            std::cout << "The action: " << notif.handle().identifier()	<< "has aborted" << std::endl;
            break;
        }

        case k_api::Base::ACTION_PAUSE:
        {
            m_bIsBusy = false;
            std::cout << "The action: " << notif.handle().identifier()	<< "has paused" << std::endl;
            break;
        }
    }    
}

void KortexRobot::subscribeToNotification()
{
   if(!m_bIsConnected){
       return;
   }
   
   k_api::Common::NotificationOptions options;
   options.set_type(k_api::Common::NOTIFICATION_TYPE_EVENT);

   using namespace std::placeholders;
   //std::function<void(k_api::Base::ActionNotification)> actionCallback = std::bind(&KortexRobot::OnActionNotificationCallback, this, _1);
    
    auto actionCallback = [this](k_api::Base::ActionNotification notif)
    {
        OnActionNotificationCallback (notif); 
        std::cout << "Notificaiton recieved" << std::endl;
    };

   auto handle = m_pBase->OnNotificationActionTopic(actionCallback, options); //subscribe
   
   m_NotificationHandleList.push_back(handle); //add the handle to the end of vecctor m_NotificationHandleList
}
     
void KortexRobot::unsubscribeToNotification()
{
   if(!m_bIsConnected){
       return;
   }

    for(auto handle:m_NotificationHandleList) 
    {
        m_pBase->Unsubscribe(handle);
    }

}

void KortexRobot::OnError(k_api::KDetailedException &ex)
{
    // You can print the error informations and error codes
        auto error_info = ex.getErrorInfo().getError();
        std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
        
        std::cout << "KError error_code: " << error_info.error_code() << std::endl;
        std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
        std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

        // Error codes by themselves are not very verbose if you don't see their corresponding enum value
        // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
        std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
        std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
}

bool KortexRobot::ExecuteExistingAction(const std::string &actionName, k_api::Base::RequestedActionType &actionType)
{
    if(!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    try
    {
        // searching the expected action
        auto action_list = m_pBase->ReadAllActions(actionType); 
        auto action_handle = k_api::Base::ActionHandle();  //this is the Reference to a specific action
        action_handle.set_identifier(0);  //set the action identifier to be 0

        for (auto action : action_list.action_list())   //looping through all the action list inside the base
        {
            if (action.name() == "Home") 
            {
                action_handle = action.handle();  //Returns the current value of handle, Reference to the action (useful when updating an existing action)
            }
        }
        if(action_handle.identifier() == 0)
        {
            return false;
        }
        else
        {
            m_pBase->ExecuteActionFromReference(action_handle);
        }
        
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(550));  //to avoid the confliction between next action
    return true;
}

bool KortexRobot::MoveTo(const tCartesianVector &position, const tCartesianVector &orientation, const k_api::Base::CartesianTrajectoryConstraint &constraint)
{
    if(!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }
    m_Action.mutable_handle()->set_action_type(k_api::Base::REACH_POSE);  //action type
    //m_Action.set_name("Move to a position to any Cartesian frame");
    //m_Action.set_application_data("");

    m_Action.mutable_reach_pose()->mutable_target_pose()->set_x(position.x); 
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_y(position.y); 
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_z(position.z); 

    m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_x(orientation.x); 
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_y(orientation.y); 
    m_Action.mutable_reach_pose()->mutable_target_pose()->set_theta_z(orientation.z); 

    switch(constraint.type_case())
    {
        case k_api::Base::CartesianTrajectoryConstraint::TypeCase::kDuration:
        {
            m_Action.mutable_reach_pose()->mutable_constraint()->set_duration(constraint.duration());
        }
        
        case k_api::Base::CartesianTrajectoryConstraint::TypeCase::kSpeed:
        {
            m_Action.mutable_reach_pose()->mutable_constraint()->mutable_speed()->set_translation(constraint.speed().translation());
            m_Action.mutable_reach_pose()->mutable_constraint()->mutable_speed()->set_orientation(constraint.speed().orientation());
        }
    }

    try
    {
        m_pBase->ExecuteAction(m_Action);
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(550));  //to avoid the confliction between next action
    return true;
}


bool KortexRobot::SetJointAngles(const std::vector<float> &angles, const k_api::Base::JointTrajectoryConstraint &J_constraint)
{
    if(!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    if (angles.size() != m_NbDOF)
    {
        return false;
    }

    m_Action.mutable_handle()->set_action_type(k_api::Base::REACH_JOINT_ANGLES);  //action type
    
    //get the constraint joint angles
    k_api::Base::ConstrainedJointAngles *constraintJointAngles = m_Action.mutable_reach_joint_angles(); //get a pointer to a list of angles
                                                                                                   //mutable keyword indicates returning a pointer to a structure 
    k_api::Base::JointAngles *jointAngles = constraintJointAngles->mutable_joint_angles(); //access all the joint angles in the list
    jointAngles->clear_joint_angles();

    //fill in the joint angles
    for(int i = 0; i < m_NbDOF; i++)
    {
        k_api::Base::JointAngle *jointAngle = jointAngles->add_joint_angles();
        jointAngle->set_joint_identifier(i);
        jointAngle->set_value(angles[i]);
    }

    //fill in the constraint with the joint angles
    if(J_constraint.type() != k_api::Base::JointTrajectoryConstraintType::UNSPECIFIED_JOINT_CONSTRAINT)
    {
        constraintJointAngles->mutable_constraint()->set_type(J_constraint.type());
        constraintJointAngles->mutable_constraint()->set_value(J_constraint.value());
    }
    try
    {
        m_pBase->ExecuteAction(m_Action);
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));  //to avoid the confliction between next action
    return true;  
}

    
bool KortexRobot::SendTwistCommand(const tCartesianVector &translation, const tCartesianVector &rotation)
{
    //twist command are priority to the regular actions
    if(!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }
    
    auto command = k_api::Base::TwistCommand();
    command.mutable_twist()->set_linear_x(translation.x);
    command.mutable_twist()->set_linear_y(translation.y);
    command.mutable_twist()->set_linear_z(translation.z);

    command.mutable_twist()->set_angular_x(rotation.x);
    command.mutable_twist()->set_angular_y(rotation.y);
    command.mutable_twist()->set_angular_z(rotation.z);

    try
    {
        m_pBase->SendTwistCommand(command);  //this function can be updated while running because the commands are not actions
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(550));  //to avoid the confliction between next action
    return true;  

}

bool KortexRobot::SetTwistReferenceFrame(const k_api::Common::CartesianReferenceFrame &frame)
{
    if(!m_bIsConnected)
    {
        return false;
    }

    k_api::ControlConfig::CartesianReferenceFrameInfo frameRequest;
    frameRequest.set_reference_frame(frame);

    try
    {
        m_pControlConfigClient->SetCartesianReferenceFrame(frameRequest);
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }
    return true;
}

bool KortexRobot::SetJointSpeeds(std::vector<float> &JointSpeeds)
{
    if(!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    if (JointSpeeds.size() != m_NbDOF)
    {
        return false;
    }

    k_api::Base::JointSpeeds jointSpeedsRequest;

    //fill in the joint angles
    for(int i = 0; i < m_NbDOF; i++)
    {
        k_api::Base::JointSpeed *speed = jointSpeedsRequest.add_joint_speeds();
        speed->set_joint_identifier(i);
        speed->set_value(JointSpeeds[i]);
    }

    try
    {
        m_pBase->SendJointSpeedsCommand(jointSpeedsRequest);
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }

    return true; 
}

//functions directly from the client and being useful outside of class
bool KortexRobot::PlaySequence(const k_api::Base::SequenceHandle &sequenceHandle)
{
   if(!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    try
    {
        m_pBase->PlaySequence(sequenceHandle);
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }
    
    return true;
}
    
bool KortexRobot::ExecuteAction(const k_api::Base::Action &action)
{
   if(!m_bIsConnected || m_bIsBusy)
    {
        return false;
    }

    try
    {
        m_pBase->ExecuteAction(action);
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }
    
    return true;

}

//nessesary for properly using the above functions, avoid problems
bool KortexRobot::WaitWhileRobotIsMoving(const int timeout)
{
    if(!m_bIsConnected)
    {
        return false;
    }

    //do nothing when the robot is moving
    int cycle = 0;
    while(m_bIsBusy && cycle < timeout)
    {
        cycle ++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (cycle > timeout)
    {
        return false;
    }
    return true;  //robot moving is over
}

    
bool KortexRobot::Stop()
{
   if(!m_bIsConnected)
    {
        return false;
    }
    try
    {
        m_pBase->Stop();
    }
    catch(k_api::KDetailedException &ex)
    {
        OnError(ex);
    }
    return true;
}
