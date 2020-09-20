/*
 * KINOVA (R) KORTEX (TM)
 *
 * Copyright (c) 2018 Kinova inc. All rights reserved.
 *
 * This software may be modified and distributed
 * under the terms of the BSD 3-Clause license.
 *
 * Refer to the LICENSE file for details.
 *
 */

#include <SessionManager.h>
#include <DeviceConfigClientRpc.h>
#include <BaseClientRpc.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <google/protobuf/util/json_util.h>

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
//#define PORT 10001 //for low level control mode


void DisplayErrorDetail(k_api::KDetailedException& data)
{
        // You can print the error informations and error codes
        auto error_info = data.getErrorInfo().getError();
        std::cout << "KDetailedoption detected what:  " << data.what() << std::endl;
        
        std::cout << "KError error_code: " << error_info.error_code() << std::endl;
        std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
        std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

        // Error codes by themselves are not very verbose if you don't see their corresponding enum value
        // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
        std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
        std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
    
}

void NotificationCallBack(Kinova::Api::Base::ConfigurationChangeNotification &data)
{
        std::cout << "Callback triger" << std::endl;

        std::string jsonString = "";
        google::protobuf::util::MessageToJsonString(data, &jsonString);
        std::cout << "********************************" << std::endl;
        std::cout << "**  Callback function called  **" << std::endl;
        std::cout << jsonString << std::endl;
        std::cout << "********************************" << std::endl;
}

k_api::Common::UserProfileHandle CreateUserProfile(k_api::Base::BaseClient *base)
{
    auto full_user_profile = k_api::Base::FullUserProfile();
    auto user_profile = full_user_profile.mutable_user_profile();
    auto user_profile_handle = user_profile->mutable_handle();
    user_profile_handle->set_identifier(0);
    user_profile->set_username("jcash");
    user_profile->set_firstname("Johnny");
    user_profile->set_lastname("Cash");
    full_user_profile.set_password("pwd");
    
    k_api::Common::UserProfileHandle returned_user_profile_handle;
    try 
    {
        std::cout << "Creating user profile to trigger notification" << std::endl;
        returned_user_profile_handle = base->CreateUserProfile(full_user_profile);
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "User profile creation failed" << std::endl;
    }
}

void DeleteUserProfile(k_api::Base::BaseClient* base, k_api::Common::UserProfileHandle& returned_user_profile_handle)
{
    try
    {
        std::cout << "Deleting previously created user profile" << std::endl;
        base->DeleteUserProfile(returned_user_profile_handle); // no notification about this modification
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "User profile deletion failed" << std::endl;
        DisplayErrorDetail (ex);
    }
    //std::this_thread::sleep_for(std::chrono::milliseconds(2000));

}



int main(int argc, char **argv)
{
    // -----------------------------------------------------------
    // How to create an API with the SessionManager, DeviceConfigClient and BaseClient services
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    auto transport = new k_api::TransportClientTcp();  //be able to send high level comments, TransportClient is the 
                                                       //structure responsable for assembling the messages of API
    auto router = new k_api::RouterClient(transport, error_callback);  //router is the structure for sending the message of API
    transport->connect(IP_ADDRESS, PORT);  //sending a connect message

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();  //create a session infos
    create_session_info.set_username("admin");
    create_session_info.set_password("admin");
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper, for send the massage to the robot 
    std::cout << "Creating session for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);  //creat a session manager
    session_manager->CreateSession(create_session_info);  //use the session manager to creat a session using the info
    std::cout << "Session created" << std::endl;

    // Create DeviceConfigClient and BaseClient for connecting the base
    auto device_config = new k_api::DeviceConfig::DeviceConfigClient(router);
    auto base = new k_api::Base::BaseClient(router);

    // -----------------------------------------------------------
    // Now you're ready to use the API
    // ...

    try
    {
        base->CreateUserProfile(k_api::Base::FullUserProfile());
    }
    catch(k_api::KDetailedException &ex)
    {
        DisplayErrorDetail (ex);
    }  
    
     
    
    //notifications

    auto function_callback = [](k_api::Base::ConfigurationChangeNotification data)
    {
        NotificationCallBack (data);  //only print ******
    };
    // this callback function receives the subscribed notification: configuration changed

    //subscribe a "handle" to configuration notifications, a service responsable by base
    auto notification_handle = base->OnNotificationConfigurationChangeTopic(function_callback, k_api::Common::NotificationOptions());

    //crate a use profile to trigger a notification
    auto returned_user_profile_handle1 = CreateUserProfile(base);

    std::this_thread::sleep_for(std::chrono::milliseconds(2000));


    //unsubscribe
    std::cout << "unsubscribe from configureationchange notifications" << std::endl;
    base->Unsubscribe(notification_handle);

    //Delete the creased profile
    DeleteUserProfile(base, returned_user_profile_handle1);
    //base->DeleteUserProfile(returned_user_profile_handle1); 

    // -----------------------------------------------------------
    // After you're done, here's how to tear down the API

    // Close API session
    session_manager->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();

    // Destroy the API, all are pointers
    delete base;
    delete device_config;
    delete session_manager;
    delete router;
    delete transport;
}
