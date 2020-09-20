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

namespace k_api = Kinova::Api;

#define IP_ADDRESS "192.168.1.10"
#define PORT 10000
//#define PORT 10001 //for low level control mode

void example_api_creation()
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

int main(int argc, char **argv)
{
    example_api_creation();
}