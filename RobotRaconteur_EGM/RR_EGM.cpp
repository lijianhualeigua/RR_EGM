#include "stdafx.h"
#include <EGM_Service.h>


int main(int argc, char** argv) {
    boost::shared_ptr<IRB6640_impl> m = boost::make_shared<IRB6640_impl>();
    // Register Local Transport
    boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
    t1->StartServerAsNodeName("example.RR_EGM");
    RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

    // Register TCP Transport on port 6510
    boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);
    t->StartServer(6510);

    try
    {
        t->LoadTlsNodeCertificate();
    }
    catch (std::exception&)
    {
        std::cout << "warning: could not load TLS certificate" << std::endl;
    }

    t->EnableNodeAnnounce();

    // Register the service type with Robot Raconteur
    RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<example__RR_EGMFactory>());


    // Register the service
    // The first is the service name, the second is the node type instead of the node name
    RobotRaconteur::RobotRaconteurNode::s()->RegisterService("RR_EGM", "example.RR_EGM", m);

    //Wait for the user to shutdown the service
    std::cout << "RR_EGM server started. Press enter to quit" << std::endl;
    getchar();
    RobotRaconteur::RobotRaconteurNode::s()->Shutdown();

    return 0;
}

