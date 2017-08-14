#include "ABB_IRB6640Solver.h"


int main(int argc, char *argv[])
{

	// Register Local Transport
	boost::shared_ptr<RobotRaconteur::LocalTransport> t1 = boost::make_shared<RobotRaconteur::LocalTransport>();
	t1->StartServerAsNodeName("example.ABB_IRB6640");
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t1);

	// Register TCP Transport on port 1234
	boost::shared_ptr<RobotRaconteur::TcpTransport> t = boost::make_shared<RobotRaconteur::TcpTransport>();
	t->StartServer(1234);
	t->EnableNodeAnnounce(RobotRaconteur::IPNodeDiscoveryFlags_LINK_LOCAL |
		RobotRaconteur::IPNodeDiscoveryFlags_NODE_LOCAL |
		RobotRaconteur::IPNodeDiscoveryFlags_SITE_LOCAL);
	RobotRaconteur::RobotRaconteurNode::s()->RegisterTransport(t);

	// Create the ABB_IRB6640Solver object
	boost::shared_ptr<ABB_IRB6640Solver> m = boost::make_shared<ABB_IRB6640Solver>();

	// Register the service type with Robot Raconteur
	RobotRaconteur::RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<example::ABB_IRB6640::example__ABB_IRB6640Factory>());


	// Register the ABB_IRB6640Solver object as a service
	RobotRaconteur::RobotRaconteurNode::s()->RegisterService("ABB_IRB6640Solver", "example.ABB_IRB6640", m);

	std::cout << "Connect to ABB_IRB6640Solver object at: " << std::endl;
	std::cout << "tcp://localhost:1234/example.ABB_IRB6640/ABB_IRB6640Solver" << std::endl;
	std::cout << "Press enter to quit" << std::endl;

	std::getline(std::cin, std::string());
	
	return 0;
}