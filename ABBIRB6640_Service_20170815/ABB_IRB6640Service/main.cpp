#include "ABB_IRB6640Solver.h"
#include "EGM.h"


//#ifdef	_WIN32
SOCKET sockfd;
struct sockaddr_in serverAddr, clientAddr;
WSADATA wsaData;
extern EGM egm;

int main(int argc, char *argv[])
{
	char temp = 'x';
	int n;
	int len;
	/* Init winsock */
	
	if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
		fprintf(stderr, "Could not open Windows connection.\n");
		exit(0);
	}

	// create socket to listen on
	sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
	memset(&serverAddr, sizeof(serverAddr), 0);
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
	serverAddr.sin_port = htons(portNumber);

	int result = bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

	if (!result) {
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
		
		//std::getline(std::cin, std::string());

			
		while (!result) {
			if (temp == '\n'){
				continue;
			}
			len = sizeof(clientAddr);
			n = recvfrom(sockfd, egm.protoMessage, 1400, 0, (struct sockaddr *)&clientAddr, &len);
			EgmRobot *pRobotMessage = new EgmRobot();
			pRobotMessage->ParseFromArray(egm.protoMessage, n);
			egm.DisplayRobotMessage(pRobotMessage);
			//std::cin.get(temp);
			//result = bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
			
		}
	}
	return 0;
}