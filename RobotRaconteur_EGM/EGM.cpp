#include "EGM.h"

// Create a simple sensor message
void EGM::CreateSensorMessage(EgmSensor* pSensorMessage, bool &joint_cmd) {
    EgmHeader* header = new EgmHeader();
    header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
    header->set_seqno(sequenceNumber++);
#ifdef _WIN32
    header->set_tm(GetTickCount());
#endif
    pSensorMessage->set_allocated_header(header);

    EgmPlanned *planned = new EgmPlanned();

    // Joint command 
    if (joint_cmd) {
        EgmJoints *pj = new EgmJoints();
        // First add desired number of joints (type: repeated)
        pj->add_joints(this->joints[0]);
        pj->add_joints(this->joints[1]);
        pj->add_joints(this->joints[2]);
        pj->add_joints(this->joints[3]);
        pj->add_joints(this->joints[4]);
        pj->add_joints(this->joints[5]);
        planned->set_allocated_joints(pj);
    }
    // Cartesian command
    else {
        EgmCartesian *pc = new EgmCartesian();
        pc->set_x(this->cart_pose.cart[0]);
        pc->set_y(this->cart_pose.cart[1]);
        pc->set_z(this->cart_pose.cart[2]);

        /*EgmQuaternion *pq = new EgmQuaternion();
        pq->set_u0(1.0);
        pq->set_u1(0.0);
        pq->set_u2(0.0);
        pq->set_u3(0.0);*/

        // Euler angle
        EgmEuler *pe = new EgmEuler();
        pe->set_x(this->cart_pose.orient[0]);
        pe->set_x(this->cart_pose.orient[1]);
        pe->set_x(this->cart_pose.orient[2]);

        EgmPose *pcartesian = new EgmPose();
        pcartesian->set_allocated_euler(pe);
        //pcartesian->set_allocated_orient(pq);
        pcartesian->set_allocated_pos(pc);
        planned->set_allocated_cartesian(pcartesian);
    }
    pSensorMessage->set_allocated_planned(planned);
}

// Display inbound robot message
void EGM::DisplayRobotMessage(EgmRobot *pRobotMessage) {
    // one interesting bug: if using ROS node, I cannot get pRobotMessage->feedback().joints().joints(3) and later ones. Google Protobuf exception raised.
    // without using ROS node, everything is fine.
    if (pRobotMessage->has_header() && pRobotMessage->header().has_seqno() && pRobotMessage->header().has_tm() && pRobotMessage->header().has_mtype()) {
        printf("SeqNo=%d || Tm=%u || Type=%d\n", pRobotMessage->header().seqno(), pRobotMessage->header().tm(), pRobotMessage->header().mtype());
#ifdef	_WIN32
        printf("Joint=%6.2lf || %6.2lf || %6.2lf || %6.2lf || %6.2lf || %6.2lf \n", pRobotMessage->feedback().joints().joints(0)*rad_deg, pRobotMessage->feedback().joints().joints(1)*rad_deg,
            pRobotMessage->feedback().joints().joints(2)*rad_deg, pRobotMessage->feedback().joints().joints(3)*rad_deg,
            pRobotMessage->feedback().joints().joints(4)*rad_deg, pRobotMessage->feedback().joints().joints(5)*rad_deg);
#elif __linux__
        printf("Joint=%6.2lf || %6.2lf || %6.2lf \n", pRobotMessage->feedback().joints().joints(0)*rad_deg, pRobotMessage->feedback().joints().joints(1)*rad_deg, pRobotMessage->feedback().joints().joints(2)*rad_deg);
#endif
    }
    else {
        printf("No header\n");
    }
}

 // Construct UDP communication
void EGM::UDP_Com(int portNumber) {
    #ifdef	_WIN32
    SOCKET sockfd;
    int n;
    struct sockaddr_in serverAddr, clientAddr;
    int len;
    /* Init winsock */
    WSADATA wsaData;
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

    // listen on all interfaces
    int result = bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

    while (!result) {
        // receive and display message from robot
        len = sizeof(clientAddr);
        n = recvfrom(sockfd, protoMessage, 1400, 0, (struct sockaddr *)&clientAddr, &len);
        if (n < 0) {
            printf("Error receive message\n");
            continue;
        }

        // parse inbound message
        EgmRobot *pRobotMessage = new EgmRobot();
        pRobotMessage->ParseFromArray(protoMessage, n);
        DisplayRobotMessage(pRobotMessage);
        delete pRobotMessage;

        // create and send a sensor message
        EgmSensor *pSensorMessage = new EgmSensor();
        CreateSensorMessage(pSensorMessage, joint_cmd);
        pSensorMessage->SerializeToString(&messageBuffer);

        // send a message to the robot
        n = sendto(sockfd, messageBuffer.c_str(), messageBuffer.length(), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
        if (n < 0) {
            printf("Error send message\n");
        }
        delete pSensorMessage;
    }
    closesocket(sockfd);
    WSACleanup();

#elif __linux__
    ros::init(argc, argv, "to_EGM");
    ros::NodeHandle nh;
    int sockfd;
    socklen_t len;
    int n;
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        printf("cannot create socket");
        return 0;
    }

    struct sockaddr_in serverAddr, clientAddr;
    memset(&serverAddr, sizeof(serverAddr), 0);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    serverAddr.sin_port = htons(portNumber);

    int result = bind(sockfd, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    while (ros::ok()) {
        while (!result) {
            // receive and display message from robot
            len = sizeof(clientAddr);
            n = recvfrom(sockfd, protoMessage, 1400, 0, (struct sockaddr *)&clientAddr, &len);
            if (n < 0) {
                printf("Error receive message\n");
                continue;
            }

            // parse inbound message
            EgmRobot *pRobotMessage = new EgmRobot();
            pRobotMessage->ParseFromArray(protoMessage, n);
            DisplayRobotMessage(pRobotMessage);
            delete pRobotMessage;

            // create and send a sensor message
            EgmSensor *pSensorMessage = new EgmSensor();
            CreateSensorMessage(pSensorMessage, joint_cmd);
            pSensorMessage->SerializeToString(&messageBuffer);

            // send a message to the robot
            n = sendto(sockfd, messageBuffer.c_str(), messageBuffer.length(), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
            if (n < 0) {
                printf("Error send message\n");
            }
            delete pSensorMessage;
        }
        ros::spinOnce();
    }
    close(sockfd);
#endif
}