#include "RobotRaconteur_EGM.h"
#include "edu__rpi__robotics__abb__egm_stubskel.h"


using namespace RobotRaconteur;

using namespace boost::asio;
using boost::mutex;

using namespace abb::egm;


int main(int argc, char* argv[])
{
	RR_SHARED_PTR<EGM> egm = RR_MAKE_SHARED<EGM>();
	egm->Start(6510);

	boost::shared_ptr<LocalTransport> t1 = boost::make_shared<LocalTransport>();
	t1->StartServerAsNodeName("edu.rpi.robotics.abb.egm");
	RobotRaconteurNode::s()->RegisterTransport(t1);

	boost::shared_ptr<TcpTransport> t = boost::make_shared<TcpTransport>();	
	try
	{
		t->LoadTlsNodeCertificate();
	}
	catch (std::exception&)
	{
		std::cout << "warning: could not load TLS certificate" << std::endl;
	}
	t->StartServer(62354);
	//t->EnableNodeAnnounce();
	RobotRaconteurNode::s()->RegisterTransport(t);

	
	RobotRaconteurNode::s()->RegisterServiceType(boost::make_shared<edu::rpi::robotics::abb::egm::edu__rpi__robotics__abb__egmFactory>());

	//Register the create object as a service so that it can be connected to
	RobotRaconteurNode::s()->RegisterService("EGM", "edu.rpi.robotics.abb.egm", egm);

	std::cout << "Press enter to exit" << std::endl;
	getchar();

	RobotRaconteurNode::s()->Shutdown();

	return 0;
}

EGM::EGM(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurNode> node)
{
	this->node = node;
	memset(joint_angle, 0, sizeof(joint_angle));
	memset(target_joint_angle, 0, sizeof(target_joint_angle));
	sequenceNumber = 0;
}

void EGM::Start(uint16_t port)
{
	boost::mutex::scoped_lock lock(this_lock);
	RR_SHARED_PTR<RobotRaconteurNode> n = node.lock();
	if (!n) throw std::runtime_error("Node not found");

	RR_SHARED_PTR<ip::udp::socket> s = RR_MAKE_SHARED<ip::udp::socket>(boost::ref(n->GetThreadPool()->get_io_service()), ip::udp::endpoint(ip::udp::v4(), port));
	s->async_receive_from(boost::asio::buffer(recv_buf, sizeof(recv_buf)), recv_from_ep,
		boost::bind(&EGM::udp_packet_received, shared_from_this(),
			placeholders::error, placeholders::bytes_transferred));
	sock = s;
}

void EGM::Stop()
{
	boost::mutex::scoped_lock lock(this_lock);
	if (sock)
	{
		sock->close();
		sock.reset();
	}
}

RR_SHARED_PTR<RRArray<double > > EGM::get_joint_angle()
{
	return AttachRRArrayCopy(joint_angle, 6);
}
void EGM::set_joint_angle(RR_SHARED_PTR<RRArray<double > > value)
{
	throw std::runtime_error("Read only property");
}

RR_SHARED_PTR<RRArray<double > > EGM::get_joint_angle_setpoint()
{
	boost::mutex::scoped_lock lock(this_lock);
	return AttachRRArrayCopy(target_joint_angle, 6);
}
void EGM::set_joint_angle_setpoint(RR_SHARED_PTR<RRArray<double > > value)
{
	RR_NULL_CHECK(value);
	if (value->size() != 6) throw std::invalid_argument("Invalid value specified");

	boost::mutex::scoped_lock lock(this_lock);
	memcpy(target_joint_angle, value->void_ptr(), sizeof(target_joint_angle));
}

static void DisplayRobotMessage(EgmRobot *pRobotMessage) {

	const double rad_deg = 180.0 / 3.1415926;

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

void EGM::udp_packet_received(const boost::system::error_code& ec, std::size_t len)
{
	if (ec || len == 0)
	{
		std::cerr << ("Network Error!") << std::endl;
		exit(1);
	}

	boost::mutex::scoped_lock lock(this_lock);

	RR_SHARED_PTR<EgmRobot> robotMessage = RR_MAKE_SHARED<EgmRobot>();
	robotMessage->ParseFromArray(recv_buf, len);
	DisplayRobotMessage(robotMessage.get());

	if (robotMessage->has_feedback())
	{
		for (size_t i = 0; i < 6; i++)
		{
			if (robotMessage->feedback().joints().joints().size() > i)
			{
				joint_angle[i] = robotMessage->feedback().joints().joints(i);
			}
		}
	}


	RR_SHARED_PTR<EgmSensor> sensorMessage = RR_MAKE_SHARED<EgmSensor>();
	
	EgmHeader* header = new EgmHeader();
	header->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
	header->set_seqno(sequenceNumber++);
#ifdef _WIN32
	header->set_tm(GetTickCount());
#endif
	sensorMessage->set_allocated_header(header);

	EgmPlanned* planned = new EgmPlanned();
	EgmJoints* pj = new EgmJoints();
	// First add desired number of joints (type: repeated)
	pj->add_joints(this->target_joint_angle[0]);
	pj->add_joints(this->target_joint_angle[1]);
	pj->add_joints(this->target_joint_angle[2]);
	pj->add_joints(this->target_joint_angle[3]);
	pj->add_joints(this->target_joint_angle[4]);
	pj->add_joints(this->target_joint_angle[5]);
	planned->set_allocated_joints(pj);

	sensorMessage->set_allocated_planned(planned);

	send_len = sensorMessage->ByteSize();
	if (send_len > sizeof(send_buf))
	{
		std::cerr << ("EmgSensor message too long") << std::endl;
		exit(1);
	}

	sensorMessage->SerializeToArray(send_buf, sizeof(send_buf));
	
	
	sock->async_send_to(boost::asio::buffer(send_buf, send_len), recv_from_ep,
		boost::bind(&EGM::udp_packet_sent, shared_from_this(),
			placeholders::error, placeholders::bytes_transferred));

	sock->async_receive_from(boost::asio::buffer(recv_buf, sizeof(recv_buf)), recv_from_ep,
		boost::bind(&EGM::udp_packet_received, shared_from_this(),
			placeholders::error, placeholders::bytes_transferred));

}

void EGM::udp_packet_sent(const boost::system::error_code& ec, std::size_t len)
{
	if (ec || len != send_len)
	{
		std::cerr << ("Network Error!") << std::endl;
		exit(1);
	}
}