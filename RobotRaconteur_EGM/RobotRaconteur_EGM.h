#include <RobotRaconteur.h>

#include "edu__rpi__robotics__abb__egm.h"
#include "egm.pb.h"

#pragma once


class EGM : public virtual edu::rpi::robotics::abb::egm::EGM, public RR_ENABLE_SHARED_FROM_THIS<EGM>
{
public:

	EGM(RR_SHARED_PTR<RobotRaconteur::RobotRaconteurNode> node = RobotRaconteur::RobotRaconteurNode::sp());
	void Start(uint16_t port);
	void Stop();

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > get_joint_angle();
	virtual void set_joint_angle(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > value);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > get_joint_angle_setpoint();
	virtual void set_joint_angle_setpoint(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > value);

protected:

	RR_WEAK_PTR<RobotRaconteur::RobotRaconteurNode> node;
	uint16_t port;
	double joint_angle[6];
	double target_joint_angle[6];

	RR_SHARED_PTR<boost::asio::ip::udp::socket> sock;
	boost::mutex this_lock;

	void udp_packet_received(const boost::system::error_code& ec, std::size_t len);
	void udp_packet_sent(const boost::system::error_code& ec, std::size_t len);

	
	uint8_t send_buf[16384];
	size_t send_len;
	boost::asio::ip::udp::endpoint recv_from_ep;
	uint8_t recv_buf[16384];
	size_t sequenceNumber;

};