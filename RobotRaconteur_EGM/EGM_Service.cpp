#include "EGM_Service.h"

static int portNumber = 6510;
unsigned int EGM::sequenceNumber = 0;

IRB6640_impl::IRB6640_impl() {}

IRB6640_impl::~IRB6640_impl() {}

void IRB6640_impl::setjoints(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > joint) {
    EGM egm;
    for (int i = 0; i < 6; i++)
        egm.joints[i] = joint->ptr()[i];
    egm.joint_cmd = true;
    egm.UDP_Com(portNumber);
}

RR_SHARED_PTR<RobotRaconteur::RRArray<double > > IRB6640_impl::readjoints() {
	// To be implemented
    EGM egm;
    std::vector<double> my_seq = std::vector<double>(6);
    EgmRobot *pRobotMessage = new EgmRobot();
    RR_SHARED_PTR<RobotRaconteur::RRArray<double > > ret_seq = RobotRaconteur::AllocateRRArray<double>(my_seq.size());
    ret_seq = RobotRaconteur::AttachRRArrayCopy<double>(&my_seq[0], my_seq.size());
    return ret_seq;
}
