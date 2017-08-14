
#include "ABB_IRB6640Solver.h"

unsigned int EGM::sequenceNumber = 0;

static int portNumber = 6510;
EGM egm;

ABB_IRB6640Solver::ABB_IRB6640Solver()
{
	this->value = 0;
}

int32_t ABB_IRB6640Solver::get_value()
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	return this->value;
}
void ABB_IRB6640Solver::set_value(int32_t value)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	this->value = value;
}

double ABB_IRB6640Solver::add(double a, double b)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	return a + b;
}

double ABB_IRB6640Solver::dot(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > a, RR_SHARED_PTR<RobotRaconteur::RRArray<double > > b)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	double val = 0;
	if (a->size() == b->size())
	{
		
		for (int i = 0; i < a->size(); ++i)
			val += a->ptr()[i] * b->ptr()[i];
	}
	return val;
}

RR_SHARED_PTR<RobotRaconteur::RRArray<double > > ABB_IRB6640Solver::sort_sequence(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > seq, std::string dir)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	std::vector<double> my_seq = std::vector<double>(seq->size());
	my_seq.assign(seq->ptr(), seq->ptr() + seq->size());
	std::sort(my_seq.begin(), my_seq.end());
	if (strcmpi(dir.c_str(), "backward") == 0)
		std::reverse(my_seq.begin(), my_seq.end());

	RR_SHARED_PTR<RobotRaconteur::RRArray<double > > ret_seq = RobotRaconteur::AllocateRRArray<double>(my_seq.size());
	ret_seq = RobotRaconteur::AttachRRArrayCopy<double>(&my_seq[0], my_seq.size()); 
	return ret_seq;
}

RR_SHARED_PTR<RobotRaconteur::RRArray<double > > ABB_IRB6640Solver::set_joints(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > seq, std::string dj)
{
	boost::lock_guard<boost::mutex> guard(mtx_);
	
	std::vector<double> my_seq = std::vector<double>(seq->size());
	my_seq.assign(seq->ptr(), seq->ptr() + seq->size());
	
	/*std::sort(my_seq.begin(), my_seq.end());
	if (strcmpi(dj.c_str(), "backward") == 0)
		std::reverse(my_seq.begin(), my_seq.end());
	RR_SHARED_PTR<RobotRaconteur::RRArray<double > > ret_seq = RobotRaconteur::AllocateRRArray<double>(my_seq.size());
	ret_seq = RobotRaconteur::AttachRRArrayCopy<double>(&my_seq[0], my_seq.size());
	*/

	if (strcmpi(dj.c_str(), "c") == 0)
	{
		egm.cart_pose.cart[0] = my_seq[0];
		egm.cart_pose.cart[1] = my_seq[1];
		egm.cart_pose.cart[2] = my_seq[2];
		egm.cart_pose.orient[0] = my_seq[3];
		egm.cart_pose.orient[1] = my_seq[4];
		egm.cart_pose.orient[2] = my_seq[5];
	}
	else
	{
		egm.joints[0] = my_seq[0];
		egm.joints[1] = my_seq[1];
		egm.joints[2] = my_seq[2];
		egm.joints[3] = my_seq[3];
		egm.joints[4] = my_seq[4];
		egm.joints[5] = my_seq[5];
		egm.joint_cmd = true;
	}

	egm.UDP_Com(portNumber);

	for (int i = 0; i < 6; i++) {
		my_seq[i] = egm.joints[i];
	}

	RR_SHARED_PTR<RobotRaconteur::RRArray<double > > ret_seq = RobotRaconteur::AllocateRRArray<double>(my_seq.size());
	ret_seq = RobotRaconteur::AttachRRArrayCopy<double>(&my_seq[0], my_seq.size());

	return ret_seq;
}