
#ifdef SendMessage
#undef SendMessage
#endif


#include <RobotRaconteur.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include "example__ABB_IRB6640.h"
#include "example__ABB_IRB6640_stubskel.h"

#include <Windows.h>
#include <algorithm>
#include <iostream>
#include <vector>
#include <cmath>

#include <boost/enable_shared_from_this.hpp>
#include <map>

#include "stdafx.h"
#include "EGM.h"

static int portNumber = 6510;

class ABB_IRB6640Solver : public example::ABB_IRB6640::ABB_IRB6640Solver, public boost::enable_shared_from_this < ABB_IRB6640Solver >
{
public:

	ABB_IRB6640Solver();

	virtual int32_t get_value();
	virtual void set_value(int32_t value);

	virtual double add(double a, double b);

	virtual double dot(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > a, RR_SHARED_PTR<RobotRaconteur::RRArray<double > > b);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > sort_sequence(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > seq, std::string dir);

	virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > set_joints(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > seq, std::string dj);

private:
	int value;
	boost::mutex mtx_;
};