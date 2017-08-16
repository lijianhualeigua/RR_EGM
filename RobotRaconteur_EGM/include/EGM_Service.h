#include "example__RR_EGM.h"
#include "example__RR_EGM_stubskel.h"
#include "EGM.h"

#include <iostream>
#include <stdio.h>
#include <boost/enable_shared_from_this.hpp>

#pragma once

using namespace example::RR_EGM;

//Class to implement the "IRB6640" object abstract class
//and also use "enable_shared_from_this" for shared_ptr support
class IRB6640_impl : public IRB6640, public boost::enable_shared_from_this<IRB6640_impl>
{
public:

    IRB6640_impl();
    ~IRB6640_impl();

    virtual void setjoints(RR_SHARED_PTR<RobotRaconteur::RRArray<double > > joint);

    virtual RR_SHARED_PTR<RobotRaconteur::RRArray<double > > readjoints();
};

//Global lock to protect from multi-threaded calls
extern boost::recursive_mutex global_lock;