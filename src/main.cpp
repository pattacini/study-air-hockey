/******************************************************************************
 *                                                                            *
 * Copyright (C) 2021 Fondazione Istituto Italiano di Tecnologia (IIT)        *
 * All Rights Reserved.                                                       *
 *                                                                            *
 ******************************************************************************/

/**
 * @authors: Ugo Pattacini <ugo.pattacini@iit.it>
 */

#include <cstdlib>
#include <cmath>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>


/************************************************************************/
class ControllerModule: public yarp::os::RFModule
{
    yarp::dev::PolyDriver driver;
    yarp::dev::ICartesianControl* arm;

    int startup_context_id;
    yarp::sig::Vector x0, o0;
    const int neck_base_axis{2};
    yarp::sig::Vector neck_base_x0;
    double period, t0;

    yarp::os::BufferedPort<yarp::os::Bottle> log;

    /********************************************************************/
    bool configure(yarp::os::ResourceFinder& rf) override {
        period = rf.check("period", yarp::os::Value(.01)).asDouble();
        auto torso_n = rf.check("torso-n", yarp::os::Value(3)).asInt();

        yarp::os::Property option;
        option.put("device", "cartesiancontrollerclient");
        option.put("remote", "/icubSim/cartesianController/left_arm");
        option.put("local", "/study-arm-hockey/left_arm");

        bool ok = false;
        t0 = yarp::os::Time::now();
        while (yarp::os::Time::now() - t0 < 10.) {
            if (driver.open(option)) {
                ok = true;
                break;
            }
            yarp::os::Time::delay(1.);
        }

        if (!ok) {
            yError() << "Unable to open the Cartesian Controller";
            return false;
        }

        log.open("/study-arm-hockey/log");

        driver.view(arm);
        arm->storeContext(&startup_context_id);

        arm->setTrajTime(.6);
        yarp::sig::Vector dof;
        arm->getDOF(dof);
        dof = 1.;
        arm->setDOF(dof, dof);
        switch (torso_n) {
            case 0:
                arm->setLimits(2, 0., 0.);
            case 1:
                arm->setLimits(0, 30., 30.);
            case 2:
                arm->setLimits(1, 0., 0.);
        }

        yarp::sig::Matrix R = yarp::math::zeros(3, 3);
        R(0, 0) = -1.; R(2, 1) = -1.; R(1, 2) = -1.;
        o0 = yarp::math::dcm2axis(R);
        x0 = yarp::sig::Vector{-.25, .0, -.05};
        arm->goToPoseSync(x0, o0);
        arm->waitMotionDone(.1, 2.);

        yarp::sig::Vector o;
        arm->getPose(neck_base_axis, neck_base_x0, o);

        t0 = yarp::os::Time::now();
        return true;
    }

    /********************************************************************/
    bool close() override {
        arm->stopControl();
        arm->restoreContext(startup_context_id);
        driver.close();
        log.close();
        return true;
    }

    /********************************************************************/
    double getPeriod() override {
        return period;
    }

    /********************************************************************/
    bool updateModule() override {
        auto t = yarp::os::Time::now();
        arm->goToPose(x0 + yarp::sig::Vector{0., .15*sin(2.*M_PI*.1*(t - t0)), 0.}, o0);
        
        yarp::sig::Vector x, o;
        arm->getPose(neck_base_axis, x, o);
        auto& b = log.prepare();
        b.clear();
        b.addDouble(yarp::math::norm(neck_base_x0 - x));
        log.writeStrict();

        return true;
    }
};


/************************************************************************/
int main(int argc, char* argv[]) {
    yarp::os::Network yarp;
    if (!yarp.checkNetwork()) {
        yError() << "YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    ControllerModule mod;
    yarp::os::ResourceFinder rf;
    rf.configure(argc, argv);
    return mod.runModule(rf);
}
