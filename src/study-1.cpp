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
#include <tuple>
#include <iterator>
#include <algorithm>
#include <string>
#include <sstream>
#include <vector>
#include <fstream>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/Math.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>


/************************************************************************/
class ControllerModule: public yarp::os::RFModule
{
    yarp::dev::PolyDriver drv_arm;
    yarp::dev::ICartesianControl* arm;

    yarp::dev::PolyDriver drv_gaze;
    yarp::dev::IGazeControl* gaze;

    int startup_context_id_arm, startup_context_id_gaze;
    yarp::sig::Vector x0{-.25, .0, -.05}, o0;
    yarp::sig::Vector fixation{-.7, .0, -.05};

    double y_max, y_delta, y;

    static constexpr double wait_ping{.1};
    static constexpr double wait_tmo{3.};

    std::string table_file;
    typedef std::tuple<double, yarp::sig::Vector, yarp::sig::Vector> Entry;
    std::vector<Entry> table;

    /********************************************************************/
    auto helperWaitDevice(yarp::dev::PolyDriver& driver,
                          const yarp::os::Property& options,
                          const std::string& device_name) {
        const auto t0 = yarp::os::Time::now();
        while (yarp::os::Time::now() - t0 < 10.) {
            if (driver.open(const_cast<yarp::os::Property&>(options))) {
                return true;
            }
            yarp::os::Time::delay(1.);
        }

        yError() << "Unable to open the Device Driver:" << device_name;
        return false;
    }

    /********************************************************************/
    void helperFillVector(yarp::os::ResourceFinder& rf,
                          const std::string& option_name,
                          yarp::sig::Vector& v) {
        if (rf.check(option_name)) {
            if (const yarp::os::Bottle* b = rf.find(option_name).asList()) {
                for (size_t i = 0; i < std::min(v.length(), b->size()); i++) {
                    v[i] = b->get(i).asDouble();
                }
            }
        }
    }

    /********************************************************************/
    const auto helperToString(const yarp::sig::Vector& v) const {
        std::ostringstream str;
        for (auto it = v.begin(); it != v.end(); it++) {
            str << *it;
            if (it != v.end() - 1) {
                str << "\t";
            }
        }
        return str.str();
    }

    /**************************************************************************/
    static auto helperCompareEntries(const Entry& e1, const Entry& e2) {
        return (std::get<0>(e1) < std::get<0>(e2)); 
    }

    /********************************************************************/
    bool configure(yarp::os::ResourceFinder& rf) override {
        table_file = rf.getHomeContextPath() + "/" +
                     rf.check("table-file", yarp::os::Value("table.tsv")).asString();
        const auto torso_joints = rf.check("torso-joints", yarp::os::Value(1)).asInt();
        const auto torso_pitch = rf.check("torso-pitch", yarp::os::Value(30.)).asDouble();
        y_max = std::abs(rf.check("y-max", yarp::os::Value(.15)).asDouble());
        y_delta = std::abs(rf.check("y-delta", yarp::os::Value(.005)).asDouble());
        helperFillVector(rf, "x0", x0);
        helperFillVector(rf, "fixation", fixation);

        yarp::os::Property options_arm;
        options_arm.put("device", "cartesiancontrollerclient");
        options_arm.put("remote", "/icubSim/cartesianController/left_arm");
        options_arm.put("local", "/study-arm-hockey/left_arm");
        if (!helperWaitDevice(drv_arm, options_arm, "Cartesian Controller")) {
            return false;
        }

        yarp::os::Property options_gaze;
        options_gaze.put("device", "gazecontrollerclient");
        options_gaze.put("remote", "/iKinGazeCtrl");
        options_gaze.put("local", "/study-arm-hockey/gaze");
        if (!helperWaitDevice(drv_gaze, options_gaze, "Gaze Controller")) {
            drv_arm.close();
            return false;
        }

        drv_arm.view(arm);
        arm->storeContext(&startup_context_id_arm);

        arm->setTrajTime(.6);
        yarp::sig::Vector dof;
        arm->getDOF(dof);
        dof = 1.;
        arm->setDOF(dof, dof);
        switch (torso_joints) {
            case 0:
                arm->setLimits(2, 0., 0.);
            case 1:
                arm->setLimits(0, torso_pitch, torso_pitch);
            case 2:
                arm->setLimits(1, 0., 0.);
        }

        yarp::sig::Matrix R = yarp::math::zeros(3, 3);
        R(0, 0) = -1.; R(2, 1) = -1.; R(1, 2) = -1.;
        o0 = yarp::math::dcm2axis(R);
        arm->goToPoseSync(x0, o0);
        arm->waitMotionDone(wait_ping, wait_tmo);

        drv_gaze.view(gaze);
        gaze->storeContext(&startup_context_id_gaze);

        gaze->setNeckTrajTime(.6);
        gaze->setEyesTrajTime(.3);
        gaze->blockNeckRoll(0.);

        gaze->lookAtFixationPoint(fixation);
        gaze->waitMotionDone(wait_ping, wait_tmo);

        y = -y_max;
        return true;
    }

    /********************************************************************/
    auto writeTable() {
        std::sort(begin(table), end(table), helperCompareEntries);
        std::ofstream fout(table_file);
        if (fout.is_open()) {
            for (auto entry = table.begin(); entry != table.end(); entry++) {
                const auto& y = std::get<0>(*entry);
                const auto& q_arm = std::get<1>(*entry);
                const auto& q_gaze = std::get<2>(*entry);
                fout << y << "\t"
                     << helperToString(q_arm) << "\t"
                     << helperToString(q_gaze);
                if (entry != table.end() - 1) {
                    fout << std::endl;
                }
            }
            fout.close();
            yInfo() << "Table written to" << table_file;
            return true;
        } else {
            yError() << "Unable to write to file" << table_file;
            return false;
        }
    }

    /********************************************************************/
    bool close() override {
        arm->stopControl();
        arm->restoreContext(startup_context_id_arm);
        drv_arm.close();

        gaze->stopControl();
        gaze->restoreContext(startup_context_id_gaze);
        drv_gaze.close();

        writeTable();
        return true;
    }

    /********************************************************************/
    double getPeriod() override {
        return 0.;
    }

    /********************************************************************/
    bool updateModule() override {
        arm->goToPoseSync(x0 + yarp::sig::Vector{0., y, 0.}, o0);
        arm->waitMotionDone(wait_ping, wait_tmo);

        gaze->lookAtFixationPoint(fixation);
        gaze->waitMotionDone(wait_ping, wait_tmo);

        yarp::sig::Vector xdhat, odhat, q_arm;
        arm->getDesired(xdhat, odhat, q_arm);
        
        yarp::sig::Vector q_gaze;
        gaze->getJointsDesired(q_gaze);

        table.push_back(std::make_tuple(y, q_arm, q_gaze));

        y += y_delta;
        return (y <= y_max + y_delta/2.);
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
    rf.setDefaultContext("study-air-hockey");
    rf.configure(argc, argv);
    return mod.runModule(rf);
}
