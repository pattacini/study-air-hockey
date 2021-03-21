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
#include <limits>
#include <iterator>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>
#include <fstream>

#include <yarp/os/Network.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Value.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>

#include <iCub/ctrl/minJerkCtrl.h>

#include "spline.h"


/************************************************************************/
class ControllerModule: public yarp::os::RFModule
{
    std::vector<yarp::dev::PolyDriver> drv{3};
    std::vector<yarp::dev::IPositionControl*> ipos{3};
    std::vector<yarp::dev::IPositionDirect*> iposd{3};

    std::string table_file;
    std::vector<std::shared_ptr<tk::spline>> interp;
    double y_min, y_max;

    yarp::os::BufferedPort<yarp::os::Bottle> targetPort;
    std::shared_ptr<iCub::ctrl::minJerkTrajGen> reference;
    yarp::sig::Vector target{0.};

    /********************************************************************/
    void helperOpenDevice(const int i, const std::string& device_name) {
        yarp::os::Property options;
        options.put("device", "remote_controlboard");
        options.put("remote", "/icubSim/" + device_name);
        options.put("local", "/study-arm-hockey/" + device_name);
        drv[i].open(options);
        
        yarp::dev::IControlMode* imod;
        drv[i].view(ipos[i]);
        drv[i].view(iposd[i]);
        drv[i].view(imod);
        int naxes;
        ipos[i]->getAxes(&naxes);
        std::vector<int> modes(naxes, VOCAB_CM_POSITION);
        std::vector<double> vels(naxes, 20.);
        std::vector<double> accs(naxes, std::numeric_limits<double>::max());
        std::vector<double> poss(naxes, 0.);
        imod->setControlModes(modes.data());
        ipos[i]->setRefSpeeds(vels.data());
        ipos[i]->setRefAccelerations(accs.data());
        switch (naxes) {
            case 3:
                for (size_t i = 0; i < 3; i++) {
                    poss[2 - i] = interp[i]->operator()(0.);
                }
                break;
            case 16:
                for (size_t i = 0; i < 7; i++) {
                    poss[i] = interp[3 + i]->operator()(0.);
                }
                break;
            case 6:
                for (size_t i = 0; i < 6; i++) {
                    poss[i] = interp[3 + 7 + i]->operator()(0.);
                }
                break;
        }
        ipos[i]->positionMove(poss.data());
        auto done = false;
        while(!done) {
            yarp::os::Time::delay(1.);
            ipos[i]->checkMotionDone(&done);
        }

        std::fill(begin(modes), end(modes), VOCAB_CM_POSITION_DIRECT);
        imod->setControlModes(modes.data());
    }

    /********************************************************************/
    auto readTable() {
        std::ifstream fin(table_file);
        if (fin.is_open()) {
            std::vector<double> y;
            std::vector<std::vector<double>> q(16);
            double read;
            while (!fin.eof()) {
                fin >> read;
                y.push_back(read);
                for (size_t j = 0; j < q.size(); j++) {
                    fin >> read;
                    q[j].push_back(read);
                }
            }
            y_min = *std::min_element(begin(y), end(y));
            y_max = *std::max_element(begin(y), end(y));
            for (const auto& q_:q) {
                interp.push_back(std::make_shared<tk::spline>(y, q_));
            }
            fin.close();
            yInfo() << "Table successfully read from" << table_file;
            return true;
        } else {
            yError() << "Unable to read from file" << table_file;
            return false;
        }
    }

    /********************************************************************/
    bool configure(yarp::os::ResourceFinder& rf) override {
        table_file = rf.findFile("table-file");
        const auto T = std::abs(rf.check("T", yarp::os::Value(1.)).asDouble());

        if (!readTable()) {
            return false;
        }

        helperOpenDevice(0, "torso");
        helperOpenDevice(1, "left_arm");
        helperOpenDevice(2, "head");

        targetPort.open("/study-arm-hockey/target");
        reference = std::make_shared<iCub::ctrl::minJerkTrajGen>(target, getPeriod(), T);
        return true;
    }

    /********************************************************************/
    bool close() override {
        for (auto& i:ipos) {
            i->stop();
        }
        for (auto& d:drv) {
            d.close();
        }

        targetPort.close();
        return true;
    }

    /********************************************************************/
    double getPeriod() override {
        return .01;
    }

    /********************************************************************/
    bool updateModule() override {
        if (auto* b = targetPort.read(false)) {
            const auto p = (b->get(0).asDouble() * (y_max - y_min) + (y_max + y_min)) / 2.;
            target[0] = std::max(std::min(p, y_max), y_min);
        }
        reference->computeNextValues(target);

        std::vector<double> pos_torso(3);
        for (size_t i = 0; i < pos_torso.size(); i++) {
            pos_torso[pos_torso.size() - 1 - i] = interp[i]->operator()(reference->getPos()[0]);
        }
        iposd[0]->setPositions(pos_torso.data());

        std::vector<double> pos_arm(7);
        for (size_t i = 0; i < pos_arm.size(); i++) {
            pos_arm[i] = interp[pos_torso.size() + i]->operator()(reference->getPos()[0]);
        }
        iposd[1]->setPositions(pos_arm.size(), std::vector<int>({0, 1, 2, 3, 4, 5, 6, 7}).data(),
                              pos_arm.data());

        std::vector<double> pos_head(6);
        for (size_t i = 0; i < pos_head.size(); i++) {
            pos_head[i] = interp[pos_torso.size() + pos_arm.size() + i]->operator()(reference->getPos()[0]);
        }
        iposd[2]->setPositions(pos_head.data());

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
    rf.setDefaultContext("study-air-hockey");
    rf.setDefault("table-file", "table.tsv");
    rf.configure(argc, argv);
    return mod.runModule(rf);
}
