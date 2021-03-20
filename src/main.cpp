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
#include <yarp/os/RFModule.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;

class CtrlThread : public PeriodicThread
{
protected:
    PolyDriver         client;
    ICartesianControl *arm;

    Vector xd;
    Vector od;

    int startup_context_id;

    double t;
    double t0;
    double t1;

public:
  CtrlThread() : PeriodicThread(1.0) {}

  virtual bool threadInit()
  {
      // open a client interface to connect to the cartesian server of the simulator
      // we suppose that:
      //
      // 1 - the iCub simulator is running
      //     (launch: iCub_SIM)
      //
      // 2 - the cartesian server is running
      //     (launch: yarprobotinterface --context simCartesianControl)
      //
      // 3 - the cartesian solver for the left arm is running too
      //     (launch: iKinCartesianSolver --context simCartesianControl --part left_arm)
      //
      Property option;
      option.put("device", "cartesiancontrollerclient");
      option.put("remote", "/icubSim/cartesianController/left_arm");
      option.put("local", "/cartesian_client/left_arm");

      // let's give the controller some time to warm up
      bool ok = false;
      double t0 = Time::now();
      while (Time::now() - t0 < 10.0)
      {
          // this might fail if controller
          // is not connected to solver yet
          if (client.open(option))
          {
              ok = true;
              break;
          }

          Time::delay(1.0);
      }

      if (!ok)
      {
          yError() << "Unable to open the Cartesian Controller";
          return false;
      }

      // open the view
      client.view(arm);

      // latch the controller context in order to preserve
      // it after closing the module
      // the context contains the dofs status, the tracking mode,
      // the resting positions, the limits and so on.
      arm->storeContext(&startup_context_id);

      // set trajectory time
      arm->setTrajTime(1.0);

      // get the torso dofs
      Vector newDof, curDof;
      arm->getDOF(curDof);
      newDof = curDof;

      // enable the torso yaw and pitch
      // disable the torso roll
      newDof[0] = 1;
      newDof[1] = 0;
      newDof[2] = 1;

      // send the request for dofs reconfiguration
      arm->setDOF(newDof, curDof);

      // impose some restriction on the torso pitch
      limitTorsoPitch();

      xd.resize(3);
      od.resize(4);

      yInfo() << "Thread started successfully";
      t = t0 = t1 = Time::now();

      return true;
    }

    virtual void run()
    {
        t=Time::now();

        generateTarget();

        // go to the target :)
        // (in streaming)
        arm->goToPose(xd,od);

        // some verbosity
        printStatus();
    }

    virtual void threadRelease()
    {
        // we require an immediate stop
        // before closing the client for safety reason
        arm->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        arm->restoreContext(startup_context_id);

        client.close();
    }

    void generateTarget()
    {
        // translational target part: a circular trajectory
        // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
        // and frequency 0.1 Hz
        xd[0]=-0.3;
        xd[1]=-0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
        xd[2]=+0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));

        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
    }

    void limitTorsoPitch()
    {
        int axis=0; // pitch joint
        double min, max;

        // sometimes it may be helpful to reduce
        // the range of variability of the joints;
        // for example here we don't want the torso
        // to lean out more than 30 degrees forward

        // we keep the lower limit
        arm->getLimits(axis,&min,&max);
        arm->setLimits(axis,min,MAX_TORSO_PITCH);
    }

    void printStatus()
    {
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

            // we get the current arm pose in the
            // operational space
            if (!arm->getPose(x,o))
                return;

            // we get the final destination of the arm
            // as found by the solver: it differs a bit
            // from the desired pose according to the tolerances
            if (!arm->getDesired(xdhat,odhat,qdhat))
                return;

            double e_x=norm(xdhat-x);
            double e_o=norm(odhat-o);

            yInfo()<<"+++++++++";
            yInfo()<<"xd          [m] = "<<xd.toString();
            yInfo()<<"xdhat       [m] = "<<xdhat.toString();
            yInfo()<<"x           [m] = "<<x.toString();
            yInfo()<<"od        [rad] = "<<od.toString();
            yInfo()<<"odhat     [rad] = "<<odhat.toString();
            yInfo()<<"o         [rad] = "<<o.toString();
            yInfo()<<"norm(e_x)   [m] = "<<e_x;
            yInfo()<<"norm(e_o) [rad] = "<<e_o;
            yInfo()<<"---------";

            t1=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        // retrieve command line options
        double period=rf.check("period",Value(CTRL_THREAD_PER)).asDouble();

        // set the thread period in [s]
        thr.setPeriod(period);

        return thr.start();
    }

    virtual bool close()
    {
        thr.stop();
        return true;
    }

    virtual double getPeriod()
    {
        return 1.0;
    }

    virtual bool updateModule()
    {
        yInfo()<<"Running happily...";
        return true;
    }
};



int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        yError()<<"YARP doesn't seem to be available";
        return EXIT_FAILURE;
    }

    CtrlModule mod;
    ResourceFinder rf;
    rf.configure(argc,argv);
    return mod.runModule(rf);
}
