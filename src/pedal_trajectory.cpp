#include <dynamic-graph/command.h>
#include <dynamic-graph/command-bind.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>
#include <dynamic-graph/factory.h>

#include "pedal_trajectory.hh"

namespace dynamicgraph {
  namespace sot {
    namespace tools {
      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(PedalTrajectory,
					 "PedalTrajectory");
      PedalTrajectory::PedalTrajectory (const std::string& name) :
	Entity (name),
	soutRFSOUT_ ("PedalTrajectory("+name+
		   ")::output(MatrixHomogeneous)::soutRF"),
	soutLFSOUT_ ("PedalTrajectory("+name+
		      ")::output(MatrixHomogeneous)::soutLF"),
	samplingPeriod_ (0.), 
	angularVelocity_(0.),
	xc_(0.15),
	zc_(0.2), //center of crank gear
	R_(0.12) //pedal-center of crank gear
      {
	signalRegistration (soutRFSOUT_);
	signalRegistration (soutLFSOUT_);
	soutRFSOUT_.setFunction (boost::bind
				 (&PedalTrajectory::computeRFSout,
				  this, _1, _2));
	soutLFSOUT_.setFunction (boost::bind
				  (&PedalTrajectory::computeLFSout,
				   this, _1, _2));
	std::string docstring;
	docstring =
	  "  Set sampling period of control discretization.\n"
	  "\n"
	  "    Input:\n"
	  "      - a floating point value.\n"
	  "\n";
	addCommand ("setSamplingPeriod",
		    new command::Setter <PedalTrajectory, double>
		    (*this, &PedalTrajectory::setSamplingPeriod, docstring));
	docstring =
	  "  Start tracking.\n"
	  "\n"
	  "    Input\n"
	  "      - duration of the motion.\n"
	  "\n"
	  "\n  Read init and goal signals, compute output trajectory and"
	  " start\n"
	  "tracking.\n";
	addCommand ("start",
		    new command::Setter <PedalTrajectory, double>
		    (*this, &PedalTrajectory::start, docstring));

	addCommand ("getAngleLeftFoot",
            command::makeDirectGetter (*this, &angleLeftFoot_,
                              command::docDirectGetter ("Get angle left foot",
                              "double")));

    addCommand ("getAngleRightFoot",
            command::makeDirectGetter (*this, &angleRightFoot_,
                              command::docDirectGetter ("Get angle right foot",
                              "double")));

	addCommand ("getSamplingPeriod",
            command::makeDirectGetter (*this, &samplingPeriod_,
                              command::docDirectGetter ("Get sampling period",
                              "double")));

	addCommand ("getAngularVelocity",
            command::makeDirectGetter (*this, &angularVelocity_,
                              command::docDirectGetter ("Get angular velocity",
                              "double")));
	addCommand ("setAngularVelocity",
            command::makeDirectSetter (*this, &angularVelocity_,
                              command::docDirectSetter ("Set angular velocity",
                              "double")));
	
	initStartingAngles();
      }

      PedalTrajectory::~PedalTrajectory ()
      {
      }

      std::string PedalTrajectory::getDocString () const
      {
	std::string doc =
	  "Perform a cubic interpolation in between two vectors.\n"
	  "\n"
	  "  Initial pose is given by signal 'init', Target position is given"
	  " by signal\n"
	  "  'goal'. Interpolation is performed with zero velocities at start"
	  " and goal\n"
	  "  positions.\n";
	return doc;
      }


      sot::MatrixHomogeneous & 
      PedalTrajectory::computeLFSout (sot::MatrixHomogeneous& sout,
				    const int&)
      {
    sout(0,1) = 0.;
	sout(0,0) = 1.0; sout(1,1)=1.0; sout(2,2)=1.0; sout(3,3)=1.0;
	double incrementAngle = angularVelocity_*samplingPeriod_;
	angleLeftFoot_ -= incrementAngle;
	double xpL=((cos(angleLeftFoot_)*R_)+xc_);
	double zpL=((sin(angleLeftFoot_)*R_)+zc_);

	sout(0,3) = xpL; sout(1,3)=0.095; sout(2,3) = zpL;
	return sout;
      }
      sot::MatrixHomogeneous & 
      PedalTrajectory::computeRFSout (sot::MatrixHomogeneous& sout,
				    const int& )
      {
    sout(0,1) = 0.;
	sout(0,0) = 1.0; sout(1,1)=1.0; sout(2,2)=1.0; sout(3,3)=1.0;
	double incrementAngle = angularVelocity_*samplingPeriod_;
	angleRightFoot_ -= incrementAngle;

	double xpR=((cos(angleRightFoot_)*R_)+xc_);
	double zpR=((sin(angleRightFoot_)*R_)+zc_);

	sout(0,3) = xpR; sout(1,3)=-0.095; sout(2,3) = zpR;
	return sout;
      }


      void PedalTrajectory::setSamplingPeriod (const double& period)
      {
	samplingPeriod_ = period;
      }

      void PedalTrajectory::start (const double& duration)
      {
	doStart (duration);
      }

      void PedalTrajectory::doStart (const double& )
      {
	// Check that sampling period has been initialized
	if (samplingPeriod_ <= 0)
	  throw ExceptionSignal (ExceptionSignal::NOT_INITIALIZED,
				 "PedalTrajectory: samplingPeriod should"
				 " be positive. Are you sure you did\n"
				 "initialize it?");

      
      
      }
      
      /// !!! We assume that the robot is starting with a specific 
      /// foot position: 
      void PedalTrajectory::initStartingAngles()
      {
	angleLeftFoot_= -5*M_PI/6;
	angleRightFoot_= M_PI/6;
      }
    } // tools
  } // namespace sot
} // namespace dynamicgraph
