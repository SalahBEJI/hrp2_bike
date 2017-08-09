#ifndef SOT_PEDAL_TRAJECTORY_HH
# define SOT_PEDAL_TRAJECTORY_HH

# include <dynamic-graph/entity.h>
# include <dynamic-graph/signal-ptr.h>
# include <dynamic-graph/signal-time-dependent.h>
# include <dynamic-graph/linear-algebra.h>
# include <sot/core/matrix-geometry.hh>

namespace dynamicgraph {
  namespace sot {
    namespace tools {
      class PedalTrajectory : public Entity
      {
	DYNAMIC_GRAPH_ENTITY_DECL();
      public:
	virtual ~PedalTrajectory ();
	PedalTrajectory (const std::string& name);
	/// Start tracking
	void start (const double& duration);
	/// Documentation
	virtual std::string getDocString () const;
	/// Set sampling period of control discretization
	void setSamplingPeriod (const double& period);
      protected:
	virtual void doStart (const double& duration);
	dynamicgraph::Signal < sot::MatrixHomogeneous, int > soutRFSOUT_;
	dynamicgraph::Signal < sot::MatrixHomogeneous, int > soutLFSOUT_;
	//	dynamicgraph::Signal < Vector, int > soutdotSOUT_;

	sot::MatrixHomogeneous& computeLFSout (sot::MatrixHomogeneous& sout, const int& inTime);
	sot::MatrixHomogeneous& computeRFSout (sot::MatrixHomogeneous& sout, const int& inTime);

	void initStartingAngles();
	double samplingPeriod_;
	double angularVelocity_;

	double xc_, zc_,R_;
	double angleLeftFoot_;
	double angleRightFoot_;

      }; // class PedalTrajectory
    } // tools
  } // namespace sot
} // namespace dynamicgraph

#endif // SOT_PEDAL_TRAJECTORY_SE3_HH
