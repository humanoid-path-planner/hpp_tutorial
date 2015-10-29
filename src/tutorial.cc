//
// Copyright (c) 2014 CNRS
// Authors: Florent Lamiraux
//
// This file is part of hpp_tutorial
// hpp_tutorial is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp_tutorial is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp_tutorial  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/util/pointer.hh>

#include <hpp/core/basic-configuration-shooter.hh>
#include <hpp/core/config-validations.hh>
#include <hpp/core/connected-component.hh>
#include <hpp/core/constraint-set.hh>
#include <hpp/core/path-planner.hh>
#include <hpp/core/path-validation.hh>
#include <hpp/core/roadmap.hh>
#include <hpp/core/steering-method.hh>

#include "hpp/corbaserver/server.hh"

namespace hpp {
  namespace tutorial {
    // forward declaration of class Planner
    HPP_PREDEF_CLASS (Planner);
    // Planner objects are manipulated only via shared pointers
    typedef boost::shared_ptr <Planner> PlannerPtr_t;

    /// Example of path planner
    class Planner : public core::PathPlanner
    {
    public:
      /// Create an instance and return a shared pointer to the instance
      static PlannerPtr_t create (const core::Problem& problem,
				  const core::RoadmapPtr_t& roadmap)
      {
	Planner* ptr = new Planner (problem, roadmap);
	PlannerPtr_t shPtr (ptr);
	ptr->init (shPtr);
	return shPtr;
      }

      /// One step of extension.
      ///
      /// This method implements one step of your algorithm. The method
      /// will be called iteratively until one goal configuration is accessible
      /// from the initial configuration.
      ///
      /// We will see how to implement a basic PRM algorithm.
      virtual void oneStep ()
      {
	// Retrieve the robot the problem has been defined for.
	model::DevicePtr_t robot (problem ().robot ());
	// Retrieve the path validation algorithm associated to the problem
	core::PathValidationPtr_t pathValidation (problem ().pathValidation ());
	// Retrieve configuration validation methods associated to the problem
	core::ConfigValidationsPtr_t configValidations
	  (problem ().configValidations ());
	// Retrieve the steering method
	core::SteeringMethodPtr_t sm (problem ().steeringMethod ());
	// Retrieve the constraints the robot is subject to
	core::ConstraintSetPtr_t constraints (problem ().constraints ());
	// Retrieve roadmap of the path planner
	core::RoadmapPtr_t r (roadmap ());
	// shoot a valid random configuration
	core::ConfigurationPtr_t qrand;
	// Report of configuration validation: unused here
	core::ValidationReportPtr_t validationReport;
	do {
	  qrand = shooter_->shoot ();
	} while (!configValidations->validate (*qrand, validationReport));
	// Add qrand as a new node
	core::NodePtr_t newNode = r->addNode (qrand);
	// try to connect the random configuration to each connected component
	// of the roadmap.
	for (core::ConnectedComponents_t::const_iterator itcc =
	       r->connectedComponents ().begin ();
	     itcc != r->connectedComponents ().end (); ++itcc) {
	  core::ConnectedComponentPtr_t cc = *itcc;
	  // except its own connected component of course
	  if (cc != newNode->connectedComponent ()) {
	    double d;
	    // Get nearest node to qrand in connected component
	    core::NodePtr_t nearest = r->nearestNode (qrand, cc, d);
	    core::ConfigurationPtr_t qnear = nearest->configuration ();
	    // Create local path between qnear and qrand
	    core::PathPtr_t localPath = (*sm) (*qnear, *qrand);
	    // validate local path
	    core::PathPtr_t validPart;
	    // report on path validation: unused here
	    core::PathValidationReportPtr_t report;
	    if (pathValidation->validate (localPath, false, validPart,
					  report)) {
	      // Create node and edges with qrand and the local path
	      r->addEdge (nearest, newNode, localPath);
	      r->addEdge (newNode, nearest, localPath->reverse ());
	    }
	  }	  
	}
      }
    protected:
      /// Protected constructor
      /// Users need to call Planner::create in order to create instances.
      Planner (const core::Problem& problem,
	       const core::RoadmapPtr_t& roadmap) :
	core::PathPlanner (problem, roadmap),
	shooter_ (core::BasicConfigurationShooter::create (problem.robot ()))
      {
      }
      /// Store weak pointer to itself
      void init (const PlannerWkPtr_t& weak)
      {
	core::PathPlanner::init (weak);
	weakPtr_ = weak;
      }
    private:
      /// Configuration shooter to uniformly shoot random configurations
      core::BasicConfigurationShooterPtr_t shooter_;
      /// weak pointer to itself
      PlannerWkPtr_t weakPtr_;
    }; // class Planner
  } // namespace tutorial
} // namespace hpp

// main function of the corba server
int main (int argc, const char* argv[])
{
  // create a ProblemSolver instance.
  // This class is a container that does the interface between hpp-core library
  // and component to be running in a middleware like CORBA or ROS.
  hpp::core::ProblemSolverPtr_t problemSolver =
    hpp::core::ProblemSolver::create ();
  // Add the new planner type in order to be able to select it from python
  // client.
  problemSolver->addPathPlannerType ("PRM", hpp::tutorial::Planner::create);
  // Create the CORBA server.
  hpp::corbaServer::Server server (problemSolver, argc, argv, true);
  // Start the CORBA server.
  server.startCorbaServer ();
  // Wait for CORBA requests.
  server.processRequest (true);
}
