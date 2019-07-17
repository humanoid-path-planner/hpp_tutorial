// Copyright (c) 2018, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-core.
// hpp-core is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-core is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-core. If not, see <http://www.gnu.org/licenses/>.

#include <hpp/pinocchio/device.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/pinocchio/urdf/util.hh>

#include <hpp/core/problem-solver.hh>
#include <hpp/core/plugin.hh>

using namespace hpp::pinocchio;
using namespace hpp::core;

int main ()
{
  ProblemSolverPtr_t ps = ProblemSolver::create ();

  // Create robot
  DevicePtr_t device = ps->createRobot ("pr2");
  hpp::pinocchio::urdf::loadRobotModel (device,
      "planar", "hpp_tutorial", "pr2", "", "");
  device->controlComputation ((Computation_t)(JOINT_POSITION || JACOBIAN));
  ps->robot (device);

  device->rootJoint()->lowerBound (0, -4);
  device->rootJoint()->upperBound (0, -3);
  device->rootJoint()->lowerBound (1, -5);
  device->rootJoint()->upperBound (1, -3);

  // Add obstacle
  DevicePtr_t obstacle = Device::create ("kitchen");
  hpp::pinocchio::urdf::loadUrdfModel (obstacle,
      "anchor", "iai_maps", "kitchen_area");
  obstacle->controlComputation (JOINT_POSITION);
  ps->addObstacle (obstacle, true, true);

  // Create initial and final configuration
  Configuration_t q_init (device->currentConfiguration());
  q_init.head<2>() << -3.2, -4;
  q_init[device->getJointByName("torso_lift_joint")->rankInConfiguration()] = 0.2;
  ps->initConfig (ConfigurationPtr_t(new Configuration_t(q_init)));

  Configuration_t q_goal (q_init);
  q_goal.head<2>() << -3.2, -4;
  q_goal[device->getJointByName("torso_lift_joint")->rankInConfiguration()] = 0;
  q_goal[device->getJointByName("l_shoulder_lift_joint")->rankInConfiguration()] =  0.5;
  q_goal[device->getJointByName("l_elbow_flex_joint"   )->rankInConfiguration()] = -0.5;
  q_goal[device->getJointByName("r_shoulder_lift_joint")->rankInConfiguration()] =  0.5;
  q_goal[device->getJointByName("r_elbow_flex_joint"   )->rankInConfiguration()] = -0.5;
  ps->addGoalConfig (ConfigurationPtr_t(new Configuration_t(q_goal)));

  bool loaded = plugin::loadPlugin ("spline-gradient-based.so", ps);
  if (loaded)
    ps->addPathOptimizer("SplineGradientBased_bezier1");
  else {
    std::cerr << "Could not load spline-gradient-based.so" << std::endl;
    ps->addPathOptimizer("RandomShortcut");
  }

  ps->solve();
  std::cout << "Problem solved.\n";

  return 0;
}
