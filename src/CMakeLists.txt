#
# Copyright (c) 2019 CNRS
# Authors: Joseph Mirabel
#
#
# This file is part of hpp_tutorial
# hpp_tutorial is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp_tutorial is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp_tutorial  If not, see
# <http://www.gnu.org/licenses/>.


### Tutorial 1

# Create and install executable running the corba server
ADD_EXECUTABLE (hpp-tutorial-1
  tutorial_1.cc
)
# Link executable with hpp-corbaserver library
PKG_CONFIG_USE_DEPENDENCY (hpp-tutorial-1 hpp-core)
# Install executable
INSTALL (TARGETS hpp-tutorial-1 DESTINATION ${CMAKE_INSTALL_BINDIR})


### Tutorial 2

# Create and install executable running the corba server
ADD_EXECUTABLE (hpp-tutorial-2-server
  tutorial_2.cc
)
# Link executable with hpp-corbaserver library
PKG_CONFIG_USE_DEPENDENCY (hpp-tutorial-2-server hpp-corbaserver)
# Install executable
INSTALL (TARGETS hpp-tutorial-2-server DESTINATION ${CMAKE_INSTALL_BINDIR})

