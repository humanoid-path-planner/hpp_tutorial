#
# Copyright (c) 2014 CNRS
# Authors: Florent Lamiraux
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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/python.cmake)

SET(PROJECT_NAME hpp_tutorial)
SET(PROJECT_DESCRIPTION
  "Tutorial for humanoid path planner platform."
)
SET(PROJECT_URL "")

FINDPYTHON(2.7 EXACT REQUIRED)

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

ADD_DOC_DEPENDENCY("hpp-core >= 4")
ADD_DOC_DEPENDENCY("hpp-corbaserver >= 4")
ADD_REQUIRED_DEPENDENCY("hpp-manipulation-corba >= 4")

# Create and install executable running the corba server
ADD_EXECUTABLE (hpp-tutorial-server
  src/tutorial.cc
)
# Link executable with hpp-corbaserver library
PKG_CONFIG_USE_DEPENDENCY (hpp-tutorial-server hpp-corbaserver)
# Install executable
INSTALL (TARGETS hpp-tutorial-server DESTINATION ${CMAKE_INSTALL_BINDIR})


SET(CATKIN_PACKAGE_SHARE_DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

install(FILES
  package.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  )
install(FILES
  urdf/pr2.urdf
  urdf/box.urdf
  urdf/cup.urdf
  urdf/door.urdf
  urdf/box_color.urdf
  urdf/rod.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  srdf/pr2.srdf
  srdf/pr2_manipulation.srdf
  srdf/box.srdf
  srdf/cup.srdf
  srdf/rod.srdf
  srdf/door.srdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/srdf
  )
install(FILES
  meshes/box.dae
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
  )
install (FILES
  src/hpp/corbaserver/pr2/robot.py
  src/hpp/corbaserver/pr2/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/pr2)
install (FILES
  src/hpp/corbaserver/rod/robot.py
  src/hpp/corbaserver/rod/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/rod)
install (FILES
  src/hpp/corbaserver/manipulation/pr2/robot.py
  src/hpp/corbaserver/manipulation/pr2/__init__.py
  DESTINATION ${PYTHON_SITELIB}/hpp/corbaserver/manipulation/pr2)

# Installation for documentation
install(FILES
  urdf/pr2.urdf
  urdf/box.urdf
  DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html/urdf
  )
install(FILES
  script/tutorial_1.py
  script/tutorial_2.py
  DESTINATION
  ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html/script
  )
SETUP_PROJECT_FINALIZE()
