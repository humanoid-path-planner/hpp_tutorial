#
# Copyright (c) 2014, 2020 CNRS
# Authors: Florent Lamiraux,  Guilhem Saurel
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

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp_tutorial)
SET(PROJECT_DESCRIPTION "Tutorial for humanoid path planner platform.")

SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/hpp.cmake)
INCLUDE(cmake/python.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

FINDPYTHON()

ADD_PROJECT_DEPENDENCY("hpp-manipulation-corba" REQUIRED)

ADD_SUBDIRECTORY(src)

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
  urdf/kitchen_area.urdf
  urdf/box_color.urdf
  urdf/rod.urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
  )
install(FILES
  srdf/pr2.srdf
  srdf/pr2_manipulation.srdf
  srdf/box.srdf
  srdf/cup.srdf
  srdf/kitchen_area.srdf
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

INSTALL(FILES
  Media/models/meshes/glasses/__Color_A05_4.png
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Media/models/meshes/glasses
)

INSTALL(FILES
  Media/materials/textures/ganz_rechts_neben_kuecheninsel.png
  Media/materials/textures/hinter_shopschraenken_an_saeule.png
  Media/materials/textures/wallpaper_ai_unihb.png
  Media/materials/textures/stove.png
  Media/materials/textures/ganz_links_vorn_bei_biolab.png
  Media/materials/textures/wallpaper2.png
  Media/materials/textures/ganz_links_neben_roboterplakat.png
  Media/materials/textures/auf_saeule_bei_shop.png
  Media/materials/textures/wallpaper_project_logos.png
  Media/materials/textures/wallpaper.png
  Media/materials/textures/sink.png
  Media/materials/textures/mittig_neben_roboterplakat.png
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/Media/materials/textures
)

# Installation for documentation
IF(INSTALL_DOCUMENTATION)
  install(FILES
    urdf/pr2.urdf
    urdf/box.urdf
    DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html/urdf
    )
  install(FILES
    script/tutorial_1.py
    script/tutorial_2.py
    script/tutorial_3.py
    DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}/doxygen-html/script
    )
ENDIF(INSTALL_DOCUMENTATION)

ADD_LIBRARY(${PROJECT_NAME} INTERFACE)
INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)
