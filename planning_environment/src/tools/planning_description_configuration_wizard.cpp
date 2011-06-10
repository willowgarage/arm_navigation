/*
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: E. Gil Jones

#include <planning_environment/tools/planning_description_configuration_wizard.h>
#include <qt4/QtGui/qapplication.h>

//in 100 hz ticks
WINDOW* left_win;
WINDOW* right_win;

PlanningDescriptionConfigurationWizard* pdcw;

bool inited = false;

void spin_function() {
  ros::WallRate r(100.0);
  unsigned int counter = 0;
  while(ros::ok()) {
    if(inited) {
      pdcw->sendTransforms();
      if(counter%CONTROL_SPEED == 0) {
        counter = 1;
        pdcw->sendMarkers();
      } else {
        counter++;
      }
    }
    r.sleep();
    ros::spinOnce();
  }
}

void quit(int sig)
{
  if(pdcw != NULL) {
    delete pdcw;
  }
  endwin();
  exit(0);
}

int main(int argc, char** argv)
{

  QApplication qtApp(argc, argv);

  srand(time(NULL));
  ros::init(argc, argv, "planning_description_configuration_wizard", ros::init_options::NoSigintHandler);

  if(argc < 3) {
    ROS_INFO_STREAM("Must specify a package and relative urdf file");
    exit(0);
  }

  std::string urdf_package = argv[1];
  std::string urdf_path = argv[2];
  pdcw = new PlanningDescriptionConfigurationWizard(urdf_package, urdf_path,NULL);

  qtApp.setActiveWindow(pdcw);

  pdcw->show();
  if(!pdcw->isInited()) {
    ROS_WARN_STREAM("Can't init. Exiting");
    exit(0);
  }

  inited = true;

  boost::thread spin_thread(boost::bind(&spin_function));

  /*
  initscr();
  use_default_colors();
  start_color();

  pdcw->outputPlanningEnvironmentLaunch();

  pdcw->setupGroups();

  pdcw->outputKinematicsLaunchFiles();
  pdcw->outputPlanningComponentVisualizerLaunchFile();
  pdcw->outputOMPLGroupYAML();
  pdcw->outputOMPLLaunchFile();
  pdcw->outputTrajectoryFilterLaunch();
  pdcw->outputJointLimitsYAML();

  pdcw->setJointsForCollisionSampling();

  pdcw->considerAlwaysAndDefaultInCollisionMarkers();

  printw("Finding often in collision pairs\n");
  refresh();
  pdcw->considerOftenInCollisionPairs();

  printw("Finding occasionally in collision pairs\n");
  refresh();
  pdcw->considerOccasionallyInCollisionPairs();

  printw("Performance testing and writing to file\n");
  refresh();
  pdcw->outputPlanningDescriptionYAML();
  printw("Press any key to exit\n");
  refresh();
  getch();
  endwin();
  ros::shutdown();
  */

  return qtApp.exec();
}





  

