//
// robot_bola:main.cpp
//
// Author: Miguel González-Fierro <mgpalaci@ing.uc3m.es>, (C) 2012
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
//

#include <iostream>
#include "navigation.h"
#include "HOAP_rave.h"

#include <openrave-core.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <openrave/planningutils.h>


int main(int argc, char ** argv)
{
  //Initialize the object
  Rave HOAP;
  string envfile="../share/simple.env.xml";

//*************************************
  HOAP.Init(envfile);
  printf("Environment initialized\n");
  //usleep(10*1000*1000);
  cout << endl << endl << endl << endl << endl;
//  cout << endl << endl << endl << endl << endl;
//  cout << endl << endl << endl << endl << endl;
  HOAP.InitialPose();

  EnvironmentBasePtr penv;
  penv=HOAP.Getpenv();
  usleep(3*1000*1000);

  //penv->StartSimulation(TIME_STEP);
  if(penv->IsSimulationRunning()){
    cout << "Simulation running" << endl;
  }
  else {
    cout << "Simulation not running" << endl;
  }


  string fdata="../share/m03.csv";
  HOAP.ReadData(fdata);

  //getchar();
  usleep(3*1000*1000);
  HOAP.Stop();

  return 0;

}


