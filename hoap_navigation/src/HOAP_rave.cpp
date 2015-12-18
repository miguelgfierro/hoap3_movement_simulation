//
// HOAP_rave.cpp
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

#include <openrave-core.h>
#include <vector>
#include <fstream>
#include "HOAP_rave.h"
#include <iostream>

using namespace OpenRAVE;
using namespace std;

void SetViewer(EnvironmentBasePtr penv, const std::string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    BOOST_ASSERT(!!viewer);

    // attach it to the environment:
    penv->AttachViewer(viewer);

    // finally you call the viewer's infinite loop (this is why you need a separate thread):
    bool showgui = true;
    viewer->main(showgui);
}


bool Rave::Init(string envfile)
{
    // Create the main environment
    //penv = CreateEnvironment(true);  // EnvironmentBasePtr

    // Initialize OpenRAVE-core
    RaveInitialize(true);  // Start openrave core
    penv = RaveCreateEnvironment();  // Create the main OpenRAVE environment, set the EnvironmentBasePtr
    penv->SetDebugLevel(Level_Debug);  // Relatively new function
    penv->StopSimulation();  // NEEDED??
    boost::thread thviewer(boost::bind(SetViewer,penv,"qtcoin"));
    usleep(0.4*1000*1000); // wait for the viewer to init, in [s]


    // load the scene
    if( !penv->Load(envfile))
    {
        cerr << "Environment can not be loaded! Leaving..." << endl;
        exit(-1);
    }

    //-- Get the robot
    std::vector<RobotBasePtr> robots;
    penv->GetRobots(robots);
    probot = robots[0];
    cout << "\nRobot: " << probot->GetName() << endl;

    //-- Load the controller
    pcontroller = RaveCreateController(penv,"IdealController");

    // lock the environment to prevent changes
    {
      EnvironmentMutex::scoped_lock lock(penv->GetMutex());
      vector<int> dofindices(probot->GetDOF());
      for(int i = 0; i < probot->GetDOF(); ++i) {
        dofindices[i] = i;
      }
      probot->SetController(pcontroller,dofindices,1);
      cout << "Controller created for " << probot->GetDOF() << " DOF" << endl;
    }
    //-- KinBody and Joints
    penv->GetBodies(bodies);
    joints = bodies[0]->GetJoints();
    cout << "Number of joints of " << probot->GetName() << ": " << joints.size() << endl;

    return true;
}

bool Rave::Stop(){
  penv->Destroy();
  return true;
}

bool Rave::InitialPose()
{
    printf("Initial Pose\n");
    float tmp[]={0,40,3695,9537,-5807,-334,18810,-2000,1,8801,0,40,-3730,-9537,5807,425,-18800,2000,0,-8800,0,0,0,0,0,0,0,0};
    float conversion=PI/180/209;
    const size_t size_container=sizeof tmp/sizeof tmp[0];
    Vector com;

    for (size_t i=0;i<size_container;i++) tmp[i]=conversion*tmp[i];
    vector<dReal> initial_pos(tmp,tmp+size_container);
    if(initial_pos.size() != probot->GetDOF())
    {
        cerr << "Mismatch in joint assingment" << endl;
        exit(-1);
    }

    probot->SetJointValues(initial_pos,false);
    com=probot->GetCenterOfMass();
    cout << "Center of mass of the robot: " << com << endl;

    return true;
}

bool Rave::MoveJoint (const int joint_number, const dReal desired_value, const float T){
  vector<dReal>  joint_position;
  dReal value,pendiente;
  float step;
  vector<dReal>::iterator it;
  probot->GetDOFValues(joint_position);
  //probot->GetJointValues(joint_position);

  //Show joint positions
  printf("Joint positions:\n");
  for (size_t i=0;i<joint_position.size();i++){
  cout << "Joint " << i << ": " << joint_position.at(i) << endl;
  }

  //Initial value
  value=joint_position.at(joint_number);
  pendiente=(desired_value-value)/T;

  step=T/TIME_STEP;
  cout << "Number of steps: " << step << endl;

  for (int i=0;i<step;i++){
    value=value+pendiente*TIME_STEP;
    it=joint_position.begin();
    joint_position.erase(it+joint_number);
    joint_position.insert(it+joint_number,value);
    //Show joint positions(another way)
    printf("Joint positions: ");
    for (it=joint_position.begin();it<joint_position.end();it++)
    {
        cout << *it << " ";
    }
    cout << endl << endl;

    //pcontroller->SetDesired(joint_position);
    probot->SetJointValues(joint_position,true);
    usleep(TIME_STEP*1000*1000);
    probot->SimulationStep(TIME_STEP);
    //penv->StepSimulation(TIME_STEP);

  }
  return true;
}



bool Rave::ReadData(string filename){

  ifstream file;
  string line;
  int npos,lpos,i,step=0;
  vector<dReal>  joint_position;
  dReal angle;
  float conversion=PI/180/209;

  file.open(filename.c_str(),ios::in);
  if(!file.is_open()) return false;

  penv->StartSimulation(TIME_STEP*10);
  while(getline(file,line))
  { //open comma separated file
    line += ';';
    npos=0;
    lpos=0;
    while ((npos = (int)line.find(';',lpos)) != string::npos)
    {
      if (npos > lpos)
      {
        istringstream iss(line.substr(lpos, npos-lpos));
        if (iss >> i)
        {
          angle=conversion*i;
          joint_position.push_back(angle);          
        }
      }
    lpos = npos + 1;
    }
    // Add extra joints
    for (int i = 1; i < (probot->GetDOF() - N_MOTORS); i++)
    {
        joint_position.push_back(0);
    }
    if(joint_position.size() != probot->GetDOF())
    {
        cerr << "Mismatch in joint assingment: Joints=" << joint_position.size()
             << " different from DOF=" << probot->GetDOF() << endl;
        exit(-1);
    }
    probot->SetJointValues(joint_position,true);
    usleep(TIME_STEP*1000*1000);
    probot->SimulationStep(TIME_STEP);
    //penv->StepSimulation(TIME_STEP);
    step++;
    //Show joint positions
    cout << "JOINT POSITIONS IN STEP " << step << endl;
    for (size_t i=0;i<joint_position.size();i++){
    cout << "  Joint " << i << ": " << joint_position.at(i);
    }
    cout << endl;
    joint_position.clear();
  }

  file.close();


  return true;
}


