//
// robot_bola.cpp
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

#include "robot_bola.h"
namespace path_planning {

void PlanningModuleExample::demothread(int argc, char ** argv)
{
      //string scenefilename = "../share/lab.env.xml";
      string scenefilename = "../share/bolakinect_complete.env.xml";

      Timer t;
      t.startTimer();
      if(!penv->Load(scenefilename))
      {
          cerr << "Environment can not be loaded! Leaving..." << endl;
          exit(-1);
      }
      t.stopTimer();
      cout << "\nLoading the environment..." << endl;
      t.getTimeInSec();

      int nMaxTries = 3; // number of tries to perform the path planning

      vector<RobotBasePtr> vrobots;
      penv->GetRobots(vrobots);
      RobotBasePtr probot = vrobots.at(0); // get the ONLY robot of the env

      vector<int> vindices;
      vector<dReal> v; //array with the goal for each DOF
      probot->SetActiveDOFs(vindices,11); //DOFAffine.X + DOFAffine.Y + DOFAffine.RotationAxis[0,0,1]

      // create the planner parameters
      PlannerBasePtr rrtplanner = RaveCreatePlanner(penv,"birrt");
      if( !rrtplanner )
      {
          cerr << "failed to create BiRRTs" << endl;
          exit(-1);
      }

      PlannerBase::PlannerParametersPtr params(new PlannerBase::PlannerParameters());
      params->_nMaxIterations = 4000; // max iterations before failure
      params->vgoalconfig.resize(probot->GetActiveDOF()); // 3 elements resize
      cout << "GetActiveDOF = " << probot->GetActiveDOF() << endl;
      cout <<"GetCenterOfMass() " << probot->GetCenterOfMass().x <<" " << probot->GetCenterOfMass().y <<" "<< probot->GetCenterOfMass().z << endl;

      vector<dReal> vlower,vupper;
      probot->GetActiveDOFLimits(vlower,vupper);
      cout << "lower limits x y theta = " << vlower[0] << " " << vlower[1] << " " << vlower[2] << endl;
      cout << "upper limits x y theta = " << vupper[0] << " " << vupper[1] << " " << vupper[2] << endl;

      ModuleBasePtr pbasemanip = RaveCreateModule(penv,"basemanipulation"); // create the module
      penv->Add(pbasemanip,true,probot->GetName()); // load the module

      string strtrajfilename = "traj_robot_bola.txt";

      while(IsOk())
      {
        GraphHandlePtr pgraph;
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(penv, probot->GetActiveDOF());

        {
          EnvironmentMutex::scoped_lock lock(penv->GetMutex()); // lock environment

          // find a set of free joint values for the robot
          RobotBase::RobotStateSaver saver(probot); // save the state

          v.push_back(probot->GetCenterOfMass().x);
          v.push_back(probot->GetCenterOfMass().y);
          v.push_back(probot->GetCenterOfMass().z);
          probot->SetActiveDOFValues(v);
          if(probot->CheckSelfCollision() ) {
            cerr << "\nFailed to initialize the robot position: self collision!" << endl;
            exit(-1);
          }
          if(penv->CheckCollision(probot))
          {
            cerr << "\nFailed to initialize the robot position: collision with the enviroment!" << endl;
            exit(-1);
          }

          vector<dReal> goal;  // Establish the robot goal values for each DOF
          goal.push_back(-0.4/*1.5*/);
          goal.push_back(-2.1/*1.0*/);
          goal.push_back(0/*0.5*/);
          cout << "Defining goal configuration: " << goal[0] << " " << goal[1] << " " << goal[2]  << endl;
          params->vgoalconfig.push_back(goal[0]);
          params->vgoalconfig.push_back(goal[1]);
          params->vgoalconfig.push_back(goal[2]);
          if( params->vgoalconfig.size() == 0 )
          {
            cout << "params->vgoalconfig.size() == 0" << endl;
            exit(-3);
          }

          params->SetRobotActiveJoints(probot);

          cout << "\nStarting planning" << endl;
          Timer t1;
          t1.startTimer();
          for(int itry = 0; itry < nMaxTries; ++itry) {

             if( !rrtplanner->InitPlan(/*probot*/RobotBasePtr(), params) ) {
                 cout << "InitPlan failed\n" << endl;
                 exit(-1);
             }
             cout << "\nPlaner initialized " << endl;
             if( !rrtplanner->PlanPath(ptraj) ) {
                 cerr << "PlanPath failed" << endl;
                 exit(-1);
             }
             else {
                 cout << "\nFinished planning" << endl;
                 break;
             }
          }
          t1.stopTimer();
          t1.getTimeInSec();

          cout << "Verifying trajectory..." << endl;
          if( RaveGetDebugLevel() & Level_VerifyPlans ) {
             planningutils::VerifyTrajectory(params, ptraj);
          }

          cout << "Drawing trajectory" << endl;
          vector<RaveVector<float> > vpoints;
          vector<dReal> vtrajdata;
          // Loop to determine the path values each 0.01 seconds
          for(dReal ftime = 0; ftime <= ptraj->GetDuration(); ftime += 0.01)
          {
            ptraj->Sample(vtrajdata,ftime,probot->GetActiveConfigurationSpecification());
            probot->SetActiveDOFValues(vtrajdata);
            //cout << vtrajdata.at(0) << " " << vtrajdata.at(1) << " " << vtrajdata.at(2) << endl;
            RaveVector<float> t;
            t.x = vtrajdata.at(0);
            t.y = vtrajdata.at(1);
            t.z = probot->GetCenterOfMass().z;
            vpoints.push_back(t);
          }
          pgraph = penv->drawlinestrip(&vpoints[0].x,vpoints.size(),sizeof(vpoints[0]),3.0f);

        } //end scope

        cout << "Control action: SetPath" << endl;
        probot->GetController()->SetPath(ptraj);

        // unlock the environment and wait for the robot to finish
        while(!probot->GetController()->IsDone() && IsOk())
          boost::this_thread::sleep(boost::posix_time::milliseconds(1));

      } //end While(is_ok)

      cout << "\nBye bye..." << endl;

  } //end demothread

}//end namespace
