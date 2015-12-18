//
// HOAP_rave.h
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

#ifndef HOAP_RAVE_H
#define HOAP_RAVE_H

#include <openrave-core.h>
#include <openrave/planningutils.h>

using namespace OpenRAVE;
using namespace std;
#define N_MOTORS 21
#define TIME_STEP 0.001f

class Rave {
  public:
    bool Init(string envfile);
    bool Stop();
    bool InitialPose();
    bool MoveJoint (const int joint_number, const dReal desired_value, const float T);
    bool ReadData(string fdata);

    EnvironmentBasePtr Getpenv() {return penv;};
    RobotBasePtr Getprobot() {return probot;};
    ControllerBasePtr Getpcontroller() {return pcontroller;};

  private:
    // Rave-specific
    EnvironmentBasePtr penv;
    PhysicsEngineBasePtr pe;
    RobotBasePtr probot;
    ControllerBasePtr pcontroller;
    vector<KinBodyPtr> bodies;
    vector<KinBody::JointPtr> joints;
};

#endif //HOAP_RAVE_H
