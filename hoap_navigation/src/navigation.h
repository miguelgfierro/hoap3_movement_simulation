//
// NAVIGATION_H
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
#include <sstream>
#include <iostream>
#include <openrave/planningutils.h>
#include "orexample.h"
#include "timer.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

using namespace OpenRAVE;
using namespace std;

namespace cppexamples {

class PlanningModuleExample : public OpenRAVEExample
{
public:
  virtual void demothread(int argc, char ** argv);

};

}//end namespace


#endif //NAVIGATION_H
