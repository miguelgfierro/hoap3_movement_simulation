//
// path_planning.h
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
#include <string>

#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include <signal.h>

namespace path_planning {

class PathPlanning
{
public:
    PathPlanning(const std::string& viewername="qtcoin") : _viewername(viewername), _bDestroyThread(false) {
        OPENRAVE_ASSERT_FORMAT0(GetSingleton()==NULL,"expecting only once instance of PathPlanning",OpenRAVE::ORE_Assert);
        GetSingleton() = this;
        signal(SIGINT,sigint_handler); // for control C
    }
    virtual ~PathPlanning()
    {
        GetSingleton() = NULL;
        this->Exit();
        if( _thopenrave.joinable() ) { //!!_thopenrave ) {
            _thopenrave.join(); // wait for the thread to exit
        }
    }

    virtual int main(int argc, char ** argv)
    {
        OpenRAVE::RaveInitialize(true);
        penv=OpenRAVE::RaveCreateEnvironment();
        penv->SetDebugLevel(OpenRAVE::Level_Debug);

        // create a viewer
        _viewer.reset();
        if( _viewername.size() > 0 ) {
            _viewer = OpenRAVE::RaveCreateViewer(penv,_viewername);
        }
        if( !!_viewer ) {
            penv->Add(_viewer);

            // create the main openrave thread
            _bDestroyThread = false;
            _thopenrave = boost::thread(boost::bind(&PathPlanning::_demothreadwrapper,this,argc,argv));

            // start the viewer main loop
            _viewer->main(true);
        }
        else {
            // just execute
            demothread(argc,argv);
        }
        return 0;
    }

    virtual bool IsOk() {
        return !_bDestroyThread;
    }

    virtual void Exit() {
        _bDestroyThread = true;
        OpenRAVE::RaveDestroy();
    }

    virtual void demothread(int argc, char ** argv) = 0;

protected:
    OpenRAVE::EnvironmentBasePtr penv;

private:
    OpenRAVE::ViewerBasePtr _viewer;
    std::string _viewername;
    bool _bDestroyThread;
    boost::thread _thopenrave;

    void quitviewer(void *) {
        if( !!_viewer ) {
            _viewer->quitmainloop();
        }
    }

    void _demothreadwrapper(int argc, char ** argv) {
        boost::shared_ptr<void> quitviewer((void*)NULL, boost::bind(&PathPlanning::quitviewer, this,_1));
        demothread(argc,argv);
    }

    static void sigint_handler(int sig)
    {
        if( !!PathPlanning::GetSingleton() ) {
            PathPlanning::GetSingleton()->Exit();
        }
    }

    static PathPlanning*& GetSingleton() {
        static PathPlanning* psingleton = NULL;
        return psingleton;
    }
};

} // end namespace 
