#include "osgviewer.hpp"
#include <openrave/plugin.h>

using namespace OpenRAVE;

#if defined(HAVE_X11_XLIB_H) && defined(Q_WS_X11)
#include "X11/Xlib.h"
#endif

// for some reason windows complains when the prototypes are different
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    //  Debug.
    RAVELOG_INFO("Initiating OSGRave plugin...!!!!.\n");

    switch(type) {
    case PT_Viewer:
#if defined(HAVE_X11_XLIB_H) && defined(Q_WS_X11)
        // always check viewers since DISPLAY could change
        if ( XOpenDisplay( NULL ) == NULL ) {
            RAVELOG_WARN("no display detected, so cannot load viewer");
            return InterfaceBasePtr();
        }
#endif
        if( interfacename == "osg" ) {
          return InterfaceBasePtr(new OSGViewer(penv));
        }
        break;
    default:
        break;
    }

    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Viewer].push_back("osg");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
