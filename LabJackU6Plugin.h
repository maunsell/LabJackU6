/*
 *  LabJack U6 Plugin for MWorks
 *
 *  Created by Mark Histed on 4/21/2010
 *    (based on Nidaq plugin code by Jon Hendry and John Maunsell)
 *
 */


#include <MWorksCore/Plugin.h>
#include <boost/shared_ptr.hpp>
    

extern "C"{
    mw::Plugin *getPlugin();
}



class LabJackU6Plugin : public mw::Plugin {
    
	virtual void registerComponents(boost::shared_ptr<mw::ComponentRegistry> registry);	
};

