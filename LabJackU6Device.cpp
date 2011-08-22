/*
 *  LabJack U6 Plugin for MWorks
 *
 *  Created by Mark Histed on 4/21/2010
 *    (based on Nidaq plugin code by Jon Hendry and John Maunsell)
 *
 */


#include "libusb-1.0/libusb.h"
#include "labjackusb.h"
#include "boost/bind.hpp"
#include <MWorksCore/Component.h>
#include <unistd.h>
#include <assert.h>
#include "u6.h"
#include "LabJackU6Device.h"

#define kBufferLength	2048
#define kDIDeadtimeUS	5000	
#define kDIReportTimeUS	5000

//#define LJU6_STROBE_FIO       7   // Use a 12-bit word; EIO0-7, CIO0-2, all encoded below
#define LJU6_LASERTRIGGER_FIO 3
#define LJU6_LEVERSOLENOID_FIO 2
#define LJU6_LEVERPRESS_FIO 1
#define LJU6_REWARD_FIO     0

#define LJU6_EMPIRICAL_DO_LATENCY_MS 1   // average when plugged into a highspeed hub.  About 8ms otherwise

using namespace mw;

/* Notes to self MH 100422
 This is how we do setup and cleanup
    * Constructor [called at plugin load time]
        Sets instant variables
    * core calls attachPhysicalDevice()
        -> variableSetup()
    * startup()  [called by core; once, I think]
    * startDeviceIO()  [called by core; every trial]
    * stopDeviceIO()   [called by core; every trial]
    * shutdown() [called by core; once, I think]
    * Destructor
        -> detachPhysicalDevice
 
 What we do:
    Constructor [sets up instance variables]
    
*/

/* Object functions **********************/

// Constructor for LabJackU6Device.  MW will later call attachToDevice, which is where we finish our initialization
LabJackU6Device::LabJackU6Device(const boost::shared_ptr <Scheduler> &a_scheduler,
                                 const boost::shared_ptr <Variable> _pulseDurationMS,
                                 const boost::shared_ptr <Variable> _pulseOn,
                                 const boost::shared_ptr <Variable> _leverPress, 
                                 const boost::shared_ptr <Variable> _leverSolenoid, 
								 const boost::shared_ptr <Variable> _laserTrigger, 
								 const boost::shared_ptr <Variable> _strobedDigitalWord)
{
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: constructor");
	}
	scheduler = a_scheduler;
	pulseDurationMS = _pulseDurationMS;
	pulseOn = _pulseOn;
	leverPress = _leverPress;
    leverSolenoid = _leverSolenoid;
	laserTrigger = _laserTrigger;
	strobedDigitalWord = _strobedDigitalWord;
	deviceIOrunning = false;
    ljHandle = NULL;
    lastLeverPressValue = -1;  // -1 means always report first value
    lastDITransitionTimeUS = 0; 
}


// Copy constructor

LabJackU6Device::LabJackU6Device(const LabJackU6Device& copy) {
    assert(0); // "Copy constructor should never be called
}

// Destructor
LabJackU6Device::~LabJackU6Device(){ 
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: destructor");
	}
	if (pulseScheduleNode != NULL) {
		boost::mutex::scoped_lock locker(pulseScheduleNodeLock); 
        pulseScheduleNode->cancel();
		pulseScheduleNode->kill();
    }
    detachPhysicalDevice();
}

// Schedule function, never scheduled if LabJack is not initialized

void *endPulse(const shared_ptr<LabJackU6Device> &gp) {
	
	shared_ptr <Clock> clock = Clock::instance();			
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: endPulse callback at %lld us", clock->getCurrentTimeUS());
    }
    gp->pulseDOLow();
    return(NULL);
}


void LabJackU6Device::pulseDOHigh(int pulseLengthUS) {
	shared_ptr <Clock> clock = Clock::instance();
    // Takes and releases pulseScheduleNodeLock
    // Takes and releases driver lock

	// Set the DO high first
    boost::mutex::scoped_lock lock(ljU6DriverLock);  //printf("lock DOhigh\n"); fflush(stdout);
	if (ljHandle == NULL) {
		return;
	}
    if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: setting pulse high %d ms (%lld)", pulseLengthUS / 1000, clock->getCurrentTimeUS());
	}
	MWTime t1 = clock->getCurrentTimeUS();  // to check elapsed time below
    if (ljU6WriteDI(ljHandle, LJU6_REWARD_FIO, 1) == false) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output high; device likely to not work from here on");
        return;
    }
    lock.unlock();      //printf("unlock DOhigh\n"); fflush(stdout);

    if (clock->getCurrentTimeUS() - t1 > 4000) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, 
               "LJU6: Writing the DO took longer than 4ms.  Is the device connected to a high-speed hub?  Pulse length is wrong.");
    }
    
	// Schedule endPulse call
    if (pulseLengthUS <= LJU6_EMPIRICAL_DO_LATENCY_MS+1) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: requested pulse length %dms too short (<%dms), not doing digital IO", 
               pulseLengthUS, LJU6_EMPIRICAL_DO_LATENCY_MS+1);
    } else {
        // long enough, do it
        boost::mutex::scoped_lock pLock(pulseScheduleNodeLock);
        shared_ptr<LabJackU6Device> this_one = shared_from_this();
        pulseScheduleNode = scheduler->scheduleMS(std::string(FILELINE ": ") + tag,
											  (pulseLengthUS / 1000.0) - LJU6_EMPIRICAL_DO_LATENCY_MS, 
											  0, 
											  1, 
											  boost::bind(endPulse, this_one),
											  M_DEFAULT_IODEVICE_PRIORITY,
											  M_DEFAULT_IODEVICE_WARN_SLOP_US,
											  M_DEFAULT_IODEVICE_FAIL_SLOP_US,
											  M_MISSED_EXECUTION_DROP);
	
        MWTime current = clock->getCurrentTimeUS();
        if (VERBOSE_IO_DEVICE >= 2) {
            mprintf("LabJackU6Device:  schedule endPulse callback at %lld us (%lld)", current, clock->getCurrentTimeUS());
        }
        highTimeUS = current;
    }
    
}

// set the DO low

void LabJackU6Device::pulseDOLow() {

	shared_ptr <Clock> clock = Clock::instance();    
    MWTime current = clock->getCurrentTimeUS();

    boost::mutex::scoped_lock lock(ljU6DriverLock);
	if (ljHandle == NULL) {
		return;
	}
    if (VERBOSE_IO_DEVICE >= 2) {
        mprintf("LabJackU6Device: pulseDOLow at %lld us (pulse %lld us long)", current, current - highTimeUS);
    }
    if (ljU6WriteDI(ljHandle, LJU6_REWARD_FIO, 0) == false) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output low; device likely to not work from here on");
    }
	// set juice variable low
	pulseDurationMS->setValue(Datum((long)0));
	
}
    
void LabJackU6Device::solenoidDO(bool state) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);

    if (eDO(ljHandle, LJU6_LEVERSOLENOID_FIO, state) < 0) {  // note eDO output convention: 0==success, negative values errorcodes
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing lever solenoid state; device likely to be broken (state %d)", state);
    }
	 
}

void LabJackU6Device::laserDO(bool state) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
	
    if (eDO(ljHandle, LJU6_LASERTRIGGER_FIO, state) < 0) {  // note eDO output convention: 0==success, negative values errorcodes
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing laser trigger state; device likely to be broken (state %d)", state);
    }
	
}

void LabJackU6Device::strobedDigitalWordDO(unsigned int digWord) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
	
    LabJackU6Device::ljU6WriteStrobedWord(ljHandle, digWord); // error checking done inside here; will call merror
	
}


bool LabJackU6Device::readDI()
// Takes the driver lock and releases it
{
    
	long int state = 0L;
	
	shared_ptr <Clock> clock = Clock::instance();
	static bool lastState = 0xff;
	static long unsigned slowCount = 0;
	
	boost::mutex::scoped_lock lock(ljU6DriverLock);
	if (ljHandle == NULL || !this->getActive()) {
		return false;
	}
    MWTime st = clock->getCurrentTimeUS();
    if (!ljU6ReadDI(ljHandle, LJU6_LEVERPRESS_FIO, &state)) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Error reading DI, stopping IO and returning FALSE");
        stopDeviceIO();  // We are seeing USB errors causing this, and the U6 doesn't work anyway, so might as well stop the threads
        //Debugger();
        return false;
    }
    MWTime elT = clock->getCurrentTimeUS()-st;
    if (elT > kDIReportTimeUS) {
		if (++slowCount < 10) {
			merror(M_IODEVICE_MESSAGE_DOMAIN, "readDI time elapsed is %.3f ms", elT / 1000.0);
		}
		else if ((slowCount < 100 && !(slowCount % 10)) || (!(slowCount % 1000))) {
			merror(M_IODEVICE_MESSAGE_DOMAIN, "readDI time elapsed >%.0f ms %d times", kDIReportTimeUS / 1000.0,
				   slowCount);
		}
    }
    
    // software debouncing
	if (state != lastState) {
		if (clock->getCurrentTimeUS() - lastDITransitionTimeUS < kDIDeadtimeUS) {
			state = lastState;				// discard changes during deadtime
			mwarning(M_IODEVICE_MESSAGE_DOMAIN, 
                     "LabJackU6Device: readDI, debounce rejecting new read (last %lld now %lld, diff %lld)", 
                     lastDITransitionTimeUS, 
                     clock->getCurrentTimeUS(),
                     clock->getCurrentTimeUS() - lastDITransitionTimeUS);
		}
		lastState = state;					// record and report the transition
		lastDITransitionTimeUS = clock->getCurrentTimeUS();
	}
    //lock.unlock();  //printf("unlock readDI\n"); fflush(stdout);
	
	return(state);
}

// External function for scheduling

void *update_lever(const weak_ptr<LabJackU6Device> &gp){
	shared_ptr <Clock> clock = Clock::instance();
	shared_ptr <LabJackU6Device> sp = gp.lock();
	sp->updateSwitch();
	sp.reset();
    return NULL;
}

bool LabJackU6Device::updateSwitch() {	
	
	
	bool switchValue = readDI();

    // Change MW variable value only if switch state is unchanged, or this is the first time through
    if ( (lastLeverPressValue == -1) // -1 means first time through
        || (switchValue != lastLeverPressValue) ) {
        
        leverPress->setValue(Datum(switchValue));
        lastLeverPressValue = switchValue;
    }
	return true;
}



/* IODevice virtual calls (made by MWorks) ***********************/

/* DEPRECATED in 0.4.4
 
bool LabJackU6Device::attachPhysicalDevice(){  
    // Attach next available device to this object
    // Also, first time configuration for this device.  
    // Opens device; reset if it is dead; and configure IO ports
    // Takes and releases driver lock

//	attached_device = new IOPhysicalDeviceReference(0, "LabJackU6");	// Seem to be no longer supported in MWorks
	
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: attachPhysicalDevice");
	}
    
    this->variableSetup();
    boost::mutex::scoped_lock lock(ljU6DriverLock);  //printf("lock %s\n", "attachPhysicalDevice");
    
    assert(ljHandle == NULL);  // should not try to configure if already open.  If we relax this in the future
    // go through this code and check that we clean up properly
    
    // Opening first found U6 over USB
    
	ljHandle = openUSBConnection(-1);
    ljU6DriverLock.unlock();
	if (ljHandle == NULL) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Error opening LabJack U6.  Is it connected to USB?");
        return false;														// no cleanup needed
    }
    setupU6PortsAndRestartIfDead();
    if (VERBOSE_IO_DEVICE >= 0) {
        mprintf("LabJackU6Device: attachPhysicalDevice: found LabJackU6"); // we should print more USB device info here
    }
    return true;
}
*/

// Attempt to find the LabJack hardware and initialize it.

bool LabJackU6Device::initialize() {
	
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: initialize");
	}
    this->variableSetup();

    boost::mutex::scoped_lock lock(ljU6DriverLock);
    assert(ljHandle == NULL);  // should not try to configure if already open.  Perhaps can relax this in the future
	ljHandle = openUSBConnection(-1);						    // Open first available U6 on USB
    ljU6DriverLock.unlock();

	if (ljHandle == NULL) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Error opening LabJack U6.  Is it connected to USB?");
        return false;														// no cleanup needed
    }
    setupU6PortsAndRestartIfDead();
    if (VERBOSE_IO_DEVICE >= 0) {
        mprintf("LabJackU6Device::initialize: found LabJackU6");
    }
    return true;
}

bool LabJackU6Device::setupU6PortsAndRestartIfDead() {
    // This is not a pleasant solution, but it works for now
    // takes and releases lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);  
    assert(ljHandle != NULL);  // you must have opened before calling this

    // Do physical port setup
    if (!ljU6ConfigPorts(ljHandle)) {
        // assume dead
        
        // Force a USB re-enumerate, and reconnect
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "LJU6 found dead, restarting.  (bug if not on restart)");

        libusb_reset_device((libusb_device_handle *)ljHandle); // patched usb library uses ReEnumerate
        closeUSBConnection(ljHandle);

        sleep(2); // histed: MaunsellMouse1 - 0.1s not enough, 0.2 works, add padding
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "Sleeping for 2 s after restarting LJU6");
    
        if( (ljHandle = openUSBConnection(-1)) == NULL) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Error: could not reopen USB U6 device after reset; U6 will not work now.");
            return false;  // no cleanup needed
        }
    
        // Redo port setup
        if (!ljU6ConfigPorts(ljHandle)) {
            merror(M_IODEVICE_MESSAGE_DOMAIN, "Error: configuring U6 after restart, U6 will not work now.  Check for patched version of libusb with reenumerate call.\n");
            return false;  // no cleanup needed
        }
    }

    return true;
}

bool LabJackU6Device::startup() {
	// Do nothing right now
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: startup");
	}
	return true;
}


bool LabJackU6Device::shutdown(){
	// Do nothing right now
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: shutdown");
	}
	return true;
}


bool LabJackU6Device::startDeviceIO(){
    // Start the scheduled IO on the LabJackU6.  This starts a thread that reads the input ports
	
	if (VERBOSE_IO_DEVICE >= 1) {
		mprintf("LabJackU6Device: startDeviceIO");
	}
	if (deviceIOrunning) {
		merror(M_IODEVICE_MESSAGE_DOMAIN, 
               "LabJackU6Device startDeviceIO:  startDeviceIO request was made without first stopping IO, aborting");
        return false;
	}
    
    // check hardware and restart if necessary
    setupU6PortsAndRestartIfDead();

//	schedule_nodes_lock.lock();			// Seems to be no longer supported in MWorks
	
	setActive(true);
	deviceIOrunning = true;

	shared_ptr<LabJackU6Device> this_one = shared_from_this();
	pollScheduleNode = scheduler->scheduleUS(std::string(FILELINE ": ") + tag,
                                             (MWTime)0, 
                                             LJU6_DITASK_UPDATE_PERIOD_US, 
                                             M_REPEAT_INDEFINITELY, 
                                             boost::bind(update_lever, weak_ptr<LabJackU6Device>(this_one)),
                                             M_DEFAULT_IODEVICE_PRIORITY,
                                             LJU6_DITASK_WARN_SLOP_US,
                                             LJU6_DITASK_FAIL_SLOP_US,                                             
                                             M_MISSED_EXECUTION_DROP);
	
	//schedule_nodes.push_back(pollScheduleNode);       
//	schedule_nodes_lock.unlock();		// Seems to be no longer supported in MWorks

	return true;
}

bool LabJackU6Device::stopDeviceIO(){

    // Stop the LabJackU6 collecting data.  This is typically called at the end of each trial.
    
	if (VERBOSE_IO_DEVICE >= 1) {
		mprintf("LabJackU6Device: stopDeviceIO");
	}
	if (!deviceIOrunning) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "stopDeviceIO: already stopped on entry; using this chance to turn off lever solenoid");
        
        // force off solenoid
        this->leverSolenoid->setValue(false);
        solenoidDO(false);
		
		return false;
	}
	
	// stop all the scheduled DI checking (i.e. stop calls to "updateChannel")
	//stopAllScheduleNodes();								// IO device base class method -- this is thread safe
	if (pollScheduleNode != NULL) {
        //merror(M_IODEVICE_MESSAGE_DOMAIN, "Error: pulseDOL
		boost::mutex::scoped_lock(pollScheduleNodeLock);
        pollScheduleNode->cancel();
		//pollScheduleNode->kill();  // MH This is not allowed!  This can make both the USB bus unhappy and also leave the lock
                                     //    in a locked state.  
                                     //    If you insist on killing a thread that may be talking to the LabJack you should reset the USB bus.
    }

	//setActive(false);   // MH - by leaving active == true, we can use the Reward window to schedule pulses when trials are not running
	deviceIOrunning = false;
	return true;
}

/* Factory: create LabJackU6 object */

boost::shared_ptr<mw::Component> LabJackU6DeviceFactory::createObject(std::map<std::string, std::string> parameters,
																   mw::ComponentRegistry *reg) {
	
	const char *PULSE_DURATION = "pulse_duration";
	const char *PULSE_ON = "pulse_on";
	const char *LEVER_PRESS = "lever_press";
	const char *LEVER_SOLENOID = "lever_solenoid";
	const char *LASER_TRIGGER = "laser_trigger";
	const char *STROBED_DIGITAL_WORD = "strobed_digital_word";
	
	REQUIRE_ATTRIBUTES(parameters, PULSE_DURATION);
	
	boost::shared_ptr<mw::Variable> pulse_duration = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_INTEGER, 0)));	
	if(parameters.find(PULSE_DURATION) != parameters.end()) {
		pulse_duration = reg->getVariable(parameters.find(PULSE_DURATION)->second);	
		checkAttribute(pulse_duration, 
					   parameters.find("reference_id")->second, 
					   PULSE_DURATION, 
					   parameters.find(PULSE_DURATION)->second);
	}
	
	boost::shared_ptr<mw::Variable> pulse_on = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_BOOLEAN, 0)));	
	if(parameters.find(PULSE_ON) != parameters.end()) {
		pulse_on = reg->getVariable(parameters.find(PULSE_ON)->second);	
		checkAttribute(pulse_on, 
					   parameters.find("reference_id")->second, 
					   PULSE_ON, 
					   parameters.find(PULSE_ON)->second);
	}
	
	boost::shared_ptr<mw::Variable> lever_press = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_INTEGER, 0)));	
	if(parameters.find(LEVER_PRESS) != parameters.end()) {
		lever_press = reg->getVariable(parameters.find(LEVER_PRESS)->second);	
		checkAttribute(lever_press, 
					   parameters.find("reference_id")->second, 
					   LEVER_PRESS, 
					   parameters.find(LEVER_PRESS)->second);
	}
	
    boost::shared_ptr<mw::Variable> lever_solenoid = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_BOOLEAN, 0)));	
	if(parameters.find(LEVER_SOLENOID) != parameters.end()) {
		lever_solenoid = reg->getVariable(parameters.find(LEVER_SOLENOID)->second);	
		checkAttribute(lever_solenoid,
					   parameters.find("reference_id")->second, 
					   LEVER_SOLENOID, 
					   parameters.find(LEVER_SOLENOID)->second);
	}

	boost::shared_ptr<mw::Variable> laser_trigger = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_BOOLEAN, 0)));	
	if(parameters.find(LASER_TRIGGER) != parameters.end()) {
		laser_trigger = reg->getVariable(parameters.find(LASER_TRIGGER)->second);	
		checkAttribute(laser_trigger,
					   parameters.find("reference_id")->second, 
					   LASER_TRIGGER, 
					   parameters.find(LASER_TRIGGER)->second);
	}

	boost::shared_ptr<mw::Variable> strobed_digital_word = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_INTEGER, 0)));	
	if(parameters.find(STROBED_DIGITAL_WORD) != parameters.end()) {
		strobed_digital_word = reg->getVariable(parameters.find(STROBED_DIGITAL_WORD)->second);	
		checkAttribute(strobed_digital_word,
					   parameters.find("reference_id")->second, 
					   STROBED_DIGITAL_WORD, 
					   parameters.find(STROBED_DIGITAL_WORD)->second);
	}
	
	
	boost::shared_ptr <mw::Scheduler> scheduler = mw::Scheduler::instance(true);
	
	boost::shared_ptr <mw::Component> new_daq = boost::shared_ptr<mw::Component>(new LabJackU6Device(scheduler,
																									 pulse_duration, 
																									 pulse_on, 
                                                                                                     lever_press,	
                                                                                                     lever_solenoid,
																									 laser_trigger,
																									 strobed_digital_word));
	return new_daq;
}	

void LabJackU6Device::variableSetup() {

	weak_ptr<LabJackU6Device> weak_self_ref(getSelfPtr<LabJackU6Device>());

	shared_ptr<Variable> doReward = this->pulseOn;
	shared_ptr<VariableNotification> notif(new LabJackU6DeviceOutputNotification(weak_self_ref));
	doReward->addNotification(notif);
    
	// leverSolenoid
	shared_ptr<Variable> doLS = this->leverSolenoid;
	shared_ptr<VariableNotification> notif2(new LabJackU6DeviceLSNotification(weak_self_ref));
	doLS->addNotification(notif2);

	// laserTrigger
	shared_ptr<Variable> doLT = this->laserTrigger;
	shared_ptr<VariableNotification> notif3(new LabJackU6DeviceLTNotification(weak_self_ref));
	doLT->addNotification(notif3);
	
	// strobedDigitalWord
	shared_ptr<Variable> doSDW = this->strobedDigitalWord;
	shared_ptr<VariableNotification> notif4(new LabJackU6DeviceSDWNotification(weak_self_ref));
	doSDW->addNotification(notif4);
	
	connected = true;	
}

void LabJackU6Device::detachPhysicalDevice() {
	if (VERBOSE_IO_DEVICE >= 1) {
		mprintf("LabJackU6Device: detachPhysicalDevice");
	}
    assert(connected == true); // "Was not connected on entry to detachPhysicalDevice");
		
    boost::mutex::scoped_lock lock(ljU6DriverLock);  
	//printf("lock in detachP\n");
    assert(ljHandle != NULL); // "Device handle is NULL before attempt to disconnect");
    
    closeUSBConnection(ljHandle);
    ljHandle = NULL;

}

/* Hardware functions *********************************/
// None of these have any locking, callers must lock

bool LabJackU6Device::ljU6ConfigPorts(HANDLE Handle) {
    /// set up IO ports
    uint8 sendDataBuff[7]; // recDataBuff[1];
	uint8 Errorcode, ErrorFrame;
	

    // Don't need to configure the laser or leversolenoid port, just use eDO
    
    ////setup one to be output, one input, and set output to zero
    //sendDataBuff[0] = 13;       //IOType is BitDirWrite
    //sendDataBuff[1] = (LJU6_LEVERPRESS_FIO & 0x0f) | (0 << 7);  //IONumber(bits 0-4) + Direction (bit 7; 1 is output)
    //sendDataBuff[2] = 13;       //IOType is BitDirWrite
    //sendDataBuff[3] = LJU6_REWARD_FIO & 0x0f | (1 << 7);  //IONumber(bits 0-4) + Direction (bit 7; 1 is output)
    //sendDataBuff[4] = 11;             //IOType is BitStateWrite
    //sendDataBuff[5] = (LJU6_REWARD_FIO & 0x0f) | (0 << 7);  //IONumber(bits 0-4) + State (bit 7)
	
	sendDataBuff[0] = 29;		// PortDirWrite
	sendDataBuff[1] = ( (0x01 << LJU6_REWARD_FIO) | (0x01 << LJU6_LEVERPRESS_FIO) | (0x01 << LJU6_LEVERSOLENOID_FIO)
					   | (0x01 << LJU6_LASERTRIGGER_FIO) | (0x01 << 7) ); 
								// FIO mask.  Note pin 7 is hardcoded to be the strobe.
	sendDataBuff[2] = 0xff;		// EIO mask
	sendDataBuff[3] = 0x0f;		// CIO mask
	sendDataBuff[4] = ( (0x01 << LJU6_REWARD_FIO) | (0x00 << LJU6_LEVERPRESS_FIO) | (0x01 << LJU6_LEVERSOLENOID_FIO)
					   | (0x01 << LJU6_LASERTRIGGER_FIO) | (0x01 << 7) ); 
								// FIO dir data: 1 is output 0 is input (only input now is leverpress)
	sendDataBuff[5] = 0xff;		// EIO dir: all output
	sendDataBuff[6] = 0x0f;		// CIO dir: all output (only 4 bits)
	

    //printf("*****************Output %x %x %x %x\n", sendDataBuff[0], sendDataBuff[1], sendDataBuff[2], sendDataBuff[3]);
    
    if(ehFeedback(Handle, sendDataBuff, 7, &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;  
    }
    if(Errorcode) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d");
        return false;
    }

	// set output ports to desired state here
    if (eDO(Handle, LJU6_LEVERSOLENOID_FIO, 0) < 0) {  // set to low, automatically configures too
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehDO error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
    if (eDO(Handle, LJU6_REWARD_FIO, 0) < 0) {  // set to low, automatically configures too
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehDO error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
	
	
    return true;

    // cleanup now done externally to this function
}


bool LabJackU6Device::ljU6ReadDI(HANDLE Handle, long Channel, long* State) {
    // returns: 1 on success, 0 on error
    // output written to State
    
    uint8 sendDataBuff[4], recDataBuff[1];
    uint8 Errorcode, ErrorFrame;

    sendDataBuff[0] = 10;       //IOType is BitStateRead
    sendDataBuff[1] = Channel;  //IONumber
    
    //printf("entering ljU6ReadDI\n"); fflush(stdout);
    if(ehFeedback(Handle, sendDataBuff, 2, &Errorcode, &ErrorFrame, recDataBuff, 1) < 0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
    if(Errorcode) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d");
        return false;
    }
    
    *State = (long int)recDataBuff[0];
    return true;
}

bool LabJackU6Device::ljU6WriteDI(HANDLE Handle, long Channel, long State) {

    uint8 sendDataBuff[2]; // recDataBuff[1];
    uint8 Errorcode, ErrorFrame;

    sendDataBuff[0] = 11;             //IOType is BitStateWrite
    sendDataBuff[1] = Channel + 128*((State > 0) ? 1 : 0);  //IONumber(bits 0-4) + State (bit 7)

    if(ehFeedback(Handle, sendDataBuff, 2, &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
    if(Errorcode) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d");
        return false;
    }
    return true;
}

bool LabJackU6Device::ljU6WriteStrobedWord(HANDLE Handle, unsigned int inWord) {
	
	uint8 outEioBits = inWord & 0xff;
	uint8 outCioBits = (inWord & 0xf00) >> 8;
	
    uint8 sendDataBuff[20]; 
    uint8 Errorcode, ErrorFrame;

	if (inWord > 0xfff) {
		merror(M_IODEVICE_MESSAGE_DOMAIN, "error writing strobed word; value is larger than 12 bits (nothing written)");
		return false;
	}
		
	
    sendDataBuff[0] = 27;			// PortStateWrite, 7 bytes total
	sendDataBuff[1] = 0x00;			// FIO: don't update
	sendDataBuff[2] = 0xff;			// EIO: update
	sendDataBuff[3] = 0x0f;			// CIO: update
	sendDataBuff[4] = 0x00;			// FIO: data
	sendDataBuff[5] = outEioBits;	// EIO: data
	sendDataBuff[6] = outCioBits;	// CIO: data
	
	sendDataBuff[7] = 5;			// WaitShort
	sendDataBuff[8] = 1;			// Time(*128us)
	
	sendDataBuff[9]  = 11;			// BitStateWrite
	sendDataBuff[10] = 7 | 0x80;	// first 4 bits: port # (FIO7); last bit, state
	
	sendDataBuff[11] = 5;			// WaitShort
	sendDataBuff[12] = 1;			// Time(*128us)
	
    sendDataBuff[13] = 27;			// PortStateWrite, 7 bytes total
	sendDataBuff[14] = 0x80;		// FIO: update pin 7
	sendDataBuff[15] = 0xff;		// EIO: update
	sendDataBuff[16] = 0x0f;		// CIO: update
	sendDataBuff[17] = 0x00;		// FIO: data
	sendDataBuff[18] = 0x00;		// EIO: data
	sendDataBuff[19] = 0x00;		// CIO: data

    if(ehFeedback(Handle, sendDataBuff, 20, &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
    if(Errorcode) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d");
        return false;
    }
    return true;
}


