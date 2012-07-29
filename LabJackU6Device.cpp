/*  -*- mode: c++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-

 *  LabJack U6 Plugin for MWorks
 *
 *  100421: Mark Histed created
 *    (based on Nidaq plugin code; Hendry, Maunsell)
 *  120708 histed - revised for two levers
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



#define LJU6_EMPIRICAL_DO_LATENCY_MS 1   // average when plugged into a highspeed hub.  About 8ms otherwise

static const char ljPortDir[3] = {  // 0 input, 1 output
    ( (0x01 << LJU6_REWARD_FIO)   
      | (0x01 << LJU6_LEVER1SOLENOID_FIO)
      | (0x01 << LJU6_LEVER2SOLENOID_FIO)
      | (0x01 << LJU6_LASERTRIGGER_FIO) 
      | (0x00 << LJU6_LEVER1_FIO)   
      | (0x00 << LJU6_LEVER2_FIO) 
      | (0x01 << LJU6_STROBE_FIO) ), 
    0xff,     // EIO
    0xf0 };   // CIO


using namespace mw;

/* helper function declarations */
void debounce_bit(unsigned int *thisState, unsigned int *lastState, MWTime *lastTransitionTimeUS, shared_ptr <Clock> clock);



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
                                 const boost::shared_ptr <Variable> _lever1Solenoid,
                                 const boost::shared_ptr <Variable> _lever2Solenoid,                                  
								 const boost::shared_ptr <Variable> _lever1, 									
								 const boost::shared_ptr <Variable> _lever2, 									
								 const boost::shared_ptr <Variable> _laserTrigger, 
								 const boost::shared_ptr <Variable> _strobedDigitalWord)
{
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf(M_IODEVICE_MESSAGE_DOMAIN, "LabJackU6Device: constructor");
	}
	scheduler = a_scheduler;
	pulseDurationMS = _pulseDurationMS;
	pulseOn = _pulseOn;
	lever1 = _lever1;
	lever2 = _lever2;
    lever1Solenoid = _lever1Solenoid;
    lever2Solenoid = _lever2Solenoid;
	laserTrigger = _laserTrigger;
	strobedDigitalWord = _strobedDigitalWord;
	deviceIOrunning = false;
    ljHandle = NULL;
	lastLever1Value = -1;  // -1 means always report first value
	lastLever2Value = -1;  // -1 means always report first value
	lastLever1TransitionTimeUS = 0; 
	lastLever2TransitionTimeUS = 0;
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
    if (ljU6WriteDO(ljHandle, LJU6_REWARD_FIO, 1) == false) {
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
    if (ljU6WriteDO(ljHandle, LJU6_REWARD_FIO, 0) == false) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing digital output low; device likely to not work from here on");
    }
	// set juice variable low
	pulseDurationMS->setValue(Datum((long)0));
	
}
    
void LabJackU6Device::leverSolenoidDO(bool state, long channel) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);

    if (ljU6WriteDO(ljHandle, channel, state) != true) {  
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing lever 1 solenoid state; device likely to be broken (state %d)", state);
    }
}

void LabJackU6Device::laserDO(bool state) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
	
    if (ljU6WriteDO(ljHandle, LJU6_LASERTRIGGER_FIO, state) != true) {  
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: writing laser trigger state; device likely to be broken (state %d)", state);
    }
	
}

void LabJackU6Device::strobedDigitalWordDO(unsigned int digWord) {
    // Takes and releases driver lock
    
    boost::mutex::scoped_lock lock(ljU6DriverLock);
	
    LabJackU6Device::ljU6WriteStrobedWord(ljHandle, digWord); // error checking done inside here; will call merror
	
}


bool LabJackU6Device::readLeverDI(bool *outLever1, bool *outLever2)
// Takes the driver lock and releases it
{
	shared_ptr <Clock> clock = Clock::instance();
    
	unsigned int lever1State = 0L;
	unsigned int lever2State = 0L;
	
	unsigned int fioState = 0L;
	unsigned int eioState = 0L;
	unsigned int cioState = 0L;
	
	static unsigned int lastLever1State = 0xff;
	static unsigned int lastLever2State = 0xff;
	static long unsigned slowCount = 0;
	
	boost::mutex::scoped_lock lock(ljU6DriverLock);
	
	if (ljHandle == NULL || !this->getActive()) {
		return false;
	}
    
	MWTime st = clock->getCurrentTimeUS();
	if (ljU6ReadPorts(ljHandle, &fioState, &eioState, &cioState) < 0 ) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "Error reading DI, stopping IO and returning FALSE");
        stopDeviceIO();  // We are seeing USB errors causing this, and the U6 doesn't work anyway, so might as well stop the threads
        return false;
    }
    MWTime elT = clock->getCurrentTimeUS()-st;
    
	if (elT > kDIReportTimeUS) {
		if (++slowCount < 10) {
			merror(M_IODEVICE_MESSAGE_DOMAIN, "ljU6ReadPorts time elapsed is %.3f ms", elT / 1000.0);
		}
		else if ((slowCount < 100 && !(slowCount % 10)) || (!(slowCount % 1000))) {
			merror(M_IODEVICE_MESSAGE_DOMAIN, "!! read port time elapsed: this time %.3f, >%.0f ms %d times", 
					elT / 1000.0, 
					kDIReportTimeUS / 1000.0,
					slowCount);
		}
    }
    lever1State = (fioState >> LJU6_LEVER1_FIO) & 0x01;
    lever2State = (fioState >> LJU6_LEVER2_FIO) & 0x01;
    
    // software debouncing
	debounce_bit(&lever1State, &lastLever1State, &lastLever1TransitionTimeUS, clock);
	debounce_bit(&lever2State, &lastLever2State, &lastLever2TransitionTimeUS, clock);
	
    *outLever1 = lever1State;
    *outLever2 = lever2State;
	
	return(1);
}

/*******************************************************************/

void debounce_bit(unsigned int *thisState, unsigned int *lastState, MWTime *lastTransitionTimeUS, shared_ptr <Clock> clock) {
	// software debouncing
	if (*thisState != *lastState) {
		if (clock->getCurrentTimeUS() - *lastTransitionTimeUS < kDIDeadtimeUS) {
			*thisState = *lastState;				// discard changes during deadtime
			mwarning(M_IODEVICE_MESSAGE_DOMAIN, 
                     "LabJackU6Device: readLeverDI, debounce rejecting new read (last %lld now %lld, diff %lld)", 
                     *lastTransitionTimeUS, 
                     clock->getCurrentTimeUS(),
                     clock->getCurrentTimeUS() - *lastTransitionTimeUS);
		}
		*lastState = *thisState;					// record and report the transition
		*lastTransitionTimeUS = clock->getCurrentTimeUS();
	}
}	
	

// External function for scheduling

void *update_lever(const weak_ptr<LabJackU6Device> &gp){
	shared_ptr <Clock> clock = Clock::instance();
	shared_ptr <LabJackU6Device> sp = gp.lock();
	sp->pollAllDI();
	sp.reset();
    return NULL;
}

bool LabJackU6Device::pollAllDI() {	
	
    bool lever1Value;
    bool lever2Value;
    bool res;
    
	res = readLeverDI(&lever1Value, &lever2Value);
    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "levers: %d %d", lever1Value, lever2Value);
    
    if (!res) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "LJU6: error in readLeverDI()");
    }

    // Change MW variable value only if switch state is unchanged, or this is the first time through
    if ( (lastLever1Value == -1) // -1 means first time through
        || (lever1Value != lastLever1Value) ) {
        
        lever1->setValue(Datum(lever1Value));
        lastLever1Value = lever1Value;
    }    
    if ( (lastLever2Value == -1) // -1 means first time through
        || (lever2Value != lastLever2Value) ) {
        
        lever2->setValue(Datum(lever2Value));
        lastLever2Value = lever2Value;
    }
    
	return true;
}



/* IODevice virtual calls (made by MWorks) ***********************/


// Attempt to find the LabJack hardware and initialize it.

bool LabJackU6Device::initialize() {
	
	if (VERBOSE_IO_DEVICE >= 2) {
		mprintf("LabJackU6Device: initialize");
	}

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

	// This configures digital output on the ports so do it after we setup the hardware lines.
	this->variableSetup();

	
    // set output ports to desired state here
    if (!ljU6WriteDO(ljHandle, LJU6_LEVER1SOLENOID_FIO, 0) == 1)
        return false; // merror is done in ljU6WriteDO
    this->lever1Solenoid->setValue(Datum(M_BOOLEAN, 0));

    if (!ljU6WriteDO(ljHandle, LJU6_LEVER2SOLENOID_FIO, 0) == 1) 
        return false; // merror is done in ljU6WriteDO
    this->lever2Solenoid->setValue(Datum(M_BOOLEAN, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_REWARD_FIO, 0) == 1) 
        return false; // merror is done in ljU6WriteDO
    this->pulseOn->setValue(Datum(M_INTEGER, 0));
    this->pulseDurationMS->setValue(Datum(M_INTEGER, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_LASERTRIGGER_FIO, 0) == 1) 
        return false; // merror is done in ljU6WriteDO    
    this->laserTrigger->setValue(Datum(M_BOOLEAN, 0));
    
    if (!ljU6WriteDO(ljHandle, LJU6_STROBE_FIO, 0) == 1) 
        return false; // merror is done in ljU6WriteDO   
    this->strobedDigitalWord->setValue(Datum(M_INTEGER, 0));
    
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
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "LJU6 found dead, restarting.  (bug if not on MWServer restart)");

        libusb_reset_device((libusb_device_handle *)ljHandle); // patched usb library uses ReEnumerate
        closeUSBConnection(ljHandle);

        sleep(1.2); // histed: MaunsellMouse1 - 0.1s not enough, 0.2 works, add padding
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "Sleeping for 1.2 s after restarting LJU6");
    
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
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "stopDeviceIO: already stopped on entry; using this chance to turn off lever solenoids");
        
        // force off solenoid
        this->lever1Solenoid->setValue(false);
        this->lever2Solenoid->setValue(false);
        leverSolenoidDO(false, LJU6_LEVER1SOLENOID_FIO);
        leverSolenoidDO(false, LJU6_LEVER2SOLENOID_FIO);
		
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
	const char *LEVER1 = "lever1";
	const char *LEVER2 = "lever2";
	const char *LEVER1_SOLENOID = "lever1_solenoid";
	const char *LEVER2_SOLENOID = "lever2_solenoid";
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
	
	boost::shared_ptr<mw::Variable> lever1 = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_BOOLEAN, 0)));	
	if(parameters.find(LEVER1) != parameters.end()) {
		lever1 = reg->getVariable(parameters.find(LEVER1)->second);	
		checkAttribute(lever1, 
					   parameters.find("reference_id")->second, 
					   LEVER1, 
					   parameters.find(LEVER1)->second);
	}

	boost::shared_ptr<mw::Variable> lever2 = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_BOOLEAN, 0)));	
	if(parameters.find(LEVER2) != parameters.end()) {
		lever2 = reg->getVariable(parameters.find(LEVER2)->second);	
		checkAttribute(lever2, 
					   parameters.find("reference_id")->second, 
					   LEVER2, 
					   parameters.find(LEVER2)->second);
	}	
	
    boost::shared_ptr<mw::Variable> lever1_solenoid = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_BOOLEAN, 0)));	
	if(parameters.find(LEVER1_SOLENOID) != parameters.end()) {
		lever1_solenoid = reg->getVariable(parameters.find(LEVER1_SOLENOID)->second);	
		checkAttribute(lever1_solenoid,
					   parameters.find("reference_id")->second, 
					   LEVER1_SOLENOID, 
					   parameters.find(LEVER1_SOLENOID)->second);
	}
    
    boost::shared_ptr<mw::Variable> lever2_solenoid = boost::shared_ptr<mw::Variable>(new mw::ConstantVariable(Datum(M_BOOLEAN, 0)));	
	if(parameters.find(LEVER2_SOLENOID) != parameters.end()) {
		lever2_solenoid = reg->getVariable(parameters.find(LEVER2_SOLENOID)->second);	
		checkAttribute(lever2_solenoid,
					   parameters.find("reference_id")->second, 
					   LEVER2_SOLENOID, 
					   parameters.find(LEVER2_SOLENOID)->second);
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
                                                                                                     lever1_solenoid,
																									 lever2_solenoid,
																									 lever1, 
																									 lever2,
																									 laser_trigger,
																									 strobed_digital_word));
	return new_daq;
}	

void LabJackU6Device::variableSetup() {

	weak_ptr<LabJackU6Device> weak_self_ref(getSelfPtr<LabJackU6Device>());

	shared_ptr<Variable> doReward = this->pulseOn;
	shared_ptr<VariableNotification> notif(new LabJackU6DeviceOutputNotification(weak_self_ref));
	doReward->addNotification(notif);
    
	// lever1Solenoid
	shared_ptr<Variable> doL1S = this->lever1Solenoid;
	shared_ptr<VariableNotification> notif2(new LabJackU6DeviceL1SNotification(weak_self_ref));
	doL1S->addNotification(notif2);	
	
	// lever2Solenoid
	shared_ptr<Variable> doL2S = this->lever2Solenoid;
	shared_ptr<VariableNotification> notif2a(new LabJackU6DeviceL2SNotification(weak_self_ref));
	doL2S->addNotification(notif2a);

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
    

    // Setup FIO as constants specify.  
    //       EIO always output
    //       CIO mask is hardcoded
    
    sendDataBuff[0] = 29;       // PortDirWrite
    sendDataBuff[1] = 0xff;     // update mask for FIO: update all
    sendDataBuff[2] = 0xff;     // update mask for EIO
    sendDataBuff[3] = 0x0f;     // update mask for CIO (only 4 bits)
    
    sendDataBuff[4] = ljPortDir[0];
    sendDataBuff[5] = ljPortDir[1];
    sendDataBuff[6] = ljPortDir[2];         
        

    //mprintf(M_IODEVICE_MESSAGE_DOMAIN, "*****************Output %02x %02x %02x %02x %02x %02x %02x\n", 
    //        sendDataBuff[0], sendDataBuff[1], sendDataBuff[2], sendDataBuff[3], 
    //        sendDataBuff[4], sendDataBuff[5], sendDataBuff[6]);
    
    if(ehFeedback(Handle, sendDataBuff, 7, &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;  
    }
    if(Errorcode) {
        mwarning(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d");
        return false;
    } 

    return true;

    // cleanup now done externally to this function
}



bool LabJackU6Device::ljU6ReadPorts(HANDLE Handle, 
									unsigned int *fioState, unsigned int *eioState, unsigned int *cioState)
{
    uint8 sendDataBuff[1], recDataBuff[3];
    uint8 Errorcode, ErrorFrame;
	
    sendDataBuff[0] = 26;       //IOType is PortStateRead
	
    if(ehFeedback(Handle, sendDataBuff, 1, &Errorcode, &ErrorFrame, recDataBuff, 3) < 0)
        return -1;
    if(Errorcode)
        return (long)Errorcode;
	
	*fioState = recDataBuff[0];
	*eioState = recDataBuff[1];
	*cioState = recDataBuff[2];

    // debug
    //mprintf("FIO 0x%x EIO 0x%x CIO 0x%x", *fioState, *eioState, *cioState);
    return 0;
    
}

bool LabJackU6Device::ljU6WriteDO(HANDLE Handle, long Channel, long State) {

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
	
    uint8 sendDataBuff[27]; 
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
	sendDataBuff[14] = 0x80;	//0x80	// FIO: update pin 7
	sendDataBuff[15] = 0xff;		// EIO: update
	sendDataBuff[16] = 0x0f;		// CIO: update
	sendDataBuff[17] = 0x00;		// FIO: data
	sendDataBuff[18] = 0x00;		// EIO: data
	sendDataBuff[19] = 0x00;		// CIO: data
	
	sendDataBuff[20] = 29;			// PortDirWrite - for some reason the above seems to reset the FIO input/output state
	sendDataBuff[21] = 0xff;		//  FIO: update
	sendDataBuff[22] = 0xff;		//  EIO: update
	sendDataBuff[23] = 0xff;		//  CIO: update
	sendDataBuff[24] = ljPortDir[0];//  FIO hardcoded above
	sendDataBuff[25] = ljPortDir[1];//  EIO hardcoded above
	sendDataBuff[26] = ljPortDir[2];//  CIO hardcoded above


    if(ehFeedback(Handle, sendDataBuff, 27, &Errorcode, &ErrorFrame, NULL, 0) < 0) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "bug: ehFeedback error, see stdout");  // note we will get a more informative error on stdout
        return false;
    }
    if(Errorcode) {
        merror(M_IODEVICE_MESSAGE_DOMAIN, "ehFeedback: error with command, errorcode was %d");
        return false;
    }
	
    return true;
}


