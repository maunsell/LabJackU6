/*
 *  LabJack U6 Plugin for MWorks
 *
 *  Created by Mark Histed on 4/21/2010
 *    (based on Nidaq plugin code by Jon Hendry and John Maunsell)
 *
 */

#ifndef	_LJU6_DEVICE_H_
#define _LJU6_DEVICE_H_

#include "MWorksCore/GenericData.h"
#include "MWorksCore/Utilities.h"
#include "MWorksCore/Plugin.h"
#include "MWorksCore/IODevice.h"
#include "MWorksCore/ComponentFactory.h"
#include "MWorksCore/ComponentRegistry.h"
#include "labjackusb.h"

#undef VERBOSE_IO_DEVICE
#define VERBOSE_IO_DEVICE 0  // verbosity level is 0-2, 2 is maximum

#define LJU6_DITASK_UPDATE_PERIOD_US 15000    
#define LJU6_DITASK_WARN_SLOP_US     50000
#define LJU6_DITASK_FAIL_SLOP_US     50000


using namespace std;

namespace mw {

class LabJackU6DeviceOutputNotification;

class LabJackU6Device : public IODevice {

protected:  
	
	bool						connected;
	MWTime						lastDITransitionTimeUS;
	boost::shared_ptr <Scheduler> scheduler;
	shared_ptr<ScheduleTask>	pulseScheduleNode;
	shared_ptr<ScheduleTask>	pollScheduleNode;
	boost::mutex				pulseScheduleNodeLock;				
	boost::mutex				pollScheduleNodeLock;				
	boost::mutex				ljU6DriverLock;		
	MWTime						highTimeUS;  // Used to compute length of scheduled high/low pulses
	
	HANDLE                      ljHandle;
    
	boost::shared_ptr <Variable> pulseDurationMS;
	boost::shared_ptr <Variable> pulseOn;
	boost::shared_ptr <Variable> leverPress;
	boost::shared_ptr <Variable> leverSolenoid;

	//MWTime update_period;  MH this is now hardcoded, users should not change this
	
	bool active;
	int lastLeverPressValue;	
	boost::mutex active_mutex;
	bool deviceIOrunning;
	
	// raw hardware functions
	bool ljU6ConfigPorts(HANDLE Handle);
	bool ljU6ReadDI(HANDLE Handle, long Channel, long* State);
	bool ljU6WriteDI(HANDLE Handle, long Channel, long State);
    
public:
	
	LabJackU6Device(const boost::shared_ptr <Scheduler> &a_scheduler,
					const boost::shared_ptr <Variable> _pulseDurationMS,
					const boost::shared_ptr <Variable> _pulseOn,
                    const boost::shared_ptr <Variable> _leverPress,
                    const boost::shared_ptr <Variable> _leverSolenoid);
	
	~LabJackU6Device();
	LabJackU6Device(const LabJackU6Device& copy);
	
	virtual bool startup();
	virtual bool shutdown();	
	//       virtual bool attachPhysicalDevice();		DEPRECATED IN 0.4.4
	virtual bool initialize();
	virtual bool startDeviceIO();
	virtual bool stopDeviceIO();		
	
	virtual bool updateSwitch();
	void detachPhysicalDevice();
	void variableSetup();
	bool setupU6PortsAndRestartIfDead();
	
	
	bool readDI();
	void pulseDOHigh(int pulseLengthUS);
	void pulseDOLow();
	void solenoidDO(bool state);
	
	virtual void dispense(Datum data){
		if(getActive()){
			bool doReward = (bool)data;
			
			// Bring DO high for pulseDurationMS ms.
			
			
			if (doReward) {
				//bring DO high
				//mprintf("Yum juice!");
				this->pulseDOHigh(pulseDurationMS->getValue());
			}
		}
	}
	virtual void setSolenoid(Datum data) {   
		if (getActive()) {
			bool solenoidState = (bool)data;
			this->solenoidDO(solenoidState);
		}
	}
    
	virtual void setActive(bool _active){
		boost::mutex::scoped_lock active_lock(active_mutex);
		active = _active;
	}
	
	virtual bool getActive(){
		boost::mutex::scoped_lock active_lock(active_mutex);
		bool is_active = active;
		return is_active;
	}
	
	shared_ptr<LabJackU6Device> shared_from_this() { return static_pointer_cast<LabJackU6Device>(IODevice::shared_from_this()); }
	
};


class LabJackU6DeviceFactory : public ComponentFactory {

	shared_ptr<Component> createObject(std::map<std::string, std::string> parameters,
													 mw::ComponentRegistry *reg);
};


class LabJackU6DeviceOutputNotification : public VariableNotification {

	protected:
	
		weak_ptr<LabJackU6Device> daq;
	
	
	public:
	
		LabJackU6DeviceOutputNotification(weak_ptr<LabJackU6Device> _daq){
			daq = _daq;
		}
	
		virtual void notify(const Datum& data, MWTime timeUS){

			shared_ptr<LabJackU6Device> shared_daq(daq);
			shared_daq->dispense(data);
			
		}
};

class LabJackU6DeviceLSNotification : public VariableNotification {

protected:
	weak_ptr<LabJackU6Device> daq;
public:
	LabJackU6DeviceLSNotification(weak_ptr<LabJackU6Device> _daq){
		daq = _daq;
	}
	virtual void notify(const Datum& data, MWTime timeUS){
		shared_ptr<LabJackU6Device> shared_daq(daq);
		shared_daq->setSolenoid(data);
	}
};
	
	
} // namespace mw

#endif






