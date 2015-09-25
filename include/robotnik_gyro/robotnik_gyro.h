/** \file robotnik_gyro.h
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2012
 *
 * \brief class for the Robotnik Gyro Board
 * 
 * (C) Robotnik Automation, SLL
*/

#include <string.h>
#include <vector>
#include <stdint.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>				// publish yaw as odom
#include <std_msgs/Float32.h>				// publish yaw raw value
#include <math.h>
#include "SerialDevice.h"
#include <std_srvs/Empty.h>

#define BUFFER_SIZE					 	1000

// ERROR FLAGS
#define DSPIC_ERROR_NONE				0
#define DSPIC_ERROR_OPENING				1
#define DSPIC_ERROR_SERIALCOMM		 	2
#define DSPIC_ERROR_TIMEOUT				3

//! Timeout for controlling the communication with the device (in seconds)
#define DSPIC_TIMEOUT_COMM			 	2.0

#define DSPIC_DEFAULT_PORT                    	"/dev/ttyUSB0"
#define DSPIC_DEFAULT_PARITY 		 	"none" 	//"even" "odd""none"
#define DSPIC_DEFAULT_TRANSFERRATE	 	38400  //9600
#define DSPIC_DEFAULT_DATA_SIZE			8

#define DSPIC_CMD_SET_ANGLE			1
#define DSPIC_CMD_READ_GYRO_BATT		2
#define DSPIC_CMD_CALIBRATE_OFFSET		3
#define DSPIC_CMD_RESET_CONTROLLER		4
#define DSPIC_CMD_DEFAULT    			5

// Mechanical constants of the device controlled by DSPIC

#define DSPIC_PI				3.14159265358979323846

// Commands	
#define RESET_CONTROLLER		"X,170"
#define GET_GYRO_BATT			"Y"
#define CALIBRATE_OFFSET		"O"
#define CHANGE_TERMINAL_MODE            "M"

// Responses from device
#define ERROR_CRC					1
#define ERROR_CMD					2
#define ERROR_OVFL 					3
#define ERROR_OVRUN 					4
#define ERROR_FRAME					5
#define ERROR_OUTRANGE 					6
#define ERROR_SPI	 				7
#define ERROR_CMDOV                                     8
#define ERROR_PARAMOV					9
#define OK_GET_GYRO_BATT		    		20
#define OK_CALIBRATE_OFFSET				21
#define ERROR_GET_GYRO_BATT			-20
#define ERROR_CALIBRATE_OFFSET			-21
#define BOARD_RESET					30

#define DSPIC_SERIAL_DELAY					10000		//! us between serial transmisions to the DSPIC controller
#define DSPIC_MAX_ERRORS					3		// Max number of consecutive communications errors

namespace robotnik_gyro_ns
{

//! struct which contains all the device data
typedef struct dsPic_data_{
	//! Value of the gyroscope's orientation
	double dGyro;
	//! Value of the gyroscope's offset
	double dOffsetGyro;
        //! Battery voltage
        double dBatt; 
	//! Yaw value = dGyro - dOffset
	double yaw;
	//! Yaw velocity 
	double vel_yaw;
} dsPic_data_t;

//! Defines standard states of a component
enum States{
        INIT_STATE,
        STANDBY_STATE,
        READY_STATE,
        EMERGENCY_STATE,
        FAILURE_STATE,
        SHUTDOWN_STATE,
        CALIBRATION_STATE
};

//! Defines return values for methods and functions
enum ReturnValue{
        OK = 0,
        ERROR = -1,
};

//! Class to operate with the ms20 magnets sensor
class robotnik_gyro {

public:
	//! Buffer for receiving data from the device
	char RecBuf[BUFFER_SIZE];

	//! Contains the data structure associated with the DSPIC device
	dsPic_data_t dsPic_data;

private:
	//! Internal component state
	int iState_; 
	//! Mutex for controlling the changes and access to odomety values
	pthread_mutex_t mutex_gyro;
	//! Contains the data for the next programmed command
	dsPic_data_t dsPic_data_tmp;
	//! Contains serial device structure for port
	SerialDevice *serial;
	//! Status of DSPIC
	int iStatusDSPIC;
	//! Auxiliar variable for controling the timeout in the communication
	ros::Time tNext;
	//! Auxiliar variable for controling the timeout in the communication
	ros::Time tNow;
	//! Contains the last ocurred error
	int iErrorType;
	//! Contains the next programmed command
	int iCmd;
	//! Flag active when the offset has been calibrated successfully
	bool bOffsetCalibration;
	//! Time of the last dsPic reply
	ros::Time tDsPicReply;
	//! Flag active when an offset calibration message has been sent to the dsPic
	bool bSentOffsetCalibration;
	//! Flag to control the calculation of the offset
	bool bSuccessCalibration;
	//! Desired frequency of the loop
	double desired_freq_;

public:
	//! Public constructor
	robotnik_gyro( std::string port );
	//! Public destructor
	~robotnik_gyro();
	//! Switches the state of the node into the desired state
	void SwitchToState(States new_state);
	//! Component State Machine
	void StateMachine();
	//!Open serial port
	int Open();
	//! Close serial port
	int Close();
    //! Gets the power voltage data
    float GetVoltage();
	//! Sets internal angle
	void SetAngle(double yaw);
	//! Sends Reset Controller message
	void ResetController();
	//! Calculates new values for the offset
	void CalibrateOffsetGyro();
	//! Reads the value of the gyroscope and the battery
	void ReadGyroBatt();
	//! True if the gyroscope's offset has been calibrated
	bool IsGyroCalibrated();
	//! Returns the value of the gyro and its offset
	void GetGyroValues(double *angle, double *offset);
	//! Gets the value of the gyro and the battery
	void GetYawBatt(double *angle, double *battery);
	//! Updates yaw and yaw_vel variables
	void UpdateYaw();
    //! Callback - force sensor recalibration 
    bool srvCallback_Recalibration(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    //! Return component internal state
    int GetState();

private:
	//! Sends Reset Controller message
	int SendResetController();
	//! Actions in the initial state
	void InitState();
	//! Actions in Ready state
	void ReadyState();
	//! Actions in Failure State
	void FailureState();
	//! Actions in standby state
	void StandbyState();
    //! Actions in Calibration State
    void CalibrationState();
	//! Read RS-232 messages from the controller
	int ReadControllerMsgs();
	//! Calculates the CRC for the message
	unsigned char ComputeCRC(char *string, int size);
	//! Process received messages from the device
	int ProcessMsg(char *msg);
    //! Calculates accurate sampling period
    double GetSamplingPeriod();
	//! Sends the message for recalibration
	int SendCalibrateOffsetGyro();
	//! Sends read gyroscope and battery msg
	int SendReadGyroBatt();
};

}

