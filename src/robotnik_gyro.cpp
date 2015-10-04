/** \file robotnik_gyro.cc
 * \author Robotnik Automation S.L.L.
 * \version 2.0
 * \date    2012
 *
 * \brief robotnik_gyro class driver
 * Component to manage the Summit XL servo controller set
 * (C) 2012 Robotnik Automation, SLL
*/

#include "robotnik_gyro/robotnik_gyro.h"
//#include "robotnik_gyro/Battery.h"  // msg
#include <math.h>
#include <cstdlib>

using namespace robotnik_gyro_ns;

std::string port;
robotnik_gyro * rbk_gyro;

/*!	\fn robotnik_gyro::robotnik_gyro()
 * 	\brief Public constructor
*/
robotnik_gyro::robotnik_gyro( std::string port ) : desired_freq_(50) {

	// Initial values
	dsPic_data.dGyro = dsPic_data.dOffsetGyro = 0.0;
	dsPic_data.dBatt = 0.0;
	dsPic_data.yaw = 0.0;
	dsPic_data.vel_yaw = 0.0;
	pthread_mutex_init(&mutex_gyro, NULL); //Initialization for gyro's mutex
	iErrorType = DSPIC_ERROR_NONE;

    	// Create serial port
	serial= new SerialDevice(port.c_str(), DSPIC_DEFAULT_TRANSFERRATE, DSPIC_DEFAULT_PARITY, DSPIC_DEFAULT_DATA_SIZE); //Creates serial device

	// Calibration flags
	bOffsetCalibration = bSentOffsetCalibration = false;
};

/*!	\fn robotnik_gyro::~robotnik_gyro()
 * 	\brief Public destructor
*/
robotnik_gyro::~robotnik_gyro(){

	// Close serial port
	if (serial!=NULL) serial->ClosePort();

        // Delete serial port
        if (serial!=NULL) delete serial;

        // Destroy gyro mutex
        pthread_mutex_destroy(& mutex_gyro );
}


/*!	\fn void robotnik_gyro::SwitchToState(States new_state)
 * 	function that switches the state of the component into the desired state
 * 	\param new_state as an integer, the new state of the component
*/
void robotnik_gyro::SwitchToState(States new_state){

	if(new_state == iState_)
		return;

	iState_ = new_state;
	switch (new_state) {
		case INIT_STATE:
			//ROS_INFO("robotnik_gyro:: switching to INIT state");
			break;
		case STANDBY_STATE:
			//ROS_INFO("robotnik_gyro:: switching to STANDBY state");
			break;
		case READY_STATE:
			//ROS_INFO("robotnik_gyro:: switching to READY state");
			break;
		case EMERGENCY_STATE:
			//ROS_INFO("robotnik_gyro:: switching to EMERGENCY state");
			break;
		case FAILURE_STATE:
			//ROS_INFO("robotnik_gyro:: switching to FAILURE state");
			break;
        case SHUTDOWN_STATE:
			//ROS_INFO("robotnik_gyro:: switching to SHUTDOWN state");
			break;
		case CALIBRATION_STATE:
			ROS_INFO("robotnik_gyro:: switching to CALIBRATION state");
		    break;
		default:
			ROS_ERROR("robotnik_gyro:: Switching to UNKNOWN state");
			break;
	}
}

/*!	\fn void robotnik_gyro::StateMachine()
 * 	function that manages internal states of the component 
*/
void robotnik_gyro::StateMachine(){

   switch (iState_){
        case INIT_STATE:
                InitState();
        break;
        case STANDBY_STATE:
                StandbyState();
        break;
        case READY_STATE:
                ReadyState();
        break;
        case FAILURE_STATE:
                FailureState();
        break;
        case CALIBRATION_STATE:
                CalibrationState();
	    break;
	default:
	break;
	}
}

/*!	\fn int robotnik_gyro::GetState()
 * 	
*/
int robotnik_gyro::GetState()
{
	return iState_;
}


/*!	\fn int robotnik_gyro::Open(char *dev)
 * 	\brief Open serial port
 * 	\returns -1 Error
 * 	\returns 0 Ok
*/
int robotnik_gyro::Open(){

	// Setup serial device
	if (this->serial->OpenPort2() == SERIAL_ERROR) {
          ROS_ERROR("robotnik_gyro::Open: Error Opening Serial Port");
	  SwitchToState(FAILURE_STATE);
	  iErrorType = DSPIC_ERROR_OPENING;
	  return ERROR;
          }
	ROS_INFO("robotnik_gyro::Open: serial port opened at %s", serial->GetDevice());

    // Store current value to keep measurement
    double yaw = dsPic_data.yaw;

	// Send 1st command to verify device is working
    usleep(50000);
    SendCalibrateOffsetGyro();
	usleep(1200000);
    SendReadGyroBatt();
    usleep(50000);
    SendCalibrateOffsetGyro();
    usleep(1200000);
    SendReadGyroBatt();
    
    // Restore offsets
    dsPic_data.dGyro = 0.0; // gyro will start with 0 after recal 
    this->SetAngle( yaw );
    
    usleep(50000);
	SwitchToState(INIT_STATE);

	return OK;
}

/*!	\fn int robotnik_gyro::Close()
 * 	\brief Closes serial port
 * 	\returns ERROR
 * 	\returns OK
*/
int robotnik_gyro::Close(){

	if (serial!=NULL) serial->ClosePort();
	return OK;
}

/*!	\fn void robotnik_gyro::InitState()
 * 	\brief Actions in the initial state
 * 	First call-> Sends start data logging message -> IF ERROR go to FAILURE_STATE
 *	-> IF OK inits communication's timers
 * 	Next call -> Reads messages from the sensor and compares timers	-> IF no responses go to FAILURE_STATE
 * 	-> IF OK go to READY_STATE
*/
void robotnik_gyro::InitState(){
	static int count_errors_reading = 0, count_errors_sending = 0;
	int ret = 0, ret2 = 0;

	ret = ReadControllerMsgs();
	switch(ret) {
		case ERROR:	// Nothing received 
			ROS_ERROR("robotnik_gyro::InitState: received ERROR");
			count_errors_reading++;
		break;
		case ERROR_CRC:
			ROS_ERROR("robotnik_gyro::InitState: Error CRC");
			count_errors_reading++;
		break;
		case ERROR_CMD:
			ROS_ERROR("robotnik_gyro::InitState: Error CMD");
			count_errors_reading++;
		break;
		case ERROR_OVFL:
			ROS_ERROR("robotnik_gyro::InitState: Error OVFL");
			count_errors_reading++;
		break;
		case ERROR_OVRUN:
			ROS_ERROR("robotnik_gyro::InitState: Error OVRUN");
			count_errors_reading++;
		break;
		case ERROR_FRAME:
			ROS_ERROR("robotnik_gyro::InitState: Error FRAME");
			count_errors_reading++;
		break;
		case ERROR_OUTRANGE:
			ROS_ERROR("robotnik_gyro::InitState: Error OUTofRANGE");
			count_errors_reading++;
		break;
		case ERROR_SPI:
			ROS_ERROR("robotnik_gyro::InitState: Error SPI");
			//count_errors_reading++;
		break;
                case ERROR_CMDOV: 
                        ROS_ERROR("robotnik_gyro::InitState: Error CMDOV - Command Overflow");
                        count_errors_reading++;
                break;
                case ERROR_PARAMOV:
                        ROS_ERROR("robotnik_gyro::InitState: Error PARAMOV - Parameter Overflow");
                        count_errors_reading++;
                break;
		case OK:
			// Inits dsPic reply timer			
			tDsPicReply = ros::Time::now();
			count_errors_reading = 0;
			SwitchToState(READY_STATE);			
		break;
	}

	ret2 = SendReadGyroBatt();
	if(ret2 == ERROR)
		count_errors_sending++;
	else
		count_errors_sending = 0;

	if((count_errors_sending >= DSPIC_MAX_ERRORS) /*|| ((count_errors_reading >= DSPIC_MAX_ERRORS))*/){
		iErrorType = DSPIC_ERROR_SERIALCOMM;
		ROS_ERROR("robotnik_gyro::InitState: %d errors sending, %d errors reading", count_errors_sending, count_errors_reading);
		SwitchToState(FAILURE_STATE);
		count_errors_sending = 0;
        }

	if (count_errors_reading > 10) {
		ROS_ERROR("robotnik_gyro::InitState: device not responding.");
		SendResetController();
		sleep(5.0);		
		count_errors_reading = 0;
		SwitchToState(FAILURE_STATE);
		iErrorType = DSPIC_ERROR_TIMEOUT;
        }
}

/*! \fn void robotnik_gyro::ReadyState()
 *  \brief Actions in Ready state
 *    First time -> Gets current time
 *    Next times -> Read from controller and checks the timeout between responses
 *    -> IF TIMEOUT go to FAILURE_STATE
 *    -> IF OK resets communication's timer
*/
void robotnik_gyro::ReadyState(){

	static int count_errors_reading = 0, count_errors_sending = 0;
	int ret = 0, ret2 = 0;

	// Actions in ReadyState
	ret = ReadControllerMsgs();
	switch(ret) {
		case ERROR:	// Nothing received
			count_errors_reading++;
		break;
		case ERROR_CRC:
			ROS_ERROR("robotnik_gyro::ReadyState: Error CRC");
			count_errors_reading++;
		break;
		case ERROR_CMD:
			ROS_ERROR("robotnik_gyro::ReadyState: Error CMD");
			count_errors_reading++;
		break;
		case ERROR_OVFL:
			ROS_ERROR("robotnik_gyro::ReadyState: Error OVFL");
			count_errors_reading++;
		break;
		case ERROR_OVRUN:
			ROS_ERROR("robotnik_gyro::ReadyState: Error OVRUN");
			count_errors_reading++;
		break;
		case ERROR_FRAME:
			ROS_ERROR("robotnik_gyro::ReadyState: Error FRAME");
			count_errors_reading++;
		break;
		case ERROR_OUTRANGE:
			ROS_ERROR("robotnik_gyro::ReadyState: Error OUTofRANGE");
			count_errors_reading++;
		break;
		case ERROR_SPI:
			ROS_ERROR("robotnik_gyro::ReadyState: Error SPI");
			//count_errors_reading++;
                break;
                case ERROR_CMDOV:
                        ROS_ERROR("robotnik_gyro::ReadyState: Error CMDOV - Command Overflow");
                        count_errors_reading++;
                break;
                case ERROR_PARAMOV:
                        ROS_ERROR("robotnik_gyro::ReadyState: Error PARAMOV - Parameter Overflow");
                        count_errors_reading++;
                break;

		default:
			// If no errors, reset counter
			count_errors_reading = 0;
			// Saves current time
			// Inits nodeguard reply timer
			tDsPicReply = ros::Time::now(); 
			//ROS_INFO("robotnik_gyro::ReadyState: Ret = %d", ret);
		break;
	}

	switch (iCmd) {

		case DSPIC_CMD_READ_GYRO_BATT:
			ret2 = SendReadGyroBatt();
	        break;
		case DSPIC_CMD_CALIBRATE_OFFSET:
			ret2 = SendCalibrateOffsetGyro();
			SwitchToState(STANDBY_STATE);	// Stays on standby until checks the new and right offset
         	break;
		case DSPIC_CMD_RESET_CONTROLLER:
			ret2 = SendResetController();
		break;
	        default:
            // Default command 
	        ret2 = SendReadGyroBatt();
	        break;
	}

	if(ret2 == ERROR)
		count_errors_sending++;
	else
		count_errors_sending = 0;

	// Update robot angle
	UpdateYaw();

	iCmd = DSPIC_CMD_READ_GYRO_BATT; 	// Next command by default

	if(count_errors_sending >= DSPIC_MAX_ERRORS){
		iErrorType = DSPIC_ERROR_SERIALCOMM;
		ROS_ERROR("robotnik_gyro::ReadyState: %d errors sending, %d errors reading", count_errors_sending, count_errors_reading);
		SwitchToState(FAILURE_STATE);
		count_errors_sending = 0;
		count_errors_reading = 0;
        }

	// Checks the communication status
	ros::Time current_time = ros::Time::now(); 		
	// if (((current_time.toSec() - tDsPicReply.toSec()) > DSPIC_TIMEOUT_COMM) && !bSentOffsetCalibration) {
	if ((current_time.toSec() - tDsPicReply.toSec()) > DSPIC_TIMEOUT_COMM) {
		ROS_ERROR("robotnik_gyro::ReadyState: Timeout in communication with the device");
		tDsPicReply = ros::Time::now();	// Resets the timer
		SwitchToState(FAILURE_STATE);
		iErrorType = DSPIC_ERROR_TIMEOUT;
        }

}

/*!	\fn void robotnik_gyro::StandbyState()
 * 	\brief Actions in standby state
 */
void robotnik_gyro::StandbyState(){
	static int count_errors_reading = 0, count_errors_sending = 0;
	int ret = 0, ret2 = 0;

	ret = ReadControllerMsgs();

	switch(ret) {
		case ERROR:	// Nothing received
			count_errors_reading++;
		break;
		case ERROR_CRC:
			ROS_ERROR("robotnik_gyro::StandbyState: Error CRC");
			count_errors_reading++;
		break;
		case ERROR_CMD:
			ROS_ERROR("robotnik_gyro::StandbyState: Error CMD");
			count_errors_reading++;
		break;
		case ERROR_OVFL:
			ROS_ERROR("robotnik_gyro::StandbyState: Error OVFL");
			count_errors_reading++;
		break;
		case ERROR_OVRUN:
			ROS_ERROR("robotnik_gyro::StandbyState: Error OVRUN");
			count_errors_reading++;
		break;
		case ERROR_FRAME:
			ROS_ERROR("robotnik_gyro::StandbyState: Error FRAME");
			count_errors_reading++;
		break;
		case ERROR_OUTRANGE:
			ROS_ERROR("robotnik_gyro::StandbyState: Error OUTofRANGE");
			count_errors_reading++;
		break;
		case ERROR_SPI:
			ROS_ERROR("robotnik_gyro::StandbyState: Error SPI");
			//count_errors_reading++;
		break;
		default:
			// Reset if no error arrives
			count_errors_reading = 0;
			// Saves current time, inits nodeguard reply timer
			tDsPicReply = ros::Time::now();
		break;
	}

	if(!bSentOffsetCalibration && !bSuccessCalibration){
		ret2 = SendCalibrateOffsetGyro();
		ROS_INFO("robotnik_gyro::Standby: Sending calibrate offset");
	}else if(bSuccessCalibration){
		iCmd = DSPIC_CMD_DEFAULT;
		SwitchToState(READY_STATE);
		// Saves current time, inits nodeguard reply timer
		tDsPicReply = ros::Time::now();
        }

	if(ret2 == ERROR)
		count_errors_sending++;
	else
		count_errors_sending = 0;

	if((count_errors_sending >= DSPIC_MAX_ERRORS)){
		iErrorType = DSPIC_ERROR_SERIALCOMM;
		ROS_ERROR("robotnik_gyro::StandbyState: %d errors sending, %d errors reading", count_errors_sending, count_errors_reading);
		SwitchToState(FAILURE_STATE);
		count_errors_sending = 0;
		count_errors_reading = 0;
        }

	// Checks the communication status
	ros::Time current_time = ros::Time::now(); 
	if ((current_time.toSec() - tDsPicReply.toSec()) > DSPIC_TIMEOUT_COMM) {
		ROS_ERROR("robotnik_gyro::Standby: Timeout in communication with the device");
		tDsPicReply = ros::Time::now();	 // Resets the timer
        }

}

/*!	\fn void robotnik_gyro::FailureState()
 * 	\brief Actions in Failure State
*/
void robotnik_gyro::FailureState(){
	int timer = 2500000; //useconds
	static int recovery_cycles = 0;

	recovery_cycles++;
	if(recovery_cycles >= 50){ //Try to recover every 'second'
		switch(iErrorType)	{
			ROS_INFO("robotnik_gyro::FailureState: Trying to recover..");
			case DSPIC_ERROR_OPENING://Try to recover
				ROS_ERROR("robotnik_gyro::FailureState: Recovering from failure state (DSPIC_ERROR_OPENING.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case DSPIC_ERROR_SERIALCOMM:
				ROS_ERROR("robotnik_gyro::FailureState: Recovering from failure state (DSPIC_ERROR_SERIALCOMM.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case DSPIC_ERROR_TIMEOUT:
				ROS_ERROR("robotnik_gyro::FailureState: Recovering from failure state (DSPIC_ERROR_TIMEOUT.)");
				this->Close();
				usleep(timer);
				this->Open();
			break;
			case DSPIC_ERROR_SPI:
				this->Close();
				ROS_ERROR("robotnik_gyro::FailureState: Continuous SPI error in board, board hw reset necessary");				
			break;
		}
		recovery_cycles = 0;
	}
}

/*!	\fn void robotnik_gyro::CalibrationState()
 * 	\brief Actions in Calibration State
*/
void robotnik_gyro::CalibrationState(){

    // Store current value to keep measurement
    double yaw = dsPic_data.yaw;

    // Send calibration command 
    SendCalibrateOffsetGyro();
    usleep(1200000);
    SendReadGyroBatt();
    usleep(50000);

    // Restore offsets
    dsPic_data.dGyro = 0.0; // gyro will start with 0 after recal 
    this->SetAngle( yaw );

    // Go on
    SwitchToState(INIT_STATE);
}

/*!	\fn int robotnik_gyro::ReadControllerMsgs()
 * 	Read RS-232 messages from the controller
 * 	Returns OK if messages readed
 * 	Return ERROR otherwise
*/
int robotnik_gyro::ReadControllerMsgs(){
	int read_bytes=0;			//Number of received bytes
	int ret = ERROR;
	char cReadBuffer[64] = "\0";

	// Read controller messages	
	if (serial->ReadPort(cReadBuffer, &read_bytes, 64)==ERROR) {
	    ROS_ERROR("robotnik_gyro::ReadControllerMsgs: Error reading port");
	    return ERROR;
        }
	while(read_bytes > 0){
		ret = ProcessMsg(cReadBuffer); // returns ERROR or Number of bytes !
		if(ret == ERROR) return ERROR;
		else ret = OK;
                if (serial->ReadPort(cReadBuffer, &read_bytes, 64)==ERROR) {
		   //ROS_ERROR("robotnik_gyro::ReadControllerMsgs: Error after 1st port read");
		   //return ERROR;
                   ret = OK;
                }
        }

	return ret;  
}

/*!	\fn int robotnik_gyro::SendResetController()
	* Allows the controller to be reset
	* \return OK
	* \return ERROR
*/
int robotnik_gyro::SendResetController(){
	char cMsg[8] = "\0";
	int written_bytes=0;

	sprintf(cMsg,"%s\r", RESET_CONTROLLER);
	ROS_INFO("robotnik_gyro::SendResetController: Sending reset command");
	// Sends the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("robotnik_gyro::SendResetController: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn int robotnik_gyro::SendReadGyroBatt()
	* Reads gyroscope's value
 	* \return OK
	* \return ERROR
*/
int robotnik_gyro::SendReadGyroBatt(){
	char cMsg[3] = "\0";
	int written_bytes=0;

        // Prepare and send message
	sprintf(cMsg,"%s\r", GET_GYRO_BATT);
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("robotnik_gyro::SendReadGyroBatt: Error sending message");
		return ERROR;
        }
	return OK;
}

/*!	\fn int robotnik_gyro::SendCalibrateOffsetGyro()
	* Sends the message to recalibrate the offset
 	* \return OK
	* \return ERROR
*/
int robotnik_gyro::SendCalibrateOffsetGyro(){
	char cMsg[3] = "\0";
	int written_bytes=0;

	sprintf(cMsg,"%s\r", CALIBRATE_OFFSET);

	// Sends the message
	if(serial->WritePort(cMsg, &written_bytes, strlen(cMsg)) != OK) {
		ROS_ERROR("robotnik_gyro::SendCalibrateOffsetGyro: Error sending message");
		return ERROR;
        }

	bSentOffsetCalibration = true;
	bSuccessCalibration = false;

	return OK;
}

/*!	\fn unsigned char robotnik_gyro::ComputeCRC(char *string, int size)
	* Calculates the CRC for the message
 	* \return OK
	* \return ERROR
*/
unsigned char robotnik_gyro::ComputeCRC(char *string, int size){

    int j=0;
    unsigned char crc=0;

    while ( (string[j] != 0) && ( j< size ) ) {
        crc +=string[j];
		//ROS_INFO(" char[%d] -> %c = %d, crc = %d", j, string[j], string[j], (int)crc);
        j++;
     }
    return crc;
}

/*!	\fn int robotnik_gyro::ProcessMsg(char *msg)
	* Process received messages from the device
 	* \return ERROR if nothing for processing
 	* \return NUM of Error from DSPIC
*/
int robotnik_gyro::ProcessMsg(char *msg){
	char *ptr;
	char cReceivedTokens[10][16];
	int i = 0;
	int num_error = 0;
	char command[64] = "\0";
	double th = 0.0;
    double batt = 0.0;
	int received_crc = 0, crc = 0;
	int MAX_TOKENS = 10;
	static int spi_error_count = 0;

	ptr = strtok (msg,",");

	// Extract all strings separated by comas
	// All tokens sent by device MUST have length < 16 or 
	// buffer ovfl will be generated 
	while ( (ptr != NULL) && (i<(MAX_TOKENS-1)) ) {
		sprintf(cReceivedTokens[i],"%s", ptr);
		ptr = strtok (NULL, ",");
		i++;
        }

	if(i == 0){
		ROS_ERROR("robotnik_gyro::ProcessMsg: nothing for processing");
		return ERROR;
        }
	//ROS_INFO(cReceivedTokens[0]);

	// Process the received response
	// Errors from device
	if(!strcmp(cReceivedTokens[0], "ERROR")){
		num_error = atoi(cReceivedTokens[1]);
		ROS_ERROR("robotnik_gyro::ProcessMsg: Error from device: %s,%s ", cReceivedTokens[0], cReceivedTokens[1]);
		if (num_error == ERROR_SPI) {
			spi_error_count++;
			if (spi_error_count > 10) {
				SwitchToState(FAILURE_STATE);
				iErrorType = DSPIC_ERROR_SPI;
				}
			}
		return num_error;
        }
    else {
		spi_error_count = 0;
		}

        // Reset detected
        if (i>=1) {
	  	if(!strcmp(cReceivedTokens[1], "Robotnik")){
			ROS_ERROR("robotnik_gyro::ProcessMsg: dsPIC board reseted !!!");
	 		return BOARD_RESET;
		        }
             }

	// Standard commands
	if(!strcmp(cReceivedTokens[0], GET_GYRO_BATT)){
                th = atof(cReceivedTokens[1]);
                batt = atof(cReceivedTokens[2]);
		received_crc = atoi(cReceivedTokens[3]);
		memset(command, 0, 64);
		sprintf(command,"%s,%2.5lf,%2.2lf", GET_GYRO_BATT, th, batt);
		//ROS_INFO(command);
		crc = ComputeCRC(command, 64);
		// If data arrived correctly
		if(crc == received_crc){
			// Update value
	                dsPic_data.dGyro = th;
	                dsPic_data.dBatt = batt;
		} else {			
			ROS_ERROR("robotnik_gyro::ProcessMsg: Get Gyro & Batt: Error crc: rec = %d, calc = %d", received_crc, crc);
			return ERROR_GET_GYRO_BATT;
            	}

		return OK_GET_GYRO_BATT;//
	  }

	if(!strcmp(cReceivedTokens[0], CALIBRATE_OFFSET)){
		//ROS_INFO(" [%s,%s]", CALIBRATE_OFFSET,cReceivedTokens[1]);
		if(!strcmp(cReceivedTokens[1], "OK\r")){
			pthread_mutex_lock(&mutex_gyro);
				dsPic_data.dOffsetGyro = -dsPic_data.dGyro;
				dsPic_data.dGyro = 0.0;
				bOffsetCalibration = true;
				bSentOffsetCalibration = false;
				bSuccessCalibration = true;
			pthread_mutex_unlock(&mutex_gyro);
			ROS_INFO("robotnik_gyro::ProcessMsg: Calibrate offset OK");
		}else{
			ROS_ERROR("robotnik_gyro::ProcessMsg: Error receiving calibrate offset confirmation");
			bSentOffsetCalibration = false;
			return ERROR_CALIBRATE_OFFSET;
                }
		return OK_CALIBRATE_OFFSET;
	  }
	return OK;
}

////////////////////////////////////////////////////////////////////////
// PUBLIC FUNCTIONS

/*!	\fn int robotnik_gyro::SetAngle(double yaw)
	* Changes current yaw value
*/
void robotnik_gyro::SetAngle(double yaw) {
	pthread_mutex_lock(&mutex_gyro);
		dsPic_data.yaw = yaw;
		dsPic_data.dOffsetGyro = (dsPic_data.dGyro - yaw);	// Changes the offset to the current value
	pthread_mutex_unlock(&mutex_gyro);
}

/*!	\fn void robotnik_gyro::ResetController()
	* Programs SendResetController Message
*/
void robotnik_gyro::ResetController()
{
	iCmd = DSPIC_CMD_RESET_CONTROLLER;
}

/*!	\fn void robotnik_gyro::ReadGyroBatt()
	* Programs SendReadGyroBatt Message
*/
void robotnik_gyro::ReadGyroBatt()
{
	iCmd = DSPIC_CMD_READ_GYRO_BATT;
}

/*!	\fn void robotnik_gyro::CalibrateOffsetGyro()
	* Programs SendCalibrateOffsetGyro Message
*/
void robotnik_gyro::CalibrateOffsetGyro()
{
	iCmd = DSPIC_CMD_CALIBRATE_OFFSET;
}

/*!	\fn bool robotnik_gyro::IsGyroCalibrated()
	* True if the gyroscope's offset has been calibrated
*/
bool robotnik_gyro::IsGyroCalibrated()
{
	if(bOffsetCalibration){
		bOffsetCalibration = false;
		return true;
	}else return false;
}

/*!	\fn float robotnik_gyro::GetVoltage()
	* Gets the power voltage value
*/
float robotnik_gyro::GetVoltage()
{
	return (float) dsPic_data.dBatt;
}

/*!	\fn void robotnik_gyro::GetGyroValues(double *angle, double *offset)
	*
*/
void robotnik_gyro::GetGyroValues(double *angle, double *offset){
	if( angle == NULL || offset == NULL) return;
	*angle = dsPic_data.dGyro;
	*offset = dsPic_data.dOffsetGyro;
}

/*!	\fn void robotnik_gyro::GetGyroBatt(double *angle, double *battery)
	*
*/
void robotnik_gyro::GetYawBatt(double *angle, double *battery){
	if( angle == NULL || battery == NULL) return;
	*angle = dsPic_data.yaw;	// dsPic_data.dGyro - dsPic_data.dOffsetGyro;
	*battery = (double) dsPic_data.dBatt;
}

/*!	\fn void robotnik_gyro::UpdateYaw()
	* Updates robot's yaw
*/
void robotnik_gyro::UpdateYaw(){

	double fSamplePeriod = 1.0 / desired_freq_;

	pthread_mutex_lock(&mutex_gyro);
		// Update yaw 
		double pa = (dsPic_data.dGyro - dsPic_data.dOffsetGyro);	// 180.0 * DSPIC_PI;
		dsPic_data.vel_yaw = (dsPic_data.yaw - pa) / fSamplePeriod;
		dsPic_data.yaw = pa;	    

		// Normalize
		// radnorm(&dsPic_data.yaw);
		while (dsPic_data.yaw >= (DSPIC_PI)) dsPic_data.yaw -= 2.0 * DSPIC_PI;
		while (dsPic_data.yaw <= (-DSPIC_PI)) dsPic_data.yaw += 2.0 * DSPIC_PI;
	pthread_mutex_unlock(&mutex_gyro);

}

/*!     \fn double robotnik_gyro::GetSamplingPeriod()
 *      \brief Returns accurate measured sampling period
 *      \return sample period in seconds
*/
double robotnik_gyro::GetSamplingPeriod() {
    static bool first = true;
    static ros::Time now, last;
    double secs=0.0;
    
    now = ros::Time::now();    

    if(first) {
        first = false;
        last= now;
        return -1.0;
        }

    secs = (now - last).toSec();
    last= now;
    return secs;
}


/*!     \fn void srvCallback_Recalibration(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
        * Callback - force sensor recalibration 
*/
bool robotnik_gyro::srvCallback_Recalibration(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  SwitchToState(CALIBRATION_STATE);    
  return true;
}

// MAIN
int main(int argc, char** argv)
{
        //robotnik_gyro::Battery battery;

	ros::init(argc, argv, "robotnik_gyro_node");
	//ROS_INFO("robotnik_gyro for ROS %.2f", NODE_VERSION);	
	//int time_remaining = -1;
	
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	
	pn.param<std::string>("port", port, "/dev/ttyS0");

        bool publish_gyro_tf = true;
        // default value version doesn't work for bool
        // pn.param<std::bool>("publish_gyro_tf", publish_gyro_tf, true);
        pn.getParam("publish_gyro_tf", publish_gyro_tf);
	
	std::string base_frame_id;
	std::string gyro_frame_id;
	pn.param<std::string>("base_frame_id", base_frame_id, "base_link");
	pn.param<std::string>("gyro_frame_id", gyro_frame_id, "gyro");

	// Create node object
	rbk_gyro = new robotnik_gyro( port );
	
	// Publishing
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/gyro_odom", 50);
	ros::Publisher yaw_pub = n.advertise<std_msgs::Float32>("/gyro", 50);
	//ros::Publisher battery_pub = n.advertise<robotnik_gyro::Battery>("/battery", 50);

    //! Service to force a gyro recalibration
	ros::ServiceServer srv_Recalibration = n.advertiseService("/recalibration", &robotnik_gyro::srvCallback_Recalibration, rbk_gyro);
	
	tf::TransformBroadcaster tf_broadcaster;
	
	if( rbk_gyro->Open()==OK ) ROS_INFO("Connected to robotnik_gyro component");
	else
	{
		ROS_FATAL("Could not connect to robotnik_gyro component");
	//	ROS_BREAK();
	}
	
	ros::Time current_time, last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(50.0);
	while(n.ok())
	{
		current_time = ros::Time::now();		

		// 1 State Machine
		rbk_gyro->StateMachine();

		// 2 Publish
		// ******************************************************************************************
		//first, we'll publish the transforms over tf
		geometry_msgs::TransformStamped gyro_trans;
		gyro_trans.header.stamp = current_time;
		gyro_trans.header.frame_id = gyro_frame_id;
		gyro_trans.child_frame_id = base_frame_id;
		gyro_trans.transform.translation.x = 0.0;   
		gyro_trans.transform.translation.y = 0.0;
		gyro_trans.transform.translation.z = 0.0;
		gyro_trans.transform.rotation = tf::createQuaternionMsgFromYaw(rbk_gyro->dsPic_data.yaw);
		if (publish_gyro_tf) tf_broadcaster.sendTransform(gyro_trans);
		
		// ******************************************************************************************
		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = gyro_frame_id;
		
		//set the position
		odom.pose.pose.position.x = 0.0;
		odom.pose.pose.position.y = 0.0;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rbk_gyro->dsPic_data.yaw);
		
		//set the velocity
		odom.child_frame_id = base_frame_id;
		odom.twist.twist.linear.x = 0.0;
		odom.twist.twist.linear.y = 0.0;
		odom.twist.twist.angular.z = rbk_gyro->dsPic_data.vel_yaw;
		
		//publish the gyro message
		odom_pub.publish(odom);

		// ******************************************************************************************
		// publish the raw angle values only if the component is in READY_STATE
		if (rbk_gyro->GetState()==READY_STATE) {
			std_msgs::Float32 yaw_msg;
			yaw_msg.data = rbk_gyro->dsPic_data.yaw;
			yaw_pub.publish(yaw_msg);
			}
		
		// ******************************************************************************************
		//publish battery
		//battery.level = robotnik_gyro->dsPic_data.dBatt;
                //battery.time_remaining = 3600;
		//battery_pub.publish(battery);	

		ros::spinOnce();
		r.sleep();
	}
	
	rbk_gyro->Close();
}
// EOF





