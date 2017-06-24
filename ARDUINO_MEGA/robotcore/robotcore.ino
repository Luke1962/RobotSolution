// COMANDI PER TEST: 
//AVANTI 20 E INDIETRO 20: 
//19,20;19,-20;


//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
//#define delay(ms) chThdSleepMilliseconds(ms) 

#include <MyRobotLibs\dbg.h>

#include <MyRobotLibs\systemConfig.h>
#include <MyRobotLibs\hw_config.h>
#include <MyRobotLibs\helper.h>
//#include <NewTone\NewTone.h>
//#include <MyRobotLibs\myStepper.h>

#pragma endregion
// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region LIBRERIE
/// ///////////////////////////////////////////////////////////////////////////////
// ROS
/// ///////////////////////////////////////////////////////////////////////////////
#include <ros.h>
#include <ros/duration.h>
#include <ros/time.h> //non serve
#include <ros/msg.h>
#include <ros_lib\sensor_msgs\Range.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_broadcaster.h>
 

#include <digitalWriteFast.h>
#include <ChibiOS_AVR.h>
#include <util/atomic.h>
//#include <TinyGPSplus/TinyGPS++.h>
//#include <StackArray.h>
#include <avr/interrupt.h>


#include <Arduino.h>	//per AttachInterrupt
//#include <Servo\src\Servo.h>
#include <Timer5/Timer5.h>


#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
#if OPT_MPU6050
	#include <I2Cdev\I2Cdev.h>

	#include <MPU6050\MPU6050_6Axis_MotionApps20.h>
//	#include <MPU6050\MPU6050.h> // not necessary if using MotionApps include file

	// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
	// is used in I2Cdev.h
	#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
	#endif

	// class default I2C address is 0x68
	// specific I2C addresses may be passed as a parameter here
	// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
	// AD0 high = 0x69
	MPU6050 mpu;
	//MPU6050 mpu(0x69); // <-- use for AD0 high


	// MPU control/status vars
	bool dmpReady = false;  // set true if DMP init was successful
	uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
	uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
	uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
	uint16_t fifoCount;     // count of all bytes currently in FIFO
	uint8_t fifoBuffer[64]; // FIFO storage buffer

							// orientation/motion vars
	Quaternion q;           // [w, x, y, z]         quaternion container
	VectorInt16 aa;         // [x, y, z]            accel sensor measurements
	VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
	VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
	VectorFloat gravity;    // [x, y, z]            gravity vector
	float euler[3];         // [psi, theta, phi]    Euler angle container
	float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

#endif // COMPASS
#if OPT_COMPASS
	#include <Wire\Wire.h>
	#include <compass\compass.h>
	MyCompass_c compass;
	//#include <Adafruit_Sensor\Adafruit_Sensor.h> //richiesto dalla liberia compass Adafruit_HMC5883_U
	//#include <HMC5883L\HMC5883L.h>
	//HMC5883L compass;
#endif // COMPASS
#if OPT_SONAR
	#include <Newping\NewPing.h>
	NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);


#endif //
#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	#include <Servo\src\Servo.h>
	#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)

	#include <PWM\PWM.h>
	Servo servoSonar;
#endif

#if OPT_GPS
	#include <TinyGPSplus\TinyGPS++.h> //deve essere incluso anche nel main

	TinyGPSPlus Gps;
#endif

#pragma region VL53L0X distanceSensor

#if OPT_LDS
	#include <Wire\Wire.h>
	#include <VL53L0X\VL53L0X.h>
	VL53L0X LDS;
	// Uncomment this line to use long range mode. This
	// increases the sensitivity of the distanceSensor and extends its
	// potential range, but increases the likelihood of getting
	// an inaccurate reading because of reflections from objects
	// other than the intended target. It works best in dark
	// conditions.
	#define LONG_RANGE
	// Uncomment ONE of these two lines to get
	// - higher speed at the cost of lower accuracy OR
	// - higher accuracy at the cost of lower speed

	//#define HIGH_SPEED
	//#define HIGH_ACCURACY

#endif // OPT_LDS



#if OPT_STEPPERLDS
//	#include <NewTone/NewTone.h> // PER GENERARE IL CLOCK DELLO STEPPER LDS
	#include <MyStepper\myStepper.h>
	myStepper_c  myStepperLDS(PIN_STEPPERLDS_ENABLE, PIN_STEPPERLDS_CK, PIN_STEPPERLDS_CW, PIN_STEPPERLDS_END);
#endif



#pragma endregion

#include <robot.h>
struct robot_c robot;	//was  struct robot_c robot;


// ////////////////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////

#if OPT_CMDMMI
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>

#endif // OPT_CMDMMI
// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI

//------------------------------------------------------------------------------
#pragma region DEFINIZIONE MAILBOX VOICE
/*
// mailbox size and memory pool object count
const size_t MBOX_COUNT = 6;

// type for a memory pool object
struct mboxObject_t {
	char* name;
	char str[100];
	int size;
};
// array of memory pool objects
mboxObject_t msgObjArray[MBOX_COUNT];

// memory pool structure
//MEMORYPOOL_DECL(memPool, MBOX_COUNT, 0);

// slots for mailbox messages
//msg_t letter[MBOX_COUNT];

// mailbox structure
//MAILBOX_DECL(mailVoice, &letter, MBOX_COUNT);
*/
#pragma endregion




/// ///////////////////////////////////////////////////////////////////////////////
// M U T E X ///////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
// Mutex for atomic access to data.
MUTEX_DECL(mutexMotion); //condiviso dai comandi di movimento e sonar per non essere in movimento quando il sonar scansiona
MUTEX_DECL(mutexSerialMMI);// accesso alla seriale
MUTEX_DECL(mutexSerialPC);// accesso alla seriale
MUTEX_DECL(mutexSensors);// accesso ai sensori in lettura e scrittura


// ////////////////////////////////////////////////////////////////////////////////////////////

#pragma endregion
#pragma region Helper Functions


// ////////////////////////////////////////////////////////////////////////////////////////////
///  bloccante
void lampeggiaLed(int pin, int freq, uint16_t nvolte) {
	int ms = 500 / freq;
	for (size_t i = 0; i < nvolte; i++)
	{
		digitalWriteFast(pin, 1);
		chThdSleepMilliseconds(ms);
		digitalWriteFast(pin, 0);
		chThdSleepMilliseconds(ms);

	}

}
// ////////////////////////////////////////////////////////////////////////////////////////////

void countDown(int seconds) {
	MSG3("CountDown in ", seconds, " sec...");
	for (size_t i = seconds; i > 0; i--)
	{
		MSG2("  -", i);
		delay(1000);
	}
}

// ////////////////////////////////////////////////////////////////////////////////////////////


// invia un  messaggio con la descrizione dell'evento e lo resetta 
void msgEventHR() {
	if (robot.status.pendingEvents.bumperF) { MSG("bumperF  EVENT"); robot.status.pendingEvents.bumperF = false; }
	if (robot.status.pendingEvents.bumperL) { MSG("bumperL  EVENT"); robot.status.pendingEvents.bumperL = false; }
	if (robot.status.pendingEvents.bumperR) { MSG("bumperR  EVENT"); robot.status.pendingEvents.bumperR = false; }
	if (robot.status.pendingEvents.EncL) { MSG("EncL  EVENT"); robot.status.pendingEvents.EncL = false; }
	if (robot.status.pendingEvents.EncR) { MSG("EncR  EVENT"); robot.status.pendingEvents.EncR = false; }
	if (robot.status.pendingEvents.irproxyB) { MSG("irproxyB  EVENT"); robot.status.pendingEvents.irproxyB = false; }
	if (robot.status.pendingEvents.irproxyF) { MSG("irproxyF  EVENT"); robot.status.pendingEvents.irproxyF = false; }
	if (robot.status.pendingEvents.irproxyFH) { MSG("irproxyFH  EVENT"); robot.status.pendingEvents.irproxyFH = false; }
	if (robot.status.pendingEvents.irproxyL) { MSG("irproxyL  EVENT"); robot.status.pendingEvents.irproxyL = false; }
	if (robot.status.pendingEvents.irproxyR) { MSG("irproxyR  EVENT"); robot.status.pendingEvents.irproxyR = false; }

	if (robot.status.pendingEvents.pirDome) { MSG("pirDome  EVENT"); robot.status.pendingEvents.pirDome = false; }
	//if (robot.status.pendingEvents.analog[0]) { MSG("POT EVENT"); robot.status.pendingEvents.analog[0]= false;	}
	//if (robot.status.pendingEvents.analog[2]) { MSG("LIGHT EVENT");robot.status.pendingEvents.analog[2]= false;	 }

}
// se pending invia messaggio di evento su Batteria,luca e GPS
void msgEventLR() {

	if (robot.status.pendingEvents.batCharge) { MSG("BATTERY EVENT"); robot.status.pendingEvents.batCharge = false; }
	if (robot.status.pendingEvents.light) { MSG("light  EVENT"); robot.status.pendingEvents.light = false; }
	if (robot.status.pendingEvents.gps) { MSG("GPS  EVENT"); robot.status.pendingEvents.gps = false; }

}

#pragma endregion



/// ///////////////////////////////////////////////////////////////////////////////
// ROS
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region ROS
#include <math.h>

#include <ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>





#include <std_msgs/Empty.h>


#include <tf/tf.h>
#include <tf/transform_broadcaster.h> 
#include <sensor_msgs/Range.h>    // ultrasound
#include <geometry_msgs/Twist.h>  // cmd_vel
#include <sensor_msgs/LaserScan.h>  // LDS
ros::NodeHandle  nh;

#define ROS_INFO(s) nh.loginfo(s);

#pragma endregion
// ////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
// PROCESSI CHIBIOS  /////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CHIBIOS


	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread ROS - pubblica Laser
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	sensor_msgs::LaserScan CDLaser_msg;
	ros::Publisher pub_Laser("LaserData", &CDLaser_msg);

	static THD_WORKING_AREA(waRosPublishLaserScan, 400);
	static THD_FUNCTION(thdRosPublishLaserScan, arg) {


		//
		float f_angle_min;
		float f_angle_max;
		float f_angle_increment;
		float f_time_increment;
		float f_scan_time;
		float f_range_min;
		float f_range_max;
		float f_ranges[5]; // max of 30 measurements
		float f_intensities[5];

		int publisher_timer;


		nh.initNode();
		nh.advertise(pub_Laser);

		f_angle_min = -1.57;
		f_angle_max = 1.57;
		f_angle_increment = 0.785;  // 3.14/4   - 5 measurement points
		f_time_increment = 10;
		f_scan_time = 4;
		f_range_min = 0.1;
		f_range_max = 30;

		CDLaser_msg.ranges_length = 5;
		CDLaser_msg.intensities_length = 5;

		// create the test data
		for (int z = 0; z<5; z++)
		{
			f_ranges[z] = z;
			f_intensities[z] = z*z;
		}
 
		while (true)
		{
			if (millis() > publisher_timer)
			{
				CDLaser_msg.header.stamp = nh.now();
				CDLaser_msg.header.frame_id = "laser_frame";
				CDLaser_msg.angle_min = f_angle_min;
				CDLaser_msg.angle_max = f_angle_max;
				CDLaser_msg.angle_increment = f_angle_increment;
				CDLaser_msg.time_increment = f_time_increment;
				CDLaser_msg.scan_time = f_scan_time;
				CDLaser_msg.range_min = f_range_min;
				CDLaser_msg.range_max = f_range_max;

				for (int z = 0; z<5; z++)
				{
					CDLaser_msg.ranges[z] = f_ranges[z];
				}

				for (int z = 0; z<5; z++)
				{
					CDLaser_msg.intensities[z] = f_intensities[z];
				}

				publisher_timer = millis() + 1000;
				pub_Laser.publish(&CDLaser_msg);
			}
			nh.spinOnce();
			chThdSleepMilliseconds(50);
			//delay(500);
		}
	 }


	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread ROS - pubblica Laser
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	static THD_WORKING_AREA(waRosPublishLDS, 400);
	static THD_FUNCTION(thdRosPublishLDS, arg) {
		//#define PIN_MOTORLDS_DIR 45
		//#define PIN_MOTORLDS_ENABLE 44
		//#define PIN_MOTORLDS_CK 11

		//chMtxLock(&mutexSensors);
		// Allo startup leggo tutti isensori
		robot.readAllSensors();	//IR proxy, Gyro, GPS
								//	chMtxUnlock(&mutexSensors);
		sensor_msgs::Range range_msg;
		ros::Publisher pub_range("/ultrasound", &range_msg);

		while (1)
		{
 
			///-------------------------------------------------------------
			// ROS Pubblicazione range_msg 
			///-------------------------------------------------------------

			range_msg.range = (float)robot.getLDSDistanceCm() / 100;
			range_msg.header.stamp = nh.now();
			pub_range.publish(&range_msg);

 

		}
		chThdSleepMilliseconds(50);
	 }

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   - SAFETY
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waSafety, 100);
static THD_FUNCTION(thdSafety, arg) {
	//robot.status.sensors.ignoreIR = true;

	while (true)
	{
		if (robot.status.isMoving && !robot.isObstacleFree())
		{
			robot.stop();
			MSG("Obstacle-Stop motor")
		}
		chThdSleepMilliseconds(200);
	}
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   - lettura sensori HR in robot.status
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

static THD_WORKING_AREA(waReadSensorsHR, 400);
static THD_FUNCTION(thdReadSensorsHR, arg) {
//	chMtxLock(&mutexSensors);
	// Allo startup leggo tutti isensori
	robot.readAllSensors();	//IR proxy, Gyro, GPS
//	chMtxUnlock(&mutexSensors);

	while (1)
	{
		LEDTOP_B_ON

//		chMtxLock(&mutexSensors);
		MSG("S HR>")
		robot.readSensorsHR();	//IR proxy, Gyro, GPS
		robot.status.tictac = !robot.status.tictac;
//		chMtxUnlock(&mutexSensors);
		LEDTOP_B_OFF


		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == MODE_AUTONOMOUS) {
			chThdSleepMilliseconds(500);// Sleep for n milliseconds.
		}
		else {
			chThdSleepMilliseconds(500);
		} 


	}
	//  return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 2B - lettura sensori LR in robot.status
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waReadSensorsLR, 400);
static THD_FUNCTION(thdReadSensorsLR, arg) {
	chMtxLock(&mutexSensors);
	// Allo startup leggo tutti isensori
	robot.readAllSensors();	//IR proxy, Gyro, GPS
//	chMtxUnlock(&mutexSensors);

	while (1)
	{
		LEDTOP_B_ON

//		chMtxLock(&mutexSensors);
		MSG("S LR>")
		robot.readSensorsLR();	//IR proxy, Gyro, GPS
		robot.status.tictac = !robot.status.tictac;
//		chMtxUnlock(&mutexSensors);
 
		LEDTOP_B_OFF

		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == MODE_AUTONOMOUS) {
			chThdSleepMilliseconds(3000);// Sleep for n milliseconds.
		}
		else {
			chThdSleepMilliseconds(5000);
		} 


	}
	//  return 0;
}





//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   MMIcommands - ricezione comandi da MMI
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#if OPT_CMDMMI
	static THD_WORKING_AREA(waMMIcommands, 200);
	static THD_FUNCTION(thdMMIcommands, arg) {
	//static msg_t MMIcommands(void *arg)


	// Setup CommandMessenger -----------------------------------------------------
	cmdMMI.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdMMI);// Attach my application's user-defined callback methods

	while (true)
	{
		dbg("1,MMI>; ");

		LEDTOP_B_ON

			///osalSysDisable(); //disabilita Interupt
			cmdMMI.feedinSerialData(); // Comando da interfaccia MMI ?, Se sì lo esegue
			///osalSysEnable();//abilita Interupt
		LEDTOP_B_OFF

			chThdSleepMilliseconds(900);
		//if (robot.operatingMode == SLAVE) 			{chThdSleepMilliseconds(200);}// Sleep for n milliseconds.
		//else if (robot.operatingMode == AUTONOMOUS)	{chThdSleepMilliseconds(500);}// Sleep for n milliseconds.
		//else										{chThdSleepMilliseconds(100);}// Sleep for n milliseconds.
	}
}



	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread   PCcommands - ricezione comandi da PC (per Debug)
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	static THD_WORKING_AREA(waPCcommands, 100);
	static THD_FUNCTION(thdPCcommands, arg) {
		// Setup CommandMessenger -----------------------------------------------------
		cmdPC.printLfCr();   // Adds newline to every command 
		attachCommandCallbacks(&cmdPC);// Attach my application's user-defined callback methods
		MSG("1,thdPCcommands STARTED;");

		while (true)
		{
			dbg("1,PC>; ");


			//osalSysDisable(); //disabilita Interupt
	//		chMtxLock(&mutexSerialPC);
			cmdPC.feedinSerialData();  // Comando da pc ?, Se sì lo esegue
	//		chMtxUnlock(&mutexSerialPC);
			//osalSysEnable();//abilita Interupt


			//yeld in base alla modalità operativa
			if (robot.status.operatingMode == MODE_SLAVE) {
				chThdSleepMilliseconds(500);// Sleep for n milliseconds.
			}
			else {
				chThdSleepMilliseconds(1000);
			}// Sleep for n milliseconds.

		}
	}
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread   SendStatusHR - invia lo stato sui vari canali
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	static THD_WORKING_AREA(waSendStatusHR, 100);
	static THD_FUNCTION(thdSendStatusHR, arg) {
		while (true)
		{
			if (robot.status.pendingEvents.EventFlag)
			{
				LEDTOP_B_ON;
				// wait to enter print region
				//chMtxLock(&mutexSerialMMI);
				msgEventHR();

				osalSysDisable(); //disabilita Interupt
				onCmdGetSensorsHRate(&cmdMMI);
				onCmdGetSensorsLRate(&cmdMMI);
				osalSysEnable();//abilita Interupt

				//chMtxUnlock(&mutexSerialMMI);


				//chMtxLock(&mutexSerialPC);
				onCmdGetSensorsHRate(&cmdPC);
				onCmdGetSensorsLRate(&cmdPC);
				//chMtxUnlock(&mutexSerialPC);



				LEDTOP_B_OFF

			}

			chThdSleepMilliseconds(800);// Sleep for n milliseconds.


		}
	}





	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread   SendStatusLR - invia lo stato sui vari canali
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	static THD_WORKING_AREA(waSendStatusLR, 200);
	static THD_FUNCTION(thdSendStatusLR, arg) {

		//chMtxLock(&mutexSerialMMI); //si blocca finchè la seriale è in uso da un altro thread
		//onCmdGetSensorsLRate(&cmdMMI);
		//chMtxUnlock(&mutexSerialMMI);

		//chMtxLock(&mutexSerialPC); //si blocca finchè la seriale è in uso da un altro thread
		//onCmdGetSensorsLRate(&cmdPC);
		//chMtxUnlock(&mutexSerialPC);

		while (true)
		{
			LEDTOP_B_ON
				if ((robot.status.operatingMode == MODE_AUTONOMOUS) &&
			 ( robot.status.pendingEvents.light
				|| robot.status.pendingEvents.batCharge
				|| robot.status.pendingEvents.gps
				))
			{
					//osalSysDisable(); //disabilita Interupt
					// wait to enter print region
					//chMtxLock(&mutexSerialMMI);
					msgEventLR();


				robot.status.pendingEvents.gps = false;
				robot.status.pendingEvents.batCharge = false;
				robot.status.pendingEvents.light = false;

				//		chMtxLock(&mutexSerialMMI); //si blocca finchè la seriale è in uso da un altro thread
				onCmdGetSensorsLRate(&cmdMMI);
				//		chMtxUnlock(&mutexSerialMMI);

				//		chMtxLock(&mutexSerialPC); //si blocca finchè la seriale è in uso da un altro thread
				onCmdGetSensorsLRate(&cmdPC);
				//		chMtxUnlock(&mutexSerialPC);

			}



			LEDTOP_B_OFF

			//yeld in base alla modalità operativa
			if (robot.status.operatingMode == MODE_AUTONOMOUS) {
				#ifdef DEBUG_ON
					chThdSleepMilliseconds(2000);// Sleep for n milliseconds.
				#else
					chThdSleepMilliseconds(5000);// Sleep for n milliseconds.
				#endif
			}
			else { //SLEEP TIME IN MODALITA' SLAVE O JOISTICK
				#ifdef DEBUG_ON
					chThdSleepMilliseconds(2000);// Sleep for n milliseconds.
				#else
					chThdSleepMilliseconds(15000);// Sleep for n milliseconds.
				#endif
			}


		}
	}




	//////////////////////////////////////////////////////////////////////////////////
	//  THREAD  B R A I N      ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	#pragma region  Processo	B R A I N    
	static THD_WORKING_AREA(waBrain, 100);
	static THD_FUNCTION(thdBrain, arg)
	{
		// Setup CommandMessenger -----------------------------------------------------
		cmdMMI.printLfCr();   // Adds newline to every command 
		attachCommandCallbacks(&cmdMMI);// Attach my application's user-defined callback methods

		int sleepTime = 500;
		const int FWDIST = 10; //distanza avanzamento
		int alfa = 0;
		int cmDone = 0; // percorso eseguito a valle di un comando moveCm o RotateDeg
		int stuckCount = 0;

		//1)inizializzo i sensori e le variabili
		int pingCm = 0;
		int ldsmm = 0;
		int estimatedDistCm = 0;
 
		MSG2("OPMODE..",(int)robot.status.operatingMode);

		robot.status.cmd.commandDir = commandDir_e::GOR;

		while (1)
		{
			LEDTOP_G_ON
			switch (robot.status.operatingMode)
			{
			case operatingMode_e::MODE_SLAVE:
				#pragma region SLAVE

					MSG("B s>")

	///				osalSysDisable(); //disabilita Interupt
					cmdMMI.feedinSerialData(); // Comando da interfaccia MMI ?, Se sì lo esegue
	//				osalSysEnable();//abilita Interupt


					//Invia i dati dei sensori se cambia qualcosa
					if (robot.raiseEvents()) {

						msgEventHR();
						msgEventLR();
						robot.resetEvents();
					}
					//MSG(" okei comanda");
					sleepTime = 1000;
					break;
				#pragma endregion

			case	operatingMode_e::MODE_AUTONOMOUS:

			#pragma region AUTONOMOUS
				sleepTime = 2000;
				MSG("B A>")


				onCmdGetSensorsHRate(&cmdMMI);
				onCmdGetSensorsHRate(&cmdPC);
				onCmdGetSensorsLRate(&cmdMMI);
				onCmdGetSensorsLRate(&cmdPC);
				onCmdGetProxy(&cmdMMI);

				// ////////////////////////////////////////////////////////////////////////////////
				/// ///////////////////////////////////////////////////////////////////////////////
				//  Esplora
				/// ///////////////////////////////////////////////////////////////////////////////
				// ////////////////////////////////////////////////////////////////////////////////
				#pragma region ESPLORA
				#if 0

							MSG("BE>")

								TOGGLEPIN(Pin_LED_TOP_B);
							robot.status.parameters.sonarStartAngle = 0;
							robot.status.parameters.sonarEndAngle = 180;
							robot.status.parameters.sonarStepAngle = 30;
							robot.status.parameters.sonarScanSweeps = 1;
							robot.status.parameters.sonarMedianSamples = 2;
							robot.status.parameters.sonarScanSpeed = 30; // map(analogRead(Pin_AnaPot1), 0, 1023, 10, 500);  //was = 30 ms di attesa tra due posizioni

							robot.LDSScanBatch();
							alfa = 90 - robot.status.parameters.SonarMaxDistAngle;
							alfa = 30;
							MSG2("Max dist @alfa:", alfa);
							MSG2("Max dist cm:", robot.status.parameters.sonarMaxDistance)
								TOGGLEPIN(Pin_LED_TOP_B);

							// invia i dati Sonar all'Host
							//onCmdSonarSendData(&cmdMMI);
							TOGGLEPIN(Pin_LED_TOP_B);

							///osalSysDisable();
							robot.rotateDeg(alfa);
							osalSysEnable();
							cmDone = robot.moveCm(robot.status.parameters.sonarMaxDistance);	// avanti
							if (cmDone < (robot.status.parameters.sonarMaxDistance - 1))	//ostacolo ?
							{
								MSG("Obstacle!");
								robot.moveCm(-FWDIST);	// torna indietro
								TOGGLEPIN(Pin_LED_TOP_B);
								stuckCount++;
								if (stuckCount > 2)
								{
									robot.rotateDeg(180); //inverto la direzione
									TOGGLEPIN(Pin_LED_TOP_B);
									stuckCount = 0;
								}
							}
							else //nessun ostacolo, azzero il contatore
							{
								stuckCount = 0;
							}
							TOGGLEPIN(Pin_LED_TOP_B);


							sleepTime = 1500;	// Sleep for 150 milliseconds.



				#endif // 0
						//  return 0;
				#pragma endregion
				#if 0//test che funzionino i sensori e movimento in ambiente chibios

							LASER_ON


							robot.go(robot.status.cmd.commandDir, robotSpeed_e::SLOW);// non bloccante
							MSG("MOTORI ATTIVATI")
								////2)avvio la rotazione del robot
								//osalSysDisable();
								////loop
								//while (true)
								//{
								//	// leggo LDS,sonar,heading
									robot.status.posCurrent.r = compass.getBearing();
									pingCm = robot.sonarPing();
									ldsmm = LDS.readRangeSingleMillimeters();
									if (!LDS.timeoutOccurred()) {
										estimatedDistCm = ldsmm / 10;
									}
									else //uso il sonar
									{
										estimatedDistCm = pingCm;
									}
									//invio i dati
									onCmdGetPose(&cmdMMI);
									MSG3("Dist: ", estimatedDistCm, "cm")
										//MSG3("Compass: ",robot.status.posCurrent.r,"°")
										//MSG3("lds: ",ldsmm, "mm")
										//	MSG3("Ping: ",pingCm, "cm")
								//}
								//osalSysEnable();

				#endif // TEST_ALL			#pragma region ESPLORA2



					break;

			#pragma endregion

			case operatingMode_e::MODE_JOYSTICK:
				//MSG(" okei comanda col gioistic");
				sleepTime = 5000;
				break;

			default:
				MSG(" modo sconosciuto");
				sleepTime = 2000;
				break;

			}

 



			LEDTOP_G_OFF

			chThdSleepMilliseconds(sleepTime);//	chThdYield();//	

		}
	}
	#pragma endregion 


#endif // OPT_CMDMMI




//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//  THREAD  N O N  A T T I V I  									/////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region THREAD NON ATTIVI
#if 0

	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread 1		- Esplora
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////

	static THD_WORKING_AREA(waThreadEsplora, 400);
	static THD_FUNCTION(ThreadEsplora, arg) {
		const int FWDIST = 10;
		int alfa = 0;
		int cmDone = 0; // percorso eseguito a valle di un comando moveCm o RotateDeg
		int stuckCount = 0;

		while (robot.status.operatingMode == AUTONOMOUS)
		{

			TOGGLEPIN(Pin_LED_TOP_B);
			robot.status.parameters.sonarStartAngle = 0;
			robot.status.parameters.sonarEndAngle = 180;
			robot.status.parameters.sonarStepAngle = 30;
			robot.status.parameters.sonarScanSweeps = 1;
			robot.status.parameters.sonarMedianSamples = 2;
			robot.status.parameters.sonarScanSpeed = 30; // map(analogRead(Pin_AnaPot1), 0, 1023, 10, 500);  //was = 30 ms di attesa tra due posizioni

			robot.SonarScanBatch(&servoSonar, &Sonar);
			alfa = 90 - robot.status.parameters.SonarMaxDistAngle;
			SERIAL_MSG.print("Max dist @alfa:"); SERIAL_MSG.println(alfa);
			dbg2("Max dist cm:", robot.status.parameters.sonarMaxDistance)
				TOGGLEPIN(Pin_LED_TOP_B);

			// invia i dati Sonar
			kbSonarSendData(&cmdMMI);
			TOGGLEPIN(Pin_LED_TOP_B);


			robot.rotateDeg(alfa);
			cmDone = robot.moveCm(robot.status.parameters.sonarMaxDistance);	// avanti
			if (cmDone < (robot.status.parameters.sonarMaxDistance - 1))	//ostacolo ?
			{
				SERIAL_MSG.println("1,Obst!;");
				robot.moveCm(-FWDIST);	// torna indietro
				TOGGLEPIN(Pin_LED_TOP_B);
				stuckCount++;
				if (stuckCount > 2)
				{
					robot.rotateDeg(180); //inverto la direzione
					TOGGLEPIN(Pin_LED_TOP_B);
					stuckCount = 0;
				}
			}
			else //nessun ostacolo, azzero il contatore
			{
				stuckCount = 0;
			}
			TOGGLEPIN(Pin_LED_TOP_B);


			chThdSleepMilliseconds(1500);	// Sleep for 150 milliseconds.

		}
		//  return 0;
	}


	//////////////////////////////////////////////////////////////////////////////////
	//  blinking LED       ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	/*
	#pragma region  Processo blinking LED
	// 64 byte stack beyond task switch and interrupt needs
	static THD_WORKING_AREA(waFlashLed, 64);
	static THD_FUNCTION(FlashLed, arg) {

	while (1) {
	// Turn LED on.
	LEDTOP_G_ON
	// Sleep for 50 milliseconds.
	chThdSleepMilliseconds(20);

	// Turn LED off.
	LEDTOP_G_OFF

	// Sleep for 150 milliseconds.
	chThdSleepMilliseconds(960);
	}
	}
	#pragma endregion
	*/


	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// thread   SCAN 
	//////////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waScan, 200);
static THD_FUNCTION(thdScan, arg) {

	while (!robot.status.operatingMode == MODE_AUTONOMOUS)
	{
		// Lock movements
		//chMtxLock(&mutexMotion);

		onCmdSonarScan(&cmdMMI);

		// Unlock data access.
		//chMtxUnlock(&mutexMotion);

		chThdSleepMilliseconds(1000);

	}
}




#endif // 0
#pragma endregion

#pragma endregion
// ////////////////////////////////////////////////////////////////////////////////////////////




// ////////////////////////////////////////////////////////////////////////////////////////////
// OS Setup (non cambia se non si aggiungono task)
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region [CHIBIOS RTOS]

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

 
void thdSetup() {
	// fill pool with msgObjArray array
	//for (size_t i = 0; i < MBOX_COUNT; i++) {
	//	chPoolFree(&memPool, &msgObjArray[i]);
	//}
	
	chThdCreateStatic(waRosPublishLaserScan, sizeof(waRosPublishLaserScan), NORMALPRIO + 6, thdRosPublishLaserScan, NULL);
	//chThdCreateStatic(waSafety, sizeof(waSafety), NORMALPRIO +10, thdSafety, NULL);
	chThdCreateStatic(waReadSensorsLR, sizeof(waReadSensorsLR), NORMALPRIO + 6, thdReadSensorsLR, NULL);
	//chThdCreateStatic(waRos, sizeof(waRos), NORMALPRIO + 6, thdRos, NULL);
	chThdCreateStatic(waReadSensorsHR, sizeof(waReadSensorsHR), NORMALPRIO + 5, thdReadSensorsHR, NULL);
//	chThdCreateStatic(waBrain, sizeof(waBrain), NORMALPRIO + 5, thdBrain, NULL);
	//chThdCreateStatic(waSendStatusHR, sizeof(waSendStatusHR), NORMALPRIO +4, thdSendStatusHR, NULL);
	//chThdCreateStatic(waSendStatusLR, sizeof(waSendStatusLR), NORMALPRIO +4, thdSendStatusLR, NULL);
	//chThdCreateStatic(waPCcommands, sizeof(waPCcommands), NORMALPRIO + 3, thdPCcommands, NULL);
	//chThdCreateStatic(waMMIcommands, sizeof(waMMIcommands), NORMALPRIO + 3, thdMMIcommands, NULL);
	//chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 2, FlashLed, NULL);
	//	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 1, ThreadMonitor, NULL);
	//	chThdCreateStatic(watestRotaryEncoder, sizeof(watestRotaryEncoder), NORMALPRIO + 2, testRotaryEncoder, NULL);
	MSG("Thread chibios avviati ..");
	while (1) {}

}





// ########################################################################################
// ########################################################################################
// ENCODER ISR
// opera solo se  targetEncoderThicks > 0 ed ogni ISR_MINMUM_INTERVAL_MSEC
// ########################################################################################
// ########################################################################################
// Interrupt service routines for the right motor's quadrature encoder
volatile bool interruptFlag = false;
volatile uint32_t robotstatussensorsEncR = 0;
volatile unsigned long isrCurrCallmSec = 0;
volatile unsigned long isrLastCallmSec = 0;

#define ISR_MINMUM_INTERVAL_MSEC 15  // 30 = circa ROBOT_MOTOR_STEPS_PER_ENCODERTICK * ROBOT_MOTOR_STEPS_PER_ENCODERTICK
void ISRencoder()
{
	if (robot.status.cmd.targetEncoderThicks > 0) //faccio lavorare l'ISR solo se targetEncoderThicks> 0
	{
		isrCurrCallmSec = millis();

		if (isrCurrCallmSec > (isrLastCallmSec + ISR_MINMUM_INTERVAL_MSEC))
		{
			LEDTOP_B_ON
			isrLastCallmSec = isrCurrCallmSec;
			robot.status.sensors.EncR += 1;  
			if (robot.status.sensors.EncR > robot.status.cmd.targetEncoderThicks)
			{
				LEDTOP_G_ON
				robot.stop();
				//MOTORS_DISABLE
				robot.status.cmd.targetEncoderThicks = 0;

			}
			LEDTOP_B_OFF
		}
 
	}

}
// ########################################################################################
// ########################################################################################
//  S E T U P
// ########################################################################################
// ########################################################################################
void setup()
{
	lampeggiaLed(Pin_LED_TOP_R, 3, 9);
	//lampeggiaLed(Pin_LED_TOP_G, 3, 5);
	//lampeggiaLed(Pin_LED_TOP_B, 3, 5);
	LEDTOP_R_ON	// Indica inizio SETUP Phase
	MOTORS_DISABLE
	attachInterrupt(digitalPinToInterrupt(3), ISRencoder, CHANGE);

	pinMode(Pin_LED_TOP_G, OUTPUT);
	InitTimersSafe(); //initialize all timers except for 0, to save time keeping functions
					  //sets the frequency for the specified pin
	//bool success = SetPinFrequencySafe(Pin_LED_TOP_R, 2);

	////if the pin frequency was set successfully, turn pin 13 on
	//if (success) {
	//	digitalWrite(Pin_LED_TOP_B, HIGH);
	//	pwmWrite(Pin_LED_TOP_R, 128); //set 128 to obtain 50%duty cyle
	//	SetPinFrequency(Pin_LED_TOP_R, 2); //setting the frequency to 10 Hz
	//}
	
	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
	MSG("####[  ROBOTCORE v0.3  ]####");
	WEBCAM_OFF
	WEBCAM_ON
	MSG3("Bat : ",robot.readBattChargeLevel(),"%");

	MSG2("A0 : ",analogRead(A0));
	MSG2("A1 : ",analogRead(A1));
	MSG2("A1 : ",analogRead(A2));

	robot.initHW(); //disabilita i motori
//	tone(Pin_LED_TOP_R, 2, 0);
	countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE
	Wire.begin(); // default timeout 1000ms

#if OPT_SEROVOSONAR
	robot.initRobot( &servoSonar);
#endif
#if OPT_GPS
	robot.initRobot( &Gps);
#endif
#if OPT_LDS
	robot.initRobot( &LDS);
	robot.initLDS( &LDS);
#endif
#if OPT_COMPASS
	robot.initRobot( &compass);
	robot.initCompass(&compass);//include compass.begin();
#endif
#if OPT_STEPPERLDS
	myStepperLDS.goRadsPerSecond(0.1);
	myStepperLDS.enable();

#endif
#if OPT_MPU6050

	MSG("Initializing MPU6050...");

	mpu.initialize();
	// load and configure the DMP
	devStatus = mpu.dmpInitialize();
	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(220);
	mpu.setYGyroOffset(76);
	mpu.setZGyroOffset(-85);
	mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
	MSG("...done");

 #endif

 
	robot.initRobot();

//	noTone(Pin_LED_TOP_R);


	MSG("ACCENDI I MOTORI");
	countDown(5);

//	LEDTOP_G_ON	// Indica inizio SETUP Phase


// test di obstacleFree()		 
#if 0		
	MSG("MOTION TEST");
	countDown(5);  //PER DARE IL TEMPO ALL COMPASS DI RISPONDERE

	MSG("MOVE FW 5cm");
	robot.moveCm(10);
	robot.moveCm(-10);
	robot.rotateDeg(45);
	robot.rotateDeg(-90);
	robot.rotateDeg(45);
	MSG("END Testing robot.moveCm()");
#endif // 1


 




	//	robot.compassCalibrate();

	LEDTOP_R_ON
	

	//#define HOME_HEADING 90
	//MSG2("## goHeading :", HOME_HEADING);
	//robot.stop();
	////MOTORS_DISABLE
	//robot.status.sensors.EncR = 0;//resetto il conteggio encoder ticks
	////robot.status.cmd.targetEncoderThicks = 0;
	//robot.goHeading(HOME_HEADING, 5);
	//LEDTOP_R_OFF



	// ########################################################################################
	// ########################################################################################
	//  TEST OPZIONALI
	// ########################################################################################
	// ########################################################################################

	#pragma region TEST OPZIONALI PRIMA DELL'AVVIO DELL'OS
			int targetAngle = 0;
			int freeDistCm = 0;
			char* charVal ;

		// test IMU
		#if 0

			MSG("TST MPU")
			long tStart = millis();
			while ( millis()- tStart < 10000){
				if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
					// reset so we can continue cleanly
					mpu.resetFIFO();
					MSG("FIFO overflow!");

					// otherwise, check for DMP data ready interrupt (this should happen frequently)
				}
				else if (mpuIntStatus & 0x02) {
					// wait for correct available data length, should be a VERY short wait
					while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

					// read a packet from FIFO
					mpu.getFIFOBytes(fifoBuffer, packetSize);

					// track FIFO count here in case there is > 1 packet available
					// (this lets us immediately read more without waiting for an interrupt)
					fifoCount -= packetSize;

					mpu.dmpGetQuaternion(&q, fifoBuffer);
					dtostrf(q.z, 4, 3, charVal);	//float to char array
					MSG("MPU")
					Serial1.print("1,quat ");
					Serial1.print(q.w);
					Serial1.print(",");
					Serial1.print(q.x);
					Serial1.print(",");
					Serial1.print(q.y);
					Serial1.print(",");
					Serial1.print (q.z);
					Serial1.println(";");

				}

				robot.readBumpers();

			}
				
		#endif // 1


		//TEST_COMPASS_3   ### BLOCCANTE ###
		#if 1		//calibra  e poi ruota verso Nord
			// Initialise the sensor 
			robot.stop();
			robot.status.sensors.EncR = 0;


			MSG("Test GO 1 giro....")
			//robot.goCWAndScan(20);
			long targetRads =2* PI; //target in radianti
			robot.status.cmd.targetEncoderThicks = 4375 / ROBOT_MOTOR_STEPS_PER_ENCODERTICK;// targetRads *  ROBOT_MOTOR_STEPS_PER_RADIANT / ROBOT_MOTOR_STEPS_PER_ENCODERTICK;
			dtostrf(robot.status.cmd.targetEncoderThicks, 2, 10, charVal);
			MSG2("Target Enc.", charVal);
			countDown(5);

			robot.go(commandDir_e::GOR, robotSpeed_e::MEDIUM);
			STEPPERLDS_ON;
			myStepperLDS.enable();
			myStepperLDS.goRadsPerSecond(0.314);// 10" per 180°
			while (true)
			{
				MSG2("Enc R.", robot.status.sensors.EncR);
				MSG2("Compass deg.", robot.readCompassDeg());

				STEPPERLDS_INVERTDIR;
				for (size_t i = 0; i < 100; i++)
				{
					STEPPERLDS_STEP;
					delay(1000/20);
					MSG2("  i: ", i);
					MSG2("LDS.", robot.getLDSDistanceCm());
				}

				delay(1000);
				
				// abilita stepper LDS


			}
			STEPPERLDS_OFF


		#endif  


	#pragma endregion //regione dei test



	//SERIAL_MSG.println(F("1,running IBIT...;"));
	//robot.runIBIT(300);



	MSG("GO HEADING 330°");
	// mi allineo parallelo al corridoio tra libreria e divano
	robot.goHeading(330, 5); //bloccante se non va il Compass

	LEDTOP_R_OFF
	lampeggiaLed(Pin_LED_TOP_G, 5, 10);

	chBegin(thdSetup);
}


void loop() {
	// not used
}

#pragma endregion


