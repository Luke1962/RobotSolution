// COMANDI PER TEST: 
//AVANTI 20 E INDIETRO 20: 
//19,20;19,-20;

//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
#define delay(ms) chThdSleepMilliseconds(ms) 
#define DEBUG_ON 
#include <MyRobotLibs\dbg.h>

#include <MyRobotLibs\systemConfig.h>
#include <MyRobotLibs\hw_config.h>

#pragma endregion


// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region LIBRERIE
#include <Wire.h>
//// ROS
//#include <ros_lib/ros/ros.h>
//#include <ros_lib/ros/duration.h>
//#include <ros_lib/ros/time.h> //non serve
//#include <ros_lib/sensor_msgs/Range.h>

#include <digitalWriteFast.h>
#include <ChibiOS_AVR.h>
#include <util/atomic.h>
#include <TinyGPSplus/TinyGPS++.h>
#include <StackArray.h>


//#include <FrequencyTimer2\FrequencyTimer2.h>	
//#include <TimerThree\TimerThree.h>
//#include <FlexiTimer2\FlexiTimer2.h>
//#include <encoder/Encoder.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
#include <Newping\NewPing.h>
#include <PWM\PWM.h>
//#include <avr/wdt.h>
//#include <robotmodel.h>
#include <robot.h>
#include <VL53L0X\VL53L0X.h>
//#include "stringlib.h"

#include <HMC5883L\HMC5883L.h>	//compass


#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
	HMC5883L compass;
#define COMPASS HMC5883L
	#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	Servo servoSonar;
	NewPing Sonar(Pin_SonarTrig, Pin_SonarEcho);
	#endif

	struct robot_c robot;	//was  struct robot_c robot;

	#pragma region VL53L0X distanceSensor
		VL53L0X distanceSensor;
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
		#define HIGH_ACCURACY

	#pragma endregion


	// ////////////////////////////////////////////////////////////////////////////////////////////


	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CmdMessenger object to the default Serial port
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#include <CmdMessenger2/CmdMessenger2.h>
	static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
	static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
	#include <MyRobotLibs\RobotInterfaceCommands2.h>

	//------------------------------------------------------------------------------
	#pragma region DEFINIZIONE MAILBOX VOICE
		// mailbox size and memory pool object count
		const size_t MB_COUNT = 6;

		// type for a memory pool object
		struct PoolObject_t {
			char* name;
			char str[100];
			int size;
		};
		// array of memory pool objects
		PoolObject_t PoolObject[MB_COUNT];

		// memory pool structure
		MEMORYPOOL_DECL(memPool, MB_COUNT, 0);

		// slots for mailbox messages
		msg_t letter[MB_COUNT];

		// mailbox structure
		MAILBOX_DECL(mailVoice, &letter, MB_COUNT);

	#pragma endregion

	// ////////////////////////////////////////////////////////////////////////////////////////////

#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  FUNZIONI E UTILITIES GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
//void playSingleNote(int pin, int freq,int noteDuration) {
//	tone(pin, freq, noteDuration);
//	noTone(pin);
//}
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



//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
// PROCESSI CHIBIOS  /////////////////////////////////////////////////////////////
//                   /////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 1		- Esplora
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/*
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
			if (stuckCount>2)
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
*/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 2 - lettura sensori in robot.status
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waReadSensors, 400);
static THD_FUNCTION(thdReadSensors, arg) {

	while (1)
	{
		// TODO METTERE SEMAFORO PER INVIARE STATO CONSISTENTE
		LEDTOP_R_ON
		robot.readSensors();	//IR proxy, Gyro, GPS
		LEDTOP_R_OFF
		robot.status.tictac = !robot.status.tictac;
		//yeld in base alla modalità operativa
		if (robot.status.operatingMode==AUTONOMOUS)	{	
			chThdSleepMilliseconds(100);// Sleep for n milliseconds.
		}else{	
			chThdSleepMilliseconds(2000);}// Sleep for n milliseconds.

		
	}
	//  return 0;
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   MMIcommands - ricezione comandi da MMI
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waMMIcommands, 200);
static THD_FUNCTION(thdMMIcommands, arg) {
	//static msg_t MMIcommands(void *arg)


	// Setup CommandMessenger -----------------------------------------------------
	cmdMMI.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdMMI);// Attach my application's user-defined callback methods

	while (true)
	{
		LEDTOP_B_ON
		
		osalSysDisable(); //disabilita Interupt
		cmdMMI.feedinSerialData(); // Comando da interfaccia MMI ?, Se sì lo esegue
		osalSysEnable();//abilita Interupt
		LEDTOP_B_OFF

		chThdSleepMilliseconds(500);
		//if (robot.operatingMode == SLAVE) 			{chThdSleepMilliseconds(200);}// Sleep for n milliseconds.
		//else if (robot.operatingMode == AUTONOMOUS)	{chThdSleepMilliseconds(500);}// Sleep for n milliseconds.
		//else										{chThdSleepMilliseconds(100);}// Sleep for n milliseconds.
	}
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   PCcommands - ricezione comandi da PC
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waPCcommands, 200);
static THD_FUNCTION(thdPCcommands, arg) {
	// Setup CommandMessenger -----------------------------------------------------
	cmdPC.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdPC);// Attach my application's user-defined callback methods

	while (true)
	{
		LEDTOP_G_ON

		osalSysDisable(); //disabilita Interupt
 		cmdPC.feedinSerialData();  // Comando da pc ?, Se sì lo esegue
		osalSysEnable();//abilita Interupt

		LEDTOP_G_OFF

		chThdSleepMilliseconds(500);

	}
}


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   SendStatus - invia lo stato sui vari canali
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waSendStatus, 200);
static THD_FUNCTION(thdSendStatus, arg) {
   
	while (true)
	{
		LEDTOP_G_ON		
 
		OnCmdGetSensorsHRate(&cmdMMI);
		OnCmdGetSensorsHRate(&cmdPC);
 
 		LEDTOP_G_OFF

		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == AUTONOMOUS) {
			chThdSleepMilliseconds(500);// Sleep for n milliseconds.
		}
		else {
			chThdSleepMilliseconds(5000);
		}// Sleep for n milliseconds.
	}
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   SCAN 
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waScan, 200);
static THD_FUNCTION(thdScan, arg) {
   
	while (!robot.status.operatingMode == AUTONOMOUS)
	{
 
		LEDTOP_G_ON
		OnCmdSonarScanDefault(&cmdMMI);
		LEDTOP_G_OFF

		chThdYield(); //		chThdSleepMilliseconds(5000);

	}
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//  THREAD  N O N  A T T I V I  									/////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

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
// M O N I T O R	   ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*
#pragma region Processo di MONITOR
volatile uint32_t count = 0;
volatile uint32_t maxDelay = 0;

static THD_WORKING_AREA(waThreadMonitor, 64);
static THD_FUNCTION(ThreadMonitor, arg) {

	while (true)
	{

		Serial.print(F("    waMMIcommands unused stack: "));
		Serial.println(chUnusedStack(waMMIcommands, sizeof(waMMIcommands)));
		//Serial.print(F("    overrun errors: "));
		//Serial.println( OverrunErrorCount);

		count++;
		//FIFO_SPEAK.push(int2str(count));
		uint32_t t = micros();
		// yield so other threads can run
		chThdYield();
		t = micros() - t;
		if (t > maxDelay) maxDelay = t;

		chThdSleepMilliseconds(3000);//	chThdYield();//	
									 //		SPEAK_CIAO
	}

}

#pragma endregion 
*/

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 3 - Invio sensori a bassa frequenza
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/*
static THD_WORKING_AREA(waThreadSendSensorsLR, 200);
static THD_FUNCTION(ThreadSendSensorsLR, arg) {
//SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
cmdWiFi.printLfCr();   // Adds newline to every command

//loop ---------------------------------------------------------------
while (1) {
robot.readGps();
SERIAL_MSG.print("1, SATS=");
SERIAL_MSG.print(robot.status.gps.sats); // Number of satellites in use (u32)
SERIAL_MSG.print(" LAT=");  SERIAL_MSG.print(robot.status.gps.lat, 6);
SERIAL_MSG.print(" LONG="); SERIAL_MSG.print(robot.status.gps.lng, 6);
SERIAL_MSG.print(" ALT=");  SERIAL_MSG.print(robot.status.gps.alt);
SERIAL_MSG.print(" Home dist=");  SERIAL_MSG.print(robot.status.gps.homeDistCm);
SERIAL_MSG.print(" Home angle=");  SERIAL_MSG.print(robot.status.gps.homeAngleDeg);
SERIAL_MSG.println(";");


chThdSleepMilliseconds(5000);// Sleep for n milliseconds.
}
//	return 0;
}*/


// ////////////////////////////////////////////////////////////////////////////////////////////
// OS Setup (non cambia se non si aggiungono task)
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region [CHIBIOS RTOS]

extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

uint16_t getFreeSram() {
	uint8_t newVariable;
	// heap is empty, use bss as start memory address 
	if ((uint16_t)__brkval == 0)
		return (((uint16_t)&newVariable) - ((uint16_t)&__bss_end));
	// use heap end as the start of the memory address 
	else
		return (((uint16_t)&newVariable) - ((uint16_t)__brkval));
};

void chSetup() {
	// fill pool with PoolObject array
	for (size_t i = 0; i < MB_COUNT; i++) {
		chPoolFree(&memPool, &PoolObject[i]);
	}

	chThdCreateStatic(waScan, sizeof(waScan), NORMALPRIO + 4, thdScan, NULL);
	chThdCreateStatic(waPCcommands, sizeof(waPCcommands), NORMALPRIO + 4, thdPCcommands, NULL);
	chThdCreateStatic(waMMIcommands, sizeof(waMMIcommands), NORMALPRIO + 2, thdMMIcommands, NULL);
	chThdCreateStatic(waReadSensors, sizeof(waReadSensors), NORMALPRIO + 2, thdReadSensors, NULL);
	chThdCreateStatic(waSendStatus, sizeof(waSendStatus), NORMALPRIO , thdSendStatus, NULL);
	//chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 2, FlashLed, NULL);
//	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 1, ThreadMonitor, NULL);
//	chThdCreateStatic(waThreadROS, sizeof(waThreadROS), NORMALPRIO + 3, ThreadROS, NULL);//-Esplora
//	chThdCreateStatic(watestRotaryEncoder, sizeof(watestRotaryEncoder), NORMALPRIO + 2, testRotaryEncoder, NULL);
	SERIAL_MSG.println("1,Thread chibios avviati ..;");
	while (1) {}

}

void setup()
{
	LEDTOP_R_ON
	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
	SERIAL_MSG.println("1,RUNNING ROBOTCORE;");
	SERIAL_MSG.println("1,test Speak;");
	SERIAL_MMI.println("28,CIAO CIAO CIAO;");

	Wire.begin();

	#pragma region distanceSensor Setup
		SERIAL_MSG.print("LaserDistance sensor init...");
		#if 0
			distanceSensor.init();

		#else
			bool initDone = false;		
			while (!initDone)
			{
				if (distanceSensor.init())
				{
					SERIAL_MSG.println("..OK");
					initDone = true;
				}
				else
				{
					SERIAL_MSG.println("..FAIL");
					delay(500);
				}
			}

		#endif
		#if defined LONG_RANGE
			// lower the return signal rate limit (default is 0.25 MCPS)
			distanceSensor.setSignalRateLimit(0.1);
			// increase laser pulse periods (defaults are 14 and 10 PCLKs)
			distanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
			distanceSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
		#endif
		#if defined HIGH_SPEED
			// reduce timing budget to 20 ms (default is about 33 ms)
			distanceSensor.setMeasurementTimingBudget(20000);
		#elif defined HIGH_ACCURACY
			// increase timing budget to 200 ms
			distanceSensor.setMeasurementTimingBudget(200000);
		#endif
	#pragma endregion

	#pragma region Initialize compass HMC5883L
		// Initialize Initialize HMC5883L
		SERIAL_MSG.println("Initialize HMC5883L");
		while (!compass.begin())
		{
			Serial.println("Could not find a valid HMC5883L sensor, check wiring!");
			delay(500);
		}

		// Set measurement range
		compass.setRange(HMC5883L_RANGE_1_3GA);

		// Set measurement mode
		compass.setMeasurementMode(HMC5883L_CONTINOUS);

		// Set data rate
		compass.setDataRate(HMC5883L_DATARATE_30HZ);

		// Set number of samples averaged
		compass.setSamples(HMC5883L_SAMPLES_8);

		// Set calibration offset. See HMC5883L_calibration.ino
		compass.setOffset(0, 0);


	#pragma endregion


	// MOTOR ENCODER
	//attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), loadEncoderPositionOnChange, CHANGE);
	//attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), loadEncoderPositionOnChange, CHANGE);

	// inizializzazione ROBOT ---------------------
	robot.initServoAndSonar(&servoSonar, &Sonar, &distanceSensor, &compass);

	#if 0//test ixproxy
	SERIAL_MSG.print("testing ixproxy...");
		while (true)
		{
			robot.readSensors();
			if (robot.status.sensors.irproxy.fw != robot.statusOld.sensors.irproxy.fw)
			{
				SERIAL_MSG.println("irproxy.fw changed");
				TOGGLEPIN(Pin_LED_TOP_B);

			}
			else if (robot.status.sensors.irproxy.fwHL != robot.statusOld.sensors.irproxy.fwHL)
			{
				SERIAL_MSG.println("irproxy.fwHl changed");
				TOGGLEPIN(Pin_LED_TOP_G);

			}
		}

	#endif // 0//test ixproxy


	#if 0		// test di obstacleFree()
		SERIAL_MSG.print("testing robot.obstacleFree()...");
		bool val = false;
		bool oldVal = false;
		robot.status.cmd.commandDir = GOF;
		while (true)
		{
			val = robot.obstacleFree();
			if (oldVal !=val )
			{
				TOGGLEPIN(Pin_LED_TOP_B);
				SERIAL_MSG.println("robot.obstacleFree() changed");
				oldVal = val;
			}
		}
	#endif // 1

	#if 0		// test di obstacleFree()
		SERIAL_MSG.print("testing robot.moveCm()...");
		int cmFatti = 0;
		#define DIST 30
		while (true)
		{
			cmFatti = robot.moveCm(DIST);
			if (cmFatti <= DIST)
			{
				TOGGLEPIN(Pin_LED_TOP_R);
				SERIAL_MSG.print("OSTACOLO DOPO CM:"); SERIAL_MSG.println(cmFatti);
				robot.moveCm(-cmFatti);
			}
			else
			{
				TOGGLEPIN(Pin_LED_TOP_G);

			}
		}
	#endif // 1

		while (true)
	{

		SERIAL_MSG.println("testing OnCmdSonarScanDefault...");
		OnCmdSonarScanDefault(&cmdPC);
		//robot.LaserScanBatch();
	}
	
	SERIAL_MSG.print("COMPASS Degress = ");
	SERIAL_MSG.println(robot.getCompassDeg());


	SERIAL_MSG.print("running IBIT...");
	robot.runIBIT(300, &servoSonar, &Sonar);

	WEBCAM_ON

	LEDTOP_R_OFF
	//while (!robot.isPowerOn())
	//{
	//	SERIAL_MSG.println("1,!!!WARNING POWER IS OFF!!!.");
	//	lampeggiaLed(Pin_LED_TOP_R, 5, 500);
	//	delay(500);
	//}
	SERIAL_MSG.println("1,starting chibios tasks..");
	//lampeggiaLed(Pin_LED_TOP_G, 5, 500);
	chBegin(chSetup);	



}


void loop() {
	// not used
}

#pragma endregion


