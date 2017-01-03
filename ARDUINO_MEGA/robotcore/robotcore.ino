// COMANDI PER TEST: 
//AVANTI 20 E INDIETRO 20: 
//19,20;19,-20;


//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA   
#define delay(ms) chThdSleepMilliseconds(ms) 

//#define DEBUG_OFF
//#define DEBUG_ON
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

//#include <I2C\I2C.h>//was
#include <Wire\Wire.h>
#include <I2Cdev\I2Cdev.h>

#include <digitalWriteFast.h>
#include <ChibiOS_AVR.h>
#include <util/atomic.h>
#include <TinyGPSplus/TinyGPS++.h>
//#include <StackArray.h>


//#include <FrequencyTimer2\FrequencyTimer2.h>	
//#include <TimerThree\TimerThree.h>
//#include <FlexiTimer2\FlexiTimer2.h>
//#include <encoder/Encoder.h>
//#include "stringlib.h"
//#include <avr/wdt.h>
//#include <robotmodel.h>
#include <robot.h>



#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
#if OPT_COMPASS
	//#include <Adafruit_Sensor\Adafruit_Sensor.h> //richiesto dalla liberia compass Adafruit_HMC5883_U
 	#include <HMC5883L\HMC5883L.h>
	HMC5883L compass;
	#define COMPASS HMC5883L
#endif // COMPASS

#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	#include <Newping\NewPing.h>
	#include <Servo\src\Servo.h>
#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)

#include <PWM\PWM.h>
	Servo servoSonar;
	NewPing Sonar(Pin_SonarTrig, Pin_SonarEcho);
#endif

#if OPT_GPS
	#include <TinyGPSplus\TinyGPS++.h> //deve essere incluso anche nel main

	TinyGPSPlus Gps;
#endif

#pragma region VL53L0X distanceSensor

#if OPT_LDS
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
	#define HIGH_ACCURACY

#endif // OPT_LDS

#pragma endregion


struct robot_c robot;	//was  struct robot_c robot;


// ////////////////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger2/CmdMessenger2.h>
static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
#include <MyRobotLibs\RobotInterfaceCommands2.h>
#define MMI_MSG(t) cmdMMI.msg(t);
#define MMI_MSG2(t,v) cmdMMI.msg(t,v);
//------------------------------------------------------------------------------
#pragma region DEFINIZIONE MAILBOX VOICE
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
						

/// ///////////////////////////////////////////////////////////////////////////////
// M U T E X ///////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
// Mutex for atomic access to data.
MUTEX_DECL(mutexMotion); //condiviso dai comandi di movimento e sonar per non essere in movimento quando il sonar scansiona
MUTEX_DECL(mutexSerialMMI);// accesso alla seriale
MUTEX_DECL(mutexSerialPC);// accesso alla seriale
MUTEX_DECL(mutexSensors);// accesso ai sensori in lettura e scrittura

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
// thread 2A - lettura sensori HR in robot.status
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waReadSensorsHR, 400);
static THD_FUNCTION(thdReadSensorsHR, arg) {
	chMtxLock(&mutexSensors);
	// Allo startup leggo tutti isensori
	robot.readAllSensors();	//IR proxy, Gyro, GPS
	chMtxUnlock(&mutexSensors);

	while (1)
	{
		dbg("1,S HR>;")
  		digitalWriteFast(Pin_LED_TOP_R, robot.status.tictac);//LEDTOP_R_ON

		chMtxLock(&mutexSensors);
		robot.readSensorsHR();	//IR proxy, Gyro, GPS
		robot.status.tictac = !robot.status.tictac;
		chMtxUnlock(&mutexSensors);
		//LEDTOP_R_OFF


		//yeld in base alla modalità operativa
		if (robot.status.operatingMode == MODE_AUTONOMOUS) {
			chThdSleepMilliseconds(2000);// Sleep for n milliseconds.
		}
		else {
			chThdSleepMilliseconds(3000);
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
	chMtxUnlock(&mutexSensors);

	while (1)
	{
		dbg("1,S LR>;")
  		digitalWriteFast(Pin_LED_TOP_R, robot.status.tictac);//LEDTOP_R_ON

		chMtxLock(&mutexSensors);
		robot.readSensorsLR();	//IR proxy, Gyro, GPS
		robot.status.tictac = !robot.status.tictac;
		chMtxUnlock(&mutexSensors);
		//LEDTOP_R_OFF


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
static THD_WORKING_AREA(waMMIcommands, 200);
static THD_FUNCTION(thdMMIcommands, arg) {
	//static msg_t MMIcommands(void *arg)


	// Setup CommandMessenger -----------------------------------------------------
	cmdMMI.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdMMI);// Attach my application's user-defined callback methods

	while (true)
	{
		dbg("1;MMI>; ");

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
// thread   PCcommands - ricezione comandi da PC (per Debug)
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waPCcommands, 100);
static THD_FUNCTION(thdPCcommands, arg) {
	// Setup CommandMessenger -----------------------------------------------------
	cmdPC.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdPC);// Attach my application's user-defined callback methods
	dbg("1,thdPCcommands STARTED;");

	while (true)
	{
		dbg("1;PC>; ");


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


		LEDTOP_G_ON
		//osalSysDisable(); //disabilita Interupt
		// wait to enter print region
		//chMtxLock(&mutexSerialMMI);

		OnCmdGetSensorsHRate(&cmdMMI);
//		OnCmdGetSensorsLRate(&cmdMMI);
		//chMtxUnlock(&mutexSerialMMI);

		//osalSysEnable();//abilita Interupt

		//chMtxLock(&mutexSerialPC);
		OnCmdGetSensorsHRate(&cmdPC);
//		OnCmdGetSensorsLRate(&cmdPC);
		//chMtxUnlock(&mutexSerialPC);
		LEDTOP_G_OFF

		//yeld in base alla modalità operativa
			if (robot.status.operatingMode == MODE_AUTONOMOUS) {
				#ifdef DEBUG_ON
				chThdSleepMilliseconds(1000);// Sleep for n milliseconds.
				#else
				chThdSleepMilliseconds(3000);// Sleep for n milliseconds.
				#endif
			}
			else { //SLEEP TIME IN MODALITA' SLAVE O JOISTICK
				#ifdef DEBUG_ON
				chThdSleepMilliseconds(2000);// Sleep for n milliseconds.
				#else
				chThdSleepMilliseconds(5000);// Sleep for n milliseconds.
				#endif
			}
	}
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread   SendStatusLR - invia lo stato sui vari canali
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waSendStatusLR, 100);
static THD_FUNCTION(thdSendStatusLR, arg) {

	//chMtxLock(&mutexSerialMMI); //si blocca finchè la seriale è in uso da un altro thread
	//OnCmdGetSensorsLRate(&cmdMMI);
	//chMtxUnlock(&mutexSerialMMI);

	//chMtxLock(&mutexSerialPC); //si blocca finchè la seriale è in uso da un altro thread
	//OnCmdGetSensorsLRate(&cmdPC);
	//chMtxUnlock(&mutexSerialPC);

	while (true)
	{


//		chMtxLock(&mutexSerialMMI); //si blocca finchè la seriale è in uso da un altro thread
		OnCmdGetSensorsLRate(&cmdMMI);
//		chMtxUnlock(&mutexSerialMMI);
 
//		chMtxLock(&mutexSerialPC); //si blocca finchè la seriale è in uso da un altro thread
		OnCmdGetSensorsLRate(&cmdPC);
//		chMtxUnlock(&mutexSerialPC);

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
//////////////////////////////////////////////////////////////////////////////////
// thread   SCAN 
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
static THD_WORKING_AREA(waScan, 200);
static THD_FUNCTION(thdScan, arg) {

	while (!robot.status.operatingMode == MODE_AUTONOMOUS)
	{
		// Lock movements
		chMtxLock(&mutexMotion);

		OnCmdSonarScan(&cmdMMI);

		// Unlock data access.
		chMtxUnlock(&mutexMotion);

		chThdSleepMilliseconds(5000);

	}
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//  THREAD  N O N  A T T I V I  									/////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region THREAD NON ATTIVI
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





#pragma endregion


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
	// fill pool with msgObjArray array
	//for (size_t i = 0; i < MBOX_COUNT; i++) {
	//	chPoolFree(&memPool, &msgObjArray[i]);
	//}

	chThdCreateStatic(waSendStatusHR, sizeof(waSendStatusHR), NORMALPRIO +5, thdSendStatusHR, NULL);
	chThdCreateStatic(waSendStatusLR, sizeof(waSendStatusLR), NORMALPRIO +5, thdSendStatusLR, NULL);
	chThdCreateStatic(waPCcommands, sizeof(waPCcommands), NORMALPRIO + 3, thdPCcommands, NULL);
	chThdCreateStatic(waMMIcommands, sizeof(waMMIcommands), NORMALPRIO + 3, thdMMIcommands, NULL);
	chThdCreateStatic(waReadSensorsHR, sizeof(waReadSensorsHR), NORMALPRIO + 2, thdReadSensorsHR, NULL);
	chThdCreateStatic(waReadSensorsLR, sizeof(waReadSensorsLR), NORMALPRIO + 2, thdReadSensorsLR, NULL);
	//chThdCreateStatic(waScan, sizeof(waScan), NORMALPRIO + 2, thdScan, NULL);
	//chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 2, FlashLed, NULL);
	//	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 1, ThreadMonitor, NULL);
	//	chThdCreateStatic(waThreadROS, sizeof(waThreadROS), NORMALPRIO + 3, ThreadROS, NULL);//-Esplora
	//	chThdCreateStatic(watestRotaryEncoder, sizeof(watestRotaryEncoder), NORMALPRIO + 2, testRotaryEncoder, NULL);
	dbg("1,Thread chibios avviati ..;");
	while (1) {}

}

void setup()
{
	LEDTOP_R_ON	// Indica inizio SETUP Phase

	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);
	SERIAL_MSG.println(F("1,RUNNING ROBOTCORE v0.1;"));

	Wire.begin(); // default timeout 1000ms

	SERIAL_MSG.println(F("1, ROBOTCORE BEGIN...;"));
	// inizializzazione ROBOT ---------------------
#pragma region I2C Scanner
#if 0


	byte error, address;
	int nDevices;

	Serial.println("Scanning...");

	nDevices = 0;
	for (address = 1; address < 127; address++)
	{
		// The i2c_scanner uses the return value of
		// the Write.endTransmisstion to see if
		// a device did acknowledge to the address.
		error = I2c.write(address, 0);


		if (error == 0)
		{
			Serial.print("I2C device found at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.print(address, HEX);
			Serial.println("  !");

			nDevices++;
		}
		else if (error == 4)
		{
			Serial.print("Unknow error at address 0x");
			if (address < 16)
				Serial.print("0");
			Serial.println(address, HEX);
		}
	}
	if (nDevices == 0)
		Serial.println("No I2C devices found\n");
	else
		Serial.println("done\n");

	delay(5000);           // wait 5 seconds for next scan

#endif // 0
#pragma endregion

	dbg2("LDS addr:", (int)&LDS)
	dbg2("Sonar  addr:", (int)&Sonar)
	dbg2("Servosonar addr:", (int)&servoSonar)
	dbg2("compass addr:", (int)&compass)
 
	robot.beginRobot(&Gps , &servoSonar, &Sonar, &LDS, &compass);

	//#if OPT_LDS
	//	LDS.init();
	//#endif // 0


	//#if OPT_COMPASS
	//	compass.begin();
	//#endif // 0
	
	#pragma region TEST OPZIONALI PRIMA DELL'AVVIO DELL'OS
		// L'output in questi test viene inviato alla seriale

		#define TEST_SPEECH 0
		#if TEST_SPEECH
			SERIAL_MSG.println(F("1,test Speak;"));
			SERIAL_MMI.println(F("28,CIAO CIAO;"));

		#endif // TEST_SPEECH

 


		#define TEST_LDS 0
		#if TEST_LDS
			SERIAL_MSG.print(F("1,LaserDistance sensor init...;"));
			bool initDone = false;
			while (!initDone)
			{
				if (distanceSensor.init())
				{
					SERIAL_MSG.println(F("1,..OK;"));
					initDone = true;
				}
				else
				{
					SERIAL_MSG.println(F("1,..FAIL;"));
					delay(500);
				}
			}
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
		#endif


		// MOTOR ENCODER
		//attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), loadEncoderPositionOnChange, CHANGE);
		//attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), loadEncoderPositionOnChange, CHANGE);

		#define TEST_IRPROXY 0
		#if TEST_IRPROXY	//test ixproxy
				SERIAL_MSG.print(F("1,testing ixproxy...;"));
			while (true)
			{
				robot.readSensors();
				if (robot.status.sensors.irproxy.fw != robot.statusOld.sensors.irproxy.fw)
				{
					SERIAL_MSG.println(F("1,irproxy.fw changed;"));
					TOGGLEPIN(Pin_LED_TOP_B);

				}
				else if (robot.status.sensors.irproxy.fwHL != robot.statusOld.sensors.irproxy.fwHL)
				{
					SERIAL_MSG.println(F("1,irproxy.fwHl changed;"));
					TOGGLEPIN(Pin_LED_TOP_G);

				}
			}

		#endif // 0//test ixproxy

		#define TEST_OBST 0
		#if TEST_OBST		// test di obstacleFree()
			SERIAL_MSG.print(F("1,testing robot.obstacleFree()...;"));
			bool val = false;
			bool oldVal = false;
			robot.status.cmd.commandDir = GOF;
			while (true)
			{
				val = robot.obstacleFree();
				if (oldVal != val)
				{
					TOGGLEPIN(Pin_LED_TOP_B);
					SERIAL_MSG.println("1,robot.obstacleFree() changed;");
					oldVal = val;
				}
			}
		#endif // 1

		#define TEST_MOTION 0
		#if TEST_MOTION		// test di obstacleFree()
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

		#define TEST_COMPASS 0
		#if TEST_COMPASS		//TEST_COMPASS  
			/* Initialise the sensor */
			if (!compass.begin())
			{
				/* There was a problem detecting the HMC5883 ... check your connections */
				SERIAL_MSG.println(F("Ooops, no HMC5883 detected ... Check your wiring!"));
				while (1);
			}
			else
			{
				SERIAL_MSG.println(F("OK, compass HMC5883 detected"));
				int x = 0;
					x = robot.readCompassDeg(&compass);
					dbg2("COMPASS Degrees = ", x);
 
			}

		#endif // COMPASS
	#pragma endregion

			
	//SERIAL_MSG.println(F("1,running IBIT...;"));
	//robot.runIBIT(300);

	WEBCAM_ON

	LEDTOP_R_OFF



		//while (!robot.isPowerOn())
		//{
		//	SERIAL_MSG.println("1,!!!WARNING POWER IS OFF!!!.");
		//	lampeggiaLed(Pin_LED_TOP_R, 5, 500);
		//	delay(500);
		//}
		SERIAL_MSG.println(F("1,starting chibios tasks..;"));
	//lampeggiaLed(Pin_LED_TOP_G, 5, 500);



	chBegin(chSetup);
}


void loop() {
	// not used
}

#pragma endregion


