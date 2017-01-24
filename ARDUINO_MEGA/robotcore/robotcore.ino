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

#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
// ///																						///
// ///       LIBRERIE 																		///
// ///		Aggiungere ciascun percorso nelle proprietà del progetto in Visual Studio 		///
// ///		Configuration Properties >C++ > Path											///
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region LIBRERIE

#include <digitalWriteFast.h>
#include <ChibiOS_AVR.h>
#include <util/atomic.h>
#include <TinyGPSplus/TinyGPS++.h>
//#include <StackArray.h>



#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
#if OPT_COMPASS
	#include <Wire\Wire.h>
	#include <compass\compass.h>
	MyCompass_c compass;
	//#include <Adafruit_Sensor\Adafruit_Sensor.h> //richiesto dalla liberia compass Adafruit_HMC5883_U
 	//#include <HMC5883L\HMC5883L.h>
	//HMC5883L compass;
#endif // COMPASS

#if OPT_SERVOSONAR
	// va messo prima dell'istanza del robot
	#include <Newping\NewPing.h>
	#include <Servo\src\Servo.h>
	#include <Servo.h> //deve restare qui altrimenti il linker s'incazza (??)

	#include <PWM\PWM.h>
	Servo servoSonar;
	NewPing sonar(Pin_SonarTrig, Pin_SonarEcho);
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

#pragma endregion

#include <robot.h>
struct robot_c robot;	//was  struct robot_c robot;


// ////////////////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CmdMessenger object to the default Serial port
// ////////////////////////////////////////////////////////////////////////////////////////////
#include <CmdMessenger2/CmdMessenger2.h>
static CmdMessenger2 cmdMMI = CmdMessenger2(SERIAL_MMI);
static CmdMessenger2 cmdPC = CmdMessenger2(SERIAL_MSG);
#include <MyRobotLibs\RobotInterfaceCommands2.h>
// usare le macro  MSG per inviare messaggi sia su Serial_PC, sia Serial_MMI
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

//Dichiarazione di funzione che punta all’indirizzo zero
void(*softReset)(void) = 0;


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

void countDown(int seconds) {
	MSG3("CountDown in ", seconds," sec...");
	for (size_t i = seconds; i > 0; i--)
	{
		MSG2("  -",i);
		delay(1000);
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
	chMtxLock(&mutexSensors);
	// Allo startup leggo tutti isensori
	robot.readAllSensors();	//IR proxy, Gyro, GPS
	chMtxUnlock(&mutexSensors);

	while (1)
	{
  		digitalWriteFast(Pin_LED_TOP_R, robot.status.tictac);//LEDTOP_R_ON

		chMtxLock(&mutexSensors);
		MSG("S HR>")
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
		dbg("1,MMI>; ");

		LEDTOP_B_ON

		osalSysDisable(); //disabilita Interupt
		cmdMMI.feedinSerialData(); // Comando da interfaccia MMI ?, Se sì lo esegue
		osalSysEnable();//abilita Interupt
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
// invia un  messaggi con la descrizione dell'evento e lo resetta 
void msgEvent() {
	if (robot.status.pendingEvents.bumperF) { MSG("bumperF  EVENT");robot.status.pendingEvents.bumperF = false; }
	if (robot.status.pendingEvents.bumperL) { MSG("bumperL  EVENT");robot.status.pendingEvents.bumperL = false; }
	if (robot.status.pendingEvents.bumperR) { MSG("bumperR  EVENT");robot.status.pendingEvents.bumperR = false; }
	if (robot.status.pendingEvents.EncL) { MSG("EncL  EVENT");robot.status.pendingEvents.EncL = false; }
	if (robot.status.pendingEvents.EncR) { MSG("EncR  EVENT");robot.status.pendingEvents.EncR = false; }
	if (robot.status.pendingEvents.irproxyB) { MSG("irproxyB  EVENT");robot.status.pendingEvents.irproxyB = false; }
	if (robot.status.pendingEvents.irproxyF) { MSG("irproxyF  EVENT");robot.status.pendingEvents.irproxyF = false; }
	if (robot.status.pendingEvents.irproxyFH) { MSG("irproxyFH  EVENT");robot.status.pendingEvents.irproxyFH = false; }
	if (robot.status.pendingEvents.irproxyL) { MSG("irproxyL  EVENT");robot.status.pendingEvents.irproxyL = false; }
	if (robot.status.pendingEvents.irproxyR) { MSG("irproxyR  EVENT");robot.status.pendingEvents.irproxyR = false; }

	if (robot.status.pendingEvents.pirDome) { MSG("pirDome  EVENT");robot.status.pendingEvents.pirDome = false; }
	//if (robot.status.pendingEvents.analog[0]) { MSG("POT EVENT"); robot.status.pendingEvents.analog[0]= false;	}
	if (robot.status.pendingEvents.batCharge) { MSG("BATTERY EVENT");robot.status.pendingEvents.batCharge= false;	 }
	if (robot.status.pendingEvents.light) { MSG("light  EVENT"); robot.status.pendingEvents.light = false;}
	//if (robot.status.pendingEvents.analog[2]) { MSG("LIGHT EVENT");robot.status.pendingEvents.analog[2]= false;	 }
	
	
	
	
	
	
	
	
	
	
	
	
	

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
		if ((robot.status.operatingMode == MODE_AUTONOMOUS) &&

		  (robot.status.pendingEvents.analog
			||robot.status.pendingEvents.bumperF
			|| robot.status.pendingEvents.bumperL
			|| robot.status.pendingEvents.bumperR
			|| robot.status.pendingEvents.EncL
			|| robot.status.pendingEvents.EncR
			|| robot.status.pendingEvents.irproxyB
			|| robot.status.pendingEvents.irproxyF
			|| robot.status.pendingEvents.irproxyFH
			|| robot.status.pendingEvents.irproxyL
			|| robot.status.pendingEvents.irproxyR
			|| robot.status.pendingEvents.light
			|| robot.status.pendingEvents.pir2
			|| robot.status.pendingEvents.pirDome
			))
		{
			LEDTOP_G_ON;
			//osalSysDisable(); //disabilita Interupt
			// wait to enter print region
			//chMtxLock(&mutexSerialMMI);
			msgEvent();

			OnCmdGetSensorsHRate(&cmdMMI);
	//		OnCmdGetSensorsLRate(&cmdMMI);
			//chMtxUnlock(&mutexSerialMMI);

			//osalSysEnable();//abilita Interupt

			//chMtxLock(&mutexSerialPC);
			OnCmdGetSensorsHRate(&cmdPC);
	//		OnCmdGetSensorsLRate(&cmdPC);
			//chMtxUnlock(&mutexSerialPC);



			LEDTOP_G_OFF
		}


		//yeld in base alla modalità operativa
			if (robot.status.operatingMode == MODE_AUTONOMOUS) {
				#ifdef DEBUG_ON
				chThdSleepMilliseconds(1000);// Sleep for n milliseconds.
				#else
				chThdSleepMilliseconds(1000);// Sleep for n milliseconds.
				#endif
			}
			else { //SLEEP TIME IN MODALITA' SLAVE O JOISTICK
				#ifdef DEBUG_ON
				chThdSleepMilliseconds(2000);// Sleep for n milliseconds.
				#else
				chThdSleepMilliseconds(11000);// Sleep for n milliseconds.
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
		if ((robot.status.operatingMode == MODE_AUTONOMOUS) &&
		 ( robot.status.pendingEvents.light
			|| robot.status.pendingEvents.batCharge
			|| robot.status.pendingEvents.gps
			))
		{
			LEDTOP_G_ON
				//osalSysDisable(); //disabilita Interupt
				// wait to enter print region
				//chMtxLock(&mutexSerialMMI);
				msgEvent();


			robot.status.pendingEvents.gps = false;
			robot.status.pendingEvents.batCharge = false;
			robot.status.pendingEvents.light = false;

			//		chMtxLock(&mutexSerialMMI); //si blocca finchè la seriale è in uso da un altro thread
			OnCmdGetSensorsLRate(&cmdMMI);
			//		chMtxUnlock(&mutexSerialMMI);

			//		chMtxLock(&mutexSerialPC); //si blocca finchè la seriale è in uso da un altro thread
			OnCmdGetSensorsLRate(&cmdPC);
			//		chMtxUnlock(&mutexSerialPC);

		}




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
		//chMtxLock(&mutexMotion);

		OnCmdSonarScan(&cmdMMI);

		// Unlock data access.
		//chMtxUnlock(&mutexMotion);

		chThdSleepMilliseconds(1000);

	}
}
//////////////////////////////////////////////////////////////////////////////////
//  THREAD  B R A I N      ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region  Processo	B R A I N    
static THD_WORKING_AREA(waBrain, 100);
static THD_FUNCTION(thdBrain, arg)
{

	int sleepTime = 500;
	const int FWDIST = 10; //distanza avanzamento
	int alfa = 0;
	int cmDone = 0; // percorso eseguito a valle di un comando moveCm o RotateDeg
	int stuckCount = 0;
				//1)inizializzo i sensori e le variabili
				int pingCm = 0;
				int ldsmm = 0;
				int estimatedDistCm = 0;
				MSG("COMPASS init");
				Wire.begin();
				compass.begin(2);
				MSG("LDS init...");
				LDS.init();
				bool initDone = false;
				while (!initDone)
				{
					if (LDS.init())
					{
						MSG("  ..OK;");
						initDone = true;
					}
					else
					{
						MSG("   ..FAIL;");
						delay(500);
					}

				}
				delay(2000);
				LDS.setTimeout(500);
				robot.status.cmd.commandDir = commandDir_e::GOR;

	while (1)
	{
		switch (robot.status.operatingMode)
		{
		case	operatingMode_e::MODE_AUTONOMOUS:

			sleepTime = 2000;



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
				//OnCmdSonarSendData(&cmdMMI);
				TOGGLEPIN(Pin_LED_TOP_B);

				osalSysDisable();
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


				sleepTime =1500;	// Sleep for 150 milliseconds.



			#endif // 0
					//  return 0;
			#pragma endregion
			#if 1  //test che funzionino i sensori e movimento in ambiente chibios

				LASER_ON



				//2)avvio la rotazione del robot
				osalSysDisable();
				robot.go(robot.status.cmd.commandDir, robotSpeed_e::SLOW);
				//loop
				while (true)
				{
					// leggo LDS,sonar,heading
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
					MSG3("Compass: ",robot.status.posCurrent.r,"°")
					MSG3("lds: ",ldsmm, "mm")
					MSG3("Ping: ",pingCm, "cm")

				}
				osalSysEnable();

			#endif // TEST_ALL			#pragma region ESPLORA2
			 


			break;
		case operatingMode_e::MODE_SLAVE:
			
			//Invia i dati dei sensori se cambia qualcosa
			if (robot.raiseEvents()) {
				OnCmdGetSensorsHRate(&cmdMMI);
				OnCmdGetSensorsHRate(&cmdPC);
				OnCmdGetSensorsLRate(&cmdMMI);
				OnCmdGetSensorsLRate(&cmdPC);
				robot.resetEvents();
			}
			//MSG(" okei comanda");
			sleepTime = 5000;
			break;

		case operatingMode_e::MODE_JOYSTICK:
			//MSG(" okei comanda col gioistic");
			sleepTime = 5000;
			break;

		default:
			MSG(" modo sconosciuto");
			sleepTime = 2000;
			break;

		}

 




		chThdSleepMilliseconds(sleepTime);//	chThdYield();//	

	}
}
#pragma endregion 

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

void thdSetup() {
	// fill pool with msgObjArray array
	//for (size_t i = 0; i < MBOX_COUNT; i++) {
	//	chPoolFree(&memPool, &msgObjArray[i]);
	//}

	chThdCreateStatic(waSafety, sizeof(waSafety), NORMALPRIO +10, thdSafety, NULL);
	chThdCreateStatic(waReadSensorsHR, sizeof(waReadSensorsHR), NORMALPRIO + 5, thdReadSensorsHR, NULL);
	chThdCreateStatic(waReadSensorsLR, sizeof(waReadSensorsLR), NORMALPRIO + 5, thdReadSensorsLR, NULL);
	chThdCreateStatic(waSendStatusHR, sizeof(waSendStatusHR), NORMALPRIO +5, thdSendStatusHR, NULL);
	chThdCreateStatic(waSendStatusLR, sizeof(waSendStatusLR), NORMALPRIO +5, thdSendStatusLR, NULL);
	chThdCreateStatic(waBrain, sizeof(waBrain), NORMALPRIO + 4, thdBrain, NULL);
	chThdCreateStatic(waPCcommands, sizeof(waPCcommands), NORMALPRIO + 3, thdPCcommands, NULL);
	chThdCreateStatic(waMMIcommands, sizeof(waMMIcommands), NORMALPRIO + 3, thdMMIcommands, NULL);
	//chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 2, FlashLed, NULL);
	//	chThdCreateStatic(waThreadMonitor, sizeof(waThreadMonitor), NORMALPRIO + 1, ThreadMonitor, NULL);
 	//	chThdCreateStatic(watestRotaryEncoder, sizeof(watestRotaryEncoder), NORMALPRIO + 2, testRotaryEncoder, NULL);
	MSG("Thread chibios avviati ..");
	while (1) {}

}
// ########################################################################################
// ########################################################################################
//  S E T U P
// ########################################################################################
// ########################################################################################
void setup()
{
	
	LEDTOP_R_ON	// Indica inizio SETUP Phase

	SERIAL_MSG.begin(SERIAL_MSG_BAUD_RATE);
	SERIAL_MMI.begin(SERIAL_MMI_BAUD_RATE);
	SERIAL_GPS.begin(SERIAL_GPS_BAUD_RATE);

	MSG("RUNNING ROBOTCORE v0.2");
	robot.stop();//serve a disabilitare i motori qualora fossero abilitati
	MSG3("Battery level: ",robot.readBattChargeLevel(),"%");
	MSG("ROBOTCORE BEGIN...PLEASE POWER ON");
	countDown(5);


	// inizializzazione ROBOT ---------------------
	MSG3("Battery level: ", robot.readBattChargeLevel(), "%");
	Wire.begin(); // default timeout 1000ms
	robot.beginRobot(&Gps, &servoSonar, &sonar, &LDS, &compass);
 

	// ########################################################################################
	// ########################################################################################
	//  TEST OPZIONALI
	// ########################################################################################
	// ########################################################################################
#if 0
#pragma region TEST OPZIONALI PRIMA DELL'AVVIO DELL'OS
#define TEST_SONAR 0
#if TEST_SONAR
	MSG("SONAR ONLY TEST..");
	MSG("INFINITE LOOP..");
	while (true)
	{
		TOGGLEPIN(Pin_LaserOn)
			MSG3("Ping: ", robot.sonarPing(), "cm");
		//MSG3("Ping: ",sonar.ping_cm(),"cm");
		delay(500);
	}
#endif


#define TEST_LDS 0
#if TEST_LDS
	MSG("LDS init...");
	//Wire.begin();

	bool initDone = false;
	while (!initDone)
	{
		if (LDS.init())
		{
			MSG("LDS..OK;");
			initDone = true;
		}
		else
		{
			MSG("LDS..FAIL;");
			delay(500);
		}

	}
	delay(2000);
	LDS.setTimeout(500);

#if defined LONG_RANGE
	// lower the return signal rate limit (default is 0.25 MCPS)
	LDS.setSignalRateLimit(0.1);
	// increase laser pulse periods (defaults are 14 and 10 PCLKs)
	LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
	LDS.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif
#if defined HIGH_SPEED
	// reduce timing budget to 20 ms (default is about 33 ms)
	LDS.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
	// increase timing budget to 200 ms
	LDS.setMeasurementTimingBudget(200000);
#endif

	//Eseguo le misure di prova------------------------------
	uint16_t d = 0;

	LASER_ON
		for (size_t i = 0; i < 10; i++)
		{
			d = LDS.readRangeSingleMillimeters();	//robot.getLDSDistance()
			if (!LDS.timeoutOccurred()) {
				MSG3("LDS :", d, "mm");
			}
			else
			{
				MSG("LDS  TIMEOUT ");
				Wire.begin();
				LDS.init();

			}

			delay(1000);
		}
	LASER_OFF
		//---------------------------------------------------------

		MSG("LDS  TEST END");
	countDown(4);
#endif


#pragma region test go()
#if 0
	// RUOTA con velocità crescente 1 secondo ogni step
	// Attenzione !! Se presente un altro tone() non funziona
	robot.stop();
	MSG("TEST robot.go()..")
		MSG("PLEASE SWITCH MOTORS ON")
		MSG("DISCONNECT CABLES")
		MSG("Start test in 10 secondi...")
		countDown(10);
	int ck = ROBOT_MOTOR_CLOCK_microsecondMAX;
	bool cont = true;
	while (cont)
	{
		MSG2("Current ck:", ck)
			robot.go(commandDir_e::GOR, ck);
		delay(1000);
		ck -= 200;
		if (ck < ROBOT_MOTOR_CLOCK_microsecondMIN)
		{
			//torno indietro per 3 secondi a velocità media
			robot.stop();
			robot.go(commandDir_e::GOL, robotSpeed_e::MEDIUM);
			delay(3000);
			cont = false;
		}
	}

	MSG("Fine test......")
		robot.stop();
	//while (true) {}

#endif // 0

#pragma endregion


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

#define TEST_GPS 0
#if TEST_GPS
//CHIAVARI LAT	44.326953 LONG 9.289679

#endif // TEST_SPEECH

#define TEST_SPEECH 0
#if TEST_SPEECH
	SERIAL_MSG.println(F("1,test Speak;"));
	SERIAL_MMI.println(F("28,CIAO CIAO;"));

#endif // TEST_SPEECH

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
	SERIAL_MSG.print(F("1,testing robot.isObstacleFree()...;"));
	bool val = false;
	bool oldVal = false;
	robot.status.cmd.commandDir = GOF;
	while (val)
	{
		val = robot.obstacleFree();
		if (oldVal != val)
		{
			TOGGLEPIN(Pin_LED_TOP_B);
			SERIAL_MSG.println("1,robot.isObstacleFree() changed;");
			oldVal = val;
		}
	}
#endif // 1

#define TEST_MOTION 0
#if TEST_MOTION		// test di obstacleFree()
	MSG("Testing robot.moveCm()...Attiva motori");
	countDown(8);
	robot.moveCm(-10);
	robot.moveCm(10);
	robot.rotateDeg(10);
	robot.rotateDeg(-20);
	robot.rotateDeg(10);
	MSG("END Testing robot.moveCm()");

	//SERIAL_MSG.print("testing robot.moveCm()...");
	//int cmFatti = 0;
	//#define DIST 30
	//while (!Serial.available())
	//{
	//	cmFatti = robot.moveCm(DIST);
	//	if (cmFatti <= DIST)
	//	{
	//		TOGGLEPIN(Pin_LED_TOP_R);
	//		SERIAL_MSG.print("OSTACOLO DOPO CM:"); SERIAL_MSG.println(cmFatti);
	//		robot.moveCm(-cmFatti);
	//	}
	//	else
	//	{
	//		TOGGLEPIN(Pin_LED_TOP_G);

	//	}
	//}
#endif // 1



#define TEST_COMPASS 0
#if TEST_COMPASS		//TEST_COMPASS  
	robot.readSensorsHR(); // legge sensori
	MSG("TEST_COMPASS. Accendere i motori...");
	// Initialise the sensor 

	robot.stop();
	compass.begin(2);
	int DestAngle = 0; // nord
	int currAngle = 0;
	int error = 0;

	//misuro
	currAngle = compass.getBearing();
	error = currAngle - DestAngle;

	MSG("Ora punto a nord SENZA CALIBRAZIONE...")
		robot.rotateDeg(-error);

	currAngle = compass.getBearing();
	MSG2(" Heading FINALE SENZA CALIBRAZIONE: ", currAngle);


#pragma region Calibrazione compass
#define CALIBRATE_COMPASS 1
#if CALIBRATE_COMPASS
	MSG("Ora CALIBRO...")
		countDown(5);
	compass.compass_debug = 0;
	//metto in moto 
	MSG("In moto..");
	if (robot.status.sensors.irproxy.bk)
	{
		robot.go(commandDir_e::GOR, (motCk_t)robotSpeed_e::MEDIUM);
	}
	else
	{
		robot.go(commandDir_e::GOL, (motCk_t)robotSpeed_e::MEDIUM);
	}

	//calibro
	MSG("Inizio calibrazione..");
	compass.compass_offset_calibration(3, 10000);

	robot.stop();
	MSG("Stop - fine calibrazione");


#endif // 0
#pragma endregion

	// al termine della calibrazione faccio puntare il robot ai 4 punti cardinali
	MSG("Ora punto a nord DOPO CALIBRAZIONE ...")
		countDown(15);
	currAngle = compass.getBearing();
	MSG3("Bearing di partenza: DOPO CALIBRAZIONE ", currAngle, "°");
	error = currAngle - DestAngle;
#if 0  // non va se gli IR sono accecati dalla luce
	robot.rotateDeg(-error);
#else
	while (true)
	{
		//misuro
		int currAngle = compass.getBearing();
		MSG2("current Bearing: ", currAngle);

		// calcolo l'errore----------------------
		int error = currAngle - DestAngle;
		if (abs(error) < 10)
		{
			robot.stop();
			break;
		}
		else
		{

			if (error < 180)
			{
				// sotto mezzo giro l'errore lo considero >0
			}
			else
			{
				// oltre mezzo giro 
				error = DestAngle + 360 - currAngle;
			}
			MSG2("Error  ", error);

			// regolo
			if (error > 0)
			{
				robot.go(commandDir_e::GOR, (motCk_t)robotSpeed_e::SLOW);
			}
			else if (error < 0)
			{
				robot.go(commandDir_e::GOL, (motCk_t)robotSpeed_e::SLOW);
			}



		}
	}

#endif // 0

	//robot.stop();
	MSG("**********Arrivato********");
	MSG3("Bearing di arrivo: ", compass.getBearing(), "°");

	while (true) {
		TOGGLEPIN(Pin_LED_TOP_G);
		delay(300);
	}
#endif // COMPASS


#define TEST_ALL 0
#if TEST_ALL
	MSG("TEST LDS,COMPASS,SONAR..");
	MSG("PLEASE SWITCH MOTORS ON")
		MSG("DISCONNECT CABLES")
		MSG("Start test in 10 secondi...")
		countDown(10);
	LASER_ON

		//1)inizializzo i sensori e le variabili
		int pingCm = 0;
	int ldsmm = 0;
	int estimatedDistCm = 0;
	MSG("COMPASS init");
	Wire.begin();
	compass.begin(2);
	MSG("LDS init...");
	LDS.init();
	bool initDone = false;
	while (!initDone)
	{
		if (LDS.init())
		{
			MSG("  ..OK;");
			initDone = true;
		}
		else
		{
			MSG("   ..FAIL;");
			delay(500);
		}

	}
	delay(2000);
	LDS.setTimeout(500);


	//2)avvio la rotazione del robot

	robot.go(commandDir_e::GOR, robotSpeed_e::MIN);
	//loop
	while (true)
	{
		// leggo LDS,sonar,heading
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
		SERIAL_PC.print(robot.status.posCurrent.r); SERIAL_PC.print(",");
		SERIAL_PC.print(ldsmm); SERIAL_PC.print(",");
		SERIAL_PC.print(pingCm); SERIAL_PC.print(",");
		SERIAL_PC.println(estimatedDistCm);
	}







#endif // TEST_ALL


#pragma endregion //regione dei test

#endif // 0

			
	//SERIAL_MSG.println(F("1,running IBIT...;"));
	//robot.runIBIT(300);





		//while (!robot.isPowerOn())
		//{
		//	SERIAL_MSG.println("1,!!!WARNING POWER IS OFF!!!.");
		//	lampeggiaLed(Pin_LED_TOP_R, 5, 500);
		//	delay(500);
		//}




#if 0

	// posiziona il servo al centro
			MSG("Posiziono Servo al centro...")
				//servoSonar.attach(Pin_ServoSonarPWM);
				robot.ServoAtPos(10); delay(1000);
			robot.ServoAtPos(90); delay(1000);
			robot.ServoAtPos(160); delay(1000);


#endif // 0

	MSG("pochi secondi all'avvio dei task...")

	lampeggiaLed(Pin_LED_TOP_G, 5, 10);
	WEBCAM_ON
	countDown(5);

	LEDTOP_R_OFF
	chBegin(thdSetup);
}


void loop() {
	// not used
}

#pragma endregion


