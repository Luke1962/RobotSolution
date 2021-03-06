// ////////////////////////////////////////////////////////////////////////////////

// I N T E R F A C C I A   M M I                //////////////////////////////////

// ////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////

// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA 
	

	//////////////////////////////////////////////////////////////////////////
	// C O N F I G U R A Z I O N E  H A R D W A R E							//
	//////////////////////////////////////////////////////////////////////////
	#pragma region Includes dei driver HW
		#include <Arduino.h>
		#include <hwMMI_config.h>
		#include <SPI.h>
//		#include <XPT2046-2\XPT2046-2.h>	//touch controller, deve stare prima di UTFT
		#include <UTFT.h>						//display TFT
		#include <buzzer\buzzer.h>
		#include <ClickEncoder/ClickEncoder.h>

		#include "hardwareIncludes.h"
//		#include <SoftwareSerial.h>

	#pragma endregion

	/// ///////////////////////////////////////////////////////////////////////
	// OPZIONI D I    D E B U G												//
	/// ///////////////////////////////////////////////////////////////////////
	#define DEBUG_ON
	#include <dbg.h>

	//////////////////////////////////////////////////////////////////////////
	//																		//
	//						 O P Z I O N I   R O B O T						//
	//																		//
	//////////////////////////////////////////////////////////////////////////

	#define OPT_SERVOSONAR 1	//INCLUDI FUNZIONALITA' SERVO-SONAR
	#define OPT_ENCODERS  0	//INCLUDI ENCODERS

	#define INPUTCHARARRAYSIZE 40 //dimensione buffer seriale


	//////////////////////////////////////////////////////////////////////////////////
	// LIBRERIE                                    ///////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	#pragma region Librerie

		#pragma region Librerie senza dipendenze
			//#include <TimerOne/TimerOne.h>     // serve a ArduinoMenu, ISR on ClickEncoder

		#pragma endregion
		#pragma region Librerie ArduinoMenu
			#include <Arduino.h>
			#include <ClickEncoder/ClickEncoder.h>
			#include <ClickEncoderStream.h> // Quad encoder
			#include <TimerOne.h>     // ISR on ClickEncoder

			#include <menu.h>
			#include <macros.h>
			#include <menuUTFT.h>
			#include <chainStream.h>// concatenate multiple input streams (this allows adding a button to the encoder)
			#include <menuFields.h>
		#pragma endregion

		#include <ChibiOS_AVR/ChibiOS_AVR.h>
		#include <TinyGPSplus\TinyGPS++.h>	//se manca non compila a causa del robotModel.cpp nella stessa cartella di robot\Commands_Enum.h

		#include <string.h> 
		#include "stringlib.h" //Funzioni di manipolazione stringhe di tipo CHARARRAY
		#include <digitalWriteFast/digitalWriteFast.h>
		//#include <CmdMessenger/CmdMessenger.h>
		//#include <robot\Commands_Enum.h>
		#include <MyRobotLibs\robotModel.h>
		#include <SoftwareSerial.h>		//C:\Program Files %28x86%29\Arduino\hardware\arduino\avr\libraries\SoftwareSerial
		#include <MyRobotLibs\SpeakSerialInterface.h>
		//#include <MyRobotLibs\CircularBuffer.h>


	#pragma endregion
#pragma endregion


/// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
/// ////////////////////////////////////////////////////////////////////////////////////////////
//
///
//  C R E A Z I O N E  O G G E T T I  G L O B A L I
///
//
/// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
/// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
	//Dichiarazione di funzione che punta all’indirizzo zero
	void(*softReset)(void) = 0;

	#pragma region TFT
		//UTFT     tft(ILI9327_8,30,31,32,33 );		//was 38, 39, 40, 41
		UTFT     tft(ILI9327_8, Pin_TFT_RS, Pin_TFT_WR, Pin_TFT_CS, Pin_TFT_RST);		//was 38, 39, 40, 41
		#ifndef NO_TOUCHSCREEN
			TS_Point p;

		#endif // !NO_TOUCHSCREEN

		MUTEX_DECL(mutexTFT);// accesso al display TFT


	#pragma endregion

	#pragma region robotModel
		//#include <servo/src/Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
	// ////////////////////////////////////////////////////////////////////////////////////////////
	// MODELLO ROBOT
		struct robotVirtualModel_c robotModel;
	#pragma endregion

	#pragma region OGGETTI COMMAND MESSENGER:  cmdRobotCore e cmdBT
		#include <CmdMessenger2/CmdMessenger2.h>
		static CmdMessenger2 cmdRobotCore = CmdMessenger2(SERIAL_ROBOT);
		static CmdMessenger2 cmdBT = CmdMessenger2(SERIAL_BT);
		//void SPEAK(char inStr[]);
		
		#include "RobotInterfaceCommandsMMI.h"
	#pragma endregion


		#include "TFT_HAL\TFT_HAL.h"

	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  MENU object  & ROTARY ENCODER SWITCH
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#pragma region OGGGETTI MENU CON ROTARY ENCODER
		menuUTFT myMenu(tft);
 
		ClickEncoder qEnc(Pin_ROT_ENCODER_A, Pin_ROT_ENCODER_B, Pin_ROT_ENCODER_SWITCH, 2, LOW);
		ClickEncoderStream enc(qEnc, 1);// simple quad encoder fake Stream
		Stream* menuInputs[] = { &enc,&Serial };
		chainStream<2> in(menuInputs);

		#pragma region Funzioni del menu

		/////////////////////////////////////////////////////////////////////////
		// MENU FUNCTIONS
		// this functions will be wired to menu options
		// meaning they will be called on option click/select
		// or on field value change/update
		bool mfSayIt(prompt& p, menuOut& o, Stream &c);
		bool mfSetModeAutonomous();
		bool mfSetModeSlave();
		bool mfTestSpeech();
		bool mfTestServoSonar();
		bool mfMoveFW();
		bool mfMoveBK();
		bool mfRotateR();
		bool mfRotateL();
		bool mfGetSensorsHR();
		bool mfGetSensorsLR();
		bool mfRebootCore();
		bool mfRebootMMI();
		bool mfLDSScanBatch();
		bool mfSetRele1On();
		bool mfSetRele1Off();
		bool mfVoid();
 
		int aValue = 50;
		float fValue = 101.1;
		/////////////////////////////////////////////////////////////////////////
		// MENU DEFINITION
		// here we define the menu structure and wire actions functions to it

		// RICORDATI CHE NEL SETUP DEVI IMPOSTARE LA POSIZIONE DEI MENU
		#define MENUPOSITION_X 5
		#define MENUPOSITION_Y 380
		MENU(subMenuTest, "Test..."
			,OP("LDS Scan", mfLDSScanBatch)
			,OP("Speech Test", mfTestSpeech)
			,OP("GetSensors HR", mfGetSensorsHR)
			,OP("GetSensors LR", mfGetSensorsLR)
			,OP("Webcam ON", mfSetRele1On)
			,OP("Webcam OFF", mfSetRele1Off)
			,OP("Servo Sonar", mfTestServoSonar)
			,FIELD(aValue, "Value", "%", 0, 100, 1, 0)
			,OP("Option C", mfSayIt)
		);

		MENU(subMenuMode, "Set Mode.."
			,OP("Autonomous", mfSetModeAutonomous)
			,OP("slave", mfSetModeSlave)
		);
		MENU(subMenuMove, "Commands.."
			,OP("Forward", mfMoveFW)
			,OP("Back", mfMoveBK)
			,OP("Right", mfRotateR)
			,OP("Left", mfRotateL)
			,FIELD(robotModel.cmdSettingDefaultMoveCm, "set FW/BK x", " cm", 0, 500, 10, 1)
			,FIELD(robotModel.cmdSettingDefaultRotateDeg, "set L/R x", " deg", 0, 360, 30, 1)
			,FIELD(robotModel.status.cmd.clock, "CK speed", " uS", 2500, 4000, 500, 100)
			,OP("GetSens HR", mfGetSensorsHR)

		);
		MENU(subMenuModifyStatus, "Modify.."
			,FIELD(robotModel.status.posCurrent.x, "Pos x", " cm", 0, 1000, 10, 1)
			,FIELD(robotModel.status.posCurrent.y, "Pos y", " cm", 0, 1000, 10, 1)
			,FIELD(robotModel.status.posCurrent.r, "Pos r", " deg", 0, 359, 15, 1)
			,FIELD(robotModel.status.sensors.pirDome, "PIR", " 0/1", 0, 1, 1, 0)
			,FIELD(robotModel.status.sensors.analog[0], "A0", " n.", 0, 1000, 20, 1)
			,FIELD(robotModel.status.sensors.analog[1], "A1", " n.", 1, 1000, 10, 1)
			,FIELD(robotModel.status.sensors.batCharge, "Batt", "%", 0, 100, 1, 0)
			,OP("Reboot CORE", mfRebootCore)
			,OP("Reboot MMI", mfRebootMMI)
				
		);
		MENU(subMenuHelp, "Help.."
			,OP( "Led Rosso: Core ", mfVoid)
			,OP( "Led Lime:   Menu ", mfVoid)
			,OP( "Led Verde: Brain  ", mfVoid)
			,OP( "Led Blu:  BTH ", mfVoid)
			,OP( "Led Wite:   TFT ", mfVoid)
		);

		MENU(mainMenu, "Main Menu"

			,SUBMENU(subMenuMode)
			,SUBMENU(subMenuTest)
			,SUBMENU(subMenuMove)
			,SUBMENU(subMenuModifyStatus)
			, SUBMENU(subMenuHelp)
		);
			
		// messe dopo per poter chiamare menu.redraw();
		bool mfSayIt(prompt& p, menuOut& o, Stream &c) {
			tft.setBackColor(0, 0, 0);
			tft.clrScr();
			tft.setColor(0, 255, 0);
			tft.print("Activated option:", 0, 0);
			tft.print(p.text, 0, 16);
			o.drawn = 0;
			delay(1000);
			tft.clrScr();
			tftPrintCaptions();
			return true;
		}
		bool mfSetModeAutonomous() {
			robotModel.cmdSetMode(operatingMode_e::MODE_AUTONOMOUS);
			playSingleNote(NOTE_A7, 80);
			myMenu.redraw();
			return true;
		}
		bool mfSetModeSlave() {
			robotModel.cmdSetMode(operatingMode_e::MODE_SLAVE);
			playSingleNote(NOTE_D7, 80);
			myMenu.redraw();
			return true;
		}
		bool mfMoveFW() {
			robotModel.cmdMoveCm( robotModel.cmdSettingDefaultMoveCm );
			playSingleNote(NOTE_D7, 80);
			myMenu.redraw();
			return true;
		}
		bool mfMoveBK() {
			robotModel.cmdMoveCm( -robotModel.cmdSettingDefaultMoveCm);
			playSingleNote(NOTE_D7, 80);
			myMenu.redraw();
			return true;
		}
		bool mfRotateR() {
			robotModel.cmdRotateDeg(robotModel.cmdSettingDefaultRotateDeg);
			playSingleNote(NOTE_D7, 80);
			myMenu.redraw();
			return true;
		}
		bool mfRotateL() {
			robotModel.cmdRotateDeg( -robotModel.cmdSettingDefaultRotateDeg);
			playSingleNote(NOTE_D7, 80);
			myMenu.redraw();
			return true;
		}
		bool mfTestSpeech() {
			SPEAK_TEST
			myMenu.redraw();
			return true;

		}
		bool mfGetSensorsHR() {
			robotModel.cmdGetSensorsHR();
			myMenu.redraw();
			return true;
		}
		bool mfGetSensorsLR() {
			robotModel.cmdGetSensorsLR();
			myMenu.redraw();
			return true;
		}

		bool mfRebootCore() {
			robotModel.cmdRiavvia();
			myMenu.redraw();
			return true;
		}
		bool mfRebootMMI() {
 
			softReset();
			myMenu.redraw();
			return true;
		}
		bool mfLDSScanBatch() {
			robotModel.cmdLDSScanBatch();
			myMenu.redraw();
			return true;
		}
		bool mfSetRele1On() {
			robotModel.cmdSetRele(1,1);
			myMenu.redraw();
			return true;
		}
		bool mfSetRele1Off() {
			robotModel.cmdSetRele(1,0);
			myMenu.redraw();
			return true;
		}
		bool mfTestServoSonar() {
			robotModel.cmdServoPos(0);
			robotModel.cmdServoPos(180);
			robotModel.cmdServoPos(90);

			myMenu.redraw();
			return true;
		}

		bool mfGetProxy() {
			robotModel.cmdGetProxy();
			myMenu.redraw();
			return true;
		}


		bool mfVoid() {
			myMenu.redraw();
			return true;
		}

		//bool mfDisable(prompt& p, menuOut& o, Stream &c) {
		//	subMenu.disable();
		//	return true;
		//}

		void timerIsr() {
			qEnc.service();
		}

	#pragma endregion

 

#pragma endregion

 
#include <RingBuf/RingBuf.h>

/// ///////////////////////////////////////////////////////////////////////////////
// M U T E X ///////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
MUTEX_DECL(mutexSerialVoice);// accesso alla seriale

#pragma endregion


// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
//
//  INCLUDES CHE VANNO DOPO LA CREAZIONE DEGLI OGGETTI GLOBALI
//
// ////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region INCLUDES che vanno dopo la definizione degli oggetti globali

#pragma endregion

// ////////////////////////////////////////////////////////////////////////////////////////////
///
//  P A R A M E T R I  E  V A R I A B I L I  G L O B A L I
/// serialRxBuffer, MAILBOX VOICE, bluetooth_rx_buffer
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region Variabili globali

	/// ///////////////////////////////////////////////////////////////////////////////
	// VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	volatile uint32_t count = 0;
	volatile uint32_t maxDelay = 0;
	char * serialRxBuffer[SERIAL_RX_BUFFER_SIZE];

	//------------------------------------------------------------------------------
	//------------------------------------------------------------------------------
	#pragma region MAILBOX VOICE

		// mailbox size and memory pool object count
		const size_t MBOX_GLOBAL_CAPACITY = 6;
		int mailBoxGlobalFreeCounter = MBOX_GLOBAL_CAPACITY; //conta quante locazioni sono disponibili
		#define INPUTCHARARRAYSIZE 20
		// type for a memory pool object
		struct PoolObject_t {
			bool isMsgString; // mettere a false se passo un numero anzichè una stringa
			int msgInt;
			char* strSpeech;
			char str[INPUTCHARARRAYSIZE];
			int size;
		};
		// array of memory pool objects
		PoolObject_t PoolObject[MBOX_GLOBAL_CAPACITY];

		// memory pool structure
		MEMORYPOOL_DECL(memPoolVoice, MBOX_GLOBAL_CAPACITY, 0);

		// slots for mailbox messages
		msg_t letter[MBOX_GLOBAL_CAPACITY];

		// mailbox structure
		MAILBOX_DECL(mailVoice, &letter, MBOX_GLOBAL_CAPACITY);

		/// ///////////////////////////////////////////////////////////////////////////////
		//  METTE IN CODA LA STRINGA DA PRONUNCIARE  AL TASK SPEAK    /////////////////// 
		/// ///////////////////////////////////////////////////////////////////////////////
		//uso: SPEAK("ei");	// output via processo FifoToSPEAK
		void sendMsg(mailbox_t mailVoice, msg_t p) {
			// send message
			msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
			if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

		}

		void speakFifo(const char inStr[]) {
			// get object from memory pool
			mailBoxGlobalFreeCounter--;
			dbg2("Free mbox-: ",mailBoxGlobalFreeCounter);
			PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPoolVoice);
			if (!p) { Serial.println("chPoolAlloc failed from speakFifo");	while (1); }


			// form message
			p->strSpeech = (char*)inStr;		// (char*)strSpeech;
																//p->strSpeech = " imbecille spostati che devo passare";		// (char*)strSpeech;
																//strcpy(p->str, "oi");
																//p->size = 2;
																//dbg2("thdFeedFifo send: ",p->strSpeech)

			// send message
			msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
			if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

			//playSingleNote(NOTE_A5, 40);
		}
		void speakFifo(__FlashStringHelper *inStr[]) {
			// get object from memory pool
			mailBoxGlobalFreeCounter--;
			dbg2("Free mbox-: ",mailBoxGlobalFreeCounter);
			PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPoolVoice);
			if (!p) { Serial.println("chPoolAlloc failed from speakFifo");	while (1); }


			// form message
			p->strSpeech = (char*)inStr;		// (char*)strSpeech;
																//p->strSpeech = " imbecille spostati che devo passare";		// (char*)strSpeech;
																//strcpy(p->str, "oi");
																//p->size = 2;
																//dbg2("thdFeedFifo send: ",p->strSpeech)

			// send message
			msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
			if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

			//playSingleNote(NOTE_A5, 40);
		}

		void speakFifo(int i) {
			
			// get object from memory pool
			mailBoxGlobalFreeCounter--;
			//dbg2("Free mbox-: ",mailBoxGlobalFreeCounter);
			PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPoolVoice);
			if (!p) { Serial.println("chPoolAlloc failed from speakFifo");	while (1); }


			// form message
			p->isMsgString = false;
			p->msgInt = i;		// (char*)strSpeech;


			// send message-----------------------------------------------------------
			msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
			if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

			//playSingleNote(NOTE_A5, 40);
		
		}

		#define SPEAK(s) speakFifo(s)
		#define SPEAK_CIAO				speakFifo("h"); 			
		#define SPEAK_OIOI				speakFifo("O"); 			
		#define SPEAK_CIAOCHISEI		speakFifo("ciao  ki sei"); 	
		#define SPEAK_OK				speakFifo("k");				
		#define SPEAK_TEST				speakFifo("t");				
		#define SPEAK_MODE_SLAVE		speakFifo("SLEIV");			
		#define SPEAK_AUTONOMO			speakFifo("AUTONOMO");		

	#pragma endregion
	//------------------------------------------------------------------------------

	// BTH Send and receive buffers
		String bluetooth_rx_buffer = "";
		String bluetooth_tx_buffer = "";
		#define	CMDDELIMITER ";"


//	bool isNumeric(const char *ch);
	int moveEfondo(char array[], int size);
	void ConvLetter(const char cifreIn[], int size);


#pragma endregion


/// ///////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////
///                   /////////////////////////////////////////////////////////////
// PROCDURE E FUNZIONI GLOBALI    /////////////////////////////////////////////////
///                   /////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
#pragma region PROCEDURE E FUNZIONI GLOBALI: VOCE,......
	/// ///////////////////////////////////////////////////////////////////////////////
	//  PROCESS VOICE COMMAND      //////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////

	//#include <avr/wdt.h>

	//void softReset(uint8_t prescaler) {
	//	 start watchdog with the provided prescaller
	//	wdt_enable(prescaler);
	//	 wait for the prescaller time to expire
	//	 without sending the reset signal by using
	//	 the wdt_reset() method
	//	while (1) {}
	//}
	/// ///////////////////////////////////////////////////////////////////////////////
	//  PROCESS VOICE COMMAND      //////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	// Interpreta il comando ricevuto ed invia a Robot il comando corrispondente
	#pragma region PROCESS VOICE COMMAND 
	#define VOICECOMMANDMAXLEN 30	//Massima lunghezza della stringa in ingresso contenente il comando
	#define WORDCOMMANDMAXLEN 10			// massima lunghezza di ciascun comando
	#define ATTENTIONCOMMAND ROBOT	//indice del comando vocale che attiva il robot
	#define COMMANDSCOUNT 10			// numero di comandi riconosciuti
	#define ATTENTIONCOMMANDSCOUNT 2	//numero di comandi di attivazione

		char AttentionWords[ATTENTIONCOMMANDSCOUNT][WORDCOMMANDMAXLEN] = { "ROBOT\0", "ARDUINO\0", }; //elenco comandi accettati come attivazione
		char Vocabolary[COMMANDSCOUNT][WORDCOMMANDMAXLEN] = { "NIL\0",  "AVANTI\0", "INDIETRO\0", "DESTRA\0","SINISTRA\0", "FERMA\0" , "SONO\0" , "LUCA\0", "ANGELICA\0", "VINICIA\0", };
		enum e_VoiceCommands { NIL, AVANTI, INDIETRO, DESTRA, SINISTRA, FERMA, SONO, LUCA, ANGELICA, VINICIA };
		static char voiceCommandCharArray[VOICECOMMANDMAXLEN] = "\0"; //array contenente la stringa vocale da riconoscere
		enum e_cmdProcessingStatus { cmdstatus_IDLE, cmdStatus_WAITCMD, cmdStatus_WAITPARAM };
		//estrae dalla stringa la prima parola e se coincide con un comando ritorna l'indice del comando
		// ritorna 0 se non riconosce alcun comando
		int  GetCommandIndex() {

			dbg2("@GetCommandIndex elabora: ", voiceCommandCharArray)


				char *cmdCandidate;
			int cmdId = 0;	// id del comando da restituire
			int i = 1;
			bool found = false;

			// estraggo il primo token-------------------------------------------
			char *rest;	//= voiceCommandCharArray
			const char sep[] = " ";
			cmdCandidate = strtok_r(voiceCommandCharArray, sep, &rest);		//http://www.mkssoftware.com/docs/man3/strtok_r.3.asp#MULTITHREAD_SAFETY_LEVEL
			dbg2("@ cmdCandidate = ", cmdCandidate)



				//	strstr()		//http://www.cplusplus.com/reference/cstring/strstr/
				//	cmdCandidate = s.substring(0, firstSpacePos);

				// confronto con ogni voce del vocabolario
				while (!found && (i < COMMANDSCOUNT))
				{
					dbg2("@ confronto con Vocabolary[i]", Vocabolary[i]);
					//if (cmdCandidate.equalsIgnoreCase(Vocabolary[i]))
					if (strcmp(cmdCandidate, Vocabolary[i]) == 0)
					{
						found = true;
						cmdId = i;	//trovato >
						dbg2("@ trovato   cmdId=", cmdId);
					}
					i++;
				}

			if (cmdId > 0)
			{

				// assegno la string rimanente a voiceCommandCharArray
				strncpy(voiceCommandCharArray, rest, strlen(rest));
				for (size_t i = strlen(rest); i < strlen(voiceCommandCharArray); i++)
				{
					voiceCommandCharArray[i] = '\0';
				}

				dbg2("@  GetCommandIndex   ritorna   cmdId=", cmdId);
				dbg2("@  Stringa rimanente", voiceCommandCharArray);

			}
			return cmdId;
		}
		//////////////////////////////////////////////////////////////////////////////////
		// blAttentionWordFound		//////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		///cerca nella stringa la parola di attenzione (es "ROBOT") 
		///Se presente ritorna true
		/// rimuove tutta la parte che precede la parola di attivazione, questa inclusa
		bool  blAttentionWordFound() {
			// copio la stringa in una di servizio
			//String cha = *InputString;
			char  *cmdCandidate;
			int cmdId;	// id del comando da restituire
			int i = 0;

			// elimino eventuali spazi all'inizio e alla fine stringa
			strtrim(voiceCommandCharArray);
			dbg2("#AttentionWordFound elabora: ", voiceCommandCharArray)

				// localizzo la parola di attenzione 
				// estraggo il primo token-------------------------------------------
				char *rest;		// = voiceCommandCharArray;  //tringa rimanente
			const char sep[] = " ";
			cmdCandidate = strtok_r(voiceCommandCharArray, sep, &rest);		//http://www.mkssoftware.com/docs/man3/strtok_r.3.asp#MULTITHREAD_SAFETY_LEVEL

			dbg2("# testing token:", cmdCandidate)
				dbg2("# string rimanente rest:", rest)

				bool found = false;
			//while ((cmdId = NIL) && (i < COMMANDSCOUNT))
			while (!found && (i < ATTENTIONCOMMANDSCOUNT))
			{
				dbg2("# confronto con:", AttentionWords[i])
					//if (cmdCandidate.equalsIgnoreCase(Vocabolary[i]))
					if (strcmp(cmdCandidate, AttentionWords[i]) == 0)		//http://www.cplusplus.com/reference/cstring/strcmp/
					{
						found = true;
						cmdId = i;	//trovato
						dbg2("# Trovato cmd #: ", cmdId)
					}
				i++;
			}



			//rimuovo il comando dalla stringa copiando rest in voiceCommandCharArray
			//s.remove(0, firstSpacePos);
			dbg2("#  Stringa restante:", rest);
			dbg2("#  len of rest:", strlen(rest));

			// assegno la string rimanente a voiceCommandCharArray
			strncpy(voiceCommandCharArray, rest, strlen(rest));
			for (size_t i = strlen(rest); i < strlen(voiceCommandCharArray); i++)
			{
				voiceCommandCharArray[i] = '\0';
			}


			dbg2("#  Stringa rimanente in uscita:", voiceCommandCharArray);



			return found;
		}


		//////////////////////////////////////////////////////////////////////////////////
		// GetCommandParamValue		//////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
		//estrae dalla stringa un valore numerico
		// ritorna -1 se non riconosce alcun comando
	#define COMMANDVALUEINVALID -999
		int  GetCommandParamValue() {
			// copio la stringa in una di servizio
			//String cha = *InputString;
			char *strCandidateValue;
			int CandidateValue = COMMANDVALUEINVALID;	// id del comando da restituire
	//		int i = 0;
			// travaso str in cha

			// estraggo il primo token-------------------------------------------
			char *rest = voiceCommandCharArray;
			const char sep[] = " ";
			strCandidateValue = strtok_r(rest, sep, &rest);		//http://www.mkssoftware.com/docs/man3/strtok_r.3.asp#MULTITHREAD_SAFETY_LEVEL

			dbg2("strCandidateValue", strCandidateValue)
				//provo a convertire
				sscanf(strCandidateValue, "%d", &CandidateValue);
			//CandidateValue = strCandidateValue.toInt();


			//rimuovo i caratteri dalla stringa
			if (CandidateValue != -999)
			{
				//rimuovo il comando dalla stringa copiando rest in voiceCommandCharArray
				//s.remove(0, firstSpacePos);
				memcpy(voiceCommandCharArray, rest, VOICECOMMANDMAXLEN);

			}
			dbg2("GetCommandValue return:", CandidateValue)

				return CandidateValue;
		}


		//////////////////////////////////////////////////////////////////////////////////
		// processVoiceCommand		//////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////

	#define CMDPROCESSING_INITIAL_STATUS cmdStatus_WAITCMD
		static e_cmdProcessingStatus cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; //0=idle,1:attn;2:cmd;3:cmdvalue
		static int currentCmdId = -1;
		static e_VoiceCommands currentCmd = NIL; // comand corrente da elaborare
	#define processVoiceCommandReset 	cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; currentCmdId = -1; currentCmd = NIL;
	//CMDPROCESSING_INITIAL_STATUS
	// 1 non utilizza il comando di attenzione 
	// 0= utilizza il comando di attenzione
		void processVoiceCommand() {
			boolean blEndVoiceCmdProcessing = false;
			//String InputString = "";
			int cmdValue = COMMANDVALUEINVALID;
			int cmdId = 0;
			dbg2("> pVC rx: ", voiceCommandCharArray);

			while (sizeof(voiceCommandCharArray) > 0 && !blEndVoiceCmdProcessing)
			{
				dbg2("> stringa rimanente da elabore:", voiceCommandCharArray);
				switch (cmdProcessingStatus)
				{
				case 0: // idle
						// si attende la parola di attenzione
						// ritorna 0 se non riconosce alcun comando
					dbg(">*  cmdProcessingStatus = 0")
						strtrim(voiceCommandCharArray);
					//cha->trim();

					if (blAttentionWordFound())
					{
						dbg(">AttentionWordFound");
						cmdProcessingStatus = cmdStatus_WAITCMD;
					}
					else
					{
						dbg("> OI OI");
						SPEAK("OI OI NON CAPITO");	//SPEAK("OI OI");


						blEndVoiceCmdProcessing = true;
						processVoiceCommandReset
					}

					break;
				case cmdStatus_WAITCMD: // attende il comando
					dbg("> *       cmdProcessingStatus = 1")
						//cha->trim();


						cmdId = GetCommandIndex();
					currentCmd = (e_VoiceCommands)cmdId;

					switch (currentCmd)
					{
					case AVANTI:
						cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
						dbg(">* OKEY AVANTI");

						SPEAK("OKEY AVANTI");
						break;

					case INDIETRO:
						cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
						dbg(">* OKEY INDIETRO");
						SPEAK("OKEY INDIETRO");
						break;

					case DESTRA:
						cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
						dbg(">* OKEY DESTRA");
						SPEAK("OKEY DESTRA");
						break;

					case SINISTRA:
						cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un valore
						dbg(">* OKEY SINISTRA");
						SPEAK("OKEY SINISTRA");
						break;
					case SONO:
						cmdProcessingStatus = cmdStatus_WAITPARAM; // attende un'altra parola chiave
						//dbg(">* OKEY SONO");
						//SPEAK("CIAO LUCA");
						break;

					case STOP:
						// wait su semaforo di commandQueue


						//todo: esegue comando di stop

						cmdRobotCore.sendCmdStart(CmdRobotStop);

						cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
						SPEAK("OKEY STOP");

						dbg("\n OK Mi fermo")
							blEndVoiceCmdProcessing = true;
						processVoiceCommandReset
							break;



					default:// comando  riconosciuto ma non gestito
						cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
						dbg("cmd riconosciuto  ma non gestito");

						blEndVoiceCmdProcessing = true;

						break;
					}
					break;

				case cmdStatus_WAITPARAM: //si aspetta un parametro
					dbg("*       cmdProcessingStatus = cmdStatus_WAITPARAM")
						//cha->trim();
						strtrim(voiceCommandCharArray);

					cmdValue = GetCommandParamValue();
					dbg2("cmdValue:", cmdValue)
						if (cmdValue == COMMANDVALUEINVALID) {
							cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
							dbg("  Valore non valido")

						}
						else // Valore numerico ok
						{

							// converto in stringa
							char inputChNumber[5];
							itoa(cmdValue, inputChNumber, 10);
							SPEAK(inputChNumber);
							//SPEAK(voiceCommandCharArray);

							dbg("*****   invia il comando ******")
								dbg2("Valore comando:", cmdValue)
								// esegue il comando
								switch (currentCmd)
								{
								case AVANTI:
									cmdRobotCore.sendCmdStart(CmdRobotMoveCm);
									cmdRobotCore.sendCmdArg(cmdValue);
									cmdRobotCore.sendCmdEnd();
									cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
									break;

								case INDIETRO:
									cmdRobotCore.sendCmdStart(CmdRobotMoveCm);
									cmdValue *= -1;
									cmdRobotCore.sendCmdArg(cmdValue);
									cmdRobotCore.sendCmdEnd();
									cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
									break;

								case DESTRA:
									cmdRobotCore.sendCmdStart(CmdRobotRotateDeg);
									cmdRobotCore.sendCmdArg(cmdValue);
									cmdRobotCore.sendCmdEnd();
									cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command

									break;
								case SINISTRA:
									cmdRobotCore.sendCmdStart(CmdRobotRotateDeg);
									cmdRobotCore.sendCmdArg(-cmdValue);
									cmdRobotCore.sendCmdEnd();
									cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
									break;

								case LUCA:
									cmdRobotCore.sendCmdStart(CmdRobotRotateDeg);
									cmdRobotCore.sendCmdArg(-cmdValue);
									cmdRobotCore.sendCmdEnd();
									cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // running command
									break;

								default:
									SPEAK_OIOI;
									cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
									break;
								} //end switch
						}
					blEndVoiceCmdProcessing = true;
					processVoiceCommandReset

						break;
					//case 3: //si aspetta un parametro non numerico

					//	cmdId = GetCommandIndex();
					//	currentCmd = (e_VoiceCommands)cmdId;

					//	switch (currentCmd)
					//	{
					//	case LUCA:
					//		cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
					//		dbg(">* LUCA");

					//		SPEAK("CIAO LUCA");
					//		// todo disattivare il sensore di movimento
					//		break;

					//	case ANGELICA:
					//		cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
					//		dbg(">* ANGELICA ");

					//		SPEAK("CIAO ANGELICA");
					//		// todo disattivare il sensore di movimento
					//		break;
					//	case VINICIA:
					//		cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
					//		dbg(">* OKEY AVANTI");

					//		SPEAK("CIAO ANGELICA");
					//		// todo disattivare il sensore di movimento
					//		break;

					//	default:// persona sconosciuta
					//		cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
					//		SPEAK("NON TI CONOSCO  MA PIACERE");

					//		break;
					//	}
					//	break;
				default: // non dovrebbe mai arrivare qui
					cmdProcessingStatus = CMDPROCESSING_INITIAL_STATUS; // torna in IDLE
					blEndVoiceCmdProcessing = true;

					break;

				}// end switch cmdProcessingStatus

			}//end while

			 //	blEndVoiceCmdProcessing = false;//ripristina lo stato iniziale

		}


	#pragma endregion
	//------------------------------------------------------------------------------


#pragma endregion

/// ///////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////
///                   /////////////////////////////////////////////////////////////
// PROCESSI CHIBIOS  /////////////////////////////////////////////////////////////
///                   /////////////////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////

	/// ///////////////////////////////////////////////////////////////////////////////
	// ROBOT INTERFACE ( SERIAL MANAGER  )    ////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	// GESTISCE I MESSAGGI DA ARDUINO CORE CON I DATI DEI SENSORI
	/*

	dati inviati da kbGetSensorsHRate
	31,1,0.00,0.00,90,1,1,1,0,2,1,10,0,1,8,16,0,1,0,0,0.00,0.00;
	31,1,0.00,0.00,90,1,1,1,0,2,1,10,0,1,8,16,0,1,0,0,1.11,2.22;
	kbGetSensorsHRate
	tictac
	robotModel.status.posCurrent.x
	robotModel.status.posCurrent.y
	robotModel.status.posCurrent.z
	(robotModel.status.sensors.irproxy.fw
	robotModel.status.sensors.irproxy.fwHL
	robotModel.status.sensors.irproxy.bk
	robotModel.status.sensors.pirDome
	robotModel.status.sensors.analog[0]		//pot
	robotModel.status.sensors.analog[1]);	//batteria
	robotModel.status.sensors.analog[2]);	//light
	robotModel.getReleStatus(0)
	robotModel.getReleStatus(1)
	digitalReadFast(Pin_MotENR));	//status motori
	digitalReadFast(Pin_MotENL))
	robotModel.status.sensors.switchTop
	robotModel.status.act.laserOn
	robotModel.readBattChargeLevel() //0-100
	robotModel.status.sensors.gps.sats
	robotModel.status.sensors.gps.lat
	robotModel.status.sensors.gps.lng

	*/ 
	#pragma region Processo: RobotCoreInterface

	static THD_WORKING_AREA(waRobotCoreInterface, 64);
	static THD_FUNCTION(thdRobotCoreInterface, arg) {
		dbg("R>>")
		char c;
		int i=0;
		bool isEndCommand;
		// inizializza il buffer di caratteri
		char chSerialRxBuffer[INPUTCHARARRAYSIZE];
		for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { chSerialRxBuffer[i] = 0; }
	
		// Setup CommandMessenger -----------------------------------------------------
		cmdRobotCore.printLfCr();   // Adds newline to every command 
		attachCommandCallbacks(&cmdRobotCore);// Attach my application's user-defined callback methods

		while (1) {
		dbg("R>")


			chThdSleepMilliseconds(300);//	chThdYield();

			//LED ON
			TFT_LED_CORE(1)


			cmdRobotCore.feedinSerialData();
			TFT_LED_CORE(0)// LED OFF

			chThdSleepMilliseconds(300);//	chThdYield();




		}

	}
	#pragma endregion



	/// ///////////////////////////////////////////////////////////////////////////////
	//  THREAD  R O T A R Y  E N C O D E R       ///////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	#pragma region  Processo GESTIONE MENU CON ROTARY ENCODER  
		static THD_WORKING_AREA(waMenu, 164);
		static THD_FUNCTION(thdMenu, arg) {
			bool ledOnOff;
			while (1) {
				ledOnOff = !ledOnOff;
				TFT_LED_MENU(1);

				chMtxLock(&mutexTFT);
				mainMenu.poll(myMenu, in);
				chMtxUnlock(&mutexTFT);
				TFT_LED_MENU(0);
				chThdSleepMilliseconds(500);//	chThdYield();//	
			}
		}
	#pragma endregion 

#define DETECTION_INTERVAL_MS 5000
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

			while (1) 
			{
				dbg("B>")
				TFT_LED_BRAIN(1);
				//playSingleNote(1200, 30);
				switch (robotModel.status.operatingMode)
					{
						case operatingMode_e::MODE_AUTONOMOUS: //Attività in modalità autonoma
								#pragma region ESPLORA
								// ////////////////////////////////////////////////////////////////////////////////
								/// ///////////////////////////////////////////////////////////////////////////////
								//  Esplora
								/// ///////////////////////////////////////////////////////////////////////////////
								// ////////////////////////////////////////////////////////////////////////////////
								#if 0
									while (robotModel.status.operatingMode == AUTONOMOUS)
									{

										TOGGLEPIN(Pin_LED_TOP_B);
										robotModel.status.parameters.sonarStartAngle = 0;
										robotModel.status.parameters.sonarEndAngle = 180;
										robotModel.status.parameters.sonarStepAngle = 30;
										robotModel.status.parameters.sonarScanSweeps = 1;
										robotModel.status.parameters.sonarMedianSamples = 2;
										robotModel.status.parameters.sonarScanSpeed = 30; // map(analogRead(Pin_AnaPot1), 0, 1023, 10, 500);  //was = 30 ms di attesa tra due posizioni

										robotModel.SonarScanBatch(&servoSonar, &Sonar);
										alfa = 90 - robotModel.status.parameters.SonarMaxDistAngle;
										SERIAL_MSG.print("Max dist @alfa:"); SERIAL_MSG.println(alfa);
										dbg2("Max dist cm:", robotModel.status.parameters.sonarMaxDistance)
											TOGGLEPIN(Pin_LED_TOP_B);

										// invia i dati Sonar all'Host
										OnkbSonarSendData(&cmdMMI);
										TOGGLEPIN(Pin_LED_TOP_B);


										robotModel.rotateDeg(alfa);
										cmDone = robotModel.moveCm(robotModel.status.parameters.sonarMaxDistance);	// avanti
										if (cmDone < (robotModel.status.parameters.sonarMaxDistance - 1))	//ostacolo ?
										{
											SERIAL_MSG.println("1,Obst!;");
											robotModel.moveCm(-FWDIST);	// torna indietro
											TOGGLEPIN(Pin_LED_TOP_B);
											stuckCount++;
											if (stuckCount > 2)
											{
												robotModel.rotateDeg(180); //inverto la direzione
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

								#endif // 0
								//  return 0;
					
							#pragma endregion


						break;
						case operatingMode_e::MODE_SLAVE:
							robotModel.cmdReadAllSensors();
							sleepTime = 2000;
						break;
						case operatingMode_e::MODE_JOYSTICK:
						break;
						case operatingMode_e::MODE_UNKNOWN:

						break;


					default:
						break;

					}


				#pragma region HUMAN DETECTION
				//-------------------------------------------------------------------
				// chiedo chi sei solo se è attivo il pir da meno di un secondo
				if (robotModel.status.pendingEvents.pirDome)
					
				{
						playSingleNote(1000, 100);
						playSingleNote(800, 100);
						playSingleNote(1000, 100);
						SPEAK_CIAOCHISEI


						//resetto l'evento
						robotModel.status.pendingEvents.pirDome = false;

						sleepTime = 2000;
				}
				else
				{
					sleepTime = 800;
				}
				#pragma endregion

				#pragma region Gestione Evento cambio modalità
				// Gestione cambio modalità-----------------------------------------
				// Switch TOP commutato da oltre un secondo?
				if (robotModel.status.pendingEvents.operatingMode)
				{
					switch (robotModel.status.operatingMode)
					{
					case	operatingMode_e::MODE_AUTONOMOUS:

						SPEAK(F(" okei esploro"));
						sleepTime = 2000;
					break;
					case operatingMode_e::MODE_SLAVE:
						SPEAK(F(" okei comanda"));
						sleepTime = 2000;
					break;

					case operatingMode_e::MODE_JOYSTICK:
						SPEAK(F(" okei comanda col gioistic"));
						sleepTime = 2000;
					break;

					default:
						SPEAK(F("non conosco questo modo "));
						sleepTime = 2000;
						break;
					}

					// clear dell'evento
					robotModel.status.pendingEvents.operatingMode=false;
				}

				#pragma endregion


				TFT_LED_BRAIN(0)

				chThdSleepMilliseconds(sleepTime);//	chThdYield();//	

			}
		}
	#pragma endregion 

	/// ///////////////////////////////////////////////////////////////////////////////
	// THREAD FiFo ->  Speech     ///////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	#pragma region Processo: FiFo -> Speech
	// in: mbox 
	//out: invia la stringa  alla seriale del modulo voce
	// usare SPEAK(s) per alimentare la fifo
	/*
	A pool is just a linked list of blocks with equal size. This list acts like a stack, chPoolFree()=PUSH, chPoolAlloc()=POP.

	chPoolFree() <- INSERTS a block in a memory pool, so it is also used to fill a pool.
	chPoolAlloc() <- REMOVES a block from a pool.

	The use requires the following steps:

	1) Creating a pool: chPoolInit() or MEMORYPOOL_DECL() if a static initialization is preferred. The test suite uses both for testing purposes, it is not required.
	2) Preloading a pool with blocks: chPoolFree(). This can be skipped if a memory provider has been defined, in that case the pool will ask the provider for blocks when needed. chCoreAllocI() is usually used as provider.
	3) Getting a block from a pool: chPoolAlloc(). The block size is intrinsically the size defined for the pool.
	4) Returning a block to a pool: chPoolFree(). The pool assumes that the block is of the correct size.

	WAs are not related to pools, that code simply uses the existing WAs as blocks for the pool in order to not waste memory, the test suite barely fits in the lesser 8bits micros so it is written to not waste RAM.

	The advantage of pools is the very high speed when freeing and allocating blocks and the constant time execution (the time of the optional memory provided should be added).
	*/

		static THD_WORKING_AREA(waFifoToSpeech, 64);
		static THD_FUNCTION(thdFifoToSpeech, arg) {
  
			SwSerialSpeech.begin(9600);
			SwSerialSpeech.print("voce okei");

  

			PoolObject_t *pVoice; //pointer al msg nella fifo

			while (1) {

				// attende l'arrivo di un messaggio
				chMBFetch(&mailVoice, (msg_t*)&pVoice, TIME_INFINITE);  // il cast di pVoice deve essere sempre di tipo msg_t
				// bip
				//playSingleNote(600, 40);

				dbg("V>")
 
				// attende che non sia busy
				while (digitalRead(Pin_SpeechSerialBusy) == 1) { chThdSleepMilliseconds(20); }		//mi assicuro che la stringa venga inviata interamente
				
				//disabilita gli interrupt per garantire che l'intera stringa venga inviata
				osalSysDisable();
				//invia la stringa o l'intero  sulla seriale
				if (pVoice->isMsgString)
				{
					SwSerialSpeech.print(pVoice->strSpeech);
				}
				else
				{
					SwSerialSpeech.print(pVoice->msgInt);						
				}
				osalSysEnable();

 
				// libera la memoria
				chPoolFree(&memPoolVoice, pVoice);
				mailBoxGlobalFreeCounter++;
				//dbg2("Free mbox+: ", mailBoxGlobalFreeCounter);

				chThdSleepMilliseconds(200);//	chThdYield();//	
			}
		}

 


	#pragma endregion 

	/// ///////////////////////////////////////////////////////////////////////////////
	//  THREAD  T F T  M O N I T O R									/////////////
	/// //////////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// Input: RobotStatus
	// Output: TFT
	#pragma region Processo di gestione T F T   M O N I T O R
		#define SONAR_RANGE_SIZE 50 // raggio in pixel della rappresentazione dei valori sonar
		static THD_WORKING_AREA(waTFT, 100);
		static THD_FUNCTION(thdTFT, arg) {
			bool HbLed = 0; //stato del led che visualizza l'ttivit� di questo Thread
			byte r = TFTCAPTION_STARTINGROW;
			char *strTmp;
			unsigned long t1;
			unsigned long t2;
			struct myMsgStruct  myTftMsg ;
			//chMtxUnlock(&mutexTFT);

			TFTCLEAR;
			while (true)// loop di visualizzazione dati dai 38 ai 50ms (in base alla lunghezza dei gauge)
			{
			tftPrintCaptions();
				dbg("T>")
				//commuta il led su TFT
				HbLed = !HbLed;	//Heartbeat Led
				TFT_LED_TFT(1);

				t1 = millis();
				//chMtxLock(&mutexTFT); //si mette in attesa se un altro thread sta scrivendo su TFT

				// imposta i colori di default
				tft.setBackColor(VGA_BLACK);
				tft.setColor(VGA_WHITE);

				//cancella  la barra di stato superiore 
				TFT_CLEAR_STATUSBAR

				// RESETTA I LED DEI PROCESSI ATTIVI
				//tft.fillCircle(LCD_LED_SPEAK_POS_X, LCD_LED_SPEAK_POS_Y, LCD_LED_HALF_SIZE, VGA_BLACK);
				//tft.fillCircle(LCD_LED_SERIALCORE_POS_X, LCD_LED_SERIALCORE_POS_Y, LCD_LED_HALF_SIZE, VGA_BLACK);
				//tft.fillCircle(LCD_LED_SERIALCORE_POS_X+50, LCD_LED_SPEAK_POS_Y, LCD_LED_HALF_SIZE, VGA_BLACK);

				//drawLedRect(TFTDATACOL, 10, robotModel.status.tictac, VGA_LIME);
 
				//cancella  l'area dati
				TFT_CLEAR_DATA
				/// Visualizza la modalità operativa --------------------------------------------------
				switch (robotModel.status.operatingMode)
				{
				case operatingMode_e::MODE_SLAVE:
					tft.setColor(VGA_RED);
					strTmp = "S L A V E  ";
					break;
				case operatingMode_e::MODE_AUTONOMOUS:
					tft.setColor(VGA_BLUE);
					strTmp = "AUTONOMOUS";
					break;
				case operatingMode_e::MODE_JOYSTICK:
					tft.setColor(VGA_WHITE);
					strTmp = "JOYSTICK";
					break;
				case operatingMode_e::MODE_UNKNOWN:
					tft.setColor(VGA_WHITE);
					strTmp = "UNKNOWN   ";
					break;

				default:
					strTmp = "MODE ????? ";
					tft.setColor(VGA_WHITE);
					break;
				}
				TFTprintAtStr(0, TFTROW(0), strTmp);
				// non FUNZIONA COSI' !!!
				//tft.setColor(VGA_RED);
				//TFTprintAtStr(0, TFTROW(0), robotModel.getOperatingModeChar());
  


				tft.setColor(VGA_WHITE);
				tft.setBackColor(0, 0, 0);
				r = TFTCAPTION_STARTINGROW;
				/// Visualizza la memoria libera---------------------------------------------------
				//TFTprintAtNumI(TFTDATACOL, TFTROW(r++), getFreeSram(), VGA_GRAY);

				/// Visualizza posizione---------------------------------------------------
 
				TFTprintAtNumF(TFTDATACOL, TFTROW(r), robotModel.status.posCurrent.x,2  , VGA_WHITE);
				TFTprintAtNumF(TFTDATACOL + 70, TFTROW(r), robotModel.status.posCurrent.y, 2, VGA_WHITE);
				TFTprintAtNumF(TFTDATACOL + 150, TFTROW(r++), robotModel.status.posCurrent.r, 2, VGA_WHITE);

				/// Visualizza dati GPS---------------------------------------------------
				TFTprintAtNumI(TFTDATACOL, TFTROW(r), robotModel.status.sensors.gps.sats, VGA_WHITE);
				TFTprintAtNumF(TFTDATACOL + 50, TFTROW(r), robotModel.status.sensors.gps.lat, 2, VGA_WHITE);
				TFTprintAtNumF(TFTDATACOL + 150, TFTROW(r++), robotModel.status.sensors.gps.lng, 2, VGA_WHITE);


				// Ingressi Analogici ----------------------------------------------------

				//TFTprintAtNumI(TFTDATACOL, TFTROW(r), robotModel.status.sensors.analog[0], VGA_GREEN);
				//drawGaugeAnalog(TFTDATACOL , TFTROW(r++), robotModel.status.sensors.analog[0], VGA_GREEN);

				//TFTprintAtNumI(TFTDATACOL, TFTROW(r), robotModel.status.sensors.analog[1], VGA_LIME);
				drawGaugePerc(TFTDATACOL, TFTROW(r++), robotModel.status.sensors.batCharge, VGA_LIME);

				//TFTprintAtNumI(TFTDATACOL, TFTROW(r), robotModel.status.sensors.analog[2], VGA_YELLOW);
				drawGaugeAnalog(TFTDATACOL, TFTROW(r++), robotModel.status.sensors.analog[2], VGA_YELLOW);

				// MOTION DETECTION
				drawLedRect(TFTDATACOL, TFTROW(r++), robotModel.status.sensors.pirDome, VGA_RED);

				// IR SENSORS DETECTION
				
				drawLedRect(TFTDATACOL, TFTROW(r), robotModel.status.sensors.irproxy.fw, VGA_FUCHSIA);
				drawLedRect(TFTDATACOL + 10, TFTROW(r), robotModel.status.sensors.irproxy.fwHL, VGA_FUCHSIA);
				drawLedRect(TFTDATACOL + 20, TFTROW(r), robotModel.status.sensors.irproxy.fl, VGA_FUCHSIA);
				drawLedRect(TFTDATACOL + 30, TFTROW(r), robotModel.status.sensors.irproxy.fr, VGA_FUCHSIA);
				drawLedRect(TFTDATACOL + 40, TFTROW(r), robotModel.status.sensors.irproxy.fw, VGA_FUCHSIA);
				drawLedRect(TFTDATACOL + 40, TFTROW(r++), robotModel.status.sensors.irproxy.bk, VGA_FUCHSIA);

				drawLedRect(TFTDATACOL, TFTROW(r), robotModel.status.sensors.bumper.right, VGA_RED);
				drawLedRect(TFTDATACOL + 10, TFTROW(r), robotModel.status.sensors.bumper.center, VGA_RED);
				drawLedRect(TFTDATACOL + 20, TFTROW(r++), robotModel.status.sensors.bumper.left, VGA_RED);
 
				#pragma region VISUALIZZA I MESSAGGI IN INGRESSO
				//TFTprintAtNumI(TFTDATACOL , TFTROW(r++), myRingBufTftMsg->elements, VGA_RED);
				#if 0

					osalSysDisable();

					if (myRingBufTftMsg->elements > 0)
					{
						dbg2("	myRingBuf->elements:", myRingBufTftMsg->elements)
						// alloca la memoria per msgOutVoice 
						memset(&myTftMsg, 0, sizeof(struct myMsgStruct));

						// estrai in myTftMsg il messaggio 
						myRingBufTftMsg->pull(myRingBufTftMsg, &myTftMsg);
					
						strTmp = myTftMsg.msgString;
						//TFTprintAtStr(TFTDATACOL, TFTROW(r++),strTmp );
						Serial.print('[');
						Serial.print( strTmp);
						Serial.println(']');
					}
					osalSysEnable();

					// Visualizza l'ultimo comando via seriale ---------------------------------------------------
					tft.setColor(VGA_WHITE);
					//TFTprintAtStr(TFTDATACOL, TFTROW(r++), *serialRxBuffer);
					while ((rxBuf.remain() > 0) && (TFTROW(r) < (TFT_Y_HEIGHT - TFT_ROWSPACING)))
					{
						sTFT = rxBuf.pop();
						TFTprintAtStr(TFTDATACOL, TFTROW(r++), sTFT);
					}


				#endif // 0
				#pragma endregion
 
				#pragma region Mappa Radar
				#if 0
						// Visualizza la mappa radar ------------------
						r++;
						tft.setColor(VGA_BLUE);
						uint16_t x = 0;
						uint16_t y = 0;
						for (int i = 0; i < 360; i++)
						{

							drawEchoPolar(maxRangePixel + 5, TFTROW(r) + maxRangePixel, (double)i, analogRead(1), VGA_GREEN);
						}
						//---------------------------------------------

				#endif // 0

				#pragma endregion
				t2= millis();

				//millisecondi impiegati dal loop sulla prima riga
				TFTprintAtNumI(TFTDATACOL+100, TFTCAPTION_STARTINGROW, t2-t1, VGA_GRAY);
				chMtxUnlock(&mutexTFT);

				TFT_LED_TFT(0);

				chThdSleepMilliseconds(1000);//chThdYield();
				
			} //end while true

		} //end thdTFT
	#pragma endregion 

	/// ///////////////////////////////////////////////////////////////////////////////
	//  THREAD GESTIONE COMANDI DA BLUETOOTH         //////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	#pragma region  ECHO SERIAL_ROBOT > SERIAL_PC 
		// IN SVILUPPO
		#define BTH_CMD_START '['
		#define BTH_CMD_END ']'
		#define VOICE_CMD_START '*'
		#define VOICE_CMD_END '#'
		#define BTH_CBK_START '*'
		#define BTH_CBK_END'#'
		enum cmdBTstatus_e
		{
			cmdWaiting = 0,
			beginBTcmd = 1,
			cmdBeginVoiceCmd = 2,
			endVoiceCmd =3
		};
		void executeBTcmd(char c) {
			switch (c)	//Esegue il comando
			{
			case 'F': //FORWARD
				SPEAK(F("OKEI AVANTI"));
				robotModel.cmdMoveCm(robotModel.cmdSettingDefaultMoveCm);
				break;
			case 'B'://BACK
				SPEAK(F("OKEI INDIETRO"));
				robotModel.cmdMoveCm(-robotModel.cmdSettingDefaultMoveCm);

				break;
			case 'R'://RIGHT
				SPEAK(F("OKEI DESTRA"));
				robotModel.cmdRotateDeg(robotModel.cmdSettingDefaultRotateDeg);

				break;
			case 'L'://LEFT
				SPEAK(F("OKEI SINSTRA"));
				robotModel.cmdRotateDeg(-robotModel.cmdSettingDefaultRotateDeg);
				break;
			case 'W'://Webcam On
				SPEAK(F("uebcam On"));

				break;
			case 'w'://Webcam Off
				SPEAK(F("uebcam spenta"));

				break;
				// Comandi di movimento no limitato
			case 'f': //FORWARD
				SPEAK("OKEI AVANTI");
				robotModel.cmdGo(commandDir_e::GOF,robotModel.status.cmd.clock);
				break;
			case 'b'://BACK
				SPEAK("OKEI INDIETRO");
				robotModel.cmdGo(commandDir_e::GOB, robotModel.status.cmd.clock);

				break;
			case 'r'://RIGHT
				SPEAK("OKEI DESTRA");
				robotModel.cmdGo(commandDir_e::GOR, robotModel.status.cmd.clock);

				break;
			case 'l'://LEFT
				SPEAK("OKEI SINSTRA");
				robotModel.cmdGo(commandDir_e::GOL, robotModel.status.cmd.clock);
				break;

			default:
				break;
			}

		}
		/////////////////////////////////////////////////////////////////
		/// Invia su BT lo stato del Robot utilizzando 
		// * per inizio
		// # per fine 
		// es. *B60#  per batteria al 60% 
		///////////////////////////////////////////////////////////////
		void BTSendStatus(){
 
			// BATTERIA
			SERIAL_BT.print(BTH_CBK_START);
			SERIAL_BT.print("B");
			SERIAL_BT.print(robotModel.status.sensors.batCharge);
			SERIAL_BT.print(BTH_CBK_END);
			// POSE
			SERIAL_BT.print(BTH_CBK_START);
			SERIAL_BT.print("P");
			SERIAL_BT.print(robotModel.status.posCurrent.x,2);
			SERIAL_BT.print(",");
			SERIAL_BT.print(robotModel.status.posCurrent.y,2);
			SERIAL_BT.print(BTH_CBK_START);
			// ROTAZIONE
			SERIAL_BT.print(BTH_CBK_START);
			SERIAL_BT.print("B");
			SERIAL_BT.print(robotModel.status.posCurrent.r);
			SERIAL_BT.print(BTH_CBK_END);

		}
		// 64 byte stack beyond task switch and interrupt needs
		static THD_WORKING_AREA(waBT, 64);
		static THD_FUNCTION(thdBT, arg) {
			bool blink;
			int inByte = 0;
			char c;
			static	cmdBTstatus_e bthStatus = cmdBTstatus_e::cmdWaiting;
			// inizializza il buffer d caratteri
			for (int i = 0; i < VOICECOMMANDMAXLEN; i++) { voiceCommandCharArray[i] = 0; }
			int i = 0;


			while (1) {
				//dbg("E>")
				blink = !blink;
				TFT_LED_BTH(1);
				if (SERIAL_BT.available())
				{
					while (SERIAL_BT.available() > 0) {
						c = SERIAL_BT.read();
						SERIAL_PC.write(c);
						SERIAL_BT.write(c);
						playSingleNote(NOTE_A6, 20);

						//PARSING ------------------
						// @= inizio comando BT
						// #= fine comando BT
						
						//formato comandi vocali: *comando#  Es.  *ROBOT AVANTI 20#
						//formato comdMessenger:  [cmdId],[parametri];  o [cmd];
						
						switch (bthStatus)
						{
						case cmdWaiting:
							//scarto tutto ciò che non è un carattere valido di inizio comando
							switch (c)
							{
							case BTH_CMD_START:
								bthStatus = cmdBTstatus_e::beginBTcmd;

								break;
							case VOICE_CMD_START:
								bthStatus = cmdBTstatus_e::cmdBeginVoiceCmd;

								break;
							case BTH_CMD_END:
								//si è perso il comando
								SPEAK_OIOI

								break;
							case VOICE_CMD_END:
								//si è perso il comando
								SPEAK_OIOI

								break;
							default: 
								break;
							}
						break;
						case cmdBeginVoiceCmd:
							switch (c)
							{
							case BTH_CMD_START:
							case VOICE_CMD_START:
								break;
							case BTH_CMD_END:
								//si è perso il comando
								SPEAK_OIOI

								break;
							case VOICE_CMD_END://carattere di fine comando >> elaboro il comando vocale
								bthStatus = endVoiceCmd;
								processVoiceCommand();
								bthStatus = cmdBTstatus_e::cmdWaiting;


								break;
							default:

								voiceCommandCharArray[i] = toupper(c); 
								i++; 
								break; // sostituisco con una pausa


							}




						break;
						case beginBTcmd:
							executeBTcmd(c);
							bthStatus = cmdBTstatus_e::cmdWaiting;


						break;

						default:
							break;
						}


					}

					SERIAL_BT.println("");
					SERIAL_PC.println("");

					// se sono arrivati comandi da BT, allora invio lo stato del robot
					BTSendStatus();
				}

				TFT_LED_BTH(0);
				chThdSleepMilliseconds(500);//	chThdYield();//	
			}
		}
	#pragma endregion 


// FINE PROCESSI CHIBIOS ////////////////////////////////////////////////////////////////////////////////


// ////////////////////////////////////////////////////////////////////////////////
/// ///////////////////////////////////////////////////////////////////////////////
//  THREAD  N O N  A T T I V I  									/////////////
/// //////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////
#pragma region THREAD NON ATTIVI
	/// ///////////////////////////////////////////////////////////////////////////////
	//  THREAD ECHO SERIAL_BT > SERIAL_PC       ///////////////////////////////////////////////////////////
	/// ///////////////////////////////////////////////////////////////////////////////
	// FOR TEST ONLY
	#pragma region  ECHO SERIAL_ROBOT > SERIAL_PC 
#if 0
						// 64 byte stack beyond task switch and interrupt needs
		static THD_WORKING_AREA(waserialBtEcho, 64);
		static THD_FUNCTION(serialBtEcho, arg) {
			bool blink;
			int inByte = 0;
			char c;
			//sdStart(&ch1, NULL);
			while (1) {
				//dbg("E>")
				blink = !blink;
				drawLed(320, 5, blink, VGA_AQUA);
				if (SERIAL_BT.available())
				{
					while (SERIAL_BT.available() > 0) {
						c = SERIAL_BT.read();
						SERIAL_PC.write(c);
						SERIAL_BT.write(c);
						playSingleNote(NOTE_A6, 20);
						//inByte = SERIAL_BT.read();
						//SERIAL_PC.write(inByte);
					}

					SERIAL_BT.println("");
					SERIAL_PC.println("");

				}

				chThdSleepMilliseconds(200);//	chThdYield();//	
			}
		}

#endif // 0
#pragma endregion 


#pragma endregion  // thread non attivi
 


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

//------------------------------------------------------------------------------
// ////////////////////////////////////////////////////////////////////////////////////////////
// PROCESSI CHIBIOS ATTIVI
// ////////////////////////////////////////////////////////////////////////////////////////////
void thd_Setup() {
	dbg("cHIBIOS chThreads...")
	dbg("R> = RobotCoreInterface")
	dbg("T> = TFT")
	dbg("B> = Brain")
	dbg("E> = ECHO BTH")
	dbg("V> = Voice")

	chThdCreateStatic(waRobotCoreInterface, sizeof(waRobotCoreInterface), NORMALPRIO + 5, thdRobotCoreInterface, NULL);
	chThdCreateStatic(waTFT, sizeof(waTFT), NORMALPRIO +4, thdTFT, NULL);
	chThdCreateStatic(waBT, sizeof(waBT), NORMALPRIO + 3, thdBT, NULL);
	chThdCreateStatic(waMenu, sizeof(waMenu), NORMALPRIO + 3, thdMenu, NULL);
	chThdCreateStatic(waBrain, sizeof(waBrain), NORMALPRIO+3, thdBrain, NULL);

	//	chThdCreateStatic(waFifoToSpeech, sizeof(waFifoToSpeech), NORMALPRIO + 2, thdFifoToSpeech, NULL);
	while (1) {}
}


void setup()
{	
	#pragma region Inizializzazione SERIALI
		SERIAL_PC.begin(SERIAL_PC_BAUD_RATE);
		SERIAL_ROBOT.begin(SERIAL_ROBOT_BAUD_RATE);
		SERIAL_ROBOT.setTimeout(5000);
		SERIAL_BT.begin(SERIAL_BT_BAUD_RATE);
		SERIAL_BT.setTimeout(5000);
		SERIAL_SPEAK.begin(SERIAL_SPEAK_BAUD_RATE);
// 		SERIAL_SPEAK.setTimeout(5000);
		SERIAL_WIFI.begin(SERIAL_WIFI_BAUD_RATE);
	#pragma endregion
	dbg("MMI");

 
	#pragma region test swSerial
		#if 0
				pinMode(Pin_SpeechSerialBusy, INPUT);
				for (size_t i = 0; i < 3; i++)
				{

					SwSerialSpeech.print("io ");
					delay(700);
				}

		#endif // 0
	#pragma endregion

	robotModel.begin(MODE_SLAVE, &cmdRobotCore);
	
	#pragma region Inizializzazione TFT
		#ifdef TFT_ili9488
			// Declare which fonts we will be using
			extern uint8_t SmallFont[];
			// extern uint8_t LargeFont[];
			// extern uint8_t hallfetica_normal[];

			// INIT
			tft.InitLCD();
			tft.setFont(SmallFont);
			tft.lcdOn();
			delay(1000);

			// MACRO
		#define TFTprintAtStr(x,y,s) tft.print(s,x,y)
		#define TFTprintAtNumF(x,y,d,decimals) tft.printNumF( d,decimals,x,y)
		#define TFTprintAtNumI(x,y,i) tft.printNumI(i,x,y)
			// ORIENTATION
			tft.orient = PORTRAIT;// LANDSCAPE;
								  // CLEAR
			tft.fillScr(LCD_BACKGROUND);
			// TEXT STYLE
			tft.setColor(GREEN);
		#endif
		#ifdef TFT_ili9341
			// INIT
			tft.begin(0x9341); // SDFP5408
							   // MACRO
		#define TFTprintAtStr(x,y,s) tft.printAt(x,y,s)
		#define TFTprintAtNumF(x,y,d) tft.printAt(x, y, dtostrf(d, 20, 8, s));

							   // ORIENTATION
			tft.setRotation(0); // Need for the Mega, please changed for your choice or rotation initial
								// CLEAR
			tft.setColor(LCD_BACKGROUND);
			tft.fillScreen(LCD_BACKGROUND);
			// TEXT STYLE
			tft.setTextSize(2);
			tft.setTextColor(GREEN);

		#endif
		#ifdef TFT_ILI9327_8
			// TFT setup-----------------------------
			tft.InitLCD(PORTRAIT);
			tft.clrScr();
			///  ts.setPrecision(PREC_HI);
			tft.setFont(SmallFont); //	tft.setFont(BigFont);

			TFTCLEAR;
			tftPrintCaptions();

			//---------------------------------------


		#endif

		//tft.setBackColor(255, 0, 0);
		//TFTprintAtStr(TFTDATACOL, TFTROW(10), (char*)robotModel.getOperatingModeChar());

		tft.setBackColor(0, 0, 0);



	#pragma endregion
	
	#pragma region Impostazione Interupt Manopola Encoder e MENU
		//al posto di Timer1 + funzione TimerIsr() uso direttamente gli interupt
		Timer1.initialize(5000); // every 0.05 seconds
		Timer1.attachInterrupt(timerIsr);
		qEnc.setAccelerationEnabled(false);
		qEnc.setDoubleClickEnabled(true); // must be on otherwise the menu library Hang

		myMenu.init();//setup geometry after tft initialized
		myMenu.maxX = 25; //larghezza max menu in caratteri (non funz.)
		myMenu.maxY = 8;  //numero di voci del menu 
		myMenu.enabledColor = VGA_WHITE; //Colore delle voci abilitate
		myMenu.bgColor = VGA_GRAY;  //Sfondo
		myMenu.disabledColor = VGA_BLACK;
		// Posizione dei menu
		mainMenu.setPosition(MENUPOSITION_X, MENUPOSITION_Y);
		subMenuMode.setPosition(MENUPOSITION_X, MENUPOSITION_Y);
		subMenuMove.setPosition(MENUPOSITION_X, MENUPOSITION_Y);
		subMenuModifyStatus.setPosition(MENUPOSITION_X, MENUPOSITION_Y);
		subMenuTest.setPosition(MENUPOSITION_X, MENUPOSITION_Y);
		subMenuHelp.setPosition(MENUPOSITION_X, MENUPOSITION_Y);
//		subMenuModifyStatus.setPosition(MENUPOSITION_X, MENUPOSITION_Y);
		//mainMenu.data[1]->enabled = false;
	#pragma endregion

	#pragma region Inizializzazione RobotModel
		robotModel.cmdSettingDefaultMoveCm = 20;
		robotModel.cmdSettingDefaultRotateDeg = 30;
		robotModel.status.operatingMode = operatingMode_e::MODE_SLAVE;

	#pragma endregion

		//	singMyMelody(myMelody1);
		playSingleNote(1000, 100);

	#pragma region Inizializzazione e Test memory pool allocation
		// fill pool with PoolObject array
		for (size_t i = 0; i < MBOX_GLOBAL_CAPACITY; i++) {
			chPoolFree(&memPoolVoice, &PoolObject[i]);
		}

		#if 0

			// get object from memory pool
			PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPoolVoice);
			if (!p) { Serial.println("chPoolAlloc failed from setup");	while (1); }
			p->strSpeech = "t ";		// (char*)strSpeech;
												//p->strSpeech = " imbecille spostati che devo passare";		// (char*)strSpeech;
												//strcpy(p->str, "oi");
												//p->size = 2;
												//dbg2("thdFeedFifo send: ",p->strSpeech)

												// send message
			msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
			if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }

			SPEAK_TEST

		#endif // 0

	#pragma endregion

	chBegin(thd_Setup);	
}
void loop() {
	// not used
}

#pragma endregion


