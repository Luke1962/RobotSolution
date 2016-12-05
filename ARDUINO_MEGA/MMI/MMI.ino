//////////////////////////////////////////////////////////////////////////////////
// I N T E R F A C C I A   M M I                //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
// CONFIGURAZIONE DEL SISTEMA                  ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region CONFIGURAZIONE DEL SISTEMA 
	

	//////////////////////////////////////////////////////////////////////////
	// C O N F I G U R A Z I O N E  H A R D W A R E							//
	//////////////////////////////////////////////////////////////////////////
	#include <hwMMI_config.h>
	//////////////////////////////////////////////////////////////////////////


	// TFT TYPE                Uncomment one of the following----------
	#define TFT_ILI9327_8	//x:320 X y:480 con driver 	
	//#define TFT_ili9341
	//#define TFT_ili9488	

	//#define T 1000

	//////////////////////////////////////////////////////////////////////////
	// OPZIONI DI D E B U G													//
	//////////////////////////////////////////////////////////////////////////
	#define DEBUG_ON
	#include <dbg.h>

	//////////////////////////////////////////////////////////////////////////
	//																		//
	//						 O P Z I O N I   R O B O T						//
	//																		//
	//////////////////////////////////////////////////////////////////////////

	#define OPT_SERVOSONAR 1	//INCLUDI FUNZIONALITA' SERVO-SONAR
	#define OPT_ENCODERS  0	//INCLUDI ENCODERS

	#define INPUTCHARARRAYSIZE 50 //dimensione buffer seriale


	//////////////////////////////////////////////////////////////////////////////////
	// LIBRERIE                                    ///////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	#pragma region Librerie
		#include <ChibiOS_AVR/ChibiOS_AVR.h>
		#include <SoftwareSerial.h>		//C:\Program Files %28x86%29\Arduino\hardware\arduino\avr\libraries\SoftwareSerial
		#include <TinyGPSplus\TinyGPS++.h>	//se manca non compila a causa del robotModel.cpp nella stessa cartella di robot\Commands_Enum.h

		#include <string.h> 
		#include "stringlib.h"
 		#include <buzzer\buzzer.h>
		//#include <digitalWriteFast.h>
		//#include <CmdMessenger/CmdMessenger.h>
		//#include <robot\Commands_Enum.h>
		#include <MyRobotLibs\robotModel.h>
		#include <MyRobotLibs\SpeakSerialInterface.h>
		#include <MyRobotLibs\CircularBuffer.h>
	#pragma endregion
#pragma endregion


// ////////////////////////////////////////////////////////////////////////////////////////////
//  CREAZIONE OGGETTI GLOBALI
// ////////////////////////////////////////////////////////////////////////////////////////////
#pragma region CREAZIONE OGGETTI GLOBALI
	//#include <servo/src/Servo.h> //deve restare qui altrimenti il linker s'incazza (??)
	// ////////////////////////////////////////////////////////////////////////////////////////////
	// MODELLO ROBOT
	struct robotModel_c robotModel;


	#pragma region ROTARY ENCODER
		#include <Rotary/Rotary.h>	//https://github.com/brianlow/Rotary


		volatile int encoder_position = 0;
		volatile int encoder_delta = 0;
		int current_encoder_position = 0;
		int current_encoder_delta = 0;

		Rotary encoder = Rotary(Pin_ROT_ENCODER_A, Pin_ROT_ENCODER_B);


		void printEncoderInfo() {
			SERIAL_PC.print("encoder_position: ");
			SERIAL_PC.print(current_encoder_position);
			SERIAL_PC.print(", delta: ");
			SERIAL_PC.print(current_encoder_delta);
			SERIAL_PC.print(", dir: ");
			if (current_encoder_delta>0) SERIAL_PC.println("right");
			else SERIAL_PC.println("left");
		}

		bool encoderPositionUpdated() {
			static int last_position = -999;

			// disable interrupts while we copy the current encoder state
			uint8_t old_SREG = SREG;
			cli();
			current_encoder_position = encoder_position;
			current_encoder_delta = encoder_delta;
			SREG = old_SREG;

			bool updated = (current_encoder_position != last_position);
			last_position = current_encoder_position;

			return updated;
		}

		/*
		Interrupt Service Routine:
		reads the encoder on pin A or B change
		*/
		void loadEncoderPositionOnChange() {
			unsigned char result = encoder.process();
			if (result == DIR_NONE) {
				// do nothing
			}
			else if (result == DIR_CW) {
				encoder_delta = 1;
				encoder_position++;
			}
			else if (result == DIR_CCW) {
				encoder_delta = -1;
				encoder_position--;
			}
		}
		// buffer circolare dei messaggi ricevuti
		CircularBuffer<String,5> rxBuf;
	#pragma endregion
	// ////////////////////////////////////////////////////////////////////////////////////////////

	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  CmdMessenger object to the default Serial port
	#pragma region COMMAND MESSENGER
		#include <CmdMessenger2/CmdMessenger2.h>
		static CmdMessenger2 cmdRobotCore = CmdMessenger2(SERIAL_ROBOT);
		static CmdMessenger2 cmdBT = CmdMessenger2(SERIAL_BT);
		//void SPEAK(char inStr[]);


		#include "RobotInterfaceCommandsMMI.h"
	#pragma endregion
	// ////////////////////////////////////////////////////////////////////////////////////////////

	// ////////////////////////////////////////////////////////////////////////////////////////////
	//  TFT object  & TOUCH SCREEN
	// ////////////////////////////////////////////////////////////////////////////////////////////
	#pragma region LCD LIBRARY & OBJECT

		#ifdef TFT_ILI9327_8 // TFT 320x480
			#pragma region TFT
				#define TFT_X_WIDTH 320
				#define TFT_Y_HEIGHT 480
				#include <UTFT\UTFT.h>
				//UTFT     tft(ILI9327_8,30,31,32,33 );		//was 38, 39, 40, 41
				UTFT     tft(ILI9327_8, Pin_TFT_RS, Pin_TFT_WR, Pin_TFT_CS, Pin_TFT_RST);		//was 38, 39, 40, 41

				//extern uint8_t BigFont[];
				extern uint8_t SmallFont[];

//#define TFTprintAtStr(x,y,s) sTFT=s; tft.print( sTFT,x,y) così non visualizza le stringhe!!
				#define TFTprintAtStr(x,y,s)   tft.print( s ,x,y)
				//#define TFTprintAtNumF(x,y,d,decimals) tft.printNumF( d,decimals,x,y)
				#define TFTprintAtNumF(x,y,d,decimals,VGAcolor) tft.setColor(VGAcolor); tft.printNumF( d,decimals,x,y)
//				#define TFTprintAtNumI(x,y,i) tft.printNumI(i,x,y)
				#define TFTprintAtNumI(x,y,i,VGAcolor) tft.setColor(VGAcolor);tft.printNumI(i,x,y)
		
			#pragma endregion

			//#pragma region touch screen
			//	#include <XPT2046/XPT2046.h>

			//	#define CS_PIN  53	///was 9
			//	#define TIRQ_PIN  2
			//	XPT2046 ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
			//#pragma endregion

			// Initialize touchscreen
			#pragma region touch screen
			#include <SPI.h>


				//#define  TS1	//Touch screen  driver XPT2046_Touchscreen
			#define  TS2	//Touch screen  driver XPT2046-2

			#ifdef  TS1
			#include <XPT2046_Touchscreen\XPT2046_Touchscreen.h>
			#define isTouching() touched()
			#else
			#include <XPT2046-2/XPT2046-2.h>

			#endif // TS1



				//class TS_Point {
				//public:
				//	TS_Point(void) : x(0), y(0), z(0) {}
				//	TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
				//	bool operator==(TS_Point p) { return ((p.x == x) && (p.y == y) && (p.z == z)); }
				//	bool operator!=(TS_Point p) { return ((p.x != x) || (p.y != y) || (p.z != z)); }
				//	uint16_t x, y, z;
				//};
				//#include <XPT2046\XPT2046.h>


				//XPT2046 ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
			#ifdef TS1
					XPT2046_Touchscreen ts((uint8_t)Pin_TS_CS, (uint8_t)Pin_TS_TIRQ);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
			#else

					XPT2046 ts((uint8_t)Pin_TS_CS, (uint8_t)Pin_TS_TIRQ);  // Param 2 - Touch IRQ Pin - interrupt enabled polling

			#endif // TS1


			#pragma endregion


		#endif
		#ifdef TFT_ili9488

			#pragma region TFT
				#define TFT_X_WIDTH 240
				#define TFT_Y_HEIGHT 320
				#include <UTFT\UTFT.h>
				UTFT    tft(ILI9327_8, 38, 39, 40, 41);

				#define TFTprintAtStr(x,y,s) tft.print(s,x,y)
				#define TFTprintAtNumF(x,y,d,decimals) tft.printNumF( d,decimals,x,y)
				#define TFTprintAtNumI(x,y,i) tft.printNumI(i,x,y)

			#pragma endregion

			#pragma region touch screen
				#include <UTouch.h>
				UTouch  myTouch(6, 5, 4, 3, 2);
			#pragma endregion
		#endif
		#ifdef  SPFD5408

		#include <SPFD5408/SPFD5408_Adafruit_GFX.h>    // Core graphics library
		#include <SPFD5408/SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
		//#include <SPFD5408_TouchScreen.h>
		// The control pins for the LCD can be assigned to any digital or
		// analog pins...but we'll use the analog pins as this allows us to
		// double up the pins with the touch screen (see the TFT paint example).
		#define LCD_CS A3 // Chip Select goes to Analog 3
		#define LCD_CD A2 // Command/Data goes to Analog 2
		#define LCD_WR A1 // LCD Write goes to Analog 1
		#define LCD_RD A0 // LCD Read goes to Analog 0

		#define LCD_RESET A4 // Can alternately just connect to Arduino's reset pin

		// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
		// For the Arduino Uno, Duemilanove, Diecimila, etc.:
		//   D0 connects to digital pin 8  (Notice these are
		//   D1 connects to digital pin 9   NOT in order!)
		//   D2 connects to digital pin 2
		//   D3 connects to digital pin 3
		//   D4 connects to digital pin 4
		//   D5 connects to digital pin 5
		//   D6 connects to digital pin 6
		//   D7 connects to digital pin 7
		// For the Arduino Mega, use digital pins 22 through 29
		// (on the 2-row header at the end of the board).

		// Assign human-readable names to some common 16-bit color values:


		#define LCD_CS A3
		#define LCD_CD A2
		#define LCD_WR A1
		#define LCD_RD A0
		// optional
		#define LCD_RESET A4


		Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
		#define TFTprintAtStr(x,y,s) tft.print(s,x,y)
		#define TFTprintAtNumF(x,y,d,decimals) tft.printNumF( d,decimals,x,y)
		#define TFTprintAtNumI(x,y,i) tft.printNumI(i,x,y)

		#endif // 1 

		#define LCD_TEXT_HEIGHT 16 // Height of text to be printed and scrolled
		#define LCD_TEXT_WIDTH 10

		#define LCD_TEXTCOLOR MAGENTA
		#define LCD_BACKGROUND BLACK
		#define LCD_TEXTSIZE 2		//20 righe di 24 caratteri (19righe utili se uso la prima come stato con size =1)
		#define LCD_CHAR_COLUMNS 24
		// The scrolling area must be a integral multiple of LCD_TEXT_HEIGHT
		#define LCD_BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
		#define LCD_TOP_FIXED_AREA 24 // 16Number of lines in top fixed area (lines counted from top of screen)
		# define LCD_SCROLL_ROWS 5

		// ///////////////////////////////////////////////////////////////////
		//  Profilazione della posizione dei vari componenti degli oggetti TFT
		#define LCD_LED_HALF_SIZE 6

		//Posizione del LED  del processo BLINK
		#define LCD_LED_POS_X 10
		#define LCD_LED_POS_Y 10
		//Posizione del LED  del processo FifoToSPEAK
		#define LCD_LED_SPEAK_POS_X 100
		#define LCD_LED_SPEAK_POS_Y 10
 
		//Posizione del LED  del processo Serial
		#define LCD_LED_SERIALCORE_POS_X 150
		#define LCD_LED_SERIALCORE_POS_Y 10
		#include "TFT_HAL\TFT_HAL.h"

	#pragma endregion
#pragma endregion







// ////////////////////////////////////////////////////////////////////////////////////////////
//  P A R A M E T R I  E  V A R I A B I L I  G L O B A L I
// ////////////////////////////////////////////////////////////////////////////////////////////

#pragma region Variabili globali

	//////////////////////////////////////////////////////////////////////////////////
	//VARIABILI GLOBALI CONDIVISE TRA I PROCESSI   ///////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	//char SpeakBuffer[INPUTCHARARRAYSIZE];	//buffer per il parlato in SP0256-AL2.h
	volatile uint32_t count = 0;
	volatile uint32_t maxDelay = 0;
	char * serialRxBuffer[SERIAL_RX_BUFFER_SIZE];

	//------------------------------------------------------------------------------
	#pragma region DEFINIZIONE MAILBOX VOICE
	// mailbox size and memory pool object count
	const size_t MB_COUNT = 6;

	// type for a memory pool object
	struct PoolObject_t {
		char* name;
		char str[INPUTCHARARRAYSIZE];
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



//	bool isNumeric(const char *ch);
	int moveEfondo(char array[], int size);
	void ConvLetter(const char cifreIn[], int size);


#pragma endregion


// ///////////////////////////////////////////////////////////////////////////////
//  METTE IN CODA LA STRINGA DA PRONUNCIARE  AL TASK SPEAK    ///////////////////////////////// 
// ///////////////////////////////////////////////////////////////////////////////
//uso: SPEAK("ei");	// output via processo FifoToSPEAK
// ///////////////////////////////////////////////////////////////////////////////
//  PROCESS VOICE COMMANDS      //////////////////////////////////////////////////
// ///////////////////////////////////////////////////////////////////////////////
// Interpreta il comando ricevuto ed invia a Robot il comando corrispondente

	#pragma region PROCESS VOICE COMMANDS  
		#define VOICECOMMANDMAXLEN 40	//Massima lunghezza della stringa in ingresso contenente il comando
		#define WORDCOMMANDMAXLEN 10			// massima lunghezza di ciascun comando
		#define ATTENTIONCOMMAND ROBOT	//indice del comando vocale che attiva il robot
		#define COMMANDSCOUNT 10			// numero di comandi riconosciuti
		#define ATTENTIONCOMMANDSCOUNT 2	//numero di comandi di attivazione

		char AttentionWords[ATTENTIONCOMMANDSCOUNT][WORDCOMMANDMAXLEN] = { "ROBOT\0", "ARDUINO\0", }; //elenco comandi accettati come attivazione
		char Vocabolary[COMMANDSCOUNT][WORDCOMMANDMAXLEN] = { "NIL\0",  "AVANTI\0", "INDIETRO\0", "DESTRA\0","SINISTRA\0", "FERMA\0" , "SONO\0" , "LUCA\0", "ANGELICA\0", "VINICIA\0",};
		enum e_VoiceCommands			   { NIL,   AVANTI,   INDIETRO,   DESTRA,  SINISTRA,   FERMA , SONO , LUCA, ANGELICA, VINICIA};
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
			char *rest ;	//= voiceCommandCharArray
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

			if (cmdId>0)
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
			while (!found && (i < ATTENTIONCOMMANDSCOUNT) )
			{
				dbg2("# confronto con:", AttentionWords[i])
					//if (cmdCandidate.equalsIgnoreCase(Vocabolary[i]))
					if (strcmp(cmdCandidate, AttentionWords[i]) == 0)		//http://www.cplusplus.com/reference/cstring/strcmp/
					{
						found = true;
						cmdId = i;	//trovato
						dbg2("# Trovato cmd #: ",cmdId)
					}
				i++;
			}



				//rimuovo il comando dalla stringa copiando rest in voiceCommandCharArray
				//s.remove(0, firstSpacePos);
				dbg2("#  Stringa restante:", rest);
				dbg2("#  len of rest:", strlen(rest));

				// assegno la string rimanente a voiceCommandCharArray
				strncpy(voiceCommandCharArray, rest, strlen(rest));
				for (size_t i =strlen(rest); i < strlen(voiceCommandCharArray); i++)
				{			voiceCommandCharArray[i] = '\0';		}


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
		// 1 non utilizza il comando di attenzione 0= utilizza il comando di attenzione
		void processVoiceCommand() {
			boolean blEndVoiceCmdProcessing = false;
			//String InputString = "";
			int cmdValue = COMMANDVALUEINVALID;
			int cmdId = 0;
			dbg2("> processVoiceCommand riceve: ", voiceCommandCharArray);

			while (sizeof(voiceCommandCharArray)>0 && !blEndVoiceCmdProcessing)
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

						cmdRobotCore.sendCmdStart(CmdRobotStopMoving);

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
#pragma region ESPLORA
				/*
	static THD_WORKING_AREA(waThreadEsplora, 400);
	static THD_FUNCTION(ThreadEsplora, arg) {
		const int FWDIST = 10;
		int alfa = 0;
		int cmDone = 0; // percorso eseguito a valle di un comando moveCm o RotateDeg
		int stuckCount = 0;

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

			// invia i dati Sonar
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
				if (stuckCount>2)
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
		//  return 0;
	}
	*/

#pragma endregion
	//////////////////////////////////////////////////////////////////////////////////
// COMMAND MANAGER ( COMANDI DA SERIALE O BLUETOOTH INTERFACE  )    ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

/*
formato comandi vocali: *comando#  Es.  *ROBOT AVANTI 20#
formato comdMessenger:  [cmdId],[parametri];  o [cmd];


*/ 
#pragma region Processo: comandi

static THD_WORKING_AREA(waComandiBT, 64);
static THD_FUNCTION(thdComandiBT, arg) {
	dbg("S")
	// inizializza il buffer di caratteri
	char chSerialBTRxBuffer[SERIAL_RX_BUFFER_SIZE];
	for (size_t i = 0; i < SERIAL_RX_BUFFER_SIZE; i++) { chSerialBTRxBuffer[i] = 0; }
	
 	// Setup CommandMessenger -----------------------------------------------------
	cmdBT.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdBT);// Attach my application's user-defined callback methods

	while (1) {

		// LED BLUE.
		tft.fillCircle(LCD_LED_SERIALCORE_POS_X+ LCD_LED_HALF_SIZE, LCD_LED_SERIALCORE_POS_Y, LCD_LED_HALF_SIZE, VGA_BLUE);


		//dbg("B>");
		//playSingleNote(NOTE_D8, 90);

		while (SERIAL_BT.available())
		{
			playSingleNote(3000, 40);//bip acuto che segnala la ricezione di caratteri sulla seriale

			// metto  il contenuto della seriale in ub Buffer
			int bytesAvailable = min(SERIAL_BT.available(), SERIAL_RX_BUFFER_SIZE);
			SERIAL_BT.readBytes(chSerialBTRxBuffer, bytesAvailable);

			//ECHO su seriale verso PC (per test)
			for (int byteNo = 0; byteNo < bytesAvailable; byteNo++)
			{
				SERIAL_BT.print(chSerialBTRxBuffer[byteNo]);
				tft.print(chSerialBTRxBuffer, 0, 400);
			}


			// come cmdRobotCore.feedinSerialData() ma prende i dati da serialRxBuffer
			cmdBT.feedinSerialDataFromBuffer(chSerialBTRxBuffer, bytesAvailable);;

			cmdBT.feedinSerialData();
		}

		chThdSleepMilliseconds(1000);//	chThdYield();



/*

		// attendo il testo sulla seriale
		while (!SERIAL_ROBOT.available()) { chThdSleepMilliseconds(50); }	// dbg(".") chThdYield();delay(50);dbg('.')

		// get object from memory pool
		PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPool);
		if (!p) { Serial.println("chPoolAlloc failed");	while (1); }


		dbg("[")
		#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

			//noInterrupts();
			int i = 0; char c = '\0';
			while (SERIAL_ROBOT.available() > 0 && i < INPUTCHARARRAYSIZE && c != '#')
			{
				// mette i dati nella FIFO------------------
				char c = SERIAL_ROBOT.read();
				//p->str[i] = c;
				chSerialRxBuffer[i] = c;
				dbg(chSerialRxBuffer[i])
					switch (c)
					{
					case '*': break;//salto il primo carattere * e non incremento i
					case '#': //ultimo carattere, lo sostituisco con fine stringa
						//p->str[i] = '\0'; 
						chSerialRxBuffer[i]='\0'; 
						i++; 
						break; // sostituisco con una pausa
					default:
						i++; 				
						break;
					}

				//dbg(char(str[i])) messa qui fa casino
			}
			//p->size = i;

		#pragma endregion
		dbg("]")

		// pulisce il resto del buffer
		for (size_t j = i; j < INPUTCHARARRAYSIZE; j++) { chSerialRxBuffer[i] = '\0';} // p->str[i] = '\0'; 
		interrupts();
				
		SPEAK(chSerialRxBuffer);// send message

		msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
		if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }
*/
	}

}
#pragma endregion

//////////////////////////////////////////////////////////////////////////////////
//RICEVE DA BT E INTERPRETA I COMANDI VOCALI      ////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region Processo: BtVoiceCommandInterpreter e comando riconosciuto su Voice FiFo
// gestisce i comandi vocali
// Monitora la SERIAL_SPEAK e mette quello che riceve in voiceCommandCharArray[]
// poi elabora il comando
static THD_WORKING_AREA(waBtCommands, 200);//was 64
static THD_FUNCTION(thdBtCommands, arg) {
	dbg("VC")
//	int i = 0;

	// inizializza il buffer d caratteri
	for (int i = 0; i < VOICECOMMANDMAXLEN; i++) { voiceCommandCharArray[i] = 0; }

	while (1) {

		// attendo il testo sulla seriale
		while (!SERIAL_BT.available()) {playSingleNote(1500, 40); chThdSleepMilliseconds(1500); }	// dbg(".") chThdYield();delay(50);dbg('.')


		dbg("Something received on BT... ")
		playSingleNote(NOTE_A1, 40);

		#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

		//noInterrupts();
		int i = 0; char c = '\0';
		while (SERIAL_BT.available() > 0 && i < VOICECOMMANDMAXLEN && c != '#')
		{
			// legge il carattere dalla seriale
			char c = SERIAL_BT.read();

				// l'app Android invia * all'inizio e # alla fine del testo
				switch (c)
				{
				case '*': break;//salto il primo carattere * e non incremento i
				case '#': voiceCommandCharArray[i]  = '\0'; i++; break; // sostituisco con una pausa
				default:	
					voiceCommandCharArray[i] = toupper(c);
					//if ((c >= 'a') && (c <= 'z'))
					//	voiceCommandCharArray[i] =c + ( 'A' - 'a');
					 
					i++; 				
					break;
				}

		}

		#pragma endregion

		// pulisce il resto del buffer
		for (size_t j = i; j < VOICECOMMANDMAXLEN; j++) {	voiceCommandCharArray[i] = '\0' ;	}
		
		SPEAK(" e ");	//prova se funziona la voce

		// elaboro il contenuto della stringa voiceCommandCharArray
		processVoiceCommand();


		chThdSleepMilliseconds(500);//	chThdYield();
	}

#pragma region cmdWiFi
	/*
	// da tenere allineata con robot.cs >  OnkbGetSensorsHighRate(ReceivedCommand arguments)
	cmdWiFi.sendCmdStart(kbGetSensorsHRate);

	cmdWiFi.sendCmdArg(robot.posCurrent.x);	//robot position X
	cmdWiFi.sendCmdArg(robot.posCurrent.y);	//robot position y
	cmdWiFi.sendCmdArg(robot.posCurrent.r);	//robot position alfa gradi

	cmdWiFi.sendCmdArg(robot.status.irproxy.fw);	// IR proxy
	cmdWiFi.sendCmdArg(robot.status.irproxy.fwHL);	// IR proxy
	cmdWiFi.sendCmdArg(robot.status.irproxy.bk);	// IR proxy
	cmdWiFi.sendCmdArg(robot.status.pirDome);		// movimento

	cmdWiFi.sendCmdArg(robot.status.analog[0]);	//pot
	cmdWiFi.sendCmdArg(robot.status.analog[1]);	//batteria
	cmdWiFi.sendCmdArg(robot.status.analog[2]);	//light

	cmdWiFi.sendCmdArg(robot.getReleStatus(0));		//rele 1
	cmdWiFi.sendCmdArg(robot.getReleStatus(1));		//rel2

	cmdWiFi.sendCmdArg(digitalReadFast(Pin_MotENR));	//status motori
	cmdWiFi.sendCmdArg(digitalReadFast(Pin_MotENL));

	cmdWiFi.sendCmdArg(robot.status.switchTop); // status switch modo Autonomo/slave
	cmdWiFi.sendCmdArg(robot.status.laserOn); // laser
	cmdWiFi.sendCmdArg(robot.readBattChargeLevel());


	cmdWiFi.sendCmdArg(robot.status.gps.sats);		//gps
	cmdWiFi.sendCmdArg(robot.status.gps.lat);
	cmdWiFi.sendCmdArg(robot.status.gps.lng);



	cmdWiFi.sendCmdEnd();
	*/

#pragma endregion

	/*

	// attendo il testo sulla seriale
	while (!SERIAL_ROBOT.available()) { chThdSleepMilliseconds(50); }	// dbg(".") chThdYield();delay(50);dbg('.')

	// get object from memory pool
	PoolObject_t* p = (PoolObject_t*)chPoolAlloc(&memPool);
	if (!p) { Serial.println("chPoolAlloc failed");	while (1); }


	dbg("[")
	#pragma region [Esegue il parsing dei caratteri in ingresso alla seriale]

	//noInterrupts();
	int i = 0; char c = '\0';
	while (SERIAL_ROBOT.available() > 0 && i < INPUTCHARARRAYSIZE && c != '#')
	{
	// mette i dati nella FIFO------------------
	char c = SERIAL_ROBOT.read();
	//p->str[i] = c;
	chSerialRxBuffer[i] = c;
	dbg(chSerialRxBuffer[i])
	switch (c)
	{
	case '*': break;//salto il primo carattere * e non incremento i
	case '#': //ultimo carattere, lo sostituisco con fine stringa
	//p->str[i] = '\0';
	chSerialRxBuffer[i]='\0';
	i++;
	break; // sostituisco con una pausa
	default:
	i++;
	break;
	}

	//dbg(char(str[i])) messa qui fa casino
	}
	//p->size = i;

	#pragma endregion
	dbg("]")

	// pulisce il resto del buffer
	for (size_t j = i; j < INPUTCHARARRAYSIZE; j++) { chSerialRxBuffer[i] = '\0';} // p->str[i] = '\0';
	interrupts();

	SPEAK(chSerialRxBuffer);// send message

	msg_t s = chMBPost(&mailVoice, (msg_t)p, TIME_IMMEDIATE);
	if (s != MSG_OK) { Serial.println("chMBPost failed");	while (1); }
	*/

}
#pragma endregion

//////////////////////////////////////////////////////////////////////////////////
// SERIAL MANAGER ( ROBOT INTERFACE  )    ////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
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
	dbg("c>")
	// inizializza il buffer di caratteri
	char chSerialRxBuffer[INPUTCHARARRAYSIZE];
	for (size_t i = 0; i < INPUTCHARARRAYSIZE; i++) { chSerialRxBuffer[i] = 0; }
	
	// Setup CommandMessenger -----------------------------------------------------
	cmdRobotCore.printLfCr();   // Adds newline to every command 
	attachCommandCallbacks(&cmdRobotCore);// Attach my application's user-defined callback methods

	while (1) {

		// LED ROSSO.
		tft.fillCircle(LCD_LED_SERIALCORE_POS_X, LCD_LED_SERIALCORE_POS_Y, LCD_LED_HALF_SIZE, VGA_RED);

		//robotModel.status.sensors.bumper.center = !robotModel.status.sensors.bumper.center;
		//robotModel.status.sensors.analog[0] = random(0, 1023);
		//robotModel.status.sensors.analog[1] = random(0, 1023);
		//dbg("c>");
		//playSingleNote(NOTE_D8, 90);



		while (SERIAL_ROBOT.available())
		{
			playSingleNote(NOTE_A7, 40);//bip acuto che segnala la ricezione di caratteri sulla seriale
			cmdRobotCore.feedinSerialData();
		}


		chThdSleepMilliseconds(1000);//	chThdYield();
	}

}
#pragma endregion




//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// Procedure grafiche 
//////////////////////////////////////////////////////////////////////////////////
#pragma region Procedure grafiche
	//#define tftLEDON(x,y,c) tft.fillCircle(x, y, LCD_LED_HALF_SIZE, c)
	#define tftLEDON(x,y,c) tft.setColor(c); tft.fillCircle(x, y, LCD_LED_HALF_SIZE, c)

	void drawFilledRect(int left, int top, int sizeX, int sizeY, int col = VGA_BLACK) {
	#ifdef TFT_ILI9327_8
		// coordinate assolute
		tft.fillRect(left, top, left + sizeX, top + sizeY, col);
	#endif
	#ifdef TFT_ili9488
		// coordinate assolute
		tft.fillRect(left, top, left + sizeX, top + sizeY, col);
	#endif
	#ifdef TFT_ili9341
		// coordinate relative
		tft.fillRect(left, top, sizeX, sizeY, col);
	#endif // 0
	}
	// disegna un rettangolo 
	void drawLedRect(int left, int top, bool value, int ledColorOn = VGA_RED) {
	#define LEDSIZE 6
	#define LEDCOLOR_OFF VGA_BLACK
		if (value)
		{
			drawFilledRect(left, top, LEDSIZE, LEDSIZE, ledColorOn);
		}
		else
		{
			drawFilledRect(left, top, LEDSIZE, LEDSIZE, LEDCOLOR_OFF);
		}
	}
	// disegna un rettangolo e lo riempie in proporzione al valore 
	void drawGaugeHoriz(int left, int top, int width, int height, int value, int min, int max, int valueColor) {
	#define GAUGE_COLOR_BCKGROUND VGA_BLACK
		int v;
		v = map(value, min, max, 0, width); //map(value, fromLow, fromHigh, toLow, toHigh).
											//tft.drawFilledRect(left, top,   width, height, BLUE);
		drawFilledRect(left, top, v, height, valueColor);
		drawFilledRect(left + v, top, width - v, height, GAUGE_COLOR_BCKGROUND);
		tft.setColor(valueColor);
		tft.drawVLine(left + width, top, height);
	}
	void drawGauge(int left, int top, int v, int color, int minValue = 0, int maxValue = 1023) {
	#define GAUGE_W 100
	#define GAUGE_H 10
		drawGaugeHoriz(left, top, GAUGE_W, GAUGE_H, v, minValue, maxValue, color);
	}
	void drawLinePolar(uint16_t x1, uint16_t y1, double alfaDeg, uint16_t lenght, uint16_t color) {
		int x2;
		int y2;
		x2 = x1 + lenght*cos(alfaDeg);
		y2 = y1 + lenght*sin(alfaDeg);
		tft.setColor(color);
		tft.drawPixel( x2, y2);
	}
	void drawPixelPolar(uint16_t x1, uint16_t y1, double alfaDeg, uint16_t lenght, uint16_t color) {
		int x2;
		int y2;
		x2 = x1 + lenght*cos(alfaDeg);
		y2 = y1 + lenght*sin(alfaDeg);
		//tft.setColor(VGA_BLACK);
		//tft.drawLine(x1, y1, x2, y2);
		tft.setColor(color);
		tft.drawPixel(x2, y2);
	}
	void drawEchoPolar(uint16_t x1, uint16_t y1, double alfaDeg, uint16_t range, uint16_t color) {
	#define maxRangePixel 50 // raggio in pixel corrispondenti alla distanza massima
		int x2;
		int y2;
		tft.setColor(VGA_BLACK);
		tft.drawLine(x1, y1, x1 + maxRangePixel*cos(alfaDeg), y1 + maxRangePixel*sin(alfaDeg));
		uint16_t l = map(range, 0, 1023, 1, maxRangePixel);
		x2 = x1 + l*cos(alfaDeg);  		
		y2 = y1 + l*sin(alfaDeg);
		tft.setColor(color);
		tft.drawPixel(x2, y2);
	}
#pragma endregion
//////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////
//  THREAD  R O T A R Y  E N C O D E R       ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region  Processo GESTIONE ROTARY ENCODER  
	static THD_WORKING_AREA(waRotaryEncoder, 164);
	static THD_FUNCTION(RotaryEncoder, arg) {
		while (1) {
			loadEncoderPositionOnChange();
			if (encoderPositionUpdated()) {
				if (encoder_delta>0)
				{
					playSingleNote(NOTE_A5, 40);
				}
				else
				{
					playSingleNote(NOTE_C4, 40);
				}
				//printEncoderInfo();
				robotModel.status.sensors.analog[Pin_AnaPot1 - Pin_AnaBase]= current_encoder_position;
			}
			if (!digitalRead(Pin_ROT_ENCODER_SWITCH))//è a logica negata!
			{
				playSingleNote(100, 40);
				current_encoder_position = 0;
				robotModel.status.sensors.analog[Pin_AnaPot1 - Pin_AnaBase] = current_encoder_position;
			}
			//digitalWrite(PIN_LED, 1);
			//chThdSleepMilliseconds(200 + current_encoder_position);
			//digitalWrite(PIN_LED, 0);// Turn LED on.
			//chThdSleepMilliseconds(200 + current_encoder_position);
			chThdSleepMilliseconds(300);//	chThdYield();//	

		}
	}
#pragma endregion 

//////////////////////////////////////////////////////////////////////////////////
//  THREAD  B R A I N      ///////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region  Processo	B R A I N    
	static THD_WORKING_AREA(waBrain, 100);
	static THD_FUNCTION(thdBrain, arg) {
		while (1) {


 			chThdSleepMilliseconds(500);//	chThdYield();//	

		}
	}
#pragma endregion 
	//////////////////////////////////////////////////////////////////////////////////
	//  THREAD  S P E E C H		//////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
#pragma region  Processo	 S P E E C H 
	// invia parole alla seriale del modulo Speech in base ai flag impostati o allo stato di RobotModel
	static THD_WORKING_AREA(waSpeech, 100);
	static THD_FUNCTION(thdSpeech, arg) {
		SPEAK(" VOCE okei ");	//prova se funziona la voce

		while (1) {
			SPEAK("A");

			#pragma region HUMAN DETECTION
				//-------------------------------------------------------------------
				// chiedo chi sei solo se è attivo il pir da meno di un secondo
				if ((robotModel.statusOld.sensors.pirDome != robotModel.status.sensors.pirDome)
					&& (robotModel.status.ts - robotModel.statusOld.ts > 1000))
				{
					SPEAK_CIAOCHISEI
						//resettto lo stato
						robotModel.statusOld.sensors.pirDome = false;
				}
			#pragma endregion

			#pragma region Gestione cambio modalità
				// Gestione cambio modalità-----------------------------------------
				// Switch TOP commutato da oltre un secondo?
				if ((robotModel.status.sensors.switchTop != robotModel.statusOld.sensors.switchTop)
					&& (robotModel.status.ts - robotModel.statusOld.ts > 1000)) {
					if (robotModel.status.sensors.switchTop)	//AUTONOMO?
					{


						SPEAK(" okei okei esploro");
					}
					else // SLAVE
					{

						SPEAK(" okei comanda");
					}
				}
			#pragma endregion

			chThdSleepMilliseconds(1500);//	chThdYield();//	

		}
	}
#pragma endregion 

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//  THREAD  T F T  M O N I T O R									/////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// Input: RobotStatus
// Output: TFT
#pragma region Processo di gestione T F T   M O N I T O R
	// x lato corto
	#define TFT_ROWSPACING 20	//Distanza in pixel tra due righe di testo
	#define TFTROW(r) r*TFT_ROWSPACING	// asse y
	#define TFTCAPTIONCOL 5 // colonna delle etichette
	#define TFTDATACOL 100 // colonna dei dati
	#define SONAR_RANGE_SIZE 50 // raggio in pixel della rappresentazione dei valori sonar
	static THD_WORKING_AREA(waThreadTFT, 100);
	static THD_FUNCTION(thdTFT, arg) {


	//char* tftMsg; //= "XYZ";
	String	sTFT; //stringa di appoggio usata dalla macro TFTprintAtStr
	// Caption fisse
	uint8_t r = 1;
	drawLedRect(TFTDATACOL, TFTROW(7),1);
	bool HbLed = 0; //stato del led che visualizza l'ttivit� di questo Thread
	tft.setColor(VGA_WHITE);

	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "Op Mode:");
	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "Free Ram:");

	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "LT,LNG,Sat:");
	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "Pot:");
	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "Vbat:");
	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "Light:");
	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "Motion det:");
	TFTprintAtStr(TFTCAPTIONCOL, TFTROW(r++), "Command:");

	//	chThdSleepMilliseconds(1);//	chThdYield();//	

	//drawButtons();
	while (true)// loop di visualizzazione dati
	{
		// RESETTA I LED DEI PROCESSI ATTIVI
		tft.setXY(0,0, tft.disp_x_size, 20);
		//tft.fillCircle(LCD_LED_SPEAK_POS_X, LCD_LED_SPEAK_POS_Y, LCD_LED_HALF_SIZE, VGA_BLACK);
		//tft.fillCircle(LCD_LED_SERIALCORE_POS_X, LCD_LED_SERIALCORE_POS_Y, LCD_LED_HALF_SIZE, VGA_BLACK);
		//tft.fillCircle(LCD_LED_SERIALCORE_POS_X+50, LCD_LED_SPEAK_POS_Y, LCD_LED_HALF_SIZE, VGA_BLACK);

		//Serial.print(chUnusedStack(waThreadTFT, sizeof(waThreadTFT)));
		HbLed = !HbLed;	//Heartbeat Led
		drawLedRect(TFTDATACOL, 10, robotModel.status.tictac, VGA_LIME);

		
 

 

 
		//cancella  l'area dati
		tft.setXY(TFTDATACOL, 21, tft.disp_x_size, tft.disp_y_size);
		r = 1;



		/// Visualizza la memoria libera---------------------------------------------------
		//tft.setColor(VGA_GRAY);
		TFTprintAtNumI(TFTDATACOL, TFTROW(r++), getFreeSram(),VGA_GRAY);

		/// Visualizza dati GPS---------------------------------------------------
		TFTprintAtNumF(TFTDATACOL, TFTROW(r), robotModel.status.sensors.gps.lat,8, VGA_BLUE);
		TFTprintAtNumF(TFTDATACOL+100, TFTROW(r), robotModel.status.sensors.gps.lng, 8, VGA_BLUE);
		TFTprintAtNumI(TFTDATACOL+200, TFTROW(r++), robotModel.status.sensors.gps.sats, VGA_BLUE);


 		
		// Ingressi Analogici ----------------------------------------------------
 		
		TFTprintAtNumI(TFTDATACOL, TFTROW(r), robotModel.status.sensors.analog[0], VGA_GREEN);
		drawGauge(TFTDATACOL+30, TFTROW(r++), robotModel.status.sensors.analog[0], VGA_GREEN);
		
		TFTprintAtNumI(TFTDATACOL, TFTROW(r), robotModel.status.sensors.analog[1], VGA_LIME);
		drawGauge(TFTDATACOL+30, TFTROW(r++), robotModel.status.sensors.analog[1], VGA_LIME);
		
		TFTprintAtNumI(TFTDATACOL, TFTROW(r), robotModel.status.sensors.analog[2], VGA_YELLOW);
		drawGauge(TFTDATACOL+30, TFTROW(r++), robotModel.status.sensors.analog[2], VGA_YELLOW);
		
 		drawLedRect(TFTDATACOL+30, TFTROW(r++), robotModel.status.sensors.pirDome, VGA_RED);
		
		
		
		/// Visualizza la modalità operativa --------------------------------------------------
		tft.setColor(VGA_RED);



		if (robotModel.status.operatingMode == 1)
		{
			TFTprintAtStr(TFTDATACOL, TFTROW(r++), "S L A V E ");

		}
		else
		{
			TFTprintAtStr(TFTDATACOL, TFTROW(r++), "AUTONOMOUS");

		}



		//tftMsg =robotModel.getOperatingModeChar();
		//TFTprintAtStr( TFTDATACOL, TFTROW(r++), tftMsg);
		//TFTprintAtStr( TFTDATACOL, TFTROW(r++), tftMsg);

		// Visualizza l'ultimo comando via seriale ---------------------------------------------------
		tft.setColor(VGA_WHITE);

		
		//TFTprintAtStr(TFTDATACOL, TFTROW(r++), *serialRxBuffer);
		while ((rxBuf.remain()>0 ) && (TFTROW(r)< (TFT_Y_HEIGHT -TFT_ROWSPACING)))
		{
			sTFT = rxBuf.pop();
			TFTprintAtStr(TFTDATACOL, TFTROW(r++), sTFT);
		}

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

		
			
		chThdSleepMilliseconds(1000);//chThdYield();
	}

}

#pragma endregion 

// FINE PROCESSI CHIBIOS ////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//  THREAD  N O N  A T T I V I  									/////////////
/////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
#pragma region THREAD NON ATTIVI
#if 0


//////////////////////////////////////////////////////////////////////////////////
//  THREAD  B L I N K I N G  L E D									/////////////
// ///////////////////////////////////////////////////////////////////////////////
#pragma region // BLINK LED
#define PIN_LED  Pin_ONBOARD_LED

// 64 byte stack beyond task switch and interrupt needs
static THD_WORKING_AREA(waFlashLed, 64);
static THD_FUNCTION(FlashLed, arg) {
	// Flash led every 200 ms.
	pinMode(PIN_LED, OUTPUT);		digitalWrite(PIN_LED, 0);	// led superiore

	while (1) {

		// Turn BOARD LED on.
		digitalWrite(PIN_LED, HIGH);
		// Turn GREEN LED on.
		tft.fillCircle(LCD_LED_POS_X, LCD_LED_POS_Y, LCD_LED_HALF_SIZE, VGA_BLACK);

		// Sleep for.. milliseconds.
		chThdSleepMilliseconds(900);

		// Turn BOARD LED off.
		digitalWrite(PIN_LED, LOW);
		// Turn GREEN LED off.
		tft.fillCircle(LCD_LED_POS_X, LCD_LED_POS_Y, LCD_LED_HALF_SIZE, VGA_GREEN);

		// Sleep for ... milliseconds.
		chThdSleepMilliseconds(900);

		//speakString("oi")  //output diretto su seriale
		//SPEAK("ei");	// output via processo FifoToSPEAK
	}
}
#pragma endregion // BLINK LED----------------------------------------------------


//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
// thread 4 - ROS SERIAL
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
/*
ros::Time time;

ros::NodeHandle  nh;

sensor_msgs::Range range_msg;
ros::Publisher pub_range("/ultrasound", &range_msg);

const int adc_pin = 0;

unsigned char frameid[] = "/ultrasound";



long range_time;

static THD_WORKING_AREA(waThreadROS, 200);
static THD_FUNCTION(ThreadROS, arg) {

// rosserial Ultrasound Example
//
// This example is for the Maxbotix Ultrasound rangers.



nh.initNode();
nh.advertise(pub_range);


range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
range_msg.header.frame_id = frameid;
range_msg.field_of_view = 0.1;  // fake
range_msg.min_range = 0.0;
range_msg.max_range = 6.47;

///pinMode(8, OUTPUT);
///digitalWrite(8, LOW);

//loop ---------------------------------------------------------------
while (1) {

//publish the adc value every 50 milliseconds
//since it takes that long for the sensor to stablize
int r = 0;

range_msg.range = (float)robotModel.getLaserDistance();
range_msg.header.stamp = range_msg.header.stamp.now();
pub_range.publish(&range_msg);
range_time = millis() + 50;

nh.spinOnce();

chThdSleepMilliseconds(50);// Sleep for n milliseconds.
}
//	return 0;
}
*/

	//////////////////////////////////////////////////////////////////////////////////
	//  THREAD ECHO SERIAL_ROBOT > SERIAL_PC       ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	// FOR TEST ONLY
	#pragma region  ECHO SERIAL_ROBOT > SERIAL_PC 
		// 64 byte stack beyond task switch and interrupt needs
		static THD_WORKING_AREA(waserialEcho, 64);
		static THD_FUNCTION(serialEcho, arg) {
			int inByte=0;
			//sdStart(&ch1, NULL);
			while (1) {
				tft.fillCircle(150, LCD_LED_SPEAK_POS_Y, LCD_LED_HALF_SIZE, VGA_LIME);

				if (SERIAL_ROBOT.available() > 0) {
					playSingleNote(NOTE_A6, 40);
					inByte = SERIAL_ROBOT.read();
					SERIAL_PC.write(inByte);
				}
				

			
				chThdSleepMilliseconds(50);//	chThdYield();//	
			}
		}
	#pragma endregion 


		// ///////////////////////////////////////////////////////////////////////////////
		//  THREAD TFT KEYBOARD       ///////////////////////////////////////////////////////////
		// ///////////////////////////////////////////////////////////////////////////////
	#pragma region //TFT KEYBOARD
		/*************************
		**   Custom functions   **
		*************************/
		int x, y;
		uint16_t xRaw;
		uint16_t yRaw;

		// Buffer input from TFT keyboard
		char stCurrent[20] = "";
		int stCurrentLen = 0;
		char stLast[20] = "";
	#define dispy 480
	#define dispx 320
	#define BUTT_W 60
	#define BUTT_H 26
	#define BUTT2_W 120
	#define BUTT2_H 28
		// R1,2,3 posizione y della mezzeria dei Buttons
	#define R0 340	// riga del buffer keyboard
	#define R1 380		///WAS 10
	#define R2 420	/// (R1 + BUTT_H +2)
	#define R3 460		/// (R1 + 2*BUTT_H +2)
	#define C1 32
	#define C2 dispx/2
	#define C3 dispx-11
	#define C_B1 80		//clear
	#define C_B2 250	//enter
	//class TS_Point {
	//public:
	//	TS_Point(void) : x(0), y(0), z(0) {}
	//	TS_Point(int16_t x, int16_t y, int16_t z) : x(x), y(y), z(z) {}
	//	bool operator==(TS_Point p) { return ((p.x == x) && (p.y == y) && (p.z == z)); }
	//	bool operator!=(TS_Point p) { return ((p.x != x) || (p.y != y) || (p.z != z)); }
	//	uint16_t x, y, z;
	//};

		void drawButtons()
		{
			// Draw the upper row of buttons
			for (x = 0; x < 5; x++)
			{
				tft.setColor(0, 0, 255);
				tft.fillRoundRect(C1 + (x *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (x * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
				tft.setColor(255, 255, 255);
				tft.drawRoundRect(C1 + (x * (BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (x * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
				tft.printNumI(x + 1, 10 + C1 + (x * (BUTT_W)) - BUTT_W / 2, R1 - 5);
			}
			// Draw the center row of buttons
			for (x = 0; x < 5; x++)
			{
				tft.setColor(0, 0, 255);
				tft.fillRoundRect(C1 + (x *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (x * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
				tft.setColor(255, 255, 255);
				tft.drawRoundRect(C1 + (x * (BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (x * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
				tft.printNumI(x + 6, 10 + C1 + (x * (BUTT_W)) - BUTT_W / 2, R2 - 5);

				//if (x<4)
				//   tft.printNumI(x+6, 27+(x*60), 87);
			}
			//tft.print("0", 267, 87);


			// Draw the lower row of buttons CLEAR & ENTER
			tft.setColor(0, 0, 255);
			tft.fillRoundRect(C_B1 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B1 + BUTT2_W / 2, R3 + BUTT2_H / 2);
			tft.setColor(255, 255, 255);
			tft.drawRoundRect(C_B1 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B1 + BUTT2_W / 2, R3 + BUTT2_H / 2);
			tft.print("Clear", C_B1 - BUTT2_W / 2 + 4, R3 - 5);

			tft.setColor(0, 0, 255);
			tft.fillRoundRect(C_B2 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B2 + BUTT2_W / 2, R3 + BUTT2_H / 2);
			tft.setColor(255, 255, 255);
			tft.drawRoundRect(C_B2 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B2 + BUTT2_W / 2, R3 + BUTT2_H / 2);
			tft.print("Enter", C_B2 - BUTT2_W / 2 + 4, R3 - 5);
			tft.setBackColor(0, 0, 0);
		}

		void updateStr(int val)
		{
			if (stCurrentLen < 20)
			{
				stCurrent[stCurrentLen] = val;
				stCurrent[stCurrentLen + 1] = '\0';
				stCurrentLen++;
				tft.setColor(VGA_LIME);
				TFTprintAtStr(C1, R0, stCurrent);
			}
			else
			{
				tft.setColor(VGA_RED);
				TFTprintAtStr(C1, R0, "BUFFER FULL!");
				delay(500);
				TFTprintAtStr(C1, R0, "            ");
				delay(500);
				TFTprintAtStr(C1, R0, "BUFFER FULL!");
				delay(500);
				TFTprintAtStr(C1, R0, "            ");
				//tft.setColor(0, 255, 0);
			}
		}

		// Draw a red frame while a button is touched
		void HighLightButtonUntilRelease(int x1, int y1, int x2, int y2)
		{
			tft.setColor(255, 0, 0);
			tft.drawRoundRect(x1, y1, x2, y2);
			///while (ts.dataAvailable())
			while (ts.isTouching()) {
				chThdSleepMilliseconds(10);//	chThdYield();//	

			};//attende il rilascio del pulsante
									//	  ts.read();
			tft.setColor(255, 255, 255);
			tft.drawRoundRect(x1, y1, x2, y2);
		}

		void buzz(int targetPin, long frequency, long length) {
			//digitalWrite(13, HIGH);
			long delayValue = 1000000 / frequency / 2; // calculate the delay value between transitions
													   //// 1 second's worth of microseconds, divided by the frequency, then split in half since
													   //// there are two phases to each cycle
			long numCycles = frequency * length / 1000; // calculate the number of cycles for proper timing
														//// multiply frequency, which is really cycles per second, by the number of seconds to
														//// get the total number of cycles to produce
			for (long i = 0; i < numCycles; i++) { // for the calculated length of time...
				digitalWrite(targetPin, HIGH); // write the buzzer pin high to push out the diaphram
				chThdSleepMicroseconds(delayValue);//	chThdYield();//	

				//delayMicroseconds(delayValue); // wait for the calculated delay value
				digitalWrite(targetPin, LOW); // write the buzzer pin low to pull back the diaphram
				//delayMicroseconds(delayValue); // wait again or the calculated delay value
				chThdSleepMicroseconds(delayValue);//	chThdYield();//	

			}
			digitalWrite(targetPin, LOW);

		}

		// 64 byte stack beyond task switch and interrupt needs
		static THD_WORKING_AREA(waTftKeyboard, 64);
		static THD_FUNCTION(TftKeyboard, arg) {
			// Touch Screen initialization---------------------------
			//ts.InitTouch(PORTRAIT);  // include gi� ts.begin();
			ts.begin(320, 480);
			ts.setRotation(XPT2046::ROT0);
			//   ts.setCalibration(209, 1759, 1775, 273);
	#define PIN_BUZZER 12
			pinMode(PIN_BUZZER, OUTPUT);//buzzer

			while (1) {
				if (ts.isTouching())
				{
					buzz(PIN_BUZZER, 250, 100);

					TS_Point p;
					ts.getPosition(p.x, p.y, XPT2046::MODE_DFR, 20);
					ts.getRaw(xRaw, yRaw, XPT2046::MODE_DFR, 20);
					x = p.x;
					y = p.y;


					// visualizzo il punto----
					tft.setColor(VGA_RED);
					tft.drawPixel(x, y);
					//------------------------------


					//// stampo le coordinate x,y --------
					//tft.setColor(VGA_LIME);
					//tft.printNumI(x, 160, 20, 6);
					//tft.printNumI(y, 160, 40, 6);


					//// stampo i dati grezzi ----------
					//tft.setColor(VGA_BLUE);
					//tft.printNumI(xRaw, 160, 70, 6);
					//tft.printNumI(yRaw, 160, 90, 6);
					//tft.printNumI(ts.getZ(), 160, 110,6);


	#if 1
	#pragma region individuazione del pulsante premuto
					int n = 0;
					if ((y >= R1 - BUTT_H / 2) && (y <= R1 + BUTT_H / 2))  // Upper row
					{
						n = 0;
						if ((x >= C1 - BUTT_W / 2) && (x <= C1 + BUTT_W / 2)) // Button: 1
						{
							buzz(PIN_BUZZER, 210, 100);
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
							updateStr('1');
						}
						n = 1;
						if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 2
						{
							buzz(PIN_BUZZER, 220, 100);

							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
							updateStr('2');
						}


						n = 2;
						if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 3
						{
							buzz(PIN_BUZZER, 330, 100);

							HighLightButtonUntilRelease(C1 + (n *BUTT_W) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
							updateStr('3');
						}


						n = 3;
						if ((x > (C1 + (n *BUTT_W) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 4
						{
							buzz(PIN_BUZZER, 350, 100);
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
							updateStr('4');
						}


						n = 4;
						if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 5
						{
							buzz(PIN_BUZZER, 360, 100);
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R1 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R1 + BUTT_H / 2);
							updateStr('5');
						}
					}
					/// PULSANTI DA 6 A 0
					if ((y >= R2 - BUTT_H / 2) && (y <= R2 + BUTT_H / 2))  // Center row
					{
						n = 0;
						if ((x >= C1 - BUTT_W / 2) && (x <= C1 + BUTT_W / 2)) // Button: 6
						{
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
							updateStr('6');
						}
						n = 1;
						if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 7
						{
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
							updateStr('7');
						}


						n = 2;
						if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 8
						{
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
							updateStr('8');
						}


						n = 3;
						if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button:9
						{
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
							updateStr('9');
						}


						n = 4;
						if ((x > (C1 + (n *(BUTT_W)) - BUTT_W / 2)) && (x <= C1 + (n * BUTT_W) + BUTT_W / 2))  // Button: 0
						{
							HighLightButtonUntilRelease(C1 + (n *(BUTT_W)) - BUTT_W / 2, R2 - BUTT_H / 2, C1 + (n * BUTT_W) + BUTT_W / 2, R2 + BUTT_H / 2);
							updateStr('0');
						}
					}
					/// PULSANTI CLEAR E ENTER -----------------------------------------
					if ((y >= R3 - BUTT2_H / 2) && (y <= R3 + BUTT2_H / 2))  // Upper row
					{
						if ((x >= C_B1 - BUTT2_W / 2) && (x <= C_B1 + BUTT2_W / 2))  // Button: Clear
						{
							// HighLightButtonUntilRelease(10, 130, 150, 180);
							HighLightButtonUntilRelease(C_B1 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B1 + BUTT2_W / 2, R3 - BUTT2_H / 2);
							stCurrent[0] = '\0';
							stCurrentLen = 0;

							// esegue un clear dell'area del display
							tft.setColor(0, 0, 0);
							tft.fillRect(0, 224, 320, R1 - BUTT_H);
						}
						if ((x >= C_B2 - BUTT2_W / 2) && (x <= C_B2 + BUTT2_W / 2))  // Button: Enter
						{
							//HighLightButtonUntilRelease(160, 130, 300, 180);
							HighLightButtonUntilRelease(C_B2 - BUTT2_W / 2, R3 - BUTT2_H / 2, C_B2 + BUTT2_W / 2, R3 - BUTT2_H / 2);
							if (stCurrentLen > 0)
							{
								for (x = 0; x < stCurrentLen + 1; x++)
								{
									stLast[x] = stCurrent[x];
								}
								stCurrent[0] = '\0';
								stCurrentLen = 0;
								tft.setColor(0, 0, 0);
								tft.fillRect(0, 208, 319, 239);
								tft.setColor(0, 255, 0);
								tft.print(stLast, LEFT, 208);
							}
							else
							{
								tft.setColor(255, 0, 0);
								tft.print("BUFFER EMPTY", CENTER, 192);
								delay(500);
								tft.print("            ", CENTER, 192);
								delay(500);
								tft.print("BUFFER EMPTY", CENTER, 192);
								delay(500);
								tft.print("            ", CENTER, 192);
								tft.setColor(0, 255, 0);
							}
						}
					}

	#pragma endregion


	#endif // 0

				}

				chThdSleepMilliseconds(100);//	chThdYield();//	

			}
		}
	#pragma endregion // TFT KEYBOARD----------------------------------------------------


		//////////////////////////////////////////////////////////////////////////////////
		// THREAD FiFo ->  SPEAK     ///////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////////
	#pragma region Processo: FiFo -> SPEAK
		/* FiFo ->  SPEAK non serve più,
		da riutilizzare per processi a bassa priorità Fifo >>Seriale
		// il formato stringhe ricevuto via Blutetooth è *...# dove ... è il testo inviato
		static THD_WORKING_AREA(waFifoToSPEAK, 64);
		static THD_FUNCTION(FifoToSPEAK, arg) {
		SERIAL_SPEAK.begin(SERIAL_SPEAK_BAUD_RATE);
		SERIAL_SPEAK.setTimeout(5000);


		#pragma region  [speakTest Iniziale speak]

		SERIAL_SPEAK.print(" Hello voce okei");


		#if dbg
		speakNumber(1);
		delay(500);
		speakNumber((uint16_t)437);
		delay(500);
		strcpy(SpeakBuffer, "126");
		speakNumberStr(SpeakBuffer);
		delay(500);
		#endif // dbg

		#pragma endregion


		while (1) {
		playSingleNote(NOTE_A7, 40);
		// Turn PROCESS LED on.
		tft.fillCircle(LCD_LED_SPEAK_POS_X, LCD_LED_SPEAK_POS_Y, LCD_LED_HALF_SIZE, VGA_OLIVE);

		PoolObject_t *p;

		// get mailVoice
		chMBFetch(&mailVoice, (msg_t*)&p, TIME_INFINITE);


		// Invia la stringa voce sulla seriale
		SERIAL_SPEAK.print(p->str);


		// put memory back into pool
		chPoolFree(&memPool, p);

		chThdSleepMilliseconds(200);//	chThdYield();//
		}
		}
		*/

	#pragma endregion

	//////////////////////////////////////////////////////////////////////////////////
	// M O N I T O R	   ///////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////
	#pragma region Processo di MONITOR
	static THD_WORKING_AREA(waThreadMonitor, 64);
	static THD_FUNCTION(ThreadMonitor, arg) {
		while (true)
		{
			dbg("M.")
			//Serial.print(F("    waFifoFeed unused stack: "));
			//Serial.println(chUnusedStack(waFifoFeed, sizeof(waFifoFeed)));
			//Serial.print(F("    overrun errors: "));
			//Serial.println( OverrunErrorCount);
			Serial.print(F("Ram: "));
			Serial.println(getFreeSram());

			//count++;
			////FIFO_SPEAK.push(int2str(count));
			//uint32_t t = micros();
			//// yield so other threads can run
			//chThdYield();
			//t = micros() - t;
			//if (t > maxDelay) maxDelay = t;

			chThdSleepMilliseconds(2000);//	chThdYield();//	
		}

	}

	#pragma endregion 
#endif // 0

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

//------------------------------------------------------------------------------
// main thread runs at NORMALPRIO
void chSetup() {
	dbg("Starting all chThreads...")

	// fill pool with PoolObject array
	for (size_t i = 0; i < MB_COUNT; i++) {
		chPoolFree(&memPool, &PoolObject[i]);
	}

	//	chThdCreateStatic(waTftKeyboard, sizeof(waTftKeyboard), NORMALPRIO+3 , TftKeyboard, NULL);
	/*
	thdTFT:  + 2		>> ok
	thdRobotCoreInterface:  + 1
	thdCommands:0 
	FifoToSPEAK= 0		>> NON sembra andare
	-------------------------
	FifoToSPEAK= +3			>> OK
	thdTFT:  + 2		>> ok
	thdRobotCoreInterface:  + 1			>> no +2 OK
	thdCommands:0  
	*/
	chThdCreateStatic(waRobotCoreInterface, sizeof(waRobotCoreInterface), NORMALPRIO + 2, thdRobotCoreInterface, NULL);
	chThdCreateStatic(waComandiBT, sizeof(waComandiBT), NORMALPRIO + 2, thdComandiBT, NULL);
	chThdCreateStatic(waThreadTFT, sizeof(waThreadTFT), NORMALPRIO + 2, thdTFT, NULL);
//	chThdCreateStatic(waBtCommands, sizeof(waBtCommands), NORMALPRIO+2, thdCommands, NULL);
	chThdCreateStatic(waRotaryEncoder, sizeof(waRotaryEncoder), NORMALPRIO + 2, RotaryEncoder, NULL);
	chThdCreateStatic(waSpeech, sizeof(waSpeech), NORMALPRIO + 2, thdSpeech, NULL);
//	chThdCreateStatic(waFlashLed, sizeof(waFlashLed), NORMALPRIO + 4, FlashLed, NULL);
	//chThdCreateStatic(waThreadEsplora, sizeof(waThreadEsplora), NORMALPRIO + 2, ThreadEsplora, NULL);//-Esplora

	//chThdCreateStatic(waserialEcho, sizeof(waserialEcho), NORMALPRIO + 4, serialEcho, NULL);
	//chThdCreateStatic(waFifoToSPEAK, sizeof(waFifoToSPEAK), NORMALPRIO+3, FifoToSPEAK, NULL);
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


			tft.setBackColor(0, 0, 0);
			//---------------------------------------


		#endif
	#pragma endregion

			
	#pragma region Setup Interrupts for Rotary Encoders
		attachInterrupt(digitalPinToInterrupt(Pin_ROT_ENCODER_A), loadEncoderPositionOnChange, CHANGE);
		attachInterrupt(digitalPinToInterrupt(Pin_ROT_ENCODER_B), loadEncoderPositionOnChange, CHANGE);

	#pragma endregion


	dbg("MMI");
	singMyMelody(myMelody1);
	SPEAK_TEST

	chBegin(chSetup);	
}
void loop() {
	// not used
}

#pragma endregion
