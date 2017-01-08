/* Robot Interface Commands */

 // We must define a callback function in our Arduino program for each entry in the list below.
#include <CmdMessenger2/CmdMessenger2.h>
#include <robotModel.h>
#include "Commands_Enum.h"
#include "SpeakSerialInterface.h"
#include <TFT_HAL\TFT_HAL.h>
///extern struct robotModel_c robot;
//Dichiarazione di funzione che punta all'indirizzo zero
void( *reboot )(void) = 0;
#define MESSAGEMAXLEN 50
char strBuff[MESSAGEMAXLEN];

#pragma region FIFO DEI MESSAGGI DA VISUALIZZARE SU TFT
#include <RingBuf/RingBuf.h>

// Fifo definitions-------------------------
#pragma region FiFoSpeech setup
#define MAX_STRING_SIZE_TFTMSG 10 //dimensione di ciascuna stringa voce
#define MAX_BUFFER_SIZE_TFTMSG 10 //quante stringe possono essere in coda
struct myMsgStruct
{
	int index;
	char msgString[MAX_STRING_SIZE_TFTMSG];

	unsigned long long timestamp;
};
//struct myMsgStruct
//{
//		char msgString[MAX_STRING_SIZE_VOICE];
//};


// Create a RinBuf object designed to hold  10 of mystructs
RingBuf *myRingBufTftMsg = RingBuf_new(sizeof(struct myMsgStruct), MAX_BUFFER_SIZE_TFTMSG);

// usati da TFT_PRINT_MSG --------------------
#define MSG_STARTINGROW 12 //riga iniziale dell'area messaggi
#define MSG_ROWS 4			// righe dedicate ai messaggi
static int msgRowCnt = MSG_STARTINGROW; //contatore
//----------------------------------------

#pragma endregion


#pragma endregion

//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////  C A L L B A C K S				//////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
 
//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   I M P O S T A Z I O N E  G E N E R A L E
//////////////////////////////////////////////////////////////////////////
// void cmdMsg(CmdMessenger2 *cmd,const char *msg){
////	cmd->sendCmd( Msg, msg );
// 	//SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
//	//ser.print("1,"); ser.print(msg); ser.print(";");
//	cmd->sendCmd(Msg);
//	cmd->sendCmdArg(msg);
//	cmd->sendCmdEnd();
//}
void cmdMsg(CmdMessenger2 *cmd,const PROGMEM char *stringLiteral){
//	cmd->sendCmd( Msg, msg );
 	//SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
	//ser.print("1,"); ser.print(msg); ser.print(";");
	cmd->sendCmd(Msg);
	cmd->sendCmdArg(stringLiteral);
	cmd->sendCmdEnd();
}
void cmdMsg(CmdMessenger2 *cmd, const char *msg, int v) {
	//	cmd->sendCmd( Msg, msg );
	//SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
	//ser.print("1,"); ser.print(msg); ser.print(";");
	cmd->sendCmd(Msg);
	cmd->sendCmdArg(msg);
	cmd->sendCmdArg(v);
	cmd->sendCmdEnd();
}
void cmdMsg(CmdMessenger2 *cmd,   const __FlashStringHelper *msg, int v) {
	//	cmd->sendCmd( Msg, msg );
	//SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
	//ser.print("1,"); ser.print(msg); ser.print(";");
	cmd->sendCmd(Msg);
	cmd->sendCmdArg(msg);
	cmd->sendCmdArg(v);
	cmd->sendCmdEnd();
}
void cmdMsg(CmdMessenger2 *cmd,const __FlashStringHelper *stringLiteral){
//	cmd->sendCmd( Msg, msg );
 	//SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
	//ser.print("1,"); ser.print(msg); ser.print(";");
	cmd->sendCmd(Msg);
	cmd->sendCmdArg(stringLiteral);
	cmd->sendCmdEnd();
}

//////////////////////////////////////////////////////////////////////////
/// GESTIONE RICEZIONE COMANDI DA REMOTO (BT O WIFI)
//////////////////////////////////////////////////////////////////////////

#pragma region GESTIONE RICEZIONE COMANDI DA REMOTO (BT O WIFI)

	void OnCmdReboot(CmdMessenger2 *cmd){
		//reset software
		SPEAK_OK
			SPEAK("MI RIAVVIO");
		cmdMsg(cmd, "Riavvio..." );
		reboot();
	//	software_Reboot();

	}

	//////////////////////////////////////////////////////////////////////////
	/// Modalità operativa : MODE_SLAVE , JOYSTICK , AUTONOMOUS
	//////////////////////////////////////////////////////////////////////////
	void OnCmdRobotSetMode(CmdMessenger2 *cmd) {
		if (robotModel.status.operatingMode != robotModel.statusOld.operatingMode) { SPEAK_OK }

		robotModel.cmdSetMode((operatingMode_e)cmd->readInt16Arg());


	}
	void OnUnknownCommand(CmdMessenger2 *cmd)
	{
		int cmdId = (int)cmd->commandID();

		playSingleNote(100, 100);
		TFT_PRINT_MSG(msgRowCnt, cmd->streamBuffer);
		msgRowCnt++; if (msgRowCnt > (MSG_STARTINGROW + MSG_ROWS-1)) { msgRowCnt = MSG_STARTINGROW; }

		dbg(cmd->streamBuffer)
		cmd->reset();


	}
	void OnCmdRobotHello(CmdMessenger2 *cmd)
	{
		//cmdMsg(cmd,"Hello I'm ready");
	}
	//////////////////////////////////////////////////////////////////////////
	/// C O M A N D I   D I   M O V I M E N T O
	//////////////////////////////////////////////////////////////////////////
	// Avanti o indietro di x cm --------------------------------------------
	void OnCmdRobotMoveCm(CmdMessenger2 *cmd)
	{
		if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }

		//String s;
		int cmPercorsi=0;
		int dist = cmd->readInt16Arg();

		char s[] = "Ok OnCmdRobotMoveCm: ";
		cmdMsg(cmd, s, dist);

		//cmd->sendCmd(Msg,dist);

		//per eseguire lo spostamento devo segnalare al processo robotCore
		// di inviare il comando
		cmPercorsi=robotModel.cmdMoveCm(dist);

		// riporto la distanza percorsa-----------
		if(dist>0){
			cmdMsg(cmd, "Moved steps Forward: ", robotModel.status.cmd.stepsDone);
	 
		}
		else{
			cmdMsg(cmd, "Moved steps Back: ", robotModel.status.cmd.stepsDone);
		}
	 
		cmdMsg(cmd, "..of targetSteps:", robotModel.status.cmd.targetSteps);
		//-----------------------------------------


		// riporto la distanza percorsa-----------
		cmd->sendCmdStart(kbMovedCm);
		cmd->sendCmdArg( cmPercorsi );
		cmd->sendCmdEnd();
		//-----------------------------------------
	}
	// ROTAZIONE IN GRADI    --------------------------------------
	void OnCmdRobotRotateDeg(CmdMessenger2 *cmd)
	{
		if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }
		int deg = cmd->readInt16Arg();
		cmdMsg(cmd, "OK rotateRadiants: ");

		int DegPercorsi = 0;

		// invia al robot il comando 
		DegPercorsi = robotModel.cmdRotateDeg(deg);


		// Messaggio step percorsi-----------------
		if (DegPercorsi > 0) {
			// Messaggio step percorsi-----------------
			cmdMsg(cmd, "Rotated stp CW: ", DegPercorsi);
		}
		else {
			cmdMsg(cmd, "Rotated stp CCW: ", -DegPercorsi);
		}
		cmdMsg(cmd, "of : ", deg);
		//-----------------------------------------
		//-----------------------------------------



		// riporto la distanza percorsa-----------
		cmd->sendCmdStart(kbRotationRad);
		cmd->sendCmdArg((float)PI*DegPercorsi / 180);
		cmd->sendCmdEnd();
		//-----------------------------------------
	}
	// ROTAZIONE IN RADIANTI --------------------------------------
	void OnCmdRobotRotateRadiants(CmdMessenger2 *cmd)
	{
		//if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }
		//float rad = cmd->readFloatArg();
	 //
		//cmdMsg(cmd,  "OK rotateRadiants: "  );

		//float RadPercorsi = 0.0;
		/////RadPercorsi = robotModel.rotateRadiants( rad );

		////dtostrf( RadPercorsi, 7, 3, s );

		//// Messaggio step percorsi-----------------
		//if (RadPercorsi>0){
		//	cmdMsg(cmd,"Rotated stp CW: ", RadPercorsi);
		//}
		//else{
		//	cmdMsg(cmd,"Rotated stp CCW: ", -RadPercorsi);
		//}
		//cmdMsg(cmd,"of : ", robotModel.status.cmd.targetSteps);
		////-----------------------------------------



		//// riporto la distanza percorsa-----------
		//cmd->sendCmdStart( kbRotationRad	);
		//cmd->sendCmdArg( RadPercorsi );
		//cmd->sendCmdEnd();
		////-----------------------------------------
	}

	void OnCmdRobotMoveCCW(CmdMessenger2 *cmd)
	{
		//int ck= cmd->readInt16Arg();
	 //
	 //	cmdMsg(cmd, "RECEIVED COMMAND [MoveCCW]", ck);
		///robotModel.goCCW(ck);
	}
	//////////////////////////////////////////////////////////////////////////
	/// C O M A N D I   D I   I M P O S T A Z I O N E  P E R I F E R I C H E
	//////////////////////////////////////////////////////////////////////////
	void OnCmdRobotRele(CmdMessenger2 *cmd)
	{
		//if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }

		//int16_t rele = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
		//int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
		/////robotModel.setRele(rele, onoff);

		//
		//cmdMsg(cmd, "Msg Rele :", rele);

		//// rimanda il medesimo comando indietro come ack
		//cmd->sendCmdStart(CmdRobotRele );
		//cmd->sendCmdArg( rele );
		//cmd->sendCmdArg( onoff );
		//cmd->sendCmdEnd();

	}
	void OnCmdSetLed(CmdMessenger2 *cmd)
	{
		//if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }

		//int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
		//digitalWriteFast( Pin_ONBOARD_LED, onoff );


		//cmdMsg(cmd, "Ack CmdSetLed :", onoff);

		//// rimanda il medesimo comando indietro come ack
		//cmd->sendCmdStart( CmdSetLed );
		//cmd->sendCmdArg( onoff );
		//cmd->sendCmdEnd();

	}
	void OnCmdSetLaser(CmdMessenger2 *cmd)
	{
		if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }
		bool blOn = cmd->readBoolArg();		//numero del rele da attivare/disattivare

		if (blOn)
		{
			SPEAK("LASER ON");

		}
		else
		{
			SPEAK("LASER OFF");

		}
		robotModel.cmdSetLaser(blOn);
	}
	void OnCmdSetPort(CmdMessenger2 *cmd)
	{
		//if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }

		//int16_t port = cmd->readInt16Arg();		//numero della porta
		//int16_t onoff = cmd->readInt16Arg();		//valore 0 1

		//digitalWriteFast( port, onoff );
		////String s = "Ack CmdSetPort " + String( port ) + ":" + String( onoff );
		//cmdMsg(cmd, "Ack CmdSetPort :", port);

	 //	// rimanda il medesimo comando indietro come ack
		//cmd->sendCmdStart( CmdSetPort );
		//cmd->sendCmdArg( port );
		//cmd->sendCmdArg( onoff );
		//cmd->sendCmdEnd();

	}

	/// ///////////////////////////////////////////////////////////////////////
	// / C O M A N D I   D I   A C Q U I S I Z I O N E
	/// ///////////////////////////////////////////////////////////////////////


	/// ///////////////////////////////////////////////////////////////////////
	//  INVIA, SU RICHIESTA DA WIFI O BTH LO STATO DEI SENSORI 
	/// ///////////////////////////////////////////////////////////////////////
	void OnCmdGetSensorsHRate(CmdMessenger2 *cmd)
	{
		cmd->sendCmdStart(kbGetSensorsHRate);
		cmd->sendCmdArg(millis());

		cmd->sendCmdArg(robotModel.status.tictac);

		cmd->sendCmdArg(robotModel.status.posCurrent.x);	//robot position X
		cmd->sendCmdArg(robotModel.status.posCurrent.y);	//robot position y
		cmd->sendCmdArg(robotModel.status.posCurrent.r);	//robot position alfa gradi

		cmd->sendCmdArg(robotModel.status.sensors.irproxy.fw);	// IR proxy
		cmd->sendCmdArg(robotModel.status.sensors.irproxy.fwHL);	// IR proxy
		cmd->sendCmdArg(robotModel.status.sensors.irproxy.bk);	// IR proxy
		cmd->sendCmdArg(robotModel.status.sensors.pirDome);		// movimento

		cmd->sendCmdArg(robotModel.status.sensors.analog[0]);	//pot
		cmd->sendCmdArg(robotModel.status.sensors.analog[1]);	//batteria
		cmd->sendCmdArg(robotModel.status.sensors.analog[2]);	//light

		cmd->sendCmdArg(robotModel.status.act.rele[0]);		//rele 1
		cmd->sendCmdArg(robotModel.status.act.rele[1]);		//rel2

														//cmd->sendCmdArg( digitalReadFast( Pin_MotENR ) );
														//cmd->sendCmdArg( digitalReadFast( Pin_MotENL ) );

		cmd->sendCmdArg(robotModel.status.sensors.switchTop); // status switch modo Autonomo/slave
		cmd->sendCmdArg(robotModel.status.act.laserOn); // laser

		cmd->sendCmdArg(robotModel.status.sensors.batCharge); //0-100

		cmd->sendCmdArg(robotModel.status.sensors.gps.sats);		//gps
		cmd->sendCmdArg(robotModel.status.sensors.gps.lat);
		cmd->sendCmdArg(robotModel.status.sensors.gps.lng);
		cmd->sendCmdEnd();

 

	}
	/// ///////////////////////////////////////////////////////////////////////
	//  RICEZIONE SENSORI A BASSO RATE
	/// ///////////////////////////////////////////////////////////////////////
	void OnCmdGetSensorsLRate(CmdMessenger2 *cmd)
	{
		cmd->sendCmdStart(kbGetSensorsLRate);
		// Percentuale di carica batteria

		cmd->sendCmdArg((int)robotModel.status.operatingMode);
		cmd->sendCmdArg(robotModel.status.sensors.batCharge);
		cmd->sendCmdArg(robotModel.status.sensors.switchTop);
		cmd->sendCmdArg(robotModel.status.sensors.gps.sats);
		cmd->sendCmdArg(robotModel.status.sensors.gps.lat);
		cmd->sendCmdArg(robotModel.status.sensors.gps.lng);
		cmd->sendCmdArg(robotModel.status.sensors.gps.alt);

		cmd->sendCmdEnd();
	}
	/// ///////////////////////////////////////////////////////////////////////
	//  RICEZIONE LETTURA PORTA
	/// ///////////////////////////////////////////////////////////////////////
	void OnCmdReadPort(CmdMessenger2 *cmd)
	{
		//int16_t port = cmd->readInt16Arg();		//numero della porta
		//if (robotModel.status.operatingMode == MODE_SLAVE) { SPEAK_OK }

		//digitalReadFast( port );
		//String s = "Ack ReadPort " + String( port ) ;
		//cmd->sendCmd( Msg, s );
		//// rimanda il medesimo comando indietro come ack
		//cmd->sendCmdStart( kbReadPort );
		//cmd->sendCmdArg( port );
	 //
		//cmd->sendCmdEnd();

	}

	//////////////////////////////////////////////////////////////////////////
	/// si muove alla  direzione e  velocità impostata
	//////////////////////////////////////////////////////////////////////////
	void OnCmdRobotGo(CmdMessenger2 *cmd)
	{
		robotModel.status.cmd.commandDir = (commandDir_e)cmd->readInt16Arg();		//direzione (enum commandDir_t {STOP, GOFW, GOBK, GOCW, GOCCW};)
		int motorCK= cmd->readInt16Arg();

		robotModel.cmdGo(robotModel.status.cmd.commandDir, motorCK);
		
	}
	void OnCmdRobotStop(CmdMessenger2 *cmd) {
		robotModel.cmdStop();

	}


	//////////////////////////////////////////////////////////////////////////
	/// C O M A N D I      S O N A R
	//////////////////////////////////////////////////////////////////////////


		//////////////////////////////////////////////////////////////
		///		Invia i dati del sonar
		/////////////////////////////////////////////////////////////
	void cmdSonarSendData(CmdMessenger2 *cmd) {
		int alfa;
		int i;


		// invio quanti dati ho da trasmettere
		//kb.sendCmdArg( robotModel.status.parameters.sonarScanSteps );

		for (i = 0; i < robotModel.status.parameters.sonarScanSteps; i++)
		{
			//kb.printLfCr();
			delay(20);
			cmd->sendCmdStart(kbGetSonarData);
			alfa = robotModel.status.parameters.sonarStartAngle + i* robotModel.status.parameters.sonarStepAngle;
			cmd->sendCmdArg(alfa);
			cmd->sendCmdArg(robotModel.status.sensors.sonarEchos[i]);
			cmd->sendCmdEnd();
		}
		cmd->sendCmd(kbSonarDataEnd);

	}


		//-------------------------------------------
		// invia i dati SONAR man mano che si sposta
		//-------------------------------------------
		void OnCmdSonarScanSync(CmdMessenger2 *cmd)
		{

			String s = "\nStart scanning...";
			cmd->sendCmd(Msg, s);

			cmd->sendCmdStart(kbGetSonarData);

			int i = 0;
			int pos = robotModel.status.parameters.sonarStartAngle; //int endPos=SONAR_ARRAY_SIZE;

			while (pos < robotModel.status.parameters.sonarEndAngle) // goes from 0 degrees to 180 degrees
			{
				//robotModel.status.sensors.sonarEchos[i] =robotModel.SonarPingAtPos(pos);



				// Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
				unsigned long t1 = millis();
				cmd->sendCmdArg(robotModel.status.sensors.sonarEchos[i]);

				while (millis() - t1 > (unsigned long)robotModel.status.parameters.sonarScanSpeed) {
					delay(1);
				}

				pos += robotModel.status.parameters.sonarStepAngle;
				i++;

			}
			cmd->sendCmdEnd();

			s = "...end scanning";
			cmd->sendCmd(Msg, s);
		}

#pragma endregion

//////////////////////////////////////////////////////////////////////////
/// GESTIONE RICEZIONE M E S S A G G I   D A  R O B O T C O E 
//////////////////////////////////////////////////////////////////////////
#pragma region MESSAGGI DA ROBOTCORE


	void OnkbGetSensorsHRate(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud
	{
		robotModel.statusOld = robotModel.status;  // save current status
		robotModel.status.ts = (unsigned long)cmd->readInt16Arg(); // (unsigned long)millis();

		robotModel.status.tictac = cmd->readBoolArg();

		robotModel.status.posCurrent.x = cmd->readDoubleArg();
		robotModel.status.posCurrent.y = cmd->readDoubleArg();
		robotModel.status.posCurrent.r = cmd->readDoubleArg();

		robotModel.status.sensors.irproxy.fw = cmd->readBoolArg();	// IR proxy
		robotModel.status.sensors.irproxy.fwHL = cmd->readBoolArg();	// IR proxy
		robotModel.status.sensors.irproxy.bk = cmd->readBoolArg();	// IR proxy
		robotModel.status.sensors.pirDome = cmd->readBoolArg();		// movimento

		robotModel.status.sensors.analog[0] = (long)cmd->readInt16Arg();	//pot
		robotModel.status.sensors.analog[1] = (long)cmd->readInt16Arg();	//batteria
		robotModel.status.sensors.analog[2] = (long)cmd->readInt16Arg();	//light

		robotModel.status.act.rele[0] = cmd->readBoolArg();		//rele 1
		robotModel.status.act.rele[1] = cmd->readBoolArg();		//rele 2

		//robotModel.status.act.MotENR = cmd->readBoolArg();
		//robotModel.status.act.MotENL = cmd->readBoolArg();

		robotModel.status.sensors.switchTop = cmd->readBoolArg();
		robotModel.status.act.laserOn = cmd->readBoolArg();

		//robotModel.status.sensors.batCharge = cmd->readInt16Arg();

		//robotModel.status.sensors.gps.sats = cmd->readInt16Arg();
		//robotModel.status.sensors.gps.lat = cmd->readFloatArg();
		//robotModel.status.sensors.gps.lng = cmd->readFloatArg();



	}

	void OnkbGetSensorsLRate(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud
	{
		robotModel.status.sensors.batCharge = cmd->readInt16Arg();
		robotModel.status.sensors.switchTop = cmd->readBoolArg();

		robotModel.status.sensors.gps.lat = cmd->readFloatArg();
		robotModel.status.sensors.gps.lng = cmd->readFloatArg();
		robotModel.status.sensors.gps.sats = cmd->readInt16Arg();

		robotModel.status.operatingMode =  (operatingMode_e)cmd->readInt16Arg();

	}
	void OnkbMovedCm(CmdMessenger2 *cmd)  
	{
		robotModel.updatePose(cmd->readDoubleArg(),0);
	}
	void OnkbRotatedDeg(CmdMessenger2 *cmd)  
	{
		robotModel.updatePose(0,cmd->readInt16Arg());
	}
	void OnkbGetPose(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud
	{
		robotModel.statusOld = robotModel.status;  // save current status

		robotModel.status.ts = (unsigned long)cmd->readInt16Arg(); // (unsigned long)millis();

		robotModel.status.posCurrent.x = cmd->readDoubleArg();
		robotModel.status.posCurrent.y = cmd->readDoubleArg();
		robotModel.status.posCurrent.r = cmd->readDoubleArg();

	}

	//////////////////////////////////////////////////////////////
	///		riceve i dati del sonar da RobotCore
	/////////////////////////////////////////////////////////////
	void OnkbSonarSendData(CmdMessenger2 *cmd) {

		int alfa = cmd->readInt16Arg();
		robotModel.status.sensors.sonarEchos[alfa] = cmd->readInt16Arg();

	}


	//////////////////////////////////////////////////////////////////////////
	/// C O M A N D O      S P E E C H
	//////////////////////////////////////////////////////////////////////////

	void	OnCmdSpeech(CmdMessenger2 *cmd) {
		char *strSpeech = cmd->readStringArg();
		/// metto in coda nella FiFO la  frase da pronunciare
		// send message
		dbg("OnCmdSpeech...");
		dbg(strSpeech)
		SPEAK(strSpeech);
		TFT_PRINT_MSG(msgRowCnt, strSpeech);
		msgRowCnt++; if (msgRowCnt > (MSG_STARTINGROW + MSG_ROWS-1)) { msgRowCnt = MSG_STARTINGROW; }

	}
	// su ricezione di una stringa messaggio, la mette nel buffer
	void	OnMsg(CmdMessenger2 *cmd) {
		static int msgRowCnt= MSG_STARTINGROW;
		// lettura stringa dalla seriale
		char *msgIn = cmd->readStringArg();
		//dbg("**MSG**")
		TFT_PRINT_MSG(msgRowCnt, msgIn);
		msgRowCnt++; if (msgRowCnt > (MSG_STARTINGROW + MSG_ROWS-1)) { msgRowCnt = MSG_STARTINGROW; }

		#if 0

				// send message
				dbg(msgIn);
				playSingleNote(NOTE_A7, 80);

				/// metto la stringa  in coda nella FiFO TFTMSG

				// Create element I want to add
				struct myMsgStruct msgOutTftMsg;
				// alloca la memoria per msgOutTftMsg 
				memset(&msgOutTftMsg, 0, sizeof(struct myMsgStruct));

				if (!myRingBufTftMsg->isFull(myRingBufTftMsg))
				{
					// metto il messaggio in ring_buf----------
					// così non funziona!! -> strcpy(tmpBuff, msgOutVoice.msgString);
					for (size_t i = 0; i < sizeof(msgIn); i++)
					{
						msgOutTftMsg.msgString[i] = msgIn[i];
					}
					msgOutTftMsg.index = myRingBufTftMsg->elements;
					msgOutTftMsg.timestamp = millis();
					//-----------------------------------------


					myRingBufTftMsg->add(myRingBufTftMsg, &msgOutTftMsg);
					//-----------------------------------------

					dbg2("msgOutTftMsg out :", msgOutTftMsg.msgString)
						dbg2("myRingBufTftMsg->elem:", myRingBufTftMsg->elements)

						chThdSleepMilliseconds(500);//	chThdYield();

				}
				else //full
				{
					dbg2("FULL myRingBufTftMsg->elem:", myRingBufTftMsg->elements)
						dbg2("........FreeSram", getFreeSram())
						//attendo più a lungo per permettere di svuotare il buffer
						chThdSleepMilliseconds(4000);//	chThdYield();
				}


		#endif // 0

	}


#pragma endregion

//////////////////////////////////////////////////////////////////////////////////////
///////					  A T T A C H    C A L L B A C K S					//////////
//////////////////////////////////////////////////////////////////////////////////////
void attachCommandCallbacks(CmdMessenger2 *cmd)		//va messa in fondo
{


		cmd->attach(OnUnknownCommand);
		cmd->attach(Msg, OnMsg);
		//cmd->attach(CmdRobotHello, OnCmdRobotHello);
		cmd->attach(CmdReboot, OnCmdReboot);

		cmd->attach(CmdRobotStartMoving, OnCmdRobotGo);
		cmd->attach(CmdRobotStop, OnCmdRobotStop);

		cmd->attach(CmdRobotMoveCm, OnCmdRobotMoveCm);
		cmd->attach(CmdRobotRotateDeg, OnCmdRobotRotateDeg);
		//cmd->attach(CmdRobotRotateRadiants, OnCmdRobotRotateRadiants);

		//cmd->attach(CmdRobotRele, OnCmdRobotRele);
		//cmd->attach(CmdSetLed, OnCmdSetLed);
		cmd->attach(CmdSetLaser, OnCmdSetLaser);
		//cmd->attach(CmdSetPort, OnCmdSetPort);

		cmd->attach(kbGetSensorsHRate, OnkbGetSensorsHRate);
		cmd->attach(kbGetSensorsLRate, OnkbGetSensorsLRate);
		cmd->attach(kbMovedCm, OnkbMovedCm);
		cmd->attach(kbRotationDeg, OnkbRotatedDeg);
		cmd->attach(kbGetPose, OnkbGetPose);
		cmd->attach(CmdGetSensorsLRate, OnCmdGetSensorsLRate);
		cmd->attach(CmdGetSensorsHRate, OnCmdGetSensorsHRate);

		cmd->attach(CmdRobotSetMode, OnCmdRobotSetMode);
		cmd->attach(cmdSpeech,OnCmdSpeech);

}
