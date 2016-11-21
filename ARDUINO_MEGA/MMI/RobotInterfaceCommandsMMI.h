/* Robot Interface Commands */

// Commands we send from the PC and want to receive on the Arduino.
// We must define a callback function in our Arduino program for each entry in the list below.
//#include <CmdMessenger2.h>  // CmdMessenger2
#include <CmdMessenger2/CmdMessenger2.h>
#include <robotModel.h>
#include "Commands_Enum.h"
#include "SpeakSerialInterface.h"
///extern struct robotModel_c robot;
//Dichiarazione di funzione che punta all'indirizzo zero
void( *Riavvia )(void) = 0;
#define MESSAGEMAXLEN 50
char strBuff[MESSAGEMAXLEN];


//////////////////////////////////////////////////////////////////////////////////////
//////////////////////////  C A L L B A C K S				//////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
 
//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   I M P O S T A Z I O N E  G E N E R A L E
//////////////////////////////////////////////////////////////////////////
void cmdMsg(CmdMessenger2 *cmd,const char *msg){
//	cmd->sendCmd( Msg, msg );
 	//SERIAL_MSG.print("1,"); SERIAL_MSG.print(msg); SERIAL_MSG.print(";");
	//ser.print("1,"); ser.print(msg); ser.print(";");
	cmd->sendCmd(Msg);
	cmd->sendCmdArg(msg);
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
void OnCmdReboot(CmdMessenger2 *cmd){
	//reset software
	cmdMsg(cmd, "Riavvio..." );
	Riavvia();
//	software_Reboot();

}
//////////////////////////////////////////////////////////////////////////
/// Modalit� operativa : SLAVE , JOYSTICK , AUTONOMOUS
//////////////////////////////////////////////////////////////////////////
void OnCmdRobotSetMode(CmdMessenger2 *cmd){
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }

	//robotModel.SetMode((operatingMode_e)cmd->readInt16Arg());
 //
	//switch (robotModel.status.operatingMode)
	//{
	//	case SLAVE:
	//		cmd->sendCmd( CmdRobotSetMode, SLAVE );
	//		SPEAK_SLAVE
	//		cmdMsg(cmd,"SetMode SLAVE");
	//		break;
	//	case JOYSTICK:
	//		cmd->sendCmd( CmdRobotSetMode, JOYSTICK );
	//		 
	//		cmdMsg(cmd, "SetMode JOYSTICK");
	//		break;
	//	case AUTONOMOUS:
	//		cmd->sendCmd( CmdRobotSetMode, AUTONOMOUS );
	//		 SPEAK_AUTONOMO
	//		cmdMsg(cmd, "SetMode AUTONOMOUS" );
	//		break;	
 //	
	//	default:
	//		cmd->sendCmd(Msg,"Unrecognised Mode");			 
	//		break;
	//}
}
void OnUnknownCommand(CmdMessenger2 *cmd)
{
	SPEAK("NON HO CAPITO");
	//String s;
	cmd->readStringArg();
//	cmd->sendCmd(kError,"\nCommand not recognised");

	char s[] = "unkwownCmd :";
	cmdMsg(cmd, s);
 
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
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }

	////String s;
	//int cmPercorsi=0;
	//int dist = cmd->readInt16Arg();

	//char s[] = "Ok OnCmdRobotMoveCm: ";
	//cmdMsg(cmd, s, dist);

	////cmd->sendCmd(Msg,dist);

	////eseguo lo spostamento
	/////cmPercorsi=robotModel.moveCm(dist);

	//// riporto la distanza percorsa-----------
	//if(dist>0){
	//	cmdMsg(cmd, "Moved steps Forward: ", robotModel.status.cmd.stepsDone);
 //
	//}
	//else{
	//	cmdMsg(cmd, "Moved steps Back: ", robotModel.status.cmd.stepsDone);
	//}
 //
	//cmdMsg(cmd, "..of targetSteps:", robotModel.status.cmd.targetSteps);
	////-----------------------------------------


	//// riporto la distanza percorsa-----------
	//cmd->sendCmdStart( kbRotationRad );
	//cmd->sendCmdArg( cmPercorsi );
	//cmd->sendCmdEnd();
	////-----------------------------------------
}
 // ROTAZIONE IN RADIANTI --------------------------------------
void OnCmdRobotRotateRadiants(CmdMessenger2 *cmd)
{
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }
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
void OnCmdRobotRotateDeg(CmdMessenger2 *cmd)
{
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }
	//int deg = cmd->readInt16Arg();
	////String s = "OK rotateRadiants: ";
	//cmdMsg(cmd, "OK rotateRadiants: ");

	//int DegPercorsi = 0;
	/////DegPercorsi = robotModel.rotateDeg(deg);

	////dtostrf( RadPercorsi, 7, 3, s );

	//// Messaggio step percorsi-----------------
	//if (DegPercorsi > 0) {
	//	// Messaggio step percorsi-----------------
	//	cmdMsg(cmd, "Rotated stp CW: ", DegPercorsi);
	//}
	//else {
	//	cmdMsg(cmd, "Rotated stp CCW: ", -DegPercorsi);
	//}
	//cmdMsg(cmd, "of : ", deg);
	////-----------------------------------------
	////-----------------------------------------



	//// riporto la distanza percorsa-----------
	//cmd->sendCmdStart(kbRotationRad);
	//cmd->sendCmdArg((float)PI*DegPercorsi/180);
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
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }

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
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }

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
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }

	//int16_t onoff = cmd->readInt16Arg();		//numero del rele da attivare/disattivare
	//digitalWriteFast( Pin_LaserOn, onoff );
	//cmdMsg(cmd, "Ack CmdSetLaser :", onoff);

	//// rimanda il medesimo comando indietro come ack
	//cmd->sendCmdStart( CmdSetLaser);
	//cmd->sendCmdArg( digitalReadFast(Pin_LaserOn ));
	//cmd->sendCmdEnd();
}
void OnCmdSetPort(CmdMessenger2 *cmd)
{
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }

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

//////////////////////////////////////////////////////////////////////////
/// C O M A N D I   D I   A C Q U I S I Z I O N E
//////////////////////////////////////////////////////////////////////////
void OnCmdGetSensorsHRate(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud
{
	//robotModel.readSensors();	//IR proxy, Gyro

	//cmd->sendCmdStart(kbGetSensorsHRate); 
	//cmd->sendCmdArg(robotModel.status.sensors.irproxy.fw);	// IR proxy
	//cmd->sendCmdArg(robotModel.status.sensors.irproxy.fwHL);	// IR proxy
	//cmd->sendCmdArg(robotModel.status.sensors.irproxy.bk);	// IR proxy
	//cmd->sendCmdArg(robotModel.status.sensors.pirDome);		// movimento

	//cmd->sendCmdArg(robotModel.status.sensors.analog[0]);	//pot
	//cmd->sendCmdArg(robotModel.status.sensors.analog[1]);	//batteria
	//cmd->sendCmdArg(robotModel.status.sensors.analog[2]);	//light

	//cmd->sendCmdArg(robotModel.getReleStatus(0));		//rele 1
	//cmd->sendCmdArg(robotModel.getReleStatus(1));		//rel2

	//cmd->sendCmdArg( digitalReadFast( Pin_MotENR ) );
	//cmd->sendCmdArg( digitalReadFast( Pin_MotENL ) );

	//cmd->sendCmdArg( digitalReadFast( Pin_BtOnOff ) );
	//cmd->sendCmdArg( digitalReadFast( Pin_LaserOn ) );
 //	cmd->sendCmdArg( robotModel.readBattChargeLevel() );	

	//cmd->sendCmdEnd();

}
void OnCmdGetSensorsLRate(CmdMessenger2 *cmd)
{
	//// Lettura Switch Modalit� Autonomo/Slave
	//if (robotModel.status.sensors.switchTop != digitalReadFast(Pin_SwitchTop))
	//{//cambio modo
	//	robotModel.status.sensors.switchTop = digitalReadFast(Pin_SwitchTop);

	//	if (robotModel.status.sensors.switchTop) { 
	//		robotModel.SetMode(AUTONOMOUS); 
	//		SPEAK_AUTONOMO
	//			dbg("Autonomo")
	//	}
	//	else {
	//		robotModel.SetMode(SLAVE);
	//		SPEAK_SLAVE	
	//			dbg("SLAVE")

	//	}

	//}

	//cmd->sendCmdStart(kbGetSensorsLRate); 
	//// Percentuale di carica batteria
	//
	//cmd->sendCmdArg( (int)robotModel.status.operatingMode );
	//cmd->sendCmdArg( robotModel.readBattChargeLevel() );	
	//cmd->sendCmdArg( robotModel.status.sensors.switchTop );
	//cmd->sendCmdArg( robotModel.status.sensors.gps.sats);
	//cmd->sendCmdArg( robotModel.status.sensors.gps.lat);
	//cmd->sendCmdArg( robotModel.status.sensors.gps.lng);
	//cmd->sendCmdArg( robotModel.status.sensors.gps.alt);

	//cmd->sendCmdEnd();
}
void OnCmdReadPort(CmdMessenger2 *cmd)
{
	//int16_t port = cmd->readInt16Arg();		//numero della porta
	//if (robotModel.status.operatingMode == SLAVE) { SPEAK_OK }

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
/// si muove alla  direzione e  velocit� impostata
//////////////////////////////////////////////////////////////////////////
void OnCmdRobotStartMoving(CmdMessenger2 *cmd)
{
	//robotModel.status.cmd.commandDir = (commandDir_e)cmd->readInt16Arg();		//direzione (enum commandDir_t {STOP, GOFW, GOBK, GOCW, GOCCW};)
	//int motorCK= cmd->readInt16Arg();
	////String s = "Mov ck" +  robotModel.status.cmd.commandDir;
	//Serial.print( "1,Mov ck" ); Serial.print( motorCK ); Serial.print( ";" );

	//
	//switch (robotModel.status.cmd.commandDir)
	//{
	//	case GOF:
	//		robotModel.goFW(motorCK);
	//		cmdMsg(cmd,"goFW");
	//		break;
	//	case GOB:
	//		robotModel.goBK(motorCK);
	//		cmdMsg(cmd,"goBK" );
	//		break;
	//	case GOR:
	//		robotModel.goCW(motorCK);
	//		cmdMsg(cmd, "goCW" );
	//		break;	
	//	case GOL:
	//		robotModel.goCCW(motorCK);
	//		cmdMsg(cmd,"goCCW" );
	//		break;		
	//	default:
	//		cmdMsg(cmd, "Error on direction" );
	//		break;
	//}
}
void OnCmdRobotStopMoving(CmdMessenger2 *cmd){
	//robotModel.stop();
	//cmdMsg(cmd, "Stopped" );

	}


//////////////////////////////////////////////////////////////////////////
/// C O M A N D I      S O N A R
//////////////////////////////////////////////////////////////////////////

#if OPT_SERVOSONAR

	//////////////////////////////////////////////////////////////
	///		Invia i dati del sonar
	/////////////////////////////////////////////////////////////
	void kbSonarSendData(CmdMessenger2 *cmd){
		int alfa;
		int i;
		

		// invio quanti dati ho da trasmettere
		//kb.sendCmdArg( robotModel.status.parameters.sonarScanSteps );
		 
		for (i = 0; i< robotModel.status.parameters.sonarScanSteps; i++)
		{
			//kb.printLfCr();
			delay( 20 );
			cmd->sendCmdStart(kbGetSonarData);
			alfa = robotModel.status.parameters.sonarStartAngle + i* robotModel.status.parameters.sonarStepAngle;
			cmd->sendCmdArg( alfa );
			cmd->sendCmdArg( robotModel.status.sensors.sonarEchos[i] );
			cmd->sendCmdEnd();
		}
		cmd->sendCmd( kbSonarDataEnd );

	}

	void OnkbGetSensorsHRate(CmdMessenger2 *cmd)  //attenzione al limite dei 9600baud
	{
		robotModel.statusOld = robotModel.status;  // save current status

		robotModel.status.tictac = cmd->readBoolArg();

		robotModel.status.posCurrent.x = cmd->readDoubleArg();
		robotModel.status.posCurrent.y = cmd->readDoubleArg();
		robotModel.status.posCurrent.r = cmd->readDoubleArg();

		robotModel.status.sensors.irproxy.fw = cmd->readBoolArg();	// IR proxy
		robotModel.status.sensors.irproxy.fwHL= cmd->readBoolArg();	// IR proxy
		robotModel.status.sensors.irproxy.bk= cmd->readBoolArg();	// IR proxy
		robotModel.status.sensors.pirDome= cmd->readBoolArg();		// movimento

		robotModel.status.sensors.analog[0] = (long)cmd->readInt16Arg();	//pot
		robotModel.status.sensors.analog[1] = (long)cmd->readInt16Arg();	//batteria
		robotModel.status.sensors.analog[2] = (long)cmd->readInt16Arg();	//light

		robotModel.status.act.rele[0] = cmd->readBoolArg();		//rele 1
		robotModel.status.act.rele[1] = cmd->readBoolArg();		//rele 2

		//robotModel.status.act.MotENR = cmd->readBoolArg();
		//robotModel.status.act.MotENL = cmd->readBoolArg();

		robotModel.status.sensors.switchTop =cmd->readBoolArg();
		robotModel.status.act.laserOn =cmd->readBoolArg();

		robotModel.status.sensors.batCharge =cmd->readInt16Arg();

		robotModel.status.sensors.gps.sats = cmd->readInt16Arg();
		robotModel.status.sensors.gps.lat = cmd->readFloatArg();
		robotModel.status.sensors.gps.lng = cmd->readFloatArg();

		robotModel.status.ts = (unsigned long)millis();


	}

	//-------------------------------------------
	// invia i dati SONAR man mano che si sposta
	//-------------------------------------------
	void OnCmdSonarScanSync(CmdMessenger2 *cmd)
	{  

		String s = "\nStart scanning...";
		cmd->sendCmd(Msg,s);

		cmd->sendCmdStart(kbGetSonarData);

		int i=0; 
		int pos = robotModel.status.parameters.sonarStartAngle; //int endPos=SONAR_ARRAY_SIZE;

		while(pos < robotModel.status.parameters.sonarEndAngle ) // goes from 0 degrees to 180 degrees
		{
			//robotModel.status.sensors.sonarEchos[i] =robotModel.SonarPingAtPos(pos);



			// Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
			unsigned long t1 = millis();
			cmd->sendCmdArg( robotModel.status.sensors.sonarEchos[i]);
			
			while (millis() - t1 > (unsigned long)robotModel.status.parameters.sonarScanSpeed) {
				delay( 1 );
			}

			pos += robotModel.status.parameters.sonarStepAngle;
			i++;

		}
		cmd->sendCmdEnd();
	
		s = "...end scanning";
		cmd->sendCmd(Msg,s);
	}
#endif


// Called when a received command has no attached function

// Callback function calculates the sum of the two received float values
//void OnCmdRobotMoveFW()
//{
  //
  //// Retreive second parameter as float
  //float b = cmdMessenger.readFloatArg();
  //
  //// Send back the result of the addition
  ////cmdMessenger.sendCmd(kFloatAdditionResult,a + b);
  //cmdMessenger.sendCmdStart(kFloatAdditionResult);
  //cmdMessenger.sendCmdArg(a+b);
  //cmdMessenger.sendCmdArg(a-b);
  //cmdMessenger.sendCmdEnd();
//}




//////////////////////////////////////////////////////////////////////////
/// C O M A N D O      S P E E C H
//////////////////////////////////////////////////////////////////////////

	void	OnCmdSpeech(CmdMessenger2 *cmd) {
		char *strSpeech = cmd->readStringArg();
		/// metto in coda nella FiFO la  frase da pronunciare
		// send message
		SPEAK(strSpeech);
		dbg("speech");

	}


//////////////////////////////////////////////////////////////////////////////////////
///////					  A T T A C H    C A L L B A C K S					//////////
//////////////////////////////////////////////////////////////////////////////////////

void attachCommandCallbacks(CmdMessenger2 *cmd)		//va messa in fondo
{
#pragma region  Attach callback methods to WiFi channel 

	//cmd->attach(OnUnknownCommand);
	//cmd->attach(CmdRobotHello, OnCmdRobotHello);
	//cmd->attach(CmdReboot, OnCmdReboot);

	//cmd->attach(CmdRobotStartMoving, OnCmdRobotStartMoving);
	//cmd->attach(CmdRobotStopMoving, OnCmdRobotStopMoving);

	//cmd->attach(CmdRobotMoveCm, OnCmdRobotMoveCm);
	//cmd->attach(CmdRobotRotateRadiants, OnCmdRobotRotateRadiants);
	//cmd->attach(CmdRobotRotateDeg, OnCmdRobotRotateDeg);

	//cmd->attach(CmdRobotRele, OnCmdRobotRele);
	//cmd->attach(CmdSetLed, OnCmdSetLed);
	//cmd->attach(CmdSetLaser, OnCmdSetLaser);
	//cmd->attach(CmdSetPort, OnCmdSetPort);

	cmd->attach(kbGetSensorsHRate, OnkbGetSensorsHRate);
	//cmd->attach(CmdGetSensorsLRate, OnCmdGetSensorsLRate);
	//cmd->attach(CmdGetSensorsHRate, OnCmdGetSensorsHRate);
	//cmd->attach(CmdRobotSetMode, OnCmdRobotSetMode);
	cmd->attach(cmdSpeech,OnCmdSpeech);


#pragma endregion
#pragma region  Attach callback methods to Voice Commands channel 
	//cmdMMI.attach(OnUnknownCommand);
	//cmdMMI.attach(CmdRobotHello, OnCmdRobotHello);
	//cmdMMI.attach(CmdReboot, OnCmdReboot);

	//cmdMMI.attach(CmdRobotStartMoving, OnCmdRobotStartMoving);
	//cmdMMI.attach(CmdRobotStopMoving, OnCmdRobotStopMoving);

	//cmdMMI.attach(CmdRobotMoveCm, OnCmdRobotMoveCm);
	//cmdMMI.attach(CmdRobotRotateRadiants, OnCmdRobotRotateRadiants);

	//cmdMMI.attach(CmdRobotRele, OnCmdRobotRele);
	//cmdMMI.attach(CmdSetLed, OnCmdSetLed);
	//cmdMMI.attach(CmdSetLaser, OnCmdSetLaser);
	//cmdMMI.attach(CmdSetPort, OnCmdSetPort);

	//cmdMMI.attach(CmdGetSensorsLRate, OnCmdGetSensorsLRate);
	//cmdMMI.attach(CmdGetSensorsHRate, OnCmdGetSensorsHRate);
	//cmdMMI.attach(CmdRobotSetMode, OnCmdRobotSetMode);
	//#if OPT_SERVOSONAR
	//cmdMMI.attach(CmdSonarScan,OnCmdSonarScan);
	//	//cmd->attach(CmdSonarScanBatch, OnCmdSonarScanBatch );
	//	//cmd->attach(CmdSonarScanSync, OnCmdSonarScanSync);
	//#endif

#pragma endregion
}