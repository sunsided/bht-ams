/*
* Demonstrations-/Kalibrierungsprogramm für Volksbot
* Autonome Mobile Systeme, WS2013
*
* VMC:
*	- file:///usr/share/doc/vmc/html/index.html
*	- /usr/share/vmc/example_apps
*
* Volksbot:
*	- Raddurchmesser:	 8.9 cm 
*	- Radabstand:		35.5 cm
*/

#include <unistd.h>
#include <stdio.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <vmc232.h>
#include <sys/time.h>
#include "robi.h"
#include "robodrive.h"

using namespace std;

#define MAX_MEAS 		(100) 		 			/*< Anzahl der Messwerte */
#define MAX_RPM 		(900)					/*< Umdrehungen/Minute */
#define MEAS_WAIT		(100)					/*< Millisekunden */
#define MEAS_VELOCITY 	(20)					/*< Meter/Sekunde */

#define MAX_ROUNDS_PER_SEC MAX_RPM*60			/*< Umwandlungsfaktor RPM in RPS */

#define MOTOR_COUNT 		(2)					/*< Anzahl der Motoren */
#define MOTOR_INDEX_LEFT	(MOTOR_ID_LEFT-1)	/*< Arrayindex für Motor links */
#define MOTOR_INDEX_RIGHT	(MOTOR_ID_RIGHT-1)	/*< Arrayindex für Motor rechts */

#define WHEEL_DISTANCE  ((double)RAD_ABSTAND)	/*< Radabstand in Metern */
#define WHEEL_RADIUS_L  ((double)RAD_DURCHMESSER/2.0)		/*< Radradius in Metern */
#define WHEEL_RADIUS_R  ((double)RAD_DURCHMESSER/2.0)		/*< Radradius in Metern */

/**
* @brief Ermittelt die Systemzeit in Sekunden
* @return Die Systemzeit in Sekunden
*/
double time_shot()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return ((double)((int)tv.tv_sec) + (double)tv.tv_usec/1000000.0);
}

/**
* @brief Konfiguriert den ROboter
*/
void configureRobot(VMC::CVmc &vmc)
{
	int    prop_val[] ={
			/*150*/90,				// P-Wert für Motor links
			/*150*/70};				// P-Wert für Motor rechts
	int    integ_val[]={
			/*9000*/25000,				// I-Wert für Motor links
			/*17000*/20000};				// I-Wert für Motor rechts

	cout << "Setze geometrische Konfiguration" << endl;

	vmc.setWheelDistance(WHEEL_DISTANCE);
	vmc.setWheelRadius(MOTOR_ID_LEFT, WHEEL_RADIUS_L);
	vmc.setWheelRadius(MOTOR_ID_RIGHT, WHEEL_RADIUS_R);

	cout << "Setze Motorparameter" << endl;

	vmc.setMaximumRPM(MAX_RPM);
	vmc.setCycleTime(30);
	vmc.setSpeedFactor(MOTOR_ID_LEFT, 	SPEED_FACTOR_LEFT);
	vmc.setSpeedFactor(MOTOR_ID_RIGHT, SPEED_FACTOR_RIGHT);
	//  vmc.setMotorPositiveRamp(MOTOR_ID_LEFT, 100);
	//  vmc.setMotorPositiveRamp(MOTOR_ID_RIGHT, 1);
	vmc.clearResponseList();
	vmc.addStateToResponseList(vmc.MOTOR_TICKS_ABSOLUTE);
	vmc.setTimeout(2000);

	cout << "Setze Reglerparameter" << endl;

	// P-Anteil
	vmc.setMotorLinearPart(MOTOR_ID_LEFT, prop_val[MOTOR_INDEX_LEFT]);
	vmc.setMotorLinearPart(MOTOR_ID_RIGHT, prop_val[MOTOR_INDEX_RIGHT]);

	// I-Anteil
	vmc.setMotorIntegralPart(MOTOR_ID_LEFT, integ_val[MOTOR_INDEX_LEFT]);
	vmc.setMotorIntegralPart(MOTOR_ID_RIGHT, integ_val[MOTOR_INDEX_RIGHT]);
}

/**
* @brief Haupteinstiegspunkt
* @param[in] argc Anzahl der Kommandozeilenparameter
* @param[in] argv Vektor von Strings der Kommandozeilenparameter
* @return Statuscode der Anwendung
*/
int main(int argc, char **argv)
{
	VMC::CVmc    vmc("/dev/ttyS0");

	cout << "Verbinde mit VMC" << endl;
	if(!vmc.isConnected())
	{
		 cout << "Verbindung zu VMC fehlgeschlagen" << endl;
		 return 1;
	}

	cout << "Vorbereitung des Roboters ..." << endl;
	configureRobot(vmc);

	cout << "Betrieb ..." << endl;

	double omegal, omegar;
	const double v = 0.15; /* [m/s] */
	const double omega = 0; /* [rad/s] */
	const double time = 6.666; /* [s] */

	/* Inverse Kinematik */
	vOmegaToOmegaWheel(v, omega, &omegal, &omegar);
	cout << "Omega links:  " << omegal << endl;
	cout << "Omega rechts: " << omegar << endl;

	/* fahren */
	driveRobot(&vmc, omegal, omegar, time);

	/* warten, dann umdrehen */
	vmc.wait(1000);
	rotateRobot(&vmc, -180);

	/* warten, dann zurückfahren */
	vmc.wait(1000);
	driveRobot(&vmc, omegal, omegar, time);

	/* Warten, dann zurückdrehen */
	vmc.wait(1000);
	rotateRobot(&vmc, 180);

	cout << "Ende." << endl;
	vmc.setMotors(0, 0);
	return 0;
}

