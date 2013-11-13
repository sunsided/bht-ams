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
#include <stdlib.h>
#include <iostream>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#include <vmc232.h>
#include <sys/time.h>
#include "robi.h"

using namespace std;

#define MAX_RPM 1000								/*< Maximale RPM */
#define MAX_ROUNDS_PER_SEC MAX_RPM*60			/*< Umwandlungsfaktor RPM in RPS */

#define MOTOR_COUNT 		(2)					/*< Anzahl der Motoren */
#define MOTOR_INDEX_LEFT	(MOTOR_ID_LEFT-1)	/*< Arrayindex für Motor links */
#define MOTOR_INDEX_RIGHT	(MOTOR_ID_RIGHT-1)	/*< Arrayindex für Motor rechts */

#define WHEEL_DISTANCE  ((double)RAD_ABSTAND)	/*< Radabstand in Metern */
#define WHEEL_RADIUS_L  ((double)RAD_DURCHMESSER/2.0)		/*< Radradius in Metern */
#define WHEEL_RADIUS_R  ((double)RAD_DURCHMESSER/2.0)		/*< Radradius in Metern */

/**
* @brief Konfiguriert den Roboter
*/
void configureRobot(VMC::CVmc &vmc, int pl, int pr, int il, int ir)
{

	int    prop_val[] ={
			90,					// P-Wert für Motor links
			80};				// P-Wert für Motor rechts
	int    integ_val[]={
			25000,				// I-Wert für Motor links
			20000};				// I-Wert für Motor rechts

	if (pl > 0) prop_val[MOTOR_INDEX_LEFT] = pl;
    if (pr > 0) prop_val[MOTOR_INDEX_RIGHT] = pr;
    if (il > 0) integ_val[MOTOR_INDEX_LEFT] = il;
    if (ir > 0) integ_val[MOTOR_INDEX_RIGHT] = ir;     

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

	cout << "* P-Wert links:  " << prop_val[MOTOR_INDEX_LEFT] << endl;
	cout << "* P-Wert rechts: " << prop_val[MOTOR_INDEX_RIGHT] << endl;
	cout << "* I-Wert links:  " << integ_val[MOTOR_INDEX_LEFT] << endl;
	cout << "* I-Wert rechts: " << integ_val[MOTOR_INDEX_RIGHT] << endl;

	// P-Anteil
	vmc.setMotorLinearPart(MOTOR_ID_LEFT, prop_val[MOTOR_INDEX_LEFT]);
	vmc.setMotorLinearPart(MOTOR_ID_RIGHT, prop_val[MOTOR_INDEX_RIGHT]);

	// I-Anteil
	vmc.setMotorIntegralPart(MOTOR_ID_LEFT, integ_val[MOTOR_INDEX_LEFT]);
	vmc.setMotorIntegralPart(MOTOR_ID_RIGHT, integ_val[MOTOR_INDEX_RIGHT]);
}

