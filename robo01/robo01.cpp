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

#include <stdint.h>
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

using namespace std;

#define MAX_MEAS 		(100) 		 			/*< Anzahl der Messwerte */
#define MAX_RPM 		(900)					/*< Umdrehungen/Minute */
#define MEAS_WAIT		(100)					/*< Millisekunden */
#define MEAS_VELOCITY 	(20)					/*< Meter/Sekunde */

#define MAX_ROUNDS_PER_SEC MAX_RPM*60			/*< Umwandlungsfaktor RPM in RPS */

#define MOTOR_COUNT 		(2)					/*< Anzahl der Motoren */
#define MOTOR_INDEX_LEFT	(MOTOR_ID_LEFT-1)	/*< Arrayindex für Motor links */
#define MOTOR_INDEX_RIGHT	(MOTOR_ID_RIGHT-1)	/*< Arrayindex für Motor rechts */

#define WHEEL_DISTANCE  ((double)35.5/100.0)	/*< Radabstand in Metern */
#define WHEEL_RADIUS_L  ((double)8.9/100.0)		/*< Radradius in Metern */
#define WHEEL_RADIUS_R  ((double)8.9/100.0)		/*< Radradius in Metern */

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
* @brief Wandelt Geschwindigkeiten in v-omega in Radgeschwindigkeiten um
* @param[in] v Die Bahngeschwindigkeit des Roboters [m/s]
* @param[in] omega Die Winkelgeschwindigkeit des Roboters [rad/s]
* @param[out] omega_l Winkelgeschwindigkeit des linken Rades [rad/s]
* @param[out] omega_r Winkelgeschwindigkeit des rechten Rades [rad/s]
*/
void vOmegaToOmegaWheel(const double v, const double omega, double *const omega_l, double *const omega_r)
{
	static const double halberRadabstand = RAD_ABSTAND * 0.5;
	static const double radRadius = RAD_DURCHMESSER * 0.5;

	/* Ermitteln der Radgeschwindigkeiten in [m/s] */
	const double v_l = v + omega * -halberRadabstand;
	const double v_r = v + omega *  halberRadabstand;

	/* Umrechnung von v_rad in omega_rad in [rad/s] */
	*omega_l = v_l / radRadius;
	*omega_r = v_r / radRadius;
}

/**
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten an
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
* @return 0 im Erfolgsfall, 1 oder 2 wenn einer der Motoren übersteuert würde
*/
uint8_t driveRobot(VMC::CVmc *const vmc, const double omega_l, const double omega_r)
{
	/* Radgeschwindigkeiten in Prozent MAX_RPM ermitteln */
	const double motorPercentL = omega_l*60.0*GETRIEBE_FAKTOR*100.0/(double)MAX_RPM;
	const double motorPercentR = omega_r*60.0*GETRIEBE_FAKTOR*100.0/(double)MAX_RPM;

	/* Werte runden und diskretisieren */
	const int motorPercentLd = (int)(0.5 + motorPercentL);
	const int motorPercentRd = (int)(0.5 + motorPercentR);

	cout << "Omega links [%]:  " << motorPercentLd << endl;
	cout << "Omega rechts [%]: " << motorPercentRd << endl;

	if (motorPercentLd > 100 || motorPercentLd < -100) return 1;
	if (motorPercentRd > 100 || motorPercentRd < -100) return 2;

	/* Motoren ansteuern */
	vmc->setMotors(motorPercentLd, motorPercentRd);
	return 0;
}

/**
* @brief Haupteinstiegspunkt
* @param[in] argc Anzahl der Kommandozeilenparameter
* @param[in] argv Vektor von Strings der Kommandozeilenparameter
* @return Statuscode der Anwendung
*/
int main(int argc, char **argv)
{
	int i;					  /*< Laufvariable für Schleifen */
	double startTime;         /*< Zeitstempel am Beginn der Messung (zum Subtrahieren) */
	double times_l[MAX_MEAS]; /*< Mess-Zeitpunkte fuer linkes Rad */
	double times_r[MAX_MEAS]; /*< Mess-Zeitpunkte fuer rechtes Rad */
	double enc_l;             /*< Abgeholte Encoderticks linkes Rad */
	double enc_r;             /*< Abgeholte Encoderticks rechtes Rad */
	int    encs_l[MAX_MEAS];  /*< Encoder-Messwerte linkes Rad */
	int    encs_r[MAX_MEAS];  /*< Encoder-Messwerte rechtes Rad */
	int    pf;                /*< Plotfile */
	int    i_p;               /*< Zaehlschleifenindex fuer P-Anteil */
	int    i_i;               /*< Zaehlschleifenindex fuer I-Anteil */
	char   filename[50];      /*< Dateiname */
	char   buffer[80];        /*< line buffer */
	double timestamp_l;       /*< Zeitstempel fuer Encoderticks Links */
	double timestamp_r;       /*< Zeitstempel fuer Encoderticks Rechts */
	double old_timestamp_l;   /*< Vorherg. Zeitstempel fuer Encoderticks Links */
	double old_timestamp_r;   /*< Vorherg. Zeitstempel fuer Encoderticks Rechts */

	int    prop_val[] ={
			100,				// P-Wert für Motor links
			100};				// P-Wert für Motor rechts
	int    integ_val[]={
			9000,				// I-Wert für Motor links
			17000};				// I-Wert für Motor rechts

	VMC::CVmc    vmc("/dev/ttyS0");

	cout << "Verbinde mit VMC" << endl;

	if(!vmc.isConnected())
	{
		 cout << "Verbindung zu VMC fehlgeschlagen" << endl;
		 return 1;
	}

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

	cout << "TEST" << endl;

	double omegal, omegar;
	const double v = 1; /* [m/s] */
	const double omega = 0; /* [rad/s] */
	vOmegaToOmegaWheel(v, omega, &omegal, &omegar);
	cout << "Omega links:  " << omegal << endl;
	cout << "Omega rechts: " << omegar << endl;

	driveRobot(&vmc, omegal, omegar);
	vmc.wait(1000);
	vmc.setMotors(0, 0);

	return 0;

	/*
	cout << "Setze Sollwerte" << endl;

	vmc.resetMotorTicks();
	vmc.setMotors(20, 20);
	vmc.wait(1000);

	cout << "Warte 1/2: Lasse Motor implizit an ..." << endl;
	cout.flush();

	//  vmc.wait(5000);
	usleep(5000*1000);

	cout << "Warte 2/2: Deaktiviere Motor manuell ..." << endl;
	cout.flush();
	*/

	vmc.setMotors(0, 0);

	/*
	cout << "Messe Sprungantwort " << MEAS_VELOCITY << " m/s" << endl;
	vmc.setMotors(MEAS_VELOCITY, MEAS_VELOCITY);

	start_time = time_shot();
	for (i = MAX_MEAS; i >= 0; --i)
	{
		vmc.getMotorState(MOTOR_ID_LEFT,  vmc.MOTOR_TICKS_ABSOLUTE, &enc_l, &timestamp_l);
		vmc.getMotorState(MOTOR_ID_RIGHT, vmc.MOTOR_TICKS_ABSOLUTE, &enc_r, &timestamp_r);
	
		// Messungen für linken Motor
		times_l[i] = timestamp_l - start_time;
		encs_l[i] = enc_l;

		// Messungen für rechten Motor
		times_r[i] = timestamp_r - start_time;
		encs_r[i] = enc_r;

		vmc.wait(MEAS_WAIT);
	}
	*/

	/*
	vmc.getMotorState(MOTOR_ID_LEFT,vmc.MOTOR_TICKS_ABSOLUTE,&enc_l,&timestamp_l);
	vmc.getMotorState(MOTOR_ID_RIGHT,vmc.MOTOR_TICKS_ABSOLUTE,&enc_r,&timestamp_r);

	old_timestamp_l=timestamp_l;
	old_timestamp_r=timestamp_r;
	encs_l[i]=(int)enc_l;
	encs_r[i]=-(int)enc_r;
	times_l[i]=timestamp_l-startTime;
	times_r[i]=timestamp_r-startTime;
	*/

	return 0;
}
