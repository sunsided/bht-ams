#include <iostream>
#include <stdint.h>
#include <math.h>

#include <vmc232.h>
#include "robi.h"
#include "robodrive.h"

using namespace std;

/**
* @brief Hält den Roboter nach der angegebenen Zeit an
* @param[in] vmc Die Steuerinstanz
* @param[in] seconds Die Anzahl der Sekunden
*/
void stopRobotAfter(VMC::CVmc *const vmc, const double seconds)
{
	/* Überwachung der Encoderticks vorbereiten */
	double startTime, timestamp_l, timestamp_r;
	double runTime = 0;

	/* So lange schleifen, bis erwartete Ticks überschritten
     * oder Zeit abgelaufen. */
	vmc->getMotorState(MOTOR_ID_LEFT, vmc->MOTOR_TICKS_ABSOLUTE, NULL, &startTime);
	while (runTime < seconds)
	{
		vmc->wait(1);
		vmc->getMotorState(MOTOR_ID_LEFT,  vmc->MOTOR_TICKS_ABSOLUTE, NULL, &timestamp_l);
		vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, NULL, &timestamp_r);

		runTime = 0.5*((timestamp_l-startTime) + (timestamp_r-startTime));
	}

	/* Motoren anhalten */
	vmc->setMotorRPM(MOTOR_ID_LEFT,  0);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, 0);
}

/**
* @brief Hält den Roboter nach der angegebenen Zeit oder den maximalen Ticks an
* @param[in] vmc Die Steuerinstanz
* @param[in] seconds Die Anzahl der Sekunden
* @param[in] maximumTicks Die maximale Anzahl Ticks, nach der gestoppt werden soll
*/
void stopRobotAfter(VMC::CVmc *const vmc, const double seconds, const double maximumTicks)
{
	/* Überwachung der Encoderticks vorbereiten */
	double startTime, timestamp_l, timestamp_r, 
		enc_l, enc_r, 
		last_enc_l=0, last_enc_r=0;
	double runTime = 0;
	double totalTicks = 0;

	vmc->resetMotorTicks();

	/* Motoren ansteuern */
	vmc->getMotorState(MOTOR_ID_LEFT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_l, &startTime);
	vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_r, NULL);

	/* So lange schleifen, bis erwartete Ticks überschritten
     * oder Zeit abgelaufen. */
	while ((runTime < seconds) && (totalTicks < maximumTicks))
	{
		vmc->wait(1);

		vmc->getMotorState(MOTOR_ID_LEFT,  vmc->MOTOR_TICKS_ABSOLUTE, &enc_l, &timestamp_l);
		vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, &enc_r, &timestamp_r);
		totalTicks += 0.5*(fabs(enc_l)-last_enc_l + fabs(enc_r)-last_enc_r);
		last_enc_l = fabs(enc_l);
		last_enc_r = fabs(enc_r);

		runTime = 0.5*((timestamp_l-startTime) + (timestamp_r-startTime));
	}

	/* Motoren anhalten */
	vmc->setMotorRPM(MOTOR_ID_LEFT,  0);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, 0);
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
	const double v_l = v - omega * halberRadabstand;
	const double v_r = v + omega * halberRadabstand;

	/* Umrechnung von v_rad in omega_rad in [rad/s] */
	*omega_l = v_l / radRadius;
	*omega_r = v_r / radRadius;
}

/**
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten an
* @param[in] vmc Die Treiberinstanz
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
*/
void driveRobot(VMC::CVmc *const vmc, const double omega_l, const double omega_r)
{
	/* Radgeschwindigkeiten in Prozent MAX_RPM ermitteln */
	const double motorRpmL =  omega_l*60.0*GETRIEBE_FAKTOR;
	const double motorRpmR = -omega_r*60.0*GETRIEBE_FAKTOR;

	/* Motoren ansteuern */
	vmc->setMotorRPM(MOTOR_ID_LEFT,  motorRpmL);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, motorRpmR);
}

/**
* @brief Wandelt Winkelgeschwindigkeit Omega des Rades in Drehzahl N des Motors um
* @param[in] omega Winkelgeschwindigkeit des Rades [1/s]
* @return Drehzahl des Motors
*/
double wheelOmegaToMotorRpm(const double omega)
{
	/* Es gilt omega = 2*pi*N <=> N = w/(2*pi) */
	return omega*60.0*GETRIEBE_FAKTOR/(2.0*PI);
}

/**
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten und der Laufzeit an
* @param[in] vmc Die Treiberinstanz
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
* @param[in] seconds Die Laufzeit
*/
void driveRobot(VMC::CVmc *const vmc, const double omega_l, const double omega_r, const double seconds)
{
	/* Radgeschwindigkeiten in Prozent MAX_RPM ermitteln  */
	const double motorRpmL = wheelOmegaToMotorRpm(omega_l;
	const double motorRpmR = wheelOmegaToMotorRpm(omega_r);

	/* Laufzeitmessung vorbereiten */
	const double length = (fabs(omega_l)+fabs(omega_r))*0.5 * seconds;
	const double ticksExpected = length * GETRIEBE_FAKTOR * ENCODER_COUNT;

	/* Motoren ansteuern */
	vmc->setMotorRPM(MOTOR_ID_LEFT,   motorRpmL);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, -motorRpmR);
	stopRobotAfter(vmc, seconds);
}

/**
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten und der Laufzeit an
* @param[in] vmc Die Treiberinstanz
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
* @param[in] seconds Die Laufzeit
*/
void driveRobotEx(VMC::CVmc *const vmc, const double omega_l, const double omega_r, const double seconds)
{
	/* Radgeschwindigkeiten in Prozent MAX_RPM ermitteln */
	const double motorRpmL = wheelOmegaToMotorRpm(omega_l;
	const double motorRpmR = wheelOmegaToMotorRpm(omega_r);

	/* Laufzeitmessung vorbereiten */
	const double length = (fabs(omega_l)+fabs(omega_r))*0.5 * seconds;
	const double ticksExpected = length * GETRIEBE_FAKTOR * ENCODER_COUNT;

	cout << "RPM: links " << motorRpmL << ", rechts " << motorRpmR << endl;
	cout << "Vergleichsstrecke: " << length << endl;
	cout << "Erwartete Ticks:   " << ticksExpected << endl;

	/* Überwachung der Encoderticks vorbereiten */
	double startTime, timestamp_l, timestamp_r, 
		enc_l, enc_r, 
		last_enc_l=0, last_enc_r=0;
	double runTime = 0;
	double totalTicks = 0;

	vmc->resetMotorTicks();

	/* Motoren ansteuern */
	vmc->setMotorRPM(MOTOR_ID_LEFT,   motorRpmL);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, -motorRpmR);
	vmc->getMotorState(MOTOR_ID_LEFT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_l, &startTime);
	vmc->getMotorState(MOTOR_ID_LEFT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_r, NULL);

	/* So lange schleifen, bis erwartete Ticks überschritten
     * oder Zeit abgelaufen. */
	while ((runTime < seconds) && (totalTicks < ticksExpected))
	{
		vmc->getMotorState(MOTOR_ID_LEFT,  vmc->MOTOR_TICKS_ABSOLUTE, &enc_l, &timestamp_l);
		vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, &enc_r, &timestamp_r);
		totalTicks += 0.5*(fabs(enc_l)-last_enc_l + fabs(enc_r)-last_enc_r);
		last_enc_l = fabs(enc_l);
		last_enc_r = fabs(enc_r);

		runTime = 0.5*((timestamp_l-startTime) + (timestamp_r-startTime));

		cout << "Laufzeit: " << runTime << ", totalTicks: " << totalTicks << "/" << ticksExpected << endl;

		vmc->wait(5); /* TODO: abkürzen, wenn Differenz zu Endticks kleiner */
	}

	/* Motoren anhalten */
	vmc->setMotorRPM(MOTOR_ID_LEFT,  0);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, 0);

	cout << "Run time: " << runTime << " of total time:  " << seconds << endl;
	cout << "Ticks:    " << totalTicks << " of expected: " << ticksExpected << endl;
}

/**
* @brief Dreht den Roboter um eine definierte Gradzahl
* @param[in] vmc Die Treiberinstanz
* @param[in] degrees Der Winkel in Grad
* @param[in] seconds Die Zeit in Sekunden
*/
void rotateRobot(VMC::CVmc *const vmc, const double degrees, const double seconds)
{
	static const double v = 0.0;
	const double radians = degrees * PI / 180.0;
	const double omega = radians / seconds;
	const int ms = (int)(0.5+seconds*1000.0);

	double omegal, omegar;
	vOmegaToOmegaWheel(v, omega, &omegal, &omegar);
//	driveRobotDirect(vmc, omegal, omegar);
//	vmc->wait(ms);
	driveRobot(vmc, omegal, omegar, seconds);
}

/**
* @brief Dreht den Roboter um eine definierte Gradzahl
* @param[in] vmc Die Treiberinstanz
* @param[in] degrees Der Winkel in Grad
* @param[in] seconds Die Zeit in Sekunden
*/
void rotateRobot(VMC::CVmc *const vmc, const double degrees)
{
	const double seconds = fabs(degrees)/ROBO_DEGREE_PER_SEC;
	rotateRobot(vmc, degrees, seconds);
}

