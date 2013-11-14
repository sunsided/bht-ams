#include <iostream>
#include <stdint.h>
#include <math.h>

#include <vmc232.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include "robi.h"
#include "physik.h"
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
		vmc->wait(5);

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
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten und der Laufzeit an
* @param[in] vmc Die Treiberinstanz
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
* @param[in] seconds Die Laufzeit
*/
void driveRobot(VMC::CVmc *const vmc, const double omega_l, const double omega_r, const double seconds)
{
	/* Radgeschwindigkeiten in Prozent MAX_RPM ermitteln  */
	const double motorRpmL = wheelOmegaToMotorRpm(omega_l);
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
	const double motorRpmL = wheelOmegaToMotorRpm(omega_l);
	const double motorRpmR = wheelOmegaToMotorRpm(omega_r);

	/* Laufzeitmessung vorbereiten */
	const double lengthl = fabs(omega_l) * seconds;
	const double lengthr = fabs(omega_r) * seconds;
	const double ticksExpectedl = lengthl * GETRIEBE_FAKTOR * ENCODER_COUNT;
	const double ticksExpectedr = lengthr * GETRIEBE_FAKTOR * ENCODER_COUNT;

	cout << "RPM: links " << motorRpmL << ", rechts " << motorRpmR << endl;
	cout << "Vergleichsstrecke: links " << lengthl << ", rechts " << lengthr << endl;
	cout << "Erwartete Ticks:  links " << ticksExpectedl << ", rechts " << ticksExpectedr << endl;

	/* Überwachung der Encoderticks vorbereiten */
	double startTimel, startTimer, timestamp_l, timestamp_r, 
		enc_l, enc_r, 
		last_enc_l=0, last_enc_r=0;
	double runTimel = 0, runTimer = 0;
	double totalTicksl = 0, totalTicksr = 0;
	double endTime = seconds; /* Fangnetz */

	vmc->resetMotorTicks();

	/* Motoren ansteuern */
	vmc->setMotorRPM(MOTOR_ID_LEFT,   motorRpmL);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, -motorRpmR);
	vmc->getMotorState(MOTOR_ID_LEFT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_l, &startTimel);
	vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_r, &startTimer);


	/* So lange schleifen, bis erwartete Ticks überschritten
     * oder Zeit abgelaufen. */
	
	while (
		(runTimel < endTime) && 
		(totalTicksl < ticksExpectedl)
	)
	{
		vmc->getMotorState(MOTOR_ID_LEFT,  vmc->MOTOR_TICKS_ABSOLUTE, &enc_l, &timestamp_l);
		vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, &enc_r, &timestamp_r);
		totalTicksl += (fabs(enc_l)-last_enc_l);
		totalTicksr += (fabs(enc_r)-last_enc_r);
		last_enc_l = fabs(enc_l);
		last_enc_r = fabs(enc_r);

		runTimel = (timestamp_l-startTimel);
		runTimer = (timestamp_r-startTimer);

		cout << "Laufzeit: links " << runTimel << ", rechts " << runTimer << ", totalTicks: links " << totalTicksl << "/" << ticksExpectedl << ", rechts " << totalTicksr << "/" << ticksExpectedr << endl;

		vmc->wait(1); /* TODO: abkürzen, wenn Differenz zu Endticks kleiner */
	}

	/* Motoren anhalten */
	vmc->setMotorRPM(MOTOR_ID_LEFT,  0);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, 0);

	cout << "Laufzeit: links " << runTimel << ", rechts " << runTimer << " of total time:  " << seconds << endl;
	cout << "Ticks: links " << totalTicksl  << " of expected: " << ticksExpectedl << ", rechts " << totalTicksr << " of expected: " << ticksExpectedr  << endl;
}

void driveRobotEx2(VMC::CVmc *const vmc, const double omega_l, const double omega_r, const double seconds)
{
	/* Systemzeit nehmen */
	struct timeval systime;   /* Sys clock struct */
	double systimeSec;

	/* Radgeschwindigkeiten in Prozent MAX_RPM ermitteln */
	const double motorRpmL = wheelOmegaToMotorRpm(omega_l);
	const double motorRpmR = wheelOmegaToMotorRpm(omega_r);

	/* Laufzeitmessung vorbereiten */
	const double lengthl = fabs(omega_l) * seconds;
	const double lengthr = fabs(omega_r) * seconds;
	const double ticksExpectedl = lengthl * GETRIEBE_FAKTOR * ENCODER_COUNT;
	const double ticksExpectedr = lengthr * GETRIEBE_FAKTOR * ENCODER_COUNT;

	cout << "RPM: links " << motorRpmL << ", rechts " << motorRpmR << endl;
	cout << "Vergleichsstrecke: links " << lengthl << ", rechts " << lengthr << endl;
	cout << "Erwartete Ticks:  links " << ticksExpectedl << ", rechts " << ticksExpectedr << endl;

	/* Überwachung der Encoderticks vorbereiten */
	double startTimel, startTimer, timestamp_l, timestamp_r, 
		enc_l, enc_r, 
		last_enc_l=0, last_enc_r=0;
	double runTimel = 0, runTimer = 0;
	double totalTicksl = 0, totalTicksr = 0;
	double endTime = seconds; /* Fangnetz */

	vmc->resetMotorTicks();

	/* Motoren ansteuern */
	vmc->setMotorRPM(MOTOR_ID_LEFT,   motorRpmL);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, -motorRpmR);
	vmc->getMotorState(MOTOR_ID_LEFT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_l, &startTimel);
	vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, &last_enc_r, &startTimer);

	

   	gettimeofday(&systime, NULL);            
   	systimeSec= ((double)((int)systime.tv_sec)+(double)systime.tv_usec/1000000.0);

	/* So lange schleifen, bis erwartete Ticks überschritten
     * oder Zeit abgelaufen. */
	
	while (
		(runTimel < endTime) && 
		(totalTicksl < ticksExpectedl)
	)
	{
		vmc->getMotorState(MOTOR_ID_LEFT,  vmc->MOTOR_TICKS_ABSOLUTE, &enc_l, &timestamp_l);
		vmc->getMotorState(MOTOR_ID_RIGHT, vmc->MOTOR_TICKS_ABSOLUTE, &enc_r, &timestamp_r);
		totalTicksl += (fabs(enc_l)-last_enc_l);
		totalTicksr += (fabs(enc_r)-last_enc_r);
		last_enc_l = fabs(enc_l);
		last_enc_r = fabs(enc_r);

		runTimel = (timestamp_l-startTimel);
		runTimer = (timestamp_r-startTimer);

		cout << "Laufzeit: links " << runTimel << ", rechts " << runTimer << ", totalTicks: links " << totalTicksl << "/" << ticksExpectedl << ", rechts " << totalTicksr << "/" << ticksExpectedr << endl;

		vmc->wait(1); /* TODO: abkürzen, wenn Differenz zu Endticks kleiner */
	}

	/* Motoren anhalten */
	vmc->setMotorRPM(MOTOR_ID_LEFT,  0);
	vmc->setMotorRPM(MOTOR_ID_RIGHT, 0);

	cout << "Laufzeit: links " << runTimel << ", rechts " << runTimer << " of total time:  " << seconds << endl;
	cout << "Ticks: links " << totalTicksl  << " of expected: " << ticksExpectedl << ", rechts " << totalTicksr << " of expected: " << ticksExpectedr  << endl;
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
	const double radians = degrees * M_PI / 180.0;
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

