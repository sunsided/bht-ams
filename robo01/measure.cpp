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
#include <math.h>
#include <deque>
#include <vector>
#include <iterator>
#include <string>
#include <sstream>

#include <vmc232.h>
#include <sys/time.h>
#include "robi.h"
#include "physik.h"
#include "configureRobot.h"
#include "commandLine.h"

using namespace std;

/**
* @brief Haupteinstiegspunkt
* @param[in] argc Anzahl der Kommandozeilenparameter
* @param[in] argv Vektor von Strings der Kommandozeilenparameter
* @return Statuscode der Anwendung
*/
int main(int argc, char **argv)
{
	double v = 0, omega = M_PI/2, time = 5;
	int    pl = 0, il = 0,
		   pr = 0, ir = 0;

	/* Kommandozeilenparameter auslesen */
	int status = parseCommandLine(argc, argv, &v, &omega, &time, &pl, &il, &pr, &ir);
	if (status != 0)
	{
		return status;
	}

	/* Überschreiben der P- und I-Werte */
	if (pl > 0)
	{
		cout << "Setze P-Wert links:          " << pl << endl;
	}
	if (pr > 0)
	{
		cout << "Setze P-Wert rechts:         " << pr << endl;
	}
	if (il > 0)
	{
		cout << "Setze I-Wert links:          " << il << endl;
	}
	if (ir > 0)
	{
		cout << "Setze I-Wert rechts:         " << ir << endl;
	}

	VMC::CVmc    vmc("/dev/ttyS0");

	cout << "Verbinde mit VMC" << endl;
	if(!vmc.isConnected())
	{
		 cerr << "Verbindung zu VMC fehlgeschlagen" << endl;
		 return 1;
	}

	cout << "Vorbereitung des Roboters ..." << endl;
	configureRobot(vmc, pl, pr, il, ir);

	cout << "Betrieb ..." << endl;

	/* Inverse Kinematik */
	double omegal, omegar;
	vOmegaToOmegaWheel(v, omega, &omegal, &omegar);
	cout << "* Omega links:  " << omegal << endl;
	cout << "* Omega rechts: " << omegar << endl;

	/* Radgeschwindigkeiten in Prozent MAX_RPM ermitteln */
	const double motorRpmL = wheelOmegaToMotorRpm(omegal);
	const double motorRpmR = wheelOmegaToMotorRpm(omegar);

	/* Laufzeitmessung vorbereiten */
	const double length = (fabs(omegal)+fabs(omegar))*0.5 * time;
	const double ticksExpected = length * GETRIEBE_FAKTOR * ENCODER_COUNT;

	cout << "RPM: links " << motorRpmL << ", rechts " << motorRpmR << endl;
	cout << "Vergleichsstrecke: " << length << endl;
	cout << "Erwartete Ticks:   " << ticksExpected << endl;

	std::vector<int> p_values;
	std::vector<int> i_values;
	p_values.push_back(70);
	p_values.push_back(80);
	p_values.push_back(90);

	i_values.push_back(9000);
	i_values.push_back(10000);
	i_values.push_back(12500);
	i_values.push_back(15000);
	i_values.push_back(17000);

	/* Parameter durchspulen */
	for (int i=0; i<10; ++i)
	{
		/* Überwachung der Encoderticks vorbereiten */
		double startTime, timestamp_l, timestamp_r, old_timestamp_l, old_timestamp_r, 
			enc_l, enc_r, 
			last_enc_l=0, last_enc_r=0;
		double runTime = 0;
		double totalTicks = 0;
		double endTime = time * 1.25; /* Fangnetz */
		std::deque<double> encs_l;
		std::deque<double> encs_r;
		std::deque<double> times_l;
		std::deque<double> times_r;

		vmc.resetMotorTicks();

		/* Motoren ansteuern */
		vmc.setMotorRPM(MOTOR_ID_LEFT,   motorRpmL);
		vmc.setMotorRPM(MOTOR_ID_RIGHT, -motorRpmR);
		vmc.getMotorState(MOTOR_ID_LEFT,  vmc.MOTOR_TICKS_ABSOLUTE, &last_enc_l, &startTime);
		vmc.getMotorState(MOTOR_ID_RIGHT, vmc.MOTOR_TICKS_ABSOLUTE, &last_enc_r, NULL);

		/* So lange schleifen, bis erwartete Ticks überschritten
		 * oder Zeit abgelaufen. */
		while (
			(runTime < endTime) && 
			(totalTicks < ticksExpected)
		)
		{
			vmc.getMotorState(MOTOR_ID_LEFT,  vmc.MOTOR_TICKS_ABSOLUTE, &enc_l, &timestamp_l);
			vmc.getMotorState(MOTOR_ID_RIGHT, vmc.MOTOR_TICKS_ABSOLUTE, &enc_r, &timestamp_r);
			totalTicks += 0.5*(fabs(enc_l)-last_enc_l + fabs(enc_r)-last_enc_r);
			last_enc_l = fabs(enc_l);
			last_enc_r = fabs(enc_r);
			runTime = 0.5*((timestamp_l-startTime) + (timestamp_r-startTime));
			cout << "Laufzeit: " << runTime << ", totalTicks: " << totalTicks << "/" << ticksExpected << endl;

			if ( (old_timestamp_l != timestamp_l) && 
				 (old_timestamp_r!=timestamp_r) )
			{
				old_timestamp_l = timestamp_l;
				old_timestamp_r = timestamp_r;

				encs_l.push_back(enc_l);
				encs_r.push_back(-enc_r);
				times_l.push_back(timestamp_l - startTime);
				times_r.push_back(timestamp_r - startTime);

				i++;
			}

			vmc.wait(10);
		}

		/* Motoren anhalten */
		vmc.setMotorRPM(MOTOR_ID_LEFT,  0);
		vmc.setMotorRPM(MOTOR_ID_RIGHT, 0);

		cout << "Run time: " << runTime << " of total time:  " << time << endl;
		cout << "Ticks:    " << totalTicks << " of expected: " << ticksExpected << endl;

		std::stringstream filename;
		filename << "robi_wheeldat_left_p" << p << "_i" << i << ".txt";

//		ofstream file;
//		file.open("lol");
		for(i=0; i<encs_l.size(); i++)
		{
			enc_l = encs_l.front();
			enc_r = encs_r.front();
			timestamp_l = times_l.front();
			timestamp_r = times_r.front();

			encs_l.pop_front();
			encs_r.pop_front();
			times_l.pop_front();
			times_r.pop_front();

#if 0
        	printf("ts: %0.3f lR: %5d (%0.3f) rR: %d (%0.3f)\n", times_l[i], encs_l[i], 
				(float)((encs_l[i]-encs_l[i-1])/((times_l[i]-times_l[i-1]))), encs_r[i], 
				(float)((encs_r[i]-encs_r[i-1])/((times_r[i]-times_r[i-1])))
			);
#endif
		} 
	}

	cout << "Ende." << endl;
	vmc.setMotors(0, 0);
	return 0;
}

