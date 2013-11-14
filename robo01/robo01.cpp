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
#include "robodrive.h"
#include "configureRobot.h"
#include "commandLine.h"
#include "physik.h"

using namespace std;

/**
* @brief Haupteinstiegspunkt
* @param[in] argc Anzahl der Kommandozeilenparameter
* @param[in] argv Vektor von Strings der Kommandozeilenparameter
* @return Statuscode der Anwendung
*/
int main(int argc, char **argv)
{
	double v = 0, omega = 0, time = 0;
	int    pl = 0, il = 0,
		   pr = 0, ir = 0;

	/* Kommandozeilenparameter auslesen */
	int status = parseCommandLine(argc, argv, &v, &omega, &time, &pl, &il, &pr, &ir);
	if (status != 0)
	{
		return status;
	}

	/* Parameter ausgeben */
	cout << "Setze Geschwindigkeit:       " << v << " m/s" << endl;
	cout << "Setze Winkelgeschwindigkeit: " << omega << " rad/s" << endl;
	cout << "Setze Zeit:                  " << time << " s" << endl;

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

	/* fahren */
	driveRobot(&vmc, omegal, omegar, time);

	cout << "Ende." << endl;
	vmc.setMotors(0, 0);
	vmc.wait(1000);
	return 0;
}

