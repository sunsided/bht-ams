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

#include <errno.h>
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

using namespace std;

#define MAX_V 0.33
#define MAX_W 1.86

/**
* @brief Parst die Kommandozeilenparameter
* @param[in] argc Anzahl der Kommandozeilenparameter
* @param[in] argv Vektor von Strings der Kommandozeilenparameter
* @param[out] v Die Geschwindigkeit
* @param[out] omega Der Winkelgeschwindigkeit
* @param[out] time Die Zeit
* @param[out] pl P-Wert linker Motor
* @param[out] il I-Wert linker Motor
* @param[out] pr P-Wert rechter Motor
* @param[out] ir I-Wert linker Motor
* @return Statuscode der Anwendung
*/
int parseCommandLine(int argc, char **argv, double* v, double* omega, double* time, int* pl, int* il, int* pr, int* ir)
{
	*v = 0;
	*omega = 0;
	*time = 0;
	*pl = 0;
	*pr = 0;
	*il = 0;
	*ir = 0;
		
	/* Hilfetext */
	if (argc == 1)
	{
		cout << "VERWENDUNG" << endl << argv[0] << " -t <ZEIT> [-v <Wert>] [-w <Wert>] [-pl <Wert>] [-pr <Wert>] [-il <Wert>] [-ir <Wert>]" << endl << endl;
		cout << "PARAMETER BEWEGUNG" << endl
			<< "-t <ZEIT>	Zeit in s" << endl
			<< "-v <WERT>	Bahngeschwindigkeit in m/s (0 .. 0.33)" << endl
			<< "-w <WERT>	Winkelgeschwindigkeit in rad/s (-1.86 .. 1.86)" << endl
			<< endl << "PARAMETER REGLER" << endl
			<< "-pl <WERT>	P-Wert des linken Motors" << endl
			<< "-pr <WERT>	P-Wert des rechten Motors" << endl
			<< "-il <WERT>	I-Wert des linken Motors" << endl
			<< "-ir <WERT>	I-Wert des rechten Motors" << endl;
		return 1;
	}

	/* alle Parameter durchlaufen */
	for(int i=0; i<argc; ++i)
	{
		char* token = argv[i];

		/* velocity */
		if (0 == strcmp(token, "-v"))
		{
			if (i==(argc-1))
			{
				cerr << "Fehlender Wert für -v" << endl;
				return -1;
			}

			/* parsen */
			char *endptr;
			*v = strtod(argv[i+1], &endptr);
			if (endptr == argv[i+1])
			{
				cerr << "Ungültiger Wert für -v" << endl;
				return -2;
			}

			/* gelesenen Wert überspringen */
			++i;
		}

		/* omega */
		if (0 == strcmp(token, "-w"))
		{
			if (i==(argc-1))
			{
				cerr << "Fehlender Wert für -w" << endl;
				return -1;
			}

			/* parsen */
			char *endptr;
			*omega = strtod(argv[i+1], &endptr);
			if (endptr == argv[i+1])
			{
				cerr << "Ungültiger Wert für -w" << endl;
				return -2;
			}

			/* gelesenen Wert überspringen */
			++i;
		}

		/* Zeit */
		if (0 == strcmp(token, "-t"))
		{
			if (i==(argc-1))
			{
				cerr << "Fehlender Wert für -t" << endl;
				return -1;
			}

			/* parsen */
			*time = strtod(argv[i+1], NULL);
			if (*time <= 0)
			{
				cerr << "Ungültiger Wert für -t" << endl;
				return -2;
			}

			/* gelesenen Wert überspringen */
			++i;
		}

		/* P-Wert links */
		if (0 == strcmp(token, "-pl"))
		{
			if (i==(argc-1))
			{
				cerr << "Fehlender Wert für -pl" << endl;
				return -1;
			}

			/* parsen */
			*pl = strtol(argv[i+1], NULL, 10);
			if (*pl <= 0)
			{
				cerr << "Ungültiger Wert für -pl" << endl;
				return -2;
			}

			/* gelesenen Wert überspringen */
			++i;
		}

		/* P-Wert rechts */
		if (0 == strcmp(token, "-pr"))
		{
			if (i==(argc-1))
			{
				cerr << "Fehlender Wert für -pr" << endl;
				return -1;
			}

			/* parsen */
			*pr = strtol(argv[i+1], NULL, 10);
			if (*pr <= 0)
			{
				cerr << "Ungültiger Wert für -pr" << endl;
				return -2;
			}

			/* gelesenen Wert überspringen */
			++i;
		}

		/* I-Wert links */
		if (0 == strcmp(token, "-il"))
		{
			if (i==(argc-1))
			{
				cerr << "Fehlender Wert für -il" << endl;
				return -1;
			}

			/* parsen */
			*il = strtol(argv[i+1], NULL, 10);
			if (*il <= 0)
			{
				cerr << "Ungültiger Wert für -il" << endl;
				return -2;
			}

			/* gelesenen Wert überspringen */
			++i;
		}

		/* I-Wert rechts */
		if (0 == strcmp(token, "-ir"))
		{
			if (i==(argc-1))
			{
				cerr << "Fehlender Wert für -ir" << endl;
				return -1;
			}

			/* parsen */
			*ir = strtol(argv[i+1], NULL, 10);
			if (*ir <= 0)
			{
				cerr << "Ungültiger Wert für -ir" << endl;
				return -2;
			}

			/* gelesenen Wert überspringen */
			++i;
		}
	}

	/* Abtesten fehlender Werte */
	if (*time <= 0)
	{
		cerr << "Fehlender Wert für Zeit (-t)" << endl;
		return -2;
	}

	if (*v == 0 && *omega == 0)
	{
		cerr << "Fehlender Wert für Bahn- (-v) oder Winkelgeschwindigkeit (-w)" << endl;
		return -2;
	}

	if (fabs(*v) > MAX_V)
	{
		cerr << "Bahngeschwindigkeit ungültig: " << *v << " (maximal " << MAX_V << ")" << endl;
		return -2;
	}

	if (fabs(*omega) > MAX_W)
	{
		cerr << "Winkelgeschwindigkeit ungültig: " << *omega << " (maximal +/-" << MAX_W << ")" << endl;
		return -2;
	}

	return 0;
}

