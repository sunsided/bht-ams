#ifndef PHYSIK_H
#define PHYSIK_H

#include <math.h>
#include "robi.h"

using namespace std;

/**
* @brief Wandelt Geschwindigkeiten in v-omega in Radgeschwindigkeiten um
* @param[in] v Die Bahngeschwindigkeit des Roboters [m/s]
* @param[in] omega Die Winkelgeschwindigkeit des Roboters [rad/s]
* @param[out] omega_l Winkelgeschwindigkeit des linken Rades [rad/s]
* @param[out] omega_r Winkelgeschwindigkeit des rechten Rades [rad/s]
*/
static inline void vOmegaToOmegaWheel(const double v, const double omega, double *const omega_l, double *const omega_r)
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
* @brief Wandelt Winkelgeschwindigkeit Omega des Rades in Drehzahl N des Motors um
* @param[in] omega Winkelgeschwindigkeit des Rades [1/s]
* @return Drehzahl des Motors
*/
static inline double wheelOmegaToMotorRpm(const double omega)
{
	/* Es gilt omega = 2*pi*N <=> N = w/(2*pi) */
	return omega*60.0*GETRIEBE_FAKTOR/(2.0*M_PI);
}

#endif
