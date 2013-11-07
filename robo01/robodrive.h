/*
* Demonstrations-/Kalibrierungsprogramm für Volksbot
* Autonome Mobile Systeme, WS2013
*
* Header für High-Level-Robotersteuerung.
*
* VMC:
*	- file:///usr/share/doc/vmc/html/index.html
*	- /usr/share/vmc/example_apps
*/

#ifndef _ROBODRIVE_H_
#define _ROBODRIVE_H_

#include <vmc232.h>

/**
* @brief Rotationsgeschwindigkeit in Grad pro Sekunde
*/
#define ROBO_DEGREE_PER_SEC	(90/2)

/**
* @brief PI
*/ 
#ifndef PI
#define PI 				(3.141592654)
#endif

/**
* @brief Wandelt Geschwindigkeiten in v-omega in Radgeschwindigkeiten um
* @param[in] v Die Bahngeschwindigkeit des Roboters [m/s]
* @param[in] omega Die Winkelgeschwindigkeit des Roboters [rad/s]
* @param[out] omega_l Winkelgeschwindigkeit des linken Rades [rad/s]
* @param[out] omega_r Winkelgeschwindigkeit des rechten Rades [rad/s]
*/
void vOmegaToOmegaWheel(const double v, const double omega, double *const omega_l, double *const omega_r);

/**
* @brief Hält den Roboter nach der angegebenen Zeit an
* @param[in] vmc Die Steuerinstanz
* @param[in] seconds Die Anzahl der Sekunden
*/
void stopRobotAfter(VMC::CVmc *const vmc, const double seconds);

/**
* @brief Hält den Roboter nach der angegebenen Zeit oder den maximalen Ticks an
* @param[in] vmc Die Steuerinstanz
* @param[in] seconds Die Anzahl der Sekunden
* @param[in] maximumTicks Die maximale Anzahl Ticks, nach der gestoppt werden soll
*/
void stopRobotAfter(VMC::CVmc *const vmc, const double seconds, const double maximumTicks);

/**
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten an
* @param[in] vmc Die Treiberinstanz
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
*/
void driveRobot(VMC::CVmc *const vmc, const double omega_l, const double omega_r);
/**
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten und der Laufzeit an
* @param[in] vmc Die Treiberinstanz
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
* @param[in] seconds Die Laufzeit
*/
void driveRobot(VMC::CVmc *const vmc, const double omega_l, const double omega_r, const double seconds);

/**
* @brief Steuert die Motoren unter Angabe der Radgeschwindigkeiten und der Laufzeit an (Debugging-Funktion)
* @param[in] vmc Die Treiberinstanz
* @param[in] omega_l Winkelgeschwindigkeit des linken Rades in [rad/s]
* @param[in] omega_r Winkelgeschwindigkeit des rechten Rades in [rad/s]
* @param[in] seconds Die Laufzeit
*/
void driveRobotEx(VMC::CVmc *const vmc, const double omega_l, const double omega_r, const double seconds);

/**
* @brief Dreht den Roboter um eine definierte Gradzahl
* @param[in] vmc Die Treiberinstanz
* @param[in] degrees Der Winkel in Grad
* @param[in] seconds Die Zeit in Sekunden
*/
void rotateRobot(VMC::CVmc *const vmc, const double degrees);

/**
* @brief Dreht den Roboter um eine definierte Gradzahl
* @param[in] vmc Die Treiberinstanz
* @param[in] degrees Der Winkel in Grad
* @param[in] seconds Die Zeit in Sekunden
*/
void rotateRobot(VMC::CVmc *const vmc, const double degrees, const double seconds);

#endif /* _ROBODRIVE_H_ */
