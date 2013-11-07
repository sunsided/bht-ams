#ifndef ROBI_H
#define ROBI_H

// Abstand zwischen den Kontaktpunkten der Räder in m
#define RAD_ABSTAND (35.5 /* [cm] */  / 100 /* [cm/m] */)

// Durchmesser eines Rades in m
#define RAD_DURCHMESSER (8.9 /* [cm] */ / 100.0 /* [cm/m] */)

// Anzahl der Encoderticks
#define ENCODER_COUNT 500

// Getriebeuntersetzungsfaktor
#define GETRIEBE_FAKTOR ((float)225.0/16.0)

// Leerlaufdrehzahl links in U/min
#define MAX_RPM_LEFT 8257

// Leerlaufdrehzahl rechts in U/min
#define MAX_RPM_RIGHT 8632

// Drehung des Roboters pro Tick in µ° (nicht Rad)
#define MICRO_TICK_DEGREE

// Serielle Schnittstelle (device)
#define CNTRL_DEV "/dev/ttyS0"

// Geschwindigkeitsfaktoren, sind definiert als:
// v (cm/s) * SPEED_FACTOR = ansteuerungswert des controllers
#define SPEED_FACTOR_LEFT 	1 // 1.035
#define SPEED_FACTOR_RIGHT	1 // 1.035

// Indexwerte der Motoren für die Controllerbefehle
#define MOTOR_ID_LEFT 1
#define MOTOR_ID_RIGHT 2

#endif
