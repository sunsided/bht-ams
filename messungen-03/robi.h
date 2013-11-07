#ifndef ROBI_H
#define ROBI_H

// Abstand zwischen den Kontaktpunkten der Räder in cm
#define RAD_ABSTAND

// Durchmesser eines Rades in cm
#define RAD_DURCHMESSER 9.81  // VALUE NEW

// Anzahl der Encoderticks  // NEW
#define ENCODER_COUNT 500   // LINE NEW

// Getriebeuntersetzungsfaktor // NEW
#define GETRIEBE_FAKTOR 14     // LINE NEW

// Leerlaufdrehzahl links in U/min
#define MAX_RPM_LEFT 8257      // VALUE NEW
// Leerlaufdrehzahl rechts in U/min
#define MAX_RPM_RIGHT 8632     // VALUE NEW

// Drehung des Roboters pro Tick in µ° (nicht Rad)
#define MICRO_TICK_DEGREE

// Serielle Schnittstelle (device)
#define CNTRL_DEV "/dev/ttyS0"

// Geschwindigkeitsfaktoren, sind definiert als:
// v (cm/s) * SPEED_FACTOR = ansteuerungswert des controllers
#define SPEED_FACTOR_LEFT
#define SPEED_FACTOR_RIGHT

// Indexwerte der Motoren für die Controllerbefehle
#define MOTOR_ID_LEFT 1
#define MOTOR_ID_RIGHT 2

#endif
