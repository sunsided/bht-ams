/*
* Demonstrations-/Kalibrierungsprogramm für Volksbot
* Autonome Mobile Systeme, WS2013
*/

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
int parseCommandLine(int argc, char **argv, 
					double* v, double* omega, double* time, 
					int* pl, int* il, int* pr, int* ir);
