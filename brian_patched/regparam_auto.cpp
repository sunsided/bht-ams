#include <unistd.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <vmc232.h>
#include "robi.h"
#include <sys/time.h>  // NEW

double time_shot();  // NEW

#define MAX_MEAS 100  // Anzahl der Messwerte
#define MAX_RPM 900	  // Beeinflusst Regelauflösung

#define MAXON_ENCODER_TICKS_PER_TURN	(ENCODER_COUNT)
#define TICKS_PER_ROUND 				((double)ENCODER_COUNT)

#define OMEGA_RAD						(1.0) // Hz
#define MOTOR_PERCENT					((int)(0.5 + OMEGA_RAD*60*GETRIEBE_FAKTOR*100.0/(double)MAX_RPM))

double time_shot()    // NEW
{
   struct timeval tv;                       // Sys clock struct

   gettimeofday(&tv, NULL);                 // Systemzeit abrufen
   return ((double)((int)tv.tv_sec)+
          (double)tv.tv_usec/1000000.0);    // Zeit in Sekunden
}


int main(int argc, char **argv)
{
  
  int i;
  double startTime;         // Zeitstempel am Beginn der Messung (zum Subtrahieren)
  double times_l[MAX_MEAS]; // Mess-Zeitpunkte fuer linkes Rad
  double times_r[MAX_MEAS]; // Mess-Zeitpunkte fuer rechtes Rad
  double enc_l;             // Abgeholte Encoderticks linkes Rad
  double enc_r;             // Abgeholte Encoderticks rechtes Rad
  int    encs_l[MAX_MEAS];  // Encoder-Messwerte linkes Rad
  int    encs_r[MAX_MEAS];  // Encoder-Messwerte rechtes Rad
  int    pf;                // Plotfile
  int    i_p;               // Zaehlschleifenindex fuer P-Anteil
  int    i_i;               // Zaehlschleifenindex fuer I-Anteil
  char   filename[50];      // Dateiname
  char   buffer[80];        // line buffer
  int    prop_val[] ={80,100,150};
  int    integ_val[]={9000,12000,17000};
  double speed_factor[] = {1.035, 1.035}; // Geschwindigkeits-Korrekturfaktoren
  double timestamp_l;       // Zeitstempel fuer Encoderticks Links
  double timestamp_r;       // Zeitstempel fuer Encoderticks Rechts
  double old_timestamp_l;   // Vorherg. Zeitstempel fuer Encoderticks Links
  double old_timestamp_r;   // Vorherg. Zeitstempel fuer Encoderticks Rechts

  VMC::CVmc    vmc("/dev/ttyS0");

  if(!vmc.isConnected())
  {
     printf ("Error connecting to VMC\n");
     return 1;
  }

  printf("------------------------------------------------------------\n"); 

  vmc.setMaximumRPM(MAX_RPM);
  vmc.setTicksPerTurn(MOTOR_ID_LEFT, TICKS_PER_ROUND);
  vmc.setTicksPerTurn(MOTOR_ID_RIGHT,TICKS_PER_ROUND);

  vmc.setCycleTime(30);
  vmc.setSpeedFactor(MOTOR_ID_LEFT, speed_factor[0]);
  vmc.setSpeedFactor(MOTOR_ID_RIGHT, speed_factor[1]);
  vmc.clearResponseList();
  vmc.addStateToResponseList(vmc.MOTOR_TICKS_ABSOLUTE);

  for (i_p=0; i_p<(sizeof(prop_val)/sizeof(int)); i_p++)
  {
    for (i_i=0; i_i<(sizeof(integ_val)/sizeof(int)); i_i++)
    {
      printf("PI-Regler konfiguration P:%d I:%d\n",
             prop_val[i_p],integ_val[i_i]);

      vmc.setMotorLinearPart(MOTOR_ID_LEFT,prop_val[i_p]);
      vmc.setMotorLinearPart(MOTOR_ID_RIGHT,prop_val[i_p]);
      vmc.setMotorIntegralPart(MOTOR_ID_LEFT,integ_val[i_i]);
      vmc.setMotorIntegralPart(MOTOR_ID_RIGHT,integ_val[i_i]);

      vmc.resetMotorTicks();
      vmc.wait(1000);
      vmc.getMotorState(MOTOR_ID_LEFT,vmc.MOTOR_TICKS_ABSOLUTE,NULL,&startTime);

      vmc.setMotors(MOTOR_PERCENT, MOTOR_PERCENT);
      old_timestamp_l=-1;
      old_timestamp_r=-1;
      for(i=0; i<MAX_MEAS;)
      {
        vmc.getMotorState(MOTOR_ID_LEFT,vmc.MOTOR_TICKS_ABSOLUTE,&enc_l,&timestamp_l);
        vmc.getMotorState(MOTOR_ID_RIGHT,vmc.MOTOR_TICKS_ABSOLUTE,&enc_r,&timestamp_r);
        if ((old_timestamp_l!=timestamp_l)&&(old_timestamp_r!=timestamp_r))
        {
          old_timestamp_l=timestamp_l;
          old_timestamp_r=timestamp_r;
          encs_l[i]=(int)enc_l;
          encs_r[i]=-(int)enc_r;
          times_l[i]=timestamp_l-startTime;
          times_r[i]=timestamp_r-startTime;
          //printf("ts: %0.3f lR: %5i rR: %i \n", timestamp_l, encs_l[i], encs_r[i]);
          i++;
        }
        vmc.wait(30);
      }
    
      for(i=0; i<MAX_MEAS; i++)
      {
        printf("ts: %0.3f lR: %5d (%0.3f) rR: %d (%0.3f)\n", times_l[i], encs_l[i], 
			(float)((encs_l[i]-encs_l[i-1])/((times_l[i]-times_l[i-1]))), encs_r[i], 
			(float)((encs_r[i]-encs_r[i-1])/((times_r[i]-times_r[i-1])))
);
      }  
    
      // Datei fuer linkes Rad und aktueller Messung anlegen
      sprintf(filename,"robi_wheeldat_left_p%d_i%d.txt",
              prop_val[i_p],
              integ_val[i_i]);
      pf=open(filename,O_CREAT|O_TRUNC|O_WRONLY,S_IRWXU);
      if (pf<=0)
      {
         perror("open");
         return 2;
      }
      write(pf,"# t  x\n\0",7);
      for(i=1; i<MAX_MEAS; i++) {
        sprintf(buffer,"   %0.3f %0.3f\n", times_l[i], (double)((encs_l[i]-encs_l[i-1])/((times_l[i]-times_l[i-1])) / ((double)GETRIEBE_FAKTOR * (double)ENCODER_COUNT) ) );
        write(pf,buffer,strlen(buffer));
      }
      close(pf);
    
      // Datei fuer rechtes Rad und aktueller Messung anlegen
      sprintf(filename,"robi_wheeldat_right_p%d_i%d.txt",
              prop_val[i_p],
              integ_val[i_i]);
      pf=open(filename,O_CREAT|O_TRUNC|O_WRONLY,S_IRWXU);
      if (pf<=0)
      {
         perror("open");
         return 2;
      }
      write(pf,"# x  t\n\0",7);
      for(i=1; i<MAX_MEAS; i++) {
        sprintf(buffer,"   %0.3f %0.3f\n", times_r[i], (double)( (encs_r[i]-encs_r[i-1])/((times_r[i]-times_r[i-1])) / ((double)GETRIEBE_FAKTOR * (double)ENCODER_COUNT) ) );
        write(pf,buffer,strlen(buffer));
      }
      close(pf);
      printf("Anhalten\n");
      for (i=MOTOR_PERCENT;i>=0;i--)
      {
        // Sanft abbremsen
        vmc.wait(50);
        vmc.setMotors(i, i);
      }
	
		/*
      vmc.setMotorLinearPart(MOTOR_ID_LEFT,100);
      vmc.setMotorLinearPart(MOTOR_ID_RIGHT,100);
      vmc.setMotorIntegralPart(MOTOR_ID_LEFT,12000);
      vmc.setMotorIntegralPart(MOTOR_ID_RIGHT,12000);
      vmc.wait(1000);
      printf("Zurueckfahren\n");
      vmc.setMotors(-MOTOR_PERCENT, -MOTOR_PERCENT);
      vmc.wait(5000);
*/

	/*
      printf("Einlenken\n");
      vmc.setMotors(-10, -10);
      vmc.wait(1000);
      vmc.setMotors(-10, 0);
      vmc.wait(500);
      vmc.setMotors(0, -10);
      vmc.wait(500);
      printf("Vorfahren\n");
      vmc.setMotors(10, 0);
      vmc.wait(500);
      vmc.setMotors(0, 10);
      vmc.wait(1000);
      vmc.setMotors(10, 0);
      vmc.wait(500);
      vmc.setMotors(5, 5);
      vmc.wait(2000);
	*/
      printf("Anhalten\n");
      vmc.setMotors(0, 0);
      vmc.wait(1000);
    } // for (i_i ...)
    //printf("ENTER fuer naechste Messung druecken\n");
    //fgets(filename,19,stdin); // filename als dummy
  } // for (i_p ...)

  return 0;
}
