#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <string.h>
#include <pthread.h>
#include <semaphore.h>
#include <mqueue.h>
#include <errno.h>
#include "dlab.h"


//Declare global variables
#define MAXS 5000			//Maximum no of samples

sem_t data_avail;			// Do not change the name of this semaphore

int k;
int squarestep;			
int motor_number=1;
int cont = 1, cont2 = 0;		//variables to control input 

float theta[MAXS];			// Array for storing motor position
float ref[MAXS];			// Array for storing reference input

double no_of_samples;
double Kp = 30;				// Kp
double Ti=0.027, Td=0.00676, N=20;	// Original PID parameters
double run_time = 3;			// Set the initial run time to 3 seconds
double Fs = 200;			// Set the initial sampling frequency to 200 Hz
double Tt = 0.01;

char selection;

double satblk(double u) 
{

	double clamp;
	if(u>1.4) {
	u = 1.4;
	}

	if(u<-1.4){
	u = -1.4;
	}

	clamp = u;
	//printf("clamp: %lf \n",clamp);
	return clamp;

}

void *Control1(void *arg) {

	double ek, uk, gain_p, gain_i, gain_d, motor_position;
	double prev_i = 0;
	double prev_d = 0;
	double prev_er = 0;
	double prev_uk = 0;
	double Ts;
	double Kt = 0.01;
	double vk = 0;
	double ak = 0;

	do {
		Ts = 1/Fs;
		no_of_samples = (run_time*Fs);



		for (k=0; k<no_of_samples; k++) 
		{
		if (cont2 != 2)
			{ 
				printf("\npress 1 to update Kp, frequency, Ti, Td, N\n");
				printf("\npress 0 to continue without updating\n");
				printf("\npress 2 to not update this whole cycle\n");

				scanf("%d", &cont2);
			}
		if (cont2 == 1)
			{
				printf("\nTune Kp\n");
				scanf("%lf", &Kp);
				printf("\nTune sampling frequency\n");
				scanf("%lf", &Fs);
				printf("\nTune Ti\n");
				scanf("%lf", &Ti);
				printf("\nTune Td\n");
				scanf("%lf", &Td);
				printf("\nTune N\n");
				scanf("%lf", &N);
				cont2 = 0; // reset
			}
			sem_wait(&data_avail);						//sampling frequency update
			motor_position = EtoR(ReadEncoder());
			ek = ref[k] - motor_position;					
			gain_p = Kp*ek;							//proportional control value
		gain_i = prev_i+((Kp/Ti)*prev_er*Ts+ak*(1/Tt)*Ts);				//calculate integral gain
		gain_d = ((Td/(N*Ts)+Td)*prev_d)+(((Kp*Td*N)/((N*Ts)+Td))*(ek-prev_er));	//calculate differential gain
			vk = gain_p + gain_i + gain_d;					
			uk = satblk(vk);						//anti-windup
			ak = uk-vk;

			DtoA(VtoD(uk));
			theta[k] = motor_position;

			//updates
			prev_i = gain_i;
			prev_d = gain_d;
			prev_er = ek;
		}

		
		//realtime control 
		printf("\npress 1 to continue or 0 to exit\n");
		//sleep(100); 
		//unsleep();
		scanf("%d", &cont);

		cont2 = 0; //reset
	}while (cont == 1);

	sem_post(&data_avail);
	pthread_exit(NULL);
}

int main(void *arg) {
	pthread_t Control;

	double mag, freq, dcyc;

	do {
		no_of_samples = (run_time*Fs);
		printf("\npress 'r' to run\npress 'p' to change Kp\npress 'f' to change the sampling frequency\npress 't' to change run time\npress 'u' to change the input (step or square)\npress 'g' to plot the results on the screen\npress 'h' to save the plot results in Postscript\npress 'i' to change the value of Ti\npress 'd' to change the value of Td\npress 'n' to change the value of N\npress 'q' to quit\n");

		scanf("%c[^\n]", &selection);
		printf("\n");
		switch (selection) {
			case 'r':
				//Run
				sem_init(&data_avail, 0, 0);
				Initialize(DLAB_SIMULATE, Fs, motor_number);
				if (pthread_create(&Control, NULL, &Control1, NULL) != 0) {
					    printf("Error in creating thread 1\n");
					    exit(1);
				    }

				cont = 0;

				pthread_join(Control, NULL);
				Terminate();
				sem_destroy(&data_avail);
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					printf("\nthe new value of Ti is: %lf \n",Ti);
					printf("\nthe new value of Td is: %lf \n",Td);
					printf("\nthe new value of N is: %lf \n",N);
				break;

			case 'p':
				//change the value of Kp
				printf("\nSelect a new value for Kp\n");
				scanf("%lf", &Kp);
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					printf("\nthe new value of Ti is: %lf \n",Ti);
					printf("\nthe new value of Td is: %lf \n",Td);
					printf("\nthe new value of N is: %lf \n",N);
				break;

			case 'f':
				//change the value of Fs
				printf("\nSelect a new sampling frequency\n");
				scanf("%lf", &Fs);
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					printf("\nthe new value of Ti is: %lf \n",Ti);
					printf("\nthe new value of Td is: %lf \n",Td);
					printf("\nthe new value of N is: %lf \n",N);
				break;

			case 't':
				//Change the value of run_time
				printf("\nSelect a new duration for the run time\n");
				scanf("%lf", &run_time);
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					printf("\nthe new value of Ti is: %lf \n",Ti);
					printf("\nthe new value of Td is: %lf \n",Td);
					printf("\nthe new value of N is: %lf \n",N);
				break;

			case 'u':
				// prompt user for type of input: Step or Square
				printf("\npress 1 for step input\npress 2 for square input\n");
				scanf("%d", &squarestep);

				if (squarestep == 1) {
					//prompt for magnitude of the step
					//set up the step reference input {ref[k]}
					printf("\nSelect a magnitude\n");
					scanf("%lf", &mag);
					for (k=0; k<no_of_samples; k++) {
						ref[k] = mag*PI/180;
					}
				}
				if (squarestep == 2) {
					//prompt for magnitude, frequency and duty cycle
					//set up {ref[k]} using DLaB function Square()
					printf("\nSelect a magnitude\n");
					scanf("%lf", &mag);
					printf("\nSelect a frequency\n");
					scanf("%lf", &freq);
					printf("\nSelect a duty cylce\n");
					scanf("%lf", &dcyc);
						printf("\nthe new Kp is: %lf \n",Kp);
						printf("\nthe new Fs is: %lf \n",Fs);
						printf("\nthe new runtime is: %lf \n",run_time);
						printf("\nthe new mag is: %lf \n",mag);
						printf("\nthe new freq is: %lf \n",freq);
						printf("\nthe new dcyc is: %lf \n",dcyc);
						printf("\nthe number of samples is: %lf \n",no_of_samples);
						printf("\nthe new value of Ti is: %lf \n",Ti);
						printf("\nthe new value of Td is: %lf \n",Td);
						printf("\nthe new value of N is: %lf \n",N);
					Square(ref, no_of_samples, Fs, mag*PI/180, freq, dcyc);
				}
				break;

			case 'g':
				//plot results
						//sem_init(&data_avail, 0, 0);
						//Initialize(DLAB_SIMULATE, Fs, motor_number);
						//if (pthread_create(&Control, NULL, &Control1, NULL) != 0) {
						//	    printf("Error in creating thread 1\n");
						//	    exit(1);
						 //   }

						//pthread_join(Control, NULL);
						//Terminate();
						//sem_destroy(&data_avail);

						plot(ref, theta, Fs, no_of_samples, SCREEN, "Graph Title", "x-axis", "y-axis");


						printf("\nthe new Kp is: %lf \n",Kp);
						printf("\nthe new Fs is: %lf \n",Fs);
						printf("\nthe new value of Ti is: %lf \n",Ti);
						printf("\nthe new value of Td is: %lf \n",Td);
						printf("\nthe new value of N is: %lf \n",N);

						cont = 1;
				break;

			case 'h':
				//Save the plot results in Postscript
				plot(ref, theta, Fs, no_of_samples, PS, "Graph Title", "x-axis", "y-axis");
				break;

			case 'i':
				//change the value of Kp
				printf("\nSelect a new value for Ti\n");
				scanf("%lf", &Ti);
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					printf("\nthe new value of Ti is: %lf \n",Ti);
					printf("\nthe new value of Td is: %lf \n",Td);
					printf("\nthe new value of N is: %lf \n",N);
				break;

			case 'd':
				//change the value of Kp
				printf("\nSelect a new value for Td\n");
				scanf("%lf", &Td);
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					printf("\nthe new value of Ti is: %lf \n",Ti);
					printf("\nthe new value of Td is: %lf \n",Td);
					printf("\nthe new value of N is: %lf \n",N);
				break;

			case 'n':
				//change the value of Kp
				printf("\nSelect a new value for N\n");
				scanf("%lf", &N);
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					printf("\nthe new value of Ti is: %lf \n",Ti);
					printf("\nthe new value of Td is: %lf \n",Td);
					printf("\nthe new value of N is: %lf \n",N);
				break;

			case 'q':
				//quit
				exit(0);
		}
	}while (selection != 'q');
}
