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
#define MAXS 5000		// Maximum no of samples set to 5000

sem_t data_avail;		// defining the semaphore

int k;
int squarestep;			// the input reference
int motor_number=7;		// Servo number (in the lab)

float theta[MAXS];		// Array for storing motor position
float ref[MAXS];		// Array for storing reference input

double no_of_samples;		// number of smaples, used by the control thread and the plot
double Kp = 1;			// Initialize Kp to 1
double run_time = 3;		// Set the initial run time to 3 seconds
double Fs = 200;		// Set the initial sampling frequency to 200 Hz

char selection;


void *Control1(void *arg) {
	
	double ek, uk, motor_position; 
	
	no_of_samples = (run_time*Fs);

	for (k=0; k<no_of_samples; k++) {
		sem_wait(&data_avail);
		motor_position = EtoR(ReadEncoder());
		ek = ref[k] - motor_position;		//calculate tracking error
		uk = Kp*ek;				//calculate control value
		DtoA(VtoD(uk));
		theta[k] = motor_position;
	}

	sem_post(&data_avail);
	pthread_exit(NULL);
}


int main(void *arg) {
	pthread_t Control;

	double mag, freq, dcyc;

	do {
		no_of_samples = (run_time*Fs);
		printf("\npress 'r' to run\npress 'p' to change the value of Kp\npress 'f' to change the sampling frequency\npress 't' to change the run time\npress 'u' to change the input (step or square)\npress 'g' to plot the results on the screen\npress 'h' to save the plot results in Postscript\npress 'q' to quit\n");
		
		scanf("%c[^\n]", &selection);
		printf("\n");
		switch (selection) {
			case 'r':
				//Run the control algorithm			
				sem_init(&data_avail, 0, 0);
				Initialize(DLAB_SIMULATE, Fs, motor_number);
				if (pthread_create(&Control, NULL, &Control1, NULL) != 0) {
					    printf("Error in creating thread 1\n");
					    exit(1);
				    }
	
				pthread_join(Control, NULL);
				Terminate();
				sem_destroy(&data_avail);
				
				// verbose output of parameters
				printf("\nthe new Kp is: %lf \n",Kp);
				printf("\nthe new Fs is: %lf \n",Fs);
				printf("\nthe new runtime is: %lf \n",run_time);
				printf("\nthe new mag is: %lf \n",mag);
				printf("\nthe new freq is: %lf \n",freq);
				printf("\nthe new dcyc is: %lf \n",dcyc);
				printf("\nthe number of samples is: %lf \n",no_of_samples);
				
				break;

			case 'p':
				//change the value of Kp
				printf("\nSelect a new value for Kp\n");
				scanf("%lf", &Kp);
				
				// verbose output of parameters
				printf("\nthe new Kp is: %lf \n",Kp);
				printf("\nthe new Fs is: %lf \n",Fs);
				printf("\nthe new runtime is: %lf \n",run_time);
				printf("\nthe new mag is: %lf \n",mag);
				printf("\nthe new freq is: %lf \n",freq);
				printf("\nthe new dcyc is: %lf \n",dcyc);
				printf("\nthe number of samples is: %lf \n",no_of_samples);
				break;

			case 'f':
				//change the value of Fs
				printf("\nSelect a new sampling frequency\n");
				scanf("%lf", &Fs);
				
				// verbose output of parameters
				printf("\nthe new Kp is: %lf \n",Kp);
				printf("\nthe new Fs is: %lf \n",Fs);
				printf("\nthe new runtime is: %lf \n",run_time);
				printf("\nthe new mag is: %lf \n",mag);
				printf("\nthe new freq is: %lf \n",freq);
				printf("\nthe new dcyc is: %lf \n",dcyc);
				printf("\nthe number of samples is: %lf \n",no_of_samples);
				break;

			case 't':
				//Change the value of run_time
				printf("\nSelect a new duration for the run time\n");
				scanf("%lf", &run_time);
				
				// verbose output of parameters
				printf("\nthe new Kp is: %lf \n",Kp);
				printf("\nthe new Fs is: %lf \n",Fs);
				printf("\nthe new runtime is: %lf \n",run_time);
				printf("\nthe new mag is: %lf \n",mag);
				printf("\nthe new freq is: %lf \n",freq);
				printf("\nthe new dcyc is: %lf \n",dcyc);
				printf("\nthe number of samples is: %lf \n",no_of_samples);
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
					
					// verbose output of parameters
					printf("\nthe new Kp is: %lf \n",Kp);
					printf("\nthe new Fs is: %lf \n",Fs);
					printf("\nthe new runtime is: %lf \n",run_time);
					printf("\nthe new mag is: %lf \n",mag);
					printf("\nthe new freq is: %lf \n",freq);
					printf("\nthe new dcyc is: %lf \n",dcyc);
					printf("\nthe number of samples is: %lf \n",no_of_samples);
					Square(ref, no_of_samples, Fs, mag*PI/180, freq, dcyc);
				}
				break;

			case 'g':
				//plot results
				plot(ref, theta, Fs, no_of_samples, SCREEN, "Graph Title", "x-axis", "y-axis");
				
				// verbose output of parameters
				printf("\nthe new Kp is: %lf \n",Kp);
				printf("\nthe new Fs is: %lf \n",Fs);
				printf("\nthe new runtime is: %lf \n",run_time);
				printf("\nthe new mag is: %lf \n",mag);
				printf("\nthe new freq is: %lf \n",freq);
				printf("\nthe new dcyc is: %lf \n",dcyc);
				printf("\nthe number of samples is: %d \n",no_of_samples);
				break;

			case 'h':
				//Save the plot results in Postscript
				plot(ref, theta, Fs, no_of_samples, PS, "Graph Title", "x-axis", "y-axis");
				break;

			case 'q':
				//quit
				exit(0);
		}
	}while (selection != 'q');
}
