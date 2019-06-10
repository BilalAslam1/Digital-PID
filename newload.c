// Oct 15, 2013: Modified NTHREAD to 20 for new systems in ENG413
//
#include <stdio.h>
#include <math.h>
#include <sys/mman.h>
#include <pthread.h>
#include <malloc.h>
#include <stdlib.h>

#define NTHREADS 20 

#define SIZE 1000000

int find_prime(int n)
{
  int candidate = 2;

  while (1) {
    int factor;
    int is_prime = 1;

    /* Test primality by successive division.  */
    for (factor = 2; factor < candidate; ++factor)
      if (candidate % factor == 0) {
        is_prime = 0;
        break;
      }
    /* Is this the prime number we're looking for?  */
    if (is_prime) {
      if (--n == 0)
        /* Return the desired prime number as the thread return value.  */
        return candidate;
    }
    ++candidate;
  }
  return 0;
}

void *func(void *arg)
{
   int i, j, rcode, *id;
   double *a;
   char msg[80];

   id = (int *)arg;

   rcode = mlockall(MCL_CURRENT | MCL_FUTURE);
   if (rcode !=0 ) {
      sprintf(msg, "Thread %d: Problem with mlockall ", *id);
	  perror(msg);
   }

   for (j = 0; ; j++) {
    a = (double *)calloc(SIZE, sizeof(double));
   	for (i = 1200; i < SIZE; i++) {
      	printf("Thread %d is simulating a load ... %d prime is %d\n", *id, i,
			find_prime((int)(i)));
      	a[i] = sqrt((double) i)*sqrt((double) i);
   	}
   	free(a);
   }

   munlockall();
}

int main(void)
{
	pthread_t loads[NTHREADS];
	int id[NTHREADS];
	int i;

	for (i = 0; i < NTHREADS; i++) {
		id[i] = i;
		if (pthread_create(&loads[i], NULL, &func, &id[i]) != 0) {
			printf("Error creating thread %d\n", i);
			exit(-1);
		}
		else {
			printf("Created Thread %d\n", i);
		}
	}

	pthread_exit(NULL);
}
