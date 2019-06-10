#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <sys/mman.h>
#include <signal.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>

#ifndef SIMULATE
#include <rtai_lxrt.h>
#include <rtai_fifos.h>
#include <rtai_sem.h>
#endif

#ifndef PI
#define PI M_PI
#endif

#define SCREEN 0
#define PS     1

#define CALLOC(n,x) ((x *)calloc(n, sizeof(x)))

#define ONE_TENTH_SEC nano2count(100000000LL)

#define DLAB_SIMULATE 1
#define DLAB_HARDWARE 0

extern int Initialize(int, float, int );
extern short int ReadEncoder(void);
extern int DtoA(short int);
extern void Terminate(void);
extern float EtoR(short int);
extern short int VtoD(float);
extern float DLaBsat(float);
extern int gch(void);
extern int getch(void);

extern int mInitialize(float, int );
extern short int mReadEncoder(void);
extern int mDtoA(short int);
extern void mTerminate(void);

extern void read_matrix(FILE *, float **, char *, int *, int *);
extern void read_vector(FILE *, float  *, char *, int *);
extern void print_matrix(FILE *, float **, char *, int, int);
extern void print_vector(FILE *, float  *, char *, int);
extern void multiply_matrix(float **, int, int, float **, int, int, float **c);
extern void add_matrix(float **, float **, int, int, float **);
extern void subtract_matrix(float **, float **, int, int, float **);
extern void scalar_multiply(float, float **, int, int, float **);
extern void copy_matrix(float **, float **, int, int);

extern double model_tk, model_dt, model_y;
extern double model_x[3];

extern void plot(float *, float *, float, int, int, char *, char *, char *);
extern void sq_wave(float *, int, float, float);
extern void Square(float *, int, float, float, float, float);
