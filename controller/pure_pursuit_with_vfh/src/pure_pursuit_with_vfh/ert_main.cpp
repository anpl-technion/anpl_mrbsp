#include <stdio.h>
#include <stdlib.h>
#include "Pure_Pursuit_With_VFH.h"
#include "Pure_Pursuit_With_VFH_private.h"
#include "rtwtypes.h"
#include "limits.h"
#include "rt_nonfinite.h"
#include "linuxinitialize.h"
#define UNUSED(x)                      x = x

static Pure_Pursuit_With_VFHModelClass Pure_Pursuit_With_VFH_Obj;/* Instance of model class */

/* Function prototype declaration*/
void exitFcn(int sig);
void *terminateTask(void *arg);
void *baseRateTask(void *arg);
void *subrateTask(void *arg);
volatile boolean_T stopRequested = false;
volatile boolean_T runModel = true;
sem_t stopSem;
sem_t baserateTaskSem;
pthread_t schedulerThread;
pthread_t baseRateThread;
unsigned long threadJoinStatus[8];
int terminatingmodel = 0;
void *baseRateTask(void *arg)
{
  runModel = (rtmGetErrorStatus(Pure_Pursuit_With_VFH_Obj.getRTM()) == (NULL)) &&
    !rtmGetStopRequested(Pure_Pursuit_With_VFH_Obj.getRTM());
  while (runModel) {
    sem_wait(&baserateTaskSem);
    Pure_Pursuit_With_VFH_Obj.step();

    /* Get model outputs here */
    stopRequested = !((rtmGetErrorStatus(Pure_Pursuit_With_VFH_Obj.getRTM()) ==
                       (NULL)) && !rtmGetStopRequested
                      (Pure_Pursuit_With_VFH_Obj.getRTM()));
    runModel = !stopRequested;
  }

  runModel = 0;
  terminateTask(arg);
  pthread_exit((void *)0);
  return NULL;
}

void exitFcn(int sig)
{
  UNUSED(sig);
  rtmSetErrorStatus(Pure_Pursuit_With_VFH_Obj.getRTM(), "stopping the model");
}

void *terminateTask(void *arg)
{
  UNUSED(arg);
  terminatingmodel = 1;

  {
    runModel = 0;
  }

  /* Disable rt_OneStep() here */

  /* Terminate model */
  Pure_Pursuit_With_VFH_Obj.terminate();
  sem_post(&stopSem);
  return NULL;
}

int main(int argc, char **argv)
{
  UNUSED(argc);
  UNUSED(argv);
  void slros_node_init(int argc, char** argv);
  slros_node_init(argc, argv);
  rtmSetErrorStatus(Pure_Pursuit_With_VFH_Obj.getRTM(), 0);

  /* Initialize model */
  Pure_Pursuit_With_VFH_Obj.initialize();

  /* Call RTOS Initialization function */
  myRTOSInit(0.05, 0);

  /* Wait for stop semaphore */
  sem_wait(&stopSem);
  return 0;
}
