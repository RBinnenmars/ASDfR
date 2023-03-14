#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>

int timer_handler(int);
void *loop();

int main()
{
  pthread_t loopthread;

  pthread_create(&loopthread,NULL,loop,NULL);

  pthread_join(loopthread,NULL);

  return 0;
}

void *loop()
{
  struct sigevent sev;
  struct itimerspec its;
  timer_t timerid;
  int sig;
  sigset_t mask;

  /* Set up a signal handler for the timer */
  struct sigaction sa;
  sa.sa_flags = SA_SIGINFO;
  sa.sa_sigaction = timer_handler;
  sigemptyset(&sa.sa_mask);
  sigaction(SIGRTMIN, &sa, NULL);

  /* Create a timer */
  sev.sigev_notify = SIGEV_SIGNAL;
  sev.sigev_signo = SIGRTMIN;
  sev.sigev_value.sival_ptr = &timerid;
  timer_create(CLOCK_REALTIME, &sev, &timerid);

  /* Start the timer */
  its.it_value.tv_sec = 5; // initial delay
  its.it_value.tv_nsec = 0;
  its.it_interval.tv_sec = 2; // interval between expirations
  its.it_interval.tv_nsec = 0;
  timer_settime(timerid, 0, &its, NULL);

  /* Wait for the signal */
  sigemptyset(&mask);
  sigaddset(&mask, SIGRTMIN);
  sigwait(&mask, &sig);

  /* Clean up the timer */
  timer_delete(timerid);

  return 0;
}

int timer_handler(int signum)
{
  printf("Timer expired.\n");
}