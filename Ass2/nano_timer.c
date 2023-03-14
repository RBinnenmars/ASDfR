#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <stdlib.h>
#include <signal.h>

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
  struct timespec ts, remaining;
  struct timespec ts_array[1000]; 
  // Set the sleep duration to 1000000 nanoseconds (1 milliseconds)
  ts.tv_sec = 0;
  ts.tv_nsec = 1000000;
  for (int j = 0; j<1000;j++)
  {
    if (clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &remaining) != 0) {
        perror("clock_nanosleep");
        return NULL;
    }
    for (int i = 0;i<1000;i++){}
    clock_gettime(CLOCK_REALTIME, &ts_array[j]);

  }
  FILE *csv_file = fopen("time_values.csv", "w");
  if (csv_file == NULL) {
      perror("Error opening file");
      return -1;
  }

  fprintf(csv_file, "Time Values\n");
  for (int i = 0; i < 1000; i++) {
      fprintf(csv_file, "%ld.%09ld\n", ts_array[i].tv_sec, ts_array[i].tv_nsec);
  }

  fclose(csv_file);
  return NULL;
}

