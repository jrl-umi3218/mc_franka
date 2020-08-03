/*
 * Copyright 2019-2020 mc_rtc development team
 */

#include <linux/sched.h>
#include <linux/sched/types.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syscall.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <unistd.h>

#include "thread.h"

int sched_setattr(pid_t pid, const struct sched_attr *attr, unsigned int flags)
{
  return syscall(__NR_sched_setattr, pid, attr, flags);
}

int main(int argc, char * argv[])
{
  struct sched_attr attr;

  /* Lock memory */
  if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
  {
    printf("mlockall failed: %m\n");
    return -2;
  }

  /* Configure deadline policy */
  memset(&attr, 0, sizeof(attr));
  attr.size = sizeof(attr);

  uint64_t cycle_ns = 1 * 1000 * 1000; // 1 ms default cycle
  char * MC_RT_FREQ = nullptr;
  if((MC_RT_FREQ = getenv("MC_RT_FREQ")) != nullptr)
  {
    cycle_ns = atoi(MC_RT_FREQ) * 1000 * 1000;
  }

  /* Initialize callback (non real-time yet) */
  void * data = init(argc, argv, cycle_ns);
  if(!data)
  {
  }

  /* Time reservation */
  attr.sched_policy = SCHED_DEADLINE;
  attr.sched_runtime = attr.sched_deadline = attr.sched_period = cycle_ns; // nanoseconds

  printf("Running real-time thread at %fms per cycle\n", cycle_ns / 1e6);

  /* Set scheduler policy */
  if(sched_setattr(0, &attr, 0) < 0)
  {
    printf("sched_setattr failed: %m\n");
    return -2;
  }

  /* Run */
  run(data);

  return 0;
}
