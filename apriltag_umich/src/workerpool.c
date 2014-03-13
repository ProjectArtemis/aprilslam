/*
 * This file is part of the AprilTag library.
 *
 * AprilTag is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * AprilTag is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with Libav; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#define __USE_GNU
#include <pthread.h>
#include <sched.h>
#include <assert.h>
#include <stdio.h>
#include "workerpool.h"
#include "timeprofile.h"
#include <inttypes.h>

struct workerpool
{
  int nthreads;
  zarray_t *tasks;
  int taskspos;

  pthread_t *threads;
  int *status;

  pthread_mutex_t mutex;
  pthread_cond_t startcond;   // used to signal the availability of work
  pthread_cond_t endcond;     // used to signal completion of all work

  int end_count; // how many threads are done?
};

struct task
{
  void (*f)(void *p);
  void *p;
};

void *worker_thread(void *p)
{
  workerpool_t *wp = (workerpool_t*) p;

  int cnt = 0;

  while (1)
  {
    struct task *task;

    pthread_mutex_lock(&wp->mutex);
    while (wp->taskspos == zarray_size(wp->tasks))
    {
      wp->end_count++;
      //          printf("%"PRId64" thread %d did %d\n", utime_now(), pthread_self(), cnt);
      pthread_cond_broadcast(&wp->endcond);
      pthread_cond_wait(&wp->startcond, &wp->mutex);
      cnt = 0;
      //            printf("%"PRId64" thread %d awake\n", utime_now(), pthread_self());
    }

    zarray_get_volatile(wp->tasks, wp->taskspos, &task);
    wp->taskspos++;
    cnt++;
    pthread_mutex_unlock(&wp->mutex);
    //        pthread_yield();
    sched_yield();

    // we've been asked to exit.
    if (task->f == NULL)
      return NULL;

    task->f(task->p);
  }

  return NULL;
}

workerpool_t *workerpool_create(int nthreads)
{
  assert(nthreads > 0);

  workerpool_t *wp = calloc(1, sizeof(workerpool_t));
  wp->nthreads = nthreads;
  wp->tasks = zarray_create(sizeof(struct task));

  if (nthreads > 1)
  {
    wp->threads = calloc(wp->nthreads, sizeof(pthread_t));

    pthread_mutex_init(&wp->mutex, NULL);
    pthread_cond_init(&wp->startcond, NULL);
    pthread_cond_init(&wp->endcond, NULL);

    for (int i = 0; i < nthreads; i++)
    {
      int res = pthread_create(&wp->threads[i], NULL, worker_thread, wp);
      if (res != 0)
      {
        perror("pthread_create");
        exit(-1);
      }
    }
  }

  return wp;
}

void workerpool_destroy(workerpool_t *wp)
{
  if (wp == NULL)
    return;

  // force all worker threads to exit.
  if (wp->nthreads > 1)
  {
    for (int i = 0; i < wp->nthreads; i++)
      workerpool_add_task(wp, NULL, NULL);

    pthread_mutex_lock(&wp->mutex);
    pthread_cond_broadcast(&wp->startcond);
    pthread_mutex_unlock(&wp->mutex);

    for (int i = 0; i < wp->nthreads; i++)
      pthread_join(wp->threads[i], NULL);

    pthread_mutex_destroy(&wp->mutex);
    pthread_cond_destroy(&wp->startcond);
    pthread_cond_destroy(&wp->endcond);
    free(wp->threads);
  }

  zarray_destroy(wp->tasks);
  free(wp);
}

int workerpool_get_nthreads(workerpool_t *wp)
{
  return wp->nthreads;
}

void workerpool_add_task(workerpool_t *wp, void (*f)(void *p), void *p)
{
  struct task t;
  t.f = f;
  t.p = p;

  zarray_add(wp->tasks, &t);
}

void workerpool_run_single(workerpool_t *wp)
{
  for (int i = 0; i < zarray_size(wp->tasks); i++)
  {
    struct task *task;
    zarray_get_volatile(wp->tasks, i, &task);
    task->f(task->p);
  }

  zarray_clear(wp->tasks);
}

// runs all added tasks, waits for them to complete.
void workerpool_run(workerpool_t *wp)
{
  if (wp->nthreads > 1)
  {
    wp->end_count = 0;

    pthread_mutex_lock(&wp->mutex);
    pthread_cond_broadcast(&wp->startcond);

    while (wp->end_count < wp->nthreads)
    {
      //            printf("caught %d\n", wp->end_count);
      pthread_cond_wait(&wp->endcond, &wp->mutex);
    }

    pthread_mutex_unlock(&wp->mutex);

    wp->taskspos = 0;

    zarray_clear(wp->tasks);

  }
  else
  {
    workerpool_run_single(wp);
  }
}
