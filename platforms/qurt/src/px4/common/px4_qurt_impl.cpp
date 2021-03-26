/****************************************************************************
 *
 *   Copyright (c) 2015 Mark Charlebois. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file px4_linux_impl.cpp
 *
 * PX4 Middleware Wrapper Linux Implementation
 */

#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/init.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/workqueue.h>
#include <dataman/dataman.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <semaphore.h>
#include <parameters/param.h>
#include "hrt_work.h"

//extern pthread_t _shell_task_id;


__BEGIN_DECLS
extern uint64_t get_ticks_per_us();

//long PX4_TICKS_PER_SEC = 1000L;

unsigned int QURT_MAX_HTHREADS = 4;

//-----------------------------------------------
// Unresolved in libpx4.so
//-----------------------------------------------

int HAP_power_request(int clock, int bus, int latency) {
    PX4_INFO("*** Fake HAP_power_request called!!! ***");
    return 0;
}

// int pthread_attr_init(pthread_attr_t *attr) {
//     PX4_INFO("*** Fake pthread_attr_init called!!! ***");
//     return -1;
// }
//
// int pthread_attr_setstacksize(pthread_attr_t *attr, size_t stacksize)  {
//     PX4_INFO("*** Fake pthread_attr_setstacksize called!!! ***");
//     return -1;
// }
//
// int pthread_attr_destroy(pthread_attr_t *attr)  {
//     PX4_INFO("*** Fake pthread_attr_destroy called!!! ***");
//     return -1;
// }
//
// int pthread_attr_getschedparam(const pthread_attr_t *restrict attr, sched_param *restrict param)  {
//     PX4_INFO("*** Fake pthread_attr_getschedparam called!!! ***");
//     return -1;
// }
//
// int pthread_attr_setschedparam(pthread_attr_t *restrict attr, const sched_param *restrict param)  {
//     PX4_INFO("*** Fake pthread_attr_setschedparam called!!! ***");
//     return -1;
// }
//
// int pthread_kill(pthread_t thread, int sig)  {
//     PX4_INFO("*** Fake pthread_kill called!!! ***");
//     return -1;
// }
//
// int pthread_cancel(pthread_t thread)  {
//     PX4_INFO("*** Fake pthread_cancel called!!! ***");
//     return -1;
// }
//
// void pthread_exit(void *value_ptr)  {
//     PX4_INFO("*** Fake pthread_exit called!!! ***");
// }
//
// int sem_init(sem_t *sem, int pshared, unsigned int value) {
//     PX4_INFO("*** Fake sem_init called!!! ***");
//     return -1;
// }
// int sem_wait(sem_t *sem) {
//     PX4_INFO("*** Fake sem_wait called!!! ***");
//     return -1;
// }
// int sem_post(sem_t *sem) {
//     PX4_INFO("*** Fake sem_post called!!! ***");
//     return -1;
// }
// int sem_destroy(sem_t *sem) {
//     PX4_INFO("*** Fake sem_destroy called!!! ***");
//     return -1;
// }
// int sem_getvalue(sem_t *sem, int *value) {
//     PX4_INFO("*** Fake sem_getvalue called!!! ***");
//     return -1;
// }
//
// int sched_get_priority_max(int policy) {
//     PX4_INFO("*** Fake sched_get_priority_max called!!! ***");
//     return -1;
// }
//
//
// int usleep(useconds_t usec) {
//     PX4_INFO("*** Fake usleep called!!! ***");
//     return -1;
// }

//-----------------------------------------------
// Unresolved in both libpx4.so and libc++.so.1
//-----------------------------------------------

static uint32_t gettime_counter = 0;

__attribute__((visibility("default"))) int clock_gettime(clockid_t clk_id, struct timespec *tp) {
    if ((gettime_counter++ % 10000) == 0) PX4_INFO("*** Fake clock_gettime called!!! ***");
    return -1;
}

//-----------------------------------------------
// Unresolved in libc++.so.1
//-----------------------------------------------

__attribute__((visibility("default"))) int nanosleep(const struct timespec *req, struct timespec *rem) {
    PX4_INFO("*** Fake nanosleep called!!! ***");
    return -1;
}

__attribute__((visibility("default"))) void free(void *ptr) {
    PX4_INFO("*** Fake free called!!! ***");
    ptr = 0;
}

__attribute__((visibility("default"))) void *malloc(size_t size) {
    PX4_INFO("*** Fake malloc called!!! ***");
    return (void*) 0;
}

__attribute__((visibility("default"))) void *calloc(size_t nmemb, size_t size) {
    PX4_INFO("*** Fake calloc called!!! ***");
    return (void*) 0;
}

__attribute__((visibility("default"))) void *realloc(void *ptr, size_t size) {
    PX4_INFO("*** Fake realloc called!!! ***");
    return (void*) 0;
}

#if 0
unsigned int sleep(unsigned int sec)
{
	for (unsigned int i = 0; i < sec; i++) {
		usleep(1000000);
	}

	return 0;
}
#endif

extern void hrt_init(void);

#if 0
void qurt_log(const char *fmt, ...)
{
	va_list	args;
	va_start(args, fmt);
	printf(fmt, args);
	printf("n");
	va_end(args);
}
#endif

//extern int _posix_init(void);

__END_DECLS

extern struct wqueue_s gwork[NWORKERS];


namespace px4
{

void init_once(void);

void init_once(void)
{
	// Required for QuRT
	//_posix_init();

//	_shell_task_id = pthread_self();
//	PX4_INFO("Shell id is %lu", _shell_task_id);

	work_queues_init();
	hrt_work_queue_init();

	px4_platform_init();
}

void init(int argc, char *argv[], const char *app_name)
{
	PX4_DEBUG("App name: %s\n", app_name);
}

}

/** Retrieve from the data manager store */
ssize_t
dm_read(
	dm_item_t item,                 /* The item type to retrieve */
	unsigned index,                 /* The index of the item */
	void *buffer,                   /* Pointer to caller data buffer */
	size_t buflen                   /* Length in bytes of data to retrieve */
)
{
	return 0;
}

/** write to the data manager store */
ssize_t
dm_write(
	dm_item_t  item,                /* The item type to store */
	unsigned index,                 /* The index of the item */
	dm_persitence_t persistence,    /* The persistence level of this item */
	const void *buffer,             /* Pointer to caller data buffer */
	size_t buflen                   /* Length in bytes of data to retrieve */
)
{
	return 0;
}

size_t strnlen(const char *s, size_t maxlen)
{
	size_t i = 0;

	while (s[i] != '\0' && i < maxlen) {
		++i;
	}

	return i;
}

int fprintf(FILE *stream, const char *format, ...)
{
	PX4_ERR("Error: Calling unresolved symbol stub:[%s(%s,...)]", __FUNCTION__, format);
	return 0;
}

int fputc(int c, FILE *stream)
{
	return c;
}

int putchar(int character)
{
	return character;
}
