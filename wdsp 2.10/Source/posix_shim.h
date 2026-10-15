/*  posix_shim.h

This file is part of a program that implements a Software-Defined Radio.

Copyright (C) 2026 Jeremy McInerney (Trident)

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

------------------------------------------------------------------------------

The Win32 primitives WDSP 2.10 uses, on POSIX. comm.h includes this instead of
<Windows.h>, <process.h>, <intrin.h> and <avrt.h> when _WIN32 is not defined.
Nothing else in the library changes.

Written from the documented semantics of each Win32 call, not from any other
port of WDSP. Every primitive below is listed with the behaviour the WDSP
sources rely on, read from the call sites:

  CRITICAL_SECTION       recursive: setters take csDSP while the DSP thread
                         holds it for a block, and a setter can call another
                         setter (channel.c, main.c). So a RECURSIVE pthread
                         mutex, never a plain one; with PRIO_INHERIT, because
                         the DSP thread runs at SCHED_FIFO (below) while the
                         setters that take csDSP from the HTTP side do not,
                         and without inheritance a preempted setter holds the
                         FIFO thread.
  Interlocked*           InterlockedBitTestAndSet/Reset return the PREVIOUS
                         value of the bit (analyzer.c:1068, calcc.c:1836 test
                         it); InterlockedAnd/_InterlockedAnd return the
                         PREVIOUS value and are used as atomic reads
                         (`_InterlockedAnd(&ch[channel].run, 1)`, main.c:37);
                         InterlockedIncrement/Decrement return the NEW value;
                         InterlockedExchange returns the previous value.
                         The operands are `volatile long` in most structs and
                         `volatile LONG` in analyzer.h; LONG is `long` here so
                         both are the same width and each macro operates on
                         the width of the object it is given.
  _beginthread           a detached thread running `void fn(void*)`; the
                         return value is only ever cast to HANDLE and handed
                         to SetThreadPriority (syncbuffs.c:31-32) or dropped
                         (channel.c:33).
  _endthread             ends the calling thread (analyzer.c:1090,
                         iobuffs.c:587, syncbuffs.c:140, utilities.c:310).
  QueueUserWorkItem      analyzer.c:1074 queues one FFT (`DWORD WINAPI
                         spectra(void*)`) per display frame; a detached thread
                         per call.
  Semaphores, events     CreateSemaphore(initial, max), ReleaseSemaphore(n) --
                         which FAILS and leaves the count alone when count + n
                         would pass max: calcc's five command semaphores and
                         Sem_Flush are created with max 1 (calcc.c:1111,
                         iobuffs.c:421), so a command released twice before the
                         worker takes it runs once --
                         CreateEvent(manualReset, initial), SetEvent,
                         ResetEvent, WaitForSingleObject(ms) with 0, 1, a
                         timeout and INFINITE (iobuffs.c:455 drains with
                         `while (!WaitForSingleObject(h, 1))`), CloseHandle,
                         and WaitForMultipleObjects(n, handles, FALSE,
                         INFINITE) returning WAIT_OBJECT_0 + the index
                         (calcc.c:1790). Objects share one lock so a
                         multi-object wait is atomic against the signals.
                         Every timed wait is on CLOCK_MONOTONIC: a Pi has no
                         RTC and steps its clock at the first NTP sync, and the
                         1 ms drain polls (iobuffs.c:455, syncbuffs.c:94) exit
                         only on timeout, so a wall-clock deadline would stall
                         a channel close by the size of the step.
  Sleep(ms)              nanosleep.
  QueryPerformanceCounter/Frequency  CLOCK_MONOTONIC (nnet.c:1289, profiling).
  AvSetMmThreadCharacteristics("Pro Audio")  main.c:32 asks for the realtime
                         audio class for a channel's DSP thread. Here it asks
                         for SCHED_FIFO at a modest priority and returns NULL
                         if the process may not (no CAP_SYS_NICE, RLIMIT_RTPRIO
                         0), in which case main.c falls back to
                         SetThreadPriority(THREAD_PRIORITY_HIGHEST), a nice
                         value for the calling thread, also best effort.
                         Neither failure is an error: the threads run at the
                         default policy, as they do on a Windows machine
                         without MMCSS.
  AllocConsole/FreeConsole/freopen_s  wisdom.c:55-56,106 redirect stdout to a
                         console it creates for the bake. Here stdout stays
                         where it is, so the progress lines reach whoever
                         started the process.
  _aligned_malloc/_aligned_free  posix_memalign and free. malloc0
                         (utilities.c:37) asks for 16-byte alignment.
  min/max                <Windows.h> defines them; RXA.c:91 and others use them.
  __declspec, __stdcall, __cdecl, WINAPI, TEXT  decorations; nothing here.
*/

#ifndef POSIX_SHIM_H
#define POSIX_SHIM_H

#if defined(_WIN32) || defined(_WIN64)
#error "posix_shim.h is for non-Windows builds; Windows uses the real headers"
#endif

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

/* WDSP's debug printer (utilities.h:84, utilities.c:587) has the name of the
   POSIX dprintf(int fd, ...) that <stdio.h> declares. The library's is renamed
   at every declaration, definition and call; the stdio one is untouched. */
#define dprintf wdsp_dprintf

/* channel.c:101 sets the SSE flush-to-zero mode for the DSP thread so denormals
   cost nothing. On x86 that is the real intrinsic; on AArch64 it is the FZ bit
   of FPCR, which is also per thread and also covers both float widths. */
#if defined(__x86_64__) || defined(__i386__)
#include <xmmintrin.h>
#elif defined(__aarch64__)
#define _MM_FLUSH_ZERO_ON 0x8000
#define _MM_FLUSH_ZERO_OFF 0x0000
static inline void _MM_SET_FLUSH_ZERO_MODE(unsigned mode)
{
	uint64_t fpcr;
	__asm__ volatile("mrs %0, fpcr" : "=r"(fpcr));
	if (mode) fpcr |= (1ull << 24); else fpcr &= ~(1ull << 24);
	__asm__ volatile("msr fpcr, %0" : : "r"(fpcr));
}
#else
#define _MM_FLUSH_ZERO_ON 0x8000
#define _MM_SET_FLUSH_ZERO_MODE(mode) ((void)(mode))
#endif

/* ---- decorations ---------------------------------------------------------- */

#define __declspec(x)
#ifndef __stdcall
#define __stdcall
#endif
#ifndef __cdecl
#define __cdecl
#endif
#define WINAPI
#define TEXT(s) s

/* ---- types ---------------------------------------------------------------- */

typedef long LONG;               /* same width as the `volatile long` flags */
typedef unsigned char byte, BYTE; /* utilities.c:289 */
typedef uint32_t DWORD;          /* wdsp.h:106: a Windows DWORD is 32-bit */
typedef int BOOL;
typedef unsigned char BOOLEAN;
typedef void *HANDLE;
typedef void *LPVOID;
typedef union { struct { uint32_t LowPart; int32_t HighPart; }; int64_t QuadPart; } LARGE_INTEGER;

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif
#define INFINITE 0xFFFFFFFFu
#define WAIT_OBJECT_0 0u
#define WAIT_TIMEOUT 258u
#define WAIT_FAILED 0xFFFFFFFFu
#define THREAD_PRIORITY_LOWEST (-2)
#define THREAD_PRIORITY_BELOW_NORMAL (-1)
#define THREAD_PRIORITY_NORMAL 0
#define THREAD_PRIORITY_ABOVE_NORMAL 1
#define THREAD_PRIORITY_HIGHEST 2
#define THREAD_PRIORITY_TIME_CRITICAL 15

#ifndef max
#define max(a, b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a, b) (((a) < (b)) ? (a) : (b))
#endif

/* ---- atomics -------------------------------------------------------------- */

/* Type-generic so each operates on the width of the object it is given. */
#define InterlockedBitTestAndSet(p, bit) \
	((__atomic_fetch_or((p), ((__typeof__(*(p)))1 << (bit)), __ATOMIC_SEQ_CST) >> (bit)) & 1)
#define InterlockedBitTestAndReset(p, bit) \
	((__atomic_fetch_and((p), ~((__typeof__(*(p)))1 << (bit)), __ATOMIC_SEQ_CST) >> (bit)) & 1)
#define InterlockedAnd(p, v) __atomic_fetch_and((p), (v), __ATOMIC_SEQ_CST)
#define _InterlockedAnd(p, v) __atomic_fetch_and((p), (v), __ATOMIC_SEQ_CST)
#define InterlockedOr(p, v) __atomic_fetch_or((p), (v), __ATOMIC_SEQ_CST)
#define InterlockedExchange(p, v) __atomic_exchange_n((p), (v), __ATOMIC_SEQ_CST)
#define InterlockedIncrement(p) __atomic_add_fetch((p), 1, __ATOMIC_SEQ_CST)
#define InterlockedDecrement(p) __atomic_sub_fetch((p), 1, __ATOMIC_SEQ_CST)

/* ---- critical sections ---------------------------------------------------- */

typedef struct posix_shim_critical_section {
	pthread_mutex_t m;
} CRITICAL_SECTION, *LPCRITICAL_SECTION;

static inline void InitializeCriticalSection(LPCRITICAL_SECTION cs)
{
	pthread_mutexattr_t a;
	pthread_mutexattr_init(&a);
	pthread_mutexattr_settype(&a, PTHREAD_MUTEX_RECURSIVE);
	pthread_mutexattr_setprotocol(&a, PTHREAD_PRIO_INHERIT);
	pthread_mutex_init(&cs->m, &a);
	pthread_mutexattr_destroy(&a);
}

static inline BOOL InitializeCriticalSectionAndSpinCount(LPCRITICAL_SECTION cs, DWORD spin)
{
	(void)spin;
	InitializeCriticalSection(cs);
	return TRUE;
}

static inline void EnterCriticalSection(LPCRITICAL_SECTION cs) { pthread_mutex_lock(&cs->m); }
static inline void LeaveCriticalSection(LPCRITICAL_SECTION cs) { pthread_mutex_unlock(&cs->m); }
static inline BOOL TryEnterCriticalSection(LPCRITICAL_SECTION cs) { return pthread_mutex_trylock(&cs->m) == 0; }
static inline void DeleteCriticalSection(LPCRITICAL_SECTION cs) { pthread_mutex_destroy(&cs->m); }

/* ---- time ----------------------------------------------------------------- */

static inline void Sleep(DWORD ms)
{
	struct timespec t;
	t.tv_sec = ms / 1000;
	t.tv_nsec = (long)(ms % 1000) * 1000000L;
	while (nanosleep(&t, &t) == -1 && errno == EINTR) {}
}

static inline BOOL QueryPerformanceFrequency(LARGE_INTEGER *f)
{
	f->QuadPart = 1000000000LL;
	return TRUE;
}

static inline BOOL QueryPerformanceCounter(LARGE_INTEGER *t)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	t->QuadPart = (int64_t)ts.tv_sec * 1000000000LL + ts.tv_nsec;
	return TRUE;
}

/* ---- semaphores and events ------------------------------------------------ */

/* One lock for every object, so WaitForMultipleObjects can examine several
   atomically against their signals. Each object has its own condition; the
   shared one is broadcast only while a multi-object wait is pending. */
enum { POSIX_SHIM_SEMAPHORE = 1, POSIX_SHIM_EVENT = 2 };

struct posix_shim_object {
	int kind;
	int manual;      /* events: manual reset */
	long count;      /* semaphores: count; events: signaled */
	long max;        /* semaphores: the ceiling ReleaseSemaphore refuses to pass */
	pthread_cond_t c;
};

struct posix_shim_registry {
	pthread_mutex_t m;
	pthread_cond_t any;
	int multi_waiters;
};

/* Every condition waits on CLOCK_MONOTONIC (see the note at the top). */
static inline void posix_shim_cond_init(pthread_cond_t *c)
{
	pthread_condattr_t a;
	pthread_condattr_init(&a);
	pthread_condattr_setclock(&a, CLOCK_MONOTONIC);
	pthread_cond_init(c, &a);
	pthread_condattr_destroy(&a);
}

/* ONE registry for the process. This header is included by 80 translation units, and a
   `static` object in a header -- or a static local inside a `static inline` function --
   is one object PER UNIT: the first version of this shim had the registry lock as a
   static local, so iobuffs.c's ReleaseSemaphore and main.c's WaitForSingleObject were
   serialising on different mutexes, and the Linux tests passed anyway. Weak definitions
   merge into one at link time (ELF, GCC and clang), which is why these are not static. */
struct posix_shim_registry posix_shim_registry_instance __attribute__((weak));
pthread_once_t posix_shim_registry_once __attribute__((weak)) = PTHREAD_ONCE_INIT;

__attribute__((weak)) void posix_shim_registry_init(void)
{
	pthread_mutex_init(&posix_shim_registry_instance.m, NULL);
	posix_shim_cond_init(&posix_shim_registry_instance.any);
	posix_shim_registry_instance.multi_waiters = 0;
}

static inline struct posix_shim_registry *posix_shim_reg(void)
{
	pthread_once(&posix_shim_registry_once, posix_shim_registry_init);
	return &posix_shim_registry_instance;
}

static inline HANDLE posix_shim_new(int kind, int manual, long count, long max)
{
	struct posix_shim_object *o = (struct posix_shim_object *)calloc(1, sizeof *o);
	if (o == NULL) return NULL;
	o->kind = kind;
	o->manual = manual;
	o->count = count;
	o->max = max;
	posix_shim_cond_init(&o->c);
	return (HANDLE)o;
}

static inline HANDLE CreateSemaphore(void *attr, long initial, long maximum, const void *name)
{
	(void)attr; (void)name;
	if (maximum < 1 || initial < 0 || initial > maximum) return NULL; /* as Win32 refuses them */
	return posix_shim_new(POSIX_SHIM_SEMAPHORE, 0, initial, maximum);
}
#define CreateSemaphoreW CreateSemaphore
#define CreateSemaphoreA CreateSemaphore

static inline HANDLE CreateEvent(void *attr, BOOL manual_reset, BOOL initial, const void *name)
{
	(void)attr; (void)name;
	return posix_shim_new(POSIX_SHIM_EVENT, manual_reset, initial ? 1 : 0, 1);
}
#define CreateEventW CreateEvent
#define CreateEventA CreateEvent

/* Caller holds the registry lock. */
static inline int posix_shim_ready(struct posix_shim_object *o) { return o->count > 0; }

/* Caller holds the registry lock, and the object is ready. */
static inline void posix_shim_take(struct posix_shim_object *o)
{
	if (o->kind == POSIX_SHIM_SEMAPHORE) o->count--;
	else if (!o->manual) o->count = 0;
}

/* Caller holds the registry lock. */
static inline void posix_shim_wake(struct posix_shim_object *o)
{
	pthread_cond_broadcast(&o->c);
	if (posix_shim_reg()->multi_waiters > 0) pthread_cond_broadcast(&posix_shim_reg()->any);
}

static inline BOOL ReleaseSemaphore(HANDLE h, long n, long *previous)
{
	struct posix_shim_object *o = (struct posix_shim_object *)h;
	if (o == NULL || o->kind != POSIX_SHIM_SEMAPHORE) return FALSE;
	if (n < 1) return FALSE;
	pthread_mutex_lock(&posix_shim_reg()->m);
	if (o->count + n > o->max) {
		/* Win32: the release fails and the count is unchanged. WDSP relies on it
		   for its max-1 semaphores: a command released twice runs once. */
		pthread_mutex_unlock(&posix_shim_reg()->m);
		return FALSE;
	}
	if (previous) *previous = o->count;
	o->count += n;
	posix_shim_wake(o);
	pthread_mutex_unlock(&posix_shim_reg()->m);
	return TRUE;
}

static inline BOOL SetEvent(HANDLE h)
{
	struct posix_shim_object *o = (struct posix_shim_object *)h;
	if (o == NULL || o->kind != POSIX_SHIM_EVENT) return FALSE;
	pthread_mutex_lock(&posix_shim_reg()->m);
	o->count = 1;
	posix_shim_wake(o);
	pthread_mutex_unlock(&posix_shim_reg()->m);
	return TRUE;
}

static inline BOOL ResetEvent(HANDLE h)
{
	struct posix_shim_object *o = (struct posix_shim_object *)h;
	if (o == NULL || o->kind != POSIX_SHIM_EVENT) return FALSE;
	pthread_mutex_lock(&posix_shim_reg()->m);
	o->count = 0;
	pthread_mutex_unlock(&posix_shim_reg()->m);
	return TRUE;
}

static inline BOOL CloseHandle(HANDLE h)
{
	struct posix_shim_object *o = (struct posix_shim_object *)h;
	if (o == NULL) return FALSE;
	pthread_cond_destroy(&o->c);
	free(o);
	return TRUE;
}

static inline DWORD WaitForSingleObject(HANDLE h, DWORD ms)
{
	struct posix_shim_object *o = (struct posix_shim_object *)h;
	struct posix_shim_registry *r = posix_shim_reg();
	struct timespec deadline;
	if (o == NULL) return WAIT_FAILED;
	if (ms != INFINITE) {
		clock_gettime(CLOCK_MONOTONIC, &deadline);
		deadline.tv_sec += ms / 1000;
		deadline.tv_nsec += (long)(ms % 1000) * 1000000L;
		if (deadline.tv_nsec >= 1000000000L) { deadline.tv_sec++; deadline.tv_nsec -= 1000000000L; }
	}
	pthread_mutex_lock(&r->m);
	while (!posix_shim_ready(o)) {
		if (ms == INFINITE) {
			pthread_cond_wait(&o->c, &r->m);
		} else if (pthread_cond_timedwait(&o->c, &r->m, &deadline) == ETIMEDOUT) {
			if (posix_shim_ready(o)) break;
			pthread_mutex_unlock(&r->m);
			return WAIT_TIMEOUT;
		}
	}
	posix_shim_take(o);
	pthread_mutex_unlock(&r->m);
	return WAIT_OBJECT_0;
}

/* Wait-any only (calcc.c:1790 passes FALSE); wait-all is not used by WDSP. */
static inline DWORD WaitForMultipleObjects(DWORD n, const HANDLE *hs, BOOL wait_all, DWORD ms)
{
	struct posix_shim_registry *r = posix_shim_reg();
	struct timespec deadline;
	DWORD i;
	if (wait_all || n == 0) return WAIT_FAILED;
	if (ms != INFINITE) {
		clock_gettime(CLOCK_MONOTONIC, &deadline);
		deadline.tv_sec += ms / 1000;
		deadline.tv_nsec += (long)(ms % 1000) * 1000000L;
		if (deadline.tv_nsec >= 1000000000L) { deadline.tv_sec++; deadline.tv_nsec -= 1000000000L; }
	}
	pthread_mutex_lock(&r->m);
	r->multi_waiters++;
	for (;;) {
		for (i = 0; i < n; i++) {
			struct posix_shim_object *o = (struct posix_shim_object *)hs[i];
			if (o != NULL && posix_shim_ready(o)) {
				posix_shim_take(o);
				r->multi_waiters--;
				pthread_mutex_unlock(&r->m);
				return WAIT_OBJECT_0 + i;
			}
		}
		if (ms == INFINITE) {
			pthread_cond_wait(&r->any, &r->m);
		} else if (pthread_cond_timedwait(&r->any, &r->m, &deadline) == ETIMEDOUT) {
			r->multi_waiters--;
			pthread_mutex_unlock(&r->m);
			return WAIT_TIMEOUT;
		}
	}
}

/* ---- threads -------------------------------------------------------------- */

struct posix_shim_thread_start {
	void (*fn)(void *);
	void *arg;
};

static inline void *posix_shim_thread_main(void *p)
{
	struct posix_shim_thread_start s = *(struct posix_shim_thread_start *)p;
	free(p);
	s.fn(s.arg);
	return NULL;
}

/* Returns the pthread_t as an integer, as the Win32 call returns a thread
   handle; (HANDLE)(uintptr_t) round-trips it for SetThreadPriority. */
static inline uintptr_t _beginthread(void (*fn)(void *), unsigned stack, void *arg)
{
	pthread_t t;
	pthread_attr_t a;
	struct posix_shim_thread_start *s = (struct posix_shim_thread_start *)malloc(sizeof *s);
	(void)stack;
	if (s == NULL) return (uintptr_t)-1;
	s->fn = fn;
	s->arg = arg;
	pthread_attr_init(&a);
	pthread_attr_setdetachstate(&a, PTHREAD_CREATE_DETACHED);
	if (pthread_create(&t, &a, posix_shim_thread_main, s) != 0) {
		pthread_attr_destroy(&a);
		free(s);
		return (uintptr_t)-1;
	}
	pthread_attr_destroy(&a);
	return (uintptr_t)t;
}

static inline void _endthread(void) { pthread_exit(NULL); }

struct posix_shim_work_item {
	DWORD (*fn)(void *);
	void *arg;
};

static inline void *posix_shim_work_main(void *p)
{
	struct posix_shim_work_item w = *(struct posix_shim_work_item *)p;
	free(p);
	w.fn(w.arg);
	return NULL;
}

static inline BOOL QueueUserWorkItem(DWORD (*fn)(void *), void *arg, DWORD flags)
{
	pthread_t t;
	pthread_attr_t a;
	struct posix_shim_work_item *w = (struct posix_shim_work_item *)malloc(sizeof *w);
	(void)flags;
	if (w == NULL) return FALSE;
	w->fn = fn;
	w->arg = arg;
	pthread_attr_init(&a);
	pthread_attr_setdetachstate(&a, PTHREAD_CREATE_DETACHED);
	if (pthread_create(&t, &a, posix_shim_work_main, w) != 0) {
		pthread_attr_destroy(&a);
		free(w);
		return FALSE;
	}
	pthread_attr_destroy(&a);
	return TRUE;
}

#define GetCurrentThread() ((HANDLE)(uintptr_t)pthread_self())

/* Best effort, for the calling thread only: a nice value of -10 for HIGHEST
   and above, 0 otherwise. Lowering nice needs CAP_SYS_NICE or an RLIMIT_NICE
   that allows it; without them this returns FALSE and the thread runs as it
   was, which is not an error. */
static inline BOOL SetThreadPriority(HANDLE h, int priority)
{
	if ((pthread_t)(uintptr_t)h != pthread_self()) return FALSE;
	int nice_value = priority >= THREAD_PRIORITY_HIGHEST ? -10 : 0;
	return setpriority(PRIO_PROCESS, (id_t)syscall(SYS_gettid), nice_value) == 0 ? TRUE : FALSE;
}

/* MMCSS "Pro Audio": SCHED_FIFO at a modest priority for the calling thread.
   NULL when the process may not (RLIMIT_RTPRIO 0 and no CAP_SYS_NICE), so
   main.c falls back to SetThreadPriority. The priority is below what a
   supervisor would give the radio's receive thread, so the DSP cannot starve
   the link that feeds it. */
#define POSIX_SHIM_RT_PRIORITY 10

static inline HANDLE AvSetMmThreadCharacteristics(const char *task, DWORD *index)
{
	struct sched_param sp;
	(void)task;
	if (index) *index = 0;
	memset(&sp, 0, sizeof sp);
	sp.sched_priority = POSIX_SHIM_RT_PRIORITY;
	if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) return NULL;
	return (HANDLE)(uintptr_t)1;
}
#define AvSetMmThreadCharacteristicsA AvSetMmThreadCharacteristics
#define AvSetMmThreadCharacteristicsW AvSetMmThreadCharacteristics

static inline BOOL AvSetMmThreadPriority(HANDLE h, int priority)
{
	(void)h; (void)priority;
	return TRUE;
}

static inline BOOL AvRevertMmThreadCharacteristics(HANDLE h)
{
	struct sched_param sp;
	(void)h;
	memset(&sp, 0, sizeof sp);
	return pthread_setschedparam(pthread_self(), SCHED_OTHER, &sp) == 0 ? TRUE : FALSE;
}

/* ---- console -------------------------------------------------------------- */

static inline BOOL AllocConsole(void) { return TRUE; }
static inline BOOL FreeConsole(void) { return TRUE; }
/* wisdom.c:56 `freopen_s(&stream, "conout$", "w", stdout)`: stdout is kept. */
#define freopen_s(pp, path, mode, stream) ((*(pp) = (stream)), 0)

static inline void OutputDebugStringA(const char *s) { fputs(s, stderr); }
#define OutputDebugString OutputDebugStringA

/* ---- memory --------------------------------------------------------------- */

static inline void *_aligned_malloc(size_t size, size_t alignment)
{
	void *p = NULL;
	if (alignment < sizeof(void *)) alignment = sizeof(void *);
	if (size == 0) size = 1;
	if (posix_memalign(&p, alignment, size) != 0) return NULL;
	return p;
}

static inline void _aligned_free(void *p) { free(p); }

#endif /* POSIX_SHIM_H */
