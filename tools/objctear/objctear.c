/*
 * objctear: provokes the torn libobjc method-cache read of report 27 (§1.3-§1.5) and says whether it crashed.
 *
 * libobjc writes a cache bucket {imp ^ isa, sel} with one STP and objc_msgSend reads it with one LDP; on the
 * Cortex-A53/A73 (no FEAT_LSE2) the pair is not single-copy atomic. A reader that sees the new sel with imp 0
 * branches to the class itself -> SIGBUS, pc = far = x15 = x16 = x17 = the class, x9 = x1 = the selector.
 *
 * Each round makes a new class (NSObject subclass) with 48 methods; N worker threads (default 4) call all of them
 * at once, each from a different starting point, so the class's empty cache is filled (and regrown) under
 * concurrent readers. Every call checks its result, so a wrong method would be noticed too.
 *
 *   objctear [ui|bg|mix] [seconds] [threads]
 *     ui  = QoS user-interactive (A73 with sweetsched=15), bg = background (A53), mix = half and half
 *     seconds: time limit (default 60); threads: 2-16 (default 4)
 *
 * Exit status: 0 survived, 2 crashed (signal; the registers are printed), 3 wrong result, 1 usage/setup error.
 * With kernel 0433 it also prints kern.sweet_objc_tear and kern.sweet_objc_tear_count before and after.
 *
 * Build: xcrun -sdk iphoneos clang -arch arm64 -miphoneos-version-min=16.0 -O2 -o objctear objctear.c -lobjc
 * (then sign it as the other tools: ldid -S objctear, or with the entitlements the lab uses).
 */
#include <objc/runtime.h>
#include <objc/message.h>
#include <pthread.h>
#include <pthread/qos.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/sysctl.h>
#include <sys/ucontext.h>
#include <time.h>
#include <unistd.h>

#define NMETHODS 48
#define MAXTHREADS 16
#define PASSES 4                /* each thread calls every method this many times per round */

static SEL sels[NMETHODS];
static int nthreads = 4;
static const char *mode = "ui";

static _Atomic(id) round_obj;
static atomic_uint round_gen;
static atomic_uint round_done;
static atomic_ulong calls;
static atomic_ulong bad;
static volatile unsigned long cur_round;

typedef uintptr_t (*send_fn)(id, SEL);

static uintptr_t
method_imp(id self, SEL _cmd)
{
	(void)self;
	return (uintptr_t)_cmd ^ 0x5a5aUL;
}

static void
read_tear_sysctls(int *mode_out, long long *count_out)
{
	int m = -1;
	long long c = -1;
	size_t len = sizeof(m);
	if (sysctlbyname("kern.sweet_objc_tear", &m, &len, NULL, 0) != 0) {
		m = -1;
	}
	len = sizeof(c);
	if (sysctlbyname("kern.sweet_objc_tear_count", &c, &len, NULL, 0) != 0) {
		c = -1;
	}
	*mode_out = m;
	*count_out = c;
}

static void
on_fault(int sig, siginfo_t *si, void *uap)
{
	ucontext_t *uc = uap;
	char buf[512];
	uint64_t pc = 0, far = 0, x1 = 0, x9 = 0, x15 = 0, x16 = 0, x17 = 0, lr = 0;
	uint32_t esr = 0;
#if defined(__arm64__)
	if (uc != NULL && uc->uc_mcontext != NULL) {
		const struct __darwin_arm_thread_state64 *ss = &uc->uc_mcontext->__ss;
		pc = ss->__pc;
		lr = ss->__lr;
		x1 = ss->__x[1];
		x9 = ss->__x[9];
		x15 = ss->__x[15];
		x16 = ss->__x[16];
		x17 = ss->__x[17];
		far = uc->uc_mcontext->__es.__far;
		esr = uc->uc_mcontext->__es.__esr;
	}
#else
	(void)uc;       /* host smoke test only; the registers are read on arm64 */
#endif
	int torn = pc != 0 && pc == far && pc == x15 && pc == x16 && pc == x17 && x1 != 0 && x9 == x1;
	int n = snprintf(buf, sizeof(buf),
	    "objctear: CRASHED: signal %d in round %lu (si_addr %p)\n"
	    "  pc 0x%llx far 0x%llx esr 0x%x lr 0x%llx\n"
	    "  x1 0x%llx x9 0x%llx x15 0x%llx x16 0x%llx x17 0x%llx\n"
	    "  torn-bucket signature (pc=far=x15=x16=x17, x9=x1): %s\n"
	    "RESULT: CRASHED\n",
	    sig, cur_round, si ? si->si_addr : NULL, pc, far, esr, lr, x1, x9, x15, x16, x17, torn ? "YES" : "no");
	write(STDOUT_FILENO, buf, (size_t)n);
	_exit(2);
}

static void *
worker(void *arg)
{
	unsigned t = (unsigned)(uintptr_t)arg;
	unsigned seen = 0;
	send_fn send = (send_fn)objc_msgSend;

	for (;;) {
		unsigned g;
		while ((g = atomic_load_explicit(&round_gen, memory_order_acquire)) == seen) {
			/* spin: keep the cores busy so all workers start together */
		}
		seen = g;
		if (g == UINT32_MAX) {
			return NULL;
		}
		id obj = atomic_load_explicit(&round_obj, memory_order_acquire);
		unsigned start = (t * NMETHODS) / (unsigned)nthreads;
		unsigned long mine = 0, wrong = 0;
		for (unsigned p = 0; p < PASSES; p++) {
			for (unsigned i = 0; i < NMETHODS; i++) {
				/* odd threads walk backwards: more distinct buckets being filled at the same moment */
				unsigned k = (t & 1) ? (start + NMETHODS - i) % NMETHODS : (start + i) % NMETHODS;
				SEL s = sels[k];
				if (send(obj, s) != ((uintptr_t)s ^ 0x5a5aUL)) {
					wrong++;
				}
				mine++;
			}
		}
		atomic_fetch_add(&calls, mine);
		if (wrong) {
			atomic_fetch_add(&bad, wrong);
		}
		atomic_fetch_add_explicit(&round_done, 1, memory_order_release);
	}
}

int
main(int argc, char **argv)
{
	int seconds = 60;
	if (argc > 1) {
		mode = argv[1];
	}
	if (argc > 2) {
		seconds = atoi(argv[2]);
	}
	if (argc > 3) {
		nthreads = atoi(argv[3]);
	}
	if ((strcmp(mode, "ui") && strcmp(mode, "bg") && strcmp(mode, "mix")) || seconds <= 0 ||
	    nthreads < 2 || nthreads > MAXTHREADS) {
		fprintf(stderr, "usage: %s [ui|bg|mix] [seconds] [threads 2-%d]\n", argv[0], MAXTHREADS);
		return 1;
	}
	setvbuf(stdout, NULL, _IOLBF, 0);

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_sigaction = on_fault;
	sa.sa_flags = SA_SIGINFO;
	sigaction(SIGBUS, &sa, NULL);
	sigaction(SIGSEGV, &sa, NULL);
	sigaction(SIGILL, &sa, NULL);

	Class base = objc_getClass("NSObject");
	if (base == Nil) {
		fprintf(stderr, "objctear: no NSObject\n");
		return 1;
	}
	char name[64];
	for (int i = 0; i < NMETHODS; i++) {
		snprintf(name, sizeof(name), "objctearMethod%02d", i);
		sels[i] = sel_registerName(name);
	}

	int tear_mode;
	long long tear_before;
	read_tear_sysctls(&tear_mode, &tear_before);
	printf("objctear: mode %s, %d threads, %d s; kern.sweet_objc_tear %d, kern.sweet_objc_tear_count %lld%s\n",
	    mode, nthreads, seconds, tear_mode, tear_before, tear_mode < 0 ? " (no 0433 in this kernel)" : "");

	pthread_t th[MAXTHREADS];
	for (int t = 0; t < nthreads; t++) {
		pthread_attr_t attr;
		pthread_attr_init(&attr);
		qos_class_t q = QOS_CLASS_USER_INTERACTIVE;
		if (!strcmp(mode, "bg") || (!strcmp(mode, "mix") && (t & 1))) {
			q = QOS_CLASS_BACKGROUND;
		}
		pthread_attr_set_qos_class_np(&attr, q, 0);
		if (pthread_create(&th[t], &attr, worker, (void *)(uintptr_t)t) != 0) {
			perror("pthread_create");
			return 1;
		}
		pthread_attr_destroy(&attr);
	}

	struct timespec t0, now;
	clock_gettime(CLOCK_MONOTONIC, &t0);
	unsigned long round = 0;
	for (;; round++) {
		clock_gettime(CLOCK_MONOTONIC, &now);
		double el = (double)(now.tv_sec - t0.tv_sec) + (double)(now.tv_nsec - t0.tv_nsec) / 1e9;
		if (el >= seconds) {
			break;
		}
		cur_round = round;
		snprintf(name, sizeof(name), "ObjcTear_%d_%lu", getpid(), round);
		Class c = objc_allocateClassPair(base, name, 0);
		if (c == Nil) {
			fprintf(stderr, "objctear: objc_allocateClassPair(%s) failed\n", name);
			return 1;
		}
		for (int i = 0; i < NMETHODS; i++) {
			class_addMethod(c, sels[i], (IMP)method_imp, "Q16@0:8");
		}
		objc_registerClassPair(c);
		id obj = class_createInstance(c, 0);

		atomic_store_explicit(&round_done, 0, memory_order_relaxed);
		atomic_store_explicit(&round_obj, obj, memory_order_release);
		atomic_fetch_add_explicit(&round_gen, 1, memory_order_release);
		while (atomic_load_explicit(&round_done, memory_order_acquire) != (unsigned)nthreads) {
			/* wait for the round */
		}
		if (atomic_load(&bad) != 0) {
			printf("objctear: WRONG RESULT: %lu calls returned another method's value (round %lu)\nRESULT: WRONG\n",
			    atomic_load(&bad), round);
			return 3;
		}
		object_dispose(obj);
		objc_disposeClassPair(c);
		if (round % 10000 == 9999) {
			printf("objctear: %lu rounds, %.1f s\n", round + 1, el);
		}
	}

	long long tear_after;
	read_tear_sysctls(&tear_mode, &tear_after);
	clock_gettime(CLOCK_MONOTONIC, &now);
	double el = (double)(now.tv_sec - t0.tv_sec) + (double)(now.tv_nsec - t0.tv_nsec) / 1e9;
	printf("objctear: %lu rounds, %lu calls in %.1f s, no fault\n", round, atomic_load(&calls), el);
	if (tear_after >= 0) {
		printf("objctear: kern.sweet_objc_tear_count %lld -> %lld (+%lld, all processes)\n",
		    tear_before, tear_after, tear_after - tear_before);
	}
	printf("RESULT: SURVIVED\n");
	return 0;
}
