
#ifndef __PERF_CNT__
#define __PERF_CNT__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct Result {
	int pass;
	unsigned long msec;
} Result;

void bench_prepare(Result *res);
void bench_done(Result *res);

#endif
