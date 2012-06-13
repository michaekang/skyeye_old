#ifndef __PROFILER_H__
#define __PROFILER_H__
#include <stdint.h>
#include <map>
#include <vector>

#include "skyeye_dyncom.h"
using namespace std;
typedef map<unsigned int, unsigned int> AddrSet;

enum {
	JIT_TYPE_TRACE,
	JIT_TYPE_TREE_GROUP
};

enum {
	COND            = (1 << 0),
	NON_BRANCH      = (1 << 1),
	DIRECT_BRANCH   = (1 << 2),
	INDIRECT_BRANCH = (1 << 3),
	CALL            = (1 << 4),
	RET             = (1 << 5),
	END_OF_PAGE     = (1 << 6),
	THUMB           = (1 << 7)
};

/* profiling data is used for direct branch only. */
typedef struct _profiling_data {
	/* type of basicblock */
	uint32_t type;
	/* how many instructions in this bb */
	uint32_t size;
	/* start physical address of this bb */
	uint32_t start;
	uint32_t addr[2];
	int32_t  vpc[2];
	uint32_t count[2];
	uint32_t in_superblock;
	uint64_t clocktime;
	uint32_t reference;
	uint32_t start_of_sb;
//	bb_stat* bb_count;
} profiling_data;

typedef struct _hotspot {
       uint64_t reference;
       uint64_t clocktime;
       int32_t id;
       AddrSet in;
       AddrSet out;
       int in_use;
       void *jit;
} hotspot;

typedef map<unsigned int, hotspot *> hotpath_map;

typedef struct _bb_stat {
	uint32_t pfn;
//	uint64_t clocktime;
	uint32_t freeze;
	uint32_t count;
} bb_stat;

void merge_page(uint32_t id);
void insert_hotpath(vector<uint32_t> &start_addr, hotspot *jit);
hotspot *find_hotpath(unsigned int addr);
void flush_hotpath_by_pfn(unsigned int pfn);
void record_trace(profiling_data *pd);
void alloc_hotspot_id();
void free_hotspot_id();
uint32_t get_hotspot_id();
hotspot *gene_native_code(cpu_t *cpu);
void resume_timing();
void pause_timing();
void update_walltime(int sig);
void set_profiler_timer();
void print_trace();
void print_entry();

#endif
