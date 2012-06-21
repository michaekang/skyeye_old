#include <algorithm>
#include "dyncom/profiler.h"
#include <skyeye_sched.h>

hotpath_map HotpathCache[65536];
uint32_t total_hotpath = 0;
uint64_t total_time = 0;
vector<uint32_t> trace_set[10240];
vector<uint32_t> start_addr[10240];
//vector<uint32_t> inner_jump[1024];
vector<uint32_t> entry_addr[10240];

#define HASH(x) ((x + (x << 2) + (x >> 6)) % 65536)
void insert_hotpath(vector<uint32_t> &start_addr, hotspot *jit)
{
	vector<uint32_t>::iterator it;
	uint32_t addr;
	#if 0
	it = start_addr.begin();
	addr = *it;
	HotpathCache[HASH(addr)][addr] = jit;
	#endif
	#if 1
	for(it = start_addr.begin(); it != start_addr.end(); ++it) {
		addr = *it;
//		printf("insert jit addr %x\n", addr);
		HotpathCache[HASH(addr)][addr] = jit;
	}
	#endif
}

void merge_page(uint32_t id)
{
	vector<uint32_t>::iterator it;
	for(it = start_addr[id].begin(); it != start_addr[id].end(); ++it) {
		start_addr[total_hotpath].push_back(*it);
	}
	for(it = trace_set[id].begin(); it != trace_set[id].end(); ++it) {
		trace_set[total_hotpath].push_back(*it);
	}
}

hotspot *find_hotpath(unsigned int addr) {
	hotpath_map::iterator _it = HotpathCache[HASH(addr)].find(addr);
	if (_it != HotpathCache[HASH(addr)].end()) {
		return _it->second;
	} else
		return NULL;
}

void flush_hotpath_by_pfn(unsigned int pfn) {
	hotpath_map::iterator it;
	uint32_t start;

	for (int i = 0; i < 65536; ++i) {
		for (it = HotpathCache[i].begin(); it != HotpathCache[i].end(); ) {
			start = static_cast<uint32_t>(it->first);
			start &= 0xfffff000;
			if (start == pfn) {
				//printf("[ERASE][0x%08x]\n", static_cast<int>(it->first));
				HotpathCache[i].erase(it ++);
			} else
				++it;
		}
	}
}

void print_out(hotspot *hs) {
	AddrSet::iterator it;
	for (it = hs->out.begin(); it != hs->out.end(); ++it) {
		printf("\t addr : %08x ref : %d\n", it->first, it->second);
	}
}



void record_trace(profiling_data *pd)
{
	int step = (pd->type & THUMB) ? 2 : 4;
	int i = 0;
	uint32_t start = pd->start;

	start_addr[total_hotpath].push_back(start);
	while (i < pd->size) {
		trace_set[total_hotpath].push_back(start);
		start += step;
		i ++;
	}
}

void insert_entry(uint32_t addr) {
	entry_addr[total_hotpath].push_back(addr);
}

void print_vector(vector<uint32_t> &v) {
	vector<uint32_t>::iterator it;
	for(it = trace_set[total_hotpath].begin(); it != trace_set[total_hotpath].end(); ++it)
		printf("pc is %x\n", *it);
}

void print_trace() {
	vector<uint32_t>::iterator it;
	printf("---------------trace begin------------------\n");
	for(it = trace_set[total_hotpath].begin(); it != trace_set[total_hotpath].end(); ++it)
		printf("pc is %x\n", *it);
	printf("---------------trace  end ------------------\n");
}

void print_entry() {
	vector<uint32_t>::iterator it;
	printf("---------------entry begin------------------\n");
	for(it = start_addr[total_hotpath].begin(); it != start_addr[total_hotpath].end(); ++it)
		printf("pc is %x\n", *it);
	printf("---------------entry  end ------------------\n");
}

void alloc_hotspot_id() {
	++total_hotpath;
//	printf("alloc hotpath %d\n", total_hotpath);
}

void free_hotspot_id() {
//	printf("free hotpath %d\n", total_hotpath);
	trace_set[total_hotpath].clear();
	start_addr[total_hotpath].clear();
//	inner_jump[total_hotpath].clear();
	entry_addr[total_hotpath].clear();
	--total_hotpath;
}

uint32_t get_hotspot_id()
{
	return total_hotpath;
}

hotspot *new_hotspot(void *jit) {
	extern uint64_t walltime;
	hotspot *hs = new hotspot;
	hs->jit = jit;
	hs->id = total_hotpath;
	hs->reference = 0;
	hs->clocktime = walltime;
	hs->in_use = 0;
	return hs;
}

int hotspot_id = 0;
hotspot *gene_native_code(cpu_t *cpu) {
	#if 0
	uint64_t t1, t2;
	uint64_t abs_time();
	t1 = abs_time();
	#endif
//	sort(trace_set[total_hotpath].begin(), trace_set[total_hotpath].end());
//	sort(start_addr[total_hotpath].begin(), start_addr[total_hotpath].end());
	hotspot *hs = NULL;
#if 0
	printf("num of instr : %d\n", trace_set[total_hotpath].size());
	if (trace_set[total_hotpath].size() < 200) {
		return NULL;
	}
#endif
	extern void tag_by_trace(cpu_t *, vector<uint32_t> &, vector<uint32_t> &);
	extern void clear_tag_table(cpu_t *cpu);
//	clear_tag_table(cpu);
	#if 1
	tag_by_trace(cpu, trace_set[total_hotpath], start_addr[total_hotpath]);
	cpu->dyncom_engine->cur_tagging_pos ++;
	void * jit = cpu_translate(cpu, cpu->f.get_pc(cpu, cpu->rf.grf));
	hs = new_hotspot(jit);
//	print_trace();
//	print_entry();
	#if 0
//	printf("alloc hotspot %d\n", hs->id);
	if (hotspot_id == hs->id) {
		printf("why...\n");
		exit(-1);
	}
	hotspot_id = hs->id;
	#endif
	insert_hotpath(start_addr[total_hotpath], hs);
	#endif
	#if 0
	t2 = abs_time();
	total_time += t2 - t1;
	#endif
	return hs;
}

uint64_t search_bb_total = 0;
uint64_t walltime;
bool switching = true;
void update_walltime(int sig)
{
	if (switching) {
		walltime ++;
	}
}
void set_profiler_timer()
{
	/* shenoubang add win32 2012-6-20 */
#ifndef __WIN32__
        struct itimerval itv, oldtv;
        itv.it_interval.tv_sec = 0;
        itv.it_interval.tv_usec = 1000;
        itv.it_value.tv_sec = 0;
        itv.it_value.tv_usec = 1000;
//        setitimer(ITIMER_REAL, &itv, &oldtv);
        setitimer(ITIMER_VIRTUAL, &itv, &oldtv);
#endif
	return ;
}

void resume_timing() {
	switching = true;
}
void pause_timing() {
switching = false;
}

