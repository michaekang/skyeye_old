#ifndef LRU_TLB_H
#define LRU_TLB_H

#include <iostream>
#include <map>
#include <list>
#include <ext/hash_map>
#include <backward/hash_fun.h>

using namespace std;
using namespace __gnu_cxx;

typedef unsigned int address_t;

#define SET_SIZE	256
#define WAY_SIZE	8
struct tlb_item {
	unsigned int pa;
	unsigned int va;
	tlb_item() {
		pa = va = 0;
	}
	tlb_item(unsigned int va, unsigned int pa) {
		this->pa = pa;
		this->va = va;
	}
	tlb_item(const tlb_item& tlb) {
		this->pa = tlb.pa;
		this->va = tlb.va;
	}
	bool operator== (const tlb_item &tlb) const {
		return tlb.pa == pa && tlb.va == va;
	}
};
typedef hash_map<unsigned int, unsigned int, hash<unsigned int>, equal_to<unsigned int> > HASH;
//typedef map<unsigned int, unsigned int> HASH;
class tlb_table {
public:
//	std::list<unsigned int> tlb_list;
//	HASH tlb_map;
	tlb_item tlb_cache[256][2048];
	friend ostream &operator<< (ostream &os, const tlb_table& t);
	tlb_table():hit(0), total(0)  {}
	int get_phys_addr(unsigned int va, unsigned int &pa);
	void insert(unsigned int va, unsigned int pa);
	void erase(unsigned int va);
	void erase_by_asid(unsigned int asid);
	void clear();
	int hit;
	int total;
};
#endif
