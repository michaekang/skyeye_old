
#include "lru_tlb.h"
#include <cstring>
#include <cstdio>

ostream & operator<< (ostream &os, const tlb_table& t)
{
	HASH::const_iterator it;
#ifdef __HASH_TLB__
	for (it = t.tlb_map.begin(); it != t.tlb_map.end(); ++it)
		os << it->first << " ";
	os << std::endl;
#endif
	return os;
}

int tlb_table::get_phys_addr(unsigned int va, unsigned int &pa)
{
	tlb_item *tlb_entry = &tlb_cache[va & 0xff][(va >> 12) % 2048];
	if (va == tlb_entry->va) {
		pa = tlb_entry->pa;
		return 0;
	} else {
		return -1;
	}
	#if 0
	for (int i = 0; i < 8; i ++) {
		if (tlb_entry[i]->va == va) {
			pa = tlb_entry->pa;
			break;
		}
	}
	#endif
#ifdef __HASH_TLB__
	HASH::const_iterator it;
	if ((it = tlb_map.find(va)) != tlb_map.end()) {
		pa = it->second;
		return 0;
	} else
		return -1;
#endif
}

void tlb_table::insert(unsigned int va, unsigned int pa)
{
//	std::pair< hash_map<unsigned int, tlb_item, hash<unsigned int>, equal_to<unsigned int> >::const_iterator, bool> ret = tlb_map.insert(std::make_pair(tlb.va, tlb));
	tlb_item* tlb_entry = &tlb_cache[va & 0xff][(va >> 12) % 2048];

	tlb_entry->va = va;
	tlb_entry->pa = pa;

#ifdef __HASH_TLB__
	std::pair< HASH::const_iterator, bool> ret = tlb_map.insert(std::make_pair(va, pa));
	if (!ret.second) {
		std::cerr << "insert a reduplicate tlb_item in tlb_table" << std::endl;
		std::cerr << "pa : " << std::hex << pa << " va : " << std::hex << va << std::endl;
	} else {
//		std::cout << "insert a tlb_item." << std::endl;
//		std::cout << "pa : " << std::hex << tlb.pa << " va : " << std::hex << tlb.va << std::endl;
	}
#endif
}

void tlb_table::erase(unsigned int va)
{
//	HASH::const_iterator it;
//	unsigned int mva = va >> 12;
//	if ((it = tlb_map.find(va)) != tlb_map.end()) {]
	tlb_cache[va & 0xff][(va >> 12) % 2048].va = 0;
#ifdef __HASH_TLB__
	tlb_map.erase(va);
#endif
//		std::cout << "erase by mva " << std::hex << va << std::endl;
//	}
}

void tlb_table::erase_by_asid(unsigned int asid)
{
	memset(&tlb_cache[asid], 0, sizeof(tlb_item) * 2048);
#ifdef __HASH_TLB__
	HASH::iterator it;
	for (it = tlb_map.begin(); it != tlb_map.end(); ) {
		if ((it->first & 0xff) == asid) {
//			std::cout << "erase by asid " << std::hex << asid << std::endl;
//			std::cout << "pa : " << std::hex << it->second.pa << " va : " << std::hex << it->second.va << std::endl;
			tlb_map.erase(it ++);
		} else
			++it;
	}
#endif

}

void tlb_table::clear()
{
	memset(tlb_cache, 0, sizeof(tlb_item) * 2048 * 256);
#ifdef __HASH_TLB__
	tlb_map.clear();
	tlb_list.clear();
#endif
}
