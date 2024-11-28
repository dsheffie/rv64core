#ifndef __pipeline_record_hh__
#define __pipeline_record_hh__

#if BOOST_VERSION >= 107400
#include <boost/serialization/library_version_type.hpp>
#endif


#include <boost/serialization/serialization.hpp>
#include <boost/serialization/list.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <cstdint>
#include <fstream>
#include <string>
#include <list>

class pipeline_record {
public:
  uint64_t uuid;
  std::string disasm;
  uint64_t pc;
  uint64_t fetch_cycle, alloc_cycle, sched_cycle, complete_cycle, retire_cycle;
  uint64_t p1_hit_cycle, p1_miss_cycle, l1d_replay;
  bool faulted;
  
  std::list<uint64_t> l1d_blocks, l1d_sd;  
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & uuid;
    ar & disasm;
    ar & pc;
    ar & fetch_cycle;
    ar & alloc_cycle;
    ar & sched_cycle;
    ar & complete_cycle;
    ar & retire_cycle;
    ar & p1_hit_cycle;
    ar & p1_miss_cycle;
    ar & l1d_replay;
    ar & faulted;    
    ar & l1d_blocks;
    ar & l1d_sd;
  }
public:
  pipeline_record(uint64_t uuid,
		  const std::string &disasm,
		  uint64_t pc,
		  uint64_t fetch_cycle,
		  uint64_t alloc_cycle,
		  uint64_t sched_cycle,
		  uint64_t complete_cycle,
		  uint64_t retire_cycle,
		  uint64_t p1_hit_cycle,
		  uint64_t p1_miss_cycle,
		  uint64_t l1d_replay,
		  const std::list<uint64_t> &l1d_blocks,
		  const std::list<uint64_t> &l1d_sd,
		  bool faulted) :
    uuid(uuid), disasm(disasm), pc(pc), fetch_cycle(fetch_cycle), alloc_cycle(alloc_cycle),
    sched_cycle(sched_cycle), complete_cycle(complete_cycle), retire_cycle(retire_cycle),
    p1_hit_cycle(p1_hit_cycle), p1_miss_cycle(p1_miss_cycle), l1d_replay(l1d_replay),
    faulted(faulted),
    l1d_blocks(l1d_blocks), l1d_sd(l1d_sd)  {}
  pipeline_record() :
    uuid(~0UL), disasm(""), pc(0), fetch_cycle(0), alloc_cycle(0),
    complete_cycle(0), retire_cycle(0),
    p1_hit_cycle(~0UL), p1_miss_cycle(~0UL), l1d_replay(~0UL),
    faulted(false) {}

  friend std::ostream &operator<<(std::ostream &out, const pipeline_record &r) {
    out << r.disasm << "," << std::hex <<  r.pc << std::dec
	<< "," << r.fetch_cycle
	<< "," << r.alloc_cycle
	<< "," << r.sched_cycle      
	<< "," << r.complete_cycle
	<< "," << r.retire_cycle
	<< "," << r.faulted;
    return out;
  }
};


class pipeline_data {
protected:
  std::list<pipeline_record> records;
  friend class boost::serialization::access;  
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & records;
  }
public:
  pipeline_data() {}
  const std::list<pipeline_record> &get_records() const {
    return records;
  }
  std::list<pipeline_record> &get_records() {
    return records;
  }  
};

class pipeline_logger : public pipeline_data {
private:
  std::ofstream *ofs = nullptr;
  boost::archive::binary_oarchive *oa = nullptr;
public:
  pipeline_logger(const std::string &out) {
    ofs = new std::ofstream(out, std::ios::binary);
    oa = new boost::archive::binary_oarchive(*ofs);
  }
  ~pipeline_logger() {
    if(oa and ofs) {
      *oa << *this;
    }
    if(oa) {
      delete oa;
    }
    if(ofs) {
      delete ofs;
    }
  }
  void append(uint64_t uuid,
	      const std::string &disasm,
	      uint64_t pc,
	      uint64_t fetch_cycle,
	      uint64_t alloc_cycle,
	      uint64_t sched_cycle,
	      uint64_t complete_cycle,
	      uint64_t retire_cycle,
	      uint64_t p1_hit_cycle,
	      uint64_t p1_miss_cycle,
	      uint64_t l1d_replay,
	      const std::list<uint64_t> &l1d_blocks,
	      const std::list<uint64_t> &l1d_sd,	      
	      bool faulted) {

    assert(fetch_cycle != (~0UL));
    assert(alloc_cycle != (~0UL));
    if(sched_cycle == (~0UL)) {
      std::cout << disasm << "\n";
    }
    assert(sched_cycle != (~0UL));
    assert(complete_cycle != (~0UL));
    assert(retire_cycle != (~0UL));    
    records.emplace_back(uuid,
			 disasm, pc,
			 fetch_cycle,
			 alloc_cycle,
			 sched_cycle,
			 complete_cycle,
			 retire_cycle,
			 p1_hit_cycle,
			 p1_miss_cycle,
			 l1d_replay,
			 l1d_blocks,
			 l1d_sd,
			 faulted);
  }
};


class pipeline_reader : public pipeline_data {
public:
  pipeline_reader() {}
  
  void read(const std::string &fname) {
    std::ifstream ifs(fname, std::ios::binary);
    boost::archive::binary_iarchive ia(ifs);
    // read class state from archive
    ia >> *this;
    std::cout << "read " << records.size() << " records\n";
  }
  
};



#endif
