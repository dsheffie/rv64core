#ifndef __pipeline_record_hh__
#define __pipeline_record_hh__

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
private:
  uint64_t pc;
  uint64_t fetch_cycle, alloc_cycle, complete_cycle, retire_cycle;
  bool faulted;
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & pc;
    ar & fetch_cycle;
    ar & alloc_cycle;
    ar & complete_cycle;
    ar & retire_cycle;
    ar & faulted;
  }
public:
  pipeline_record(uint64_t pc,
		  uint64_t fetch_cycle,
		  uint64_t alloc_cycle,
		  uint64_t complete_cycle,
		  uint64_t retire_cycle,
		  bool faulted) :
    pc(pc), fetch_cycle(fetch_cycle), alloc_cycle(alloc_cycle),
    complete_cycle(complete_cycle), retire_cycle(retire_cycle),
    faulted(faulted) {}
};

class pipeline_logger {
private:
  std::ofstream *ofs = nullptr;
  boost::archive::binary_oarchive *oa = nullptr;
  std::list<pipeline_record> records;
  friend class boost::serialization::access;  
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version) {
    ar & records;
  }
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
  void append(uint64_t pc,
	      uint64_t fetch_cycle,
	      uint64_t alloc_cycle,
	      uint64_t complete_cycle,
	      uint64_t retire_cycle,
	      bool faulted) {
    records.emplace_back(pc, fetch_cycle, alloc_cycle, complete_cycle,
			 retire_cycle, faulted);
  }
};



#endif
