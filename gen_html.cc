#include <iostream>
#include <regex>
#include <fstream>
#include <boost/program_options.hpp>
#include "pipeline_record.hh"

using namespace std;

static void read_template(list<string> &pre, list<string> &post) {
  string line;
  fstream in("traceTemplate.html");
  bool use_pre = true;
  //var tableData = {}
  while(getline(in, line)) {
    if(regex_search(line, regex("var tableData"))) {
      use_pre = false;
      continue;
    }
    if(use_pre) {
      pre.push_back(line);
    }
    else {
      post.push_back(line);
    }

  }
}

enum class eventtype {FETCH, ALLOC, COMPLETE, RETIRE};

struct event_t {
  eventtype t;
  pipeline_record *rec;
  event_t(eventtype t, pipeline_record *rec) :
    t(t), rec(rec) {}
};

void generate_kanata(pipeline_reader &r, size_t start, size_t len) {
  size_t cnt = 0;
  /* at each cycle, these events happen */
  std::map<uint64_t, std::list<event_t>> cycle_map;
  std::map<uint64_t, uint64_t> remap_table;
  
  std::ofstream out("kanata.log");
  out << "Kanata\t004\n";


  uint64_t first_cycle = ~(0UL);
  uint64_t id = 0;
  for(pipeline_record &rec : r.get_records()) {
    ++cnt;
    if(cnt < start)
      continue;
    if(cnt > (start + len))
      break;

    first_cycle = std::min(first_cycle, rec.fetch_cycle);
    remap_table[rec.uuid] = id++;
    cycle_map[rec.fetch_cycle].emplace_back(eventtype::FETCH, &rec);
    cycle_map[rec.alloc_cycle].emplace_back(eventtype::ALLOC, &rec);
    cycle_map[rec.complete_cycle].emplace_back(eventtype::COMPLETE, &rec);
    cycle_map[rec.retire_cycle].emplace_back(eventtype::RETIRE, &rec);        
  }

  uint64_t last_cycle = ~(0UL);
  for(auto &p : cycle_map) {
    std::cout << "cycle " << p.first << " has " << p.second.size() << " events\n";
    if(last_cycle == (~(0UL))) {
      out << "C=\t"<<first_cycle<<"\n";
    }
    else {
      uint64_t d = p.first - last_cycle;
      out << "C\t"<<d<<"\n";
    }
    for(event_t &e : p.second) {
      uint64_t ii = remap_table.at(e.rec->uuid);
      switch(e.t)
	{
	case eventtype::FETCH:
	  out << "I\t"<<ii<<"\t"<<ii<<"\t"<<"0\n";
	  out << "L\t"<<ii<<"\t0\t"<< e.rec->disasm << "\n";
	  out << "S\t"<<ii<<"0\t"<<"F\n";
	  //out << "E\t"<<ii<<"0\t"<<"F\n";
	  break;
	case eventtype::ALLOC:
	case eventtype::COMPLETE:
	case eventtype::RETIRE:

	  out << "R\t"<<ii<<"\t"<<ii<<"\t"<<"0\n";
	  break;
	}
    }
    last_cycle = p.first;
  }
  
  out.close();

}

int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  string fname, oname;
  size_t len = 0, start = 0;
  try {
    po::options_description desc("options");
    desc.add_options()
      ("in,i", po::value<string>(&fname), "input dump")
      ("out,o", po::value<string>(&oname), "html out")
      ("start,s", po::value<size_t>(&start)->default_value(1UL<<20), "start icnt")
      ("len,l", po::value<size_t>(&len)->default_value(512), "len")      
      ;
       po::variables_map vm;
       po::store(po::parse_command_line(argc, argv, desc), vm);
       po::notify(vm); 
  }
  catch(po::error &e) {
    std::cerr <<"command-line error : " << e.what() << "\n";
    return -1;
  }
  list<string> pre, post, ops;
  pipeline_reader r;
  read_template(pre, post);
  try {
    r.read(fname);
  }
  catch(...) {
    cout << "exception occured loading " << fname << "\n";
    exit(-1);
  }
  cout << r.get_records().size() << " records read\n";
  cout << "Start at " << start << " and complete at " << len+start << "\n";
  
  //generate_kanata(r, start, len);
  
  size_t cnt = 0;


  for(auto &rec : r.get_records()) {
    ++cnt;
    if(cnt < start)
      continue;
    if(cnt > (start + len))
      break;
    stringstream ss;
    ss << "{" << "\"str\":\""
       << hex << rec.pc << dec
       << " " << rec.disasm << "\""
       << ",uops:[{"
       << "\"uuid\":"
       << "\"" << rec.uuid << "\","
       << "\"events\":{"
       << "\"" << rec.fetch_cycle << "\":\"F\","
       << "\"" << rec.alloc_cycle << "\":\"A\","
       << "\"" << rec.sched_cycle << "\":\"S\",";

    for(uint64_t c : rec.l1d_blocks) {
      ss << "\"" << c << "\":\"B\",";
    }
    
    for(uint64_t c : rec.l1d_sd) {
      ss << "\"" << c << "\":\"Z\",";
    }

    if(rec.p1_hit_cycle != (~0UL)) {
      ss << "\"" << rec.p1_hit_cycle << "\":\"H\",";
    }
    if(rec.p1_miss_cycle != (~0UL)) {
      ss << "\"" << rec.p1_miss_cycle << "\":\"M\",";
    }
    if(rec.l1d_replay != (~0UL)) {
      ss << "\"" << rec.l1d_replay << "\":\"L\",";
    }
    ss << "\"" << rec.complete_cycle << "\":\"C\","
       << "\"" << rec.retire_cycle << "\":\"R\""                  
       << "}}]"
       << "}";
    ops.push_back(ss.str());
    //std::cout << ss.str() << "\n";
  }
  std::ofstream o(oname);
  for(auto &l : pre) {
    o << l << "\n";;
  }
  o << "var tableData = [";
  for(auto it = ops.begin(), E = ops.end(); it != E; ++it) {
    string &s = *it;
    o << s;
    if(std::next(it) != E) {
      o << ",";
    }
  }
  o << "]\n";
  for(auto &l : post) {
    o << l << "\n";
  }
  return 0;
}
