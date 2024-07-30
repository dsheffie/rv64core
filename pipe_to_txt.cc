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


int main(int argc, char *argv[]) {
  namespace po = boost::program_options;
  string fname, oname;

  try {
    po::options_description desc("options");
    desc.add_options()
      ("in,i", po::value<string>(&fname), "input dump")
      ("out,o", po::value<string>(&oname), "text output file")
      ;
       po::variables_map vm;
       po::store(po::parse_command_line(argc, argv, desc), vm);
       po::notify(vm); 
  }
  catch(po::error &e) {
    std::cerr <<"command-line error : " << e.what() << "\n";
    return -1;
  }
  pipeline_reader r;
  try {
    r.read(fname);
  }
  catch(...) {
    cout << "exception occured loading " << fname << "\n";
    exit(-1);
  }

  if(oname.empty()) {
    return -1;
  }
  std::ofstream t(oname);
  for(auto &rec : r.get_records()) {
    t << hex << rec.pc << dec
      << "," << rec.disasm
      << "," << rec.uuid
      << "," << rec.fetch_cycle
      << "," << rec.alloc_cycle
      << "," << rec.l1d_port1_cycle
      << "," << rec.l1d_port2_cycle  
      << "," << rec.complete_cycle
      << "," << rec.retire_cycle << "\n";
  }
  t.close();
  return 0;
}
