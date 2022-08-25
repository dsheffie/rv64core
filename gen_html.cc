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
  r.read(fname);
  size_t cnt = 0;
  cout << "Start at " << start << " and complete at " << len+start << "\n";
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
       << "\"events\":{"
       << "\"" << rec.fetch_cycle << "\":\"F\","
       << "\"" << rec.alloc_cycle << "\":\"A\","
       << "\"" << rec.complete_cycle << "\":\"C\","
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
