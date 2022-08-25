#include <iostream>
#include <regex>
#include <fstream>
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
  list<string> pre, post, ops;
  pipeline_reader r;
  read_template(pre, post);
  r.read("dhry.log");
  size_t cnt = 0;
  static const size_t start = 2*1024*1024;
  for(auto &rec : r.get_records()) {
    ++cnt;
    if(cnt < start)
      continue;
    if(cnt > (start + 256))
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
    std::cout << ss.str() << "\n";
  }
  std::ofstream o("out.html");
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
