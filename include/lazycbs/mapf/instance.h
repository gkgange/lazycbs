#ifndef LAZYCBS__INSTANCE__H
#define LAZYCBS__INSTANCE__H
#include <string>
#include <utility>
// Alternate implementations of Map and Agent loaders,
// so we can be fully standalone from ECBS.

namespace mapf {

struct Map {
  Map(int r, int c);
  ~Map(void);

  const char* get_map(void) const { return mem; }
  
  int rows;
  int cols;
  char* mem;
};

struct Agents {
  typedef std::pair<int, int> coord;

  struct agent {
    agent(int rs, int cs, int re, int ce)
    : origin(std::make_pair(rs, cs)), goal(std::make_pair(re, ce)) { }
    coord origin;
    coord goal;
  };

  agent operator[](int ai) const { return agents[ai]; }
  void push(const agent& a) { agents.push_back(a); }
  int size(void) const { return agents.size(); }

  std::vector<agent> agents;
};

Map load_ecbs_map(std::string fname);
Map load_movingai_map(std::string fname);

Agents load_ecbs_scenario(std::string fname);
Agents load_movingai_scenario(std::string fname, int upto);

}
#endif
