#include <geas/mtl/Vec.h>
#include <iostream>
#include <algorithm>
#include <lazycbs/pf/pf.hh>
#include <lazycbs/pf/pf.hpp>
#include <lazycbs/pf/sipp.hh>

using std::cout;

const char map[] =
             {0, 0, 1, 0, 0,
              0, 0, 1, 0, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 1, 0,
              0, 0, 0, 0, 0} ;

int find_coord(vec< std::pair<int, int> >& map, int r, int c) {
  auto ptr = std::lower_bound(map.begin(), map.end(), std::make_pair(r, c));
  assert(*ptr == std::make_pair(r, c));
  return ptr - map.begin();
}

const char* move_str[] =
  { "right",
    "left",
    "down",
    "up",
    "wait",
    "stop" };
  
void dump_path(const vec< std::pair<int, mapf::pf::Move> >& path) {
  auto it = path.begin();
  auto en = path.end();
  cout << "Path: ";
  if(it != en) {
    cout << it->first << ":" << move_str[it->second];
    for(++it; it != en; ++it) {
      cout << ", " << it->first << ":" << move_str[it->second];
    }
  }
  cout << std::endl;
}

int main(int argc, char** argv) {
  vec< std::pair<int, int> > loc_coord;
  mapf::navigation nav(mapf::navigation::of_obstacle_array(5, 5, map, loc_coord));

  cout << "(4, 3) => " << find_coord(loc_coord, 4, 3) << std::endl;
  
  int* heur = nav.fwd_heuristic(3); 
  int* rheur = nav.rev_heuristic(1);

  mapf::sipp_ctx sctx(nav.size());

  int sz = nav.size();
  /*
  mapf::constraints cons(sz);
  */
  mapf::constraints res(sz);

  cout << "[" << heur[0];
  for(int ii = 1 ; ii < sz; ++ii)
    cout << ", " << heur[ii];
  cout << "]" << std::endl;

  mapf::sipp_pathfinder spf(nav);
  int dist = spf.search(1, 3, sctx, heur, res);
  cout << "1 -> 3 : " << dist << std::endl;

  mapf::sipp_explainer ex(nav);
  
  sctx.lock(18, 5);
  dist = spf.search(1, 3, sctx, heur, res);
  cout << "1 -> 3 : " << dist << std::endl;

  vec<mapf::sipp_explainer::cst> expl;
  ex.explain(1, 3, 100, sctx, heur, rheur, expl);

  cout << "explanation: [";
  auto it = expl.begin();
  auto en = expl.end();
  if(it != en) {
    cout << "(" << (*it).loc << ":" << move_str[(*it).move] << ":" << (*it).time << ")";
    for(++it; it != en; ++it) {
      cout << ", (" << (*it).loc << ":" << move_str[(*it).move] << ":" << (*it).time << ")";
    }
  }
  cout << "]" << std::endl;

  sctx.unlock(18);
  sctx.forbid(mapf::pf::M_WAIT, 18, 5);
  dist = spf.search(1, 3, sctx, heur, res);
  cout << "1 -> 3 : " << dist << std::endl;
  dump_path(spf.path);

  res.forbid(mapf::pf::M_WAIT, find_coord(loc_coord, 2, 1), 2);
  res.forbid(mapf::pf::M_WAIT, find_coord(loc_coord, 2, 2), 3);
  dist = spf.search(1, 3, sctx, heur, res);
  cout << "1 -> 3 : " << dist << std::endl;
  dump_path(spf.path);

  sctx.forbid(mapf::pf::M_WAIT, 19, 7);
  dist = spf.search(1, 3, sctx, heur, res);
  cout << "1 -> 3 : " << dist << std::endl;

  sctx.release(mapf::pf::M_WAIT, 18, 5);
  dist = spf.search(1, 3, sctx, heur, res);
  cout << "1 -> 3 : " << dist << std::endl;
  /*
  mapf::simple_pathfinder pf(nav);
  int dist = pf.search(1, 3, heur, cons, res);

  cout << "1 -> 3 : " << dist << std::endl;

  cons.forbid(mapf::pf::M_WAIT, 18, 5);
  dist = pf.search(1, 3, heur, cons, res);
  cout << "1 -> 3 : " << dist << std::endl;

  cons.release(mapf::pf::M_WAIT, 18, 5);
  dist = pf.search(1, 3, heur, cons, res);
  cout << "1 -> 3 : " << dist << std::endl;
  */

  delete[] heur;

  return 0;
}
