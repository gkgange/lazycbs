#include <iostream>
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
int main(int argc, char** argv) {
  mapf::navigation nav(mapf::navigation::of_obstacle_array(5, 5, map));

  int* heur = nav.fwd_heuristic(3); 

  mapf::sipp_ctx sctx(nav);

  int sz = nav.size();
  /*
  mapf::constraints cons(sz);
  mapf::constraints res(sz);
  */

  cout << "[" << heur[0];
  for(int ii = 1 ; ii < sz; ++ii)
    cout << ", " << heur[ii];
  cout << "]" << std::endl;

  mapf::sipp_pathfinder spf(nav);
  int dist = spf.search(1, 3, sctx.ptr, heur);
  cout << "1 -> 3 : " << dist << std::endl;

  sctx.ptr[18].Tend = 5;
  dist = spf.search(1, 3, sctx.ptr, heur);
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
