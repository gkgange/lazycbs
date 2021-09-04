#include <lazycbs/mapf/mapf-solver.h>
#include <lazycbs/mapf/instance.h>

#include <string>
#include <cstring>
#include <fstream>
#include <ctime>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <boost/program_options.hpp>


/* This flag controls early solver termination */
volatile static sig_atomic_t terminated = 0;

/* The signal handler just sets the flag. */
void catch_int (int sig) {
  terminated = 1;
  // signal (sig, catch_int);
}
void set_handlers(void) {
  signal(SIGINT, catch_int);
}
void clear_handlers(void) {
  signal(SIGINT, SIG_DFL);
}


using namespace boost::program_options;
using namespace std;


int main(int argc, char** argv) {

  // Reading arguments ----------------------------------------------------------------
  string map_fname, agents_fname;
  /* double w_hwy = 1.0 , w_focal = 1.0 */;
  int time_limit;  // random restarts iterations number
  int agents_upto;
  bool opt_makespan;
  try {
    options_description desc("Options");
    desc.add_options() 
      ("help", "Print help messages")
      ("map", value<string>(&map_fname)->required(), "Map filename")
      ("agents", value<string>(&agents_fname)->required(), "Agents filename")
      ("upto", value<int>(&agents_upto)->default_value(INT_MAX), "Number of agents to read (expects movingai format).")
      ("makespan", value<bool>(&opt_makespan)->default_value(false), "Optimize makespan, rather than cost (Default false)")
      ("time_limit",value<int>(&time_limit)->default_value(300), "Time limit cutoff [seconds]")
      ;

    variables_map vm;
    store(parse_command_line(argc, argv, desc), vm);
    if (vm.count("help")) { 
      cout << endl << desc << endl;
      return 0;
    }
    notify(vm);
  } catch(boost::program_options::required_option& e) {
    cout << endl << e.what() << endl;
    cout << "Use --help for arguments info." << endl << endl;
    return 0;
  }
  catch (exception& e) {
    cerr << endl << e.what() << endl;
    cout << "Use --help for arguments info." << endl << endl;
    return -1;
  }
  // ---------------------------------------------------------------------------------- 

//cout << std::boolalpha<< rand_succ_gen << endl;

  // In case we get timed out during initialisation.

  std::clock_t start(std::clock());

  set_handlers();
  // read the map file and construct its two-dim array

  mapf::Map ml(mapf::load_movingai_map(map_fname));
  
  // read agents' start and goal locations
  mapf::Agents al(agents_upto < INT_MAX
                ? mapf::load_movingai_scenario(agents_fname, agents_upto)
                : mapf::load_ecbs_scenario(agents_fname));

  mapf::MAPF_Solver mapf(ml, al, 1e8);

  clear_handlers();
  bool okay = false;
  try {
    if(terminated)
      throw mapf::MAPF_Solver::SolveAborted {};

    // bool res = mapf.minimizeCost();
    okay = opt_makespan ? MAPF_MinMakespan(mapf) : MAPF_MinCost_REC(mapf) ;
    // bool res = MAPF_MinMakespan(mapf);
  } catch (mapf::MAPF_Solver::SolveAborted& s) {
    // fprintf(stderr, "%% Solve aborted.\n");
  }
  if(okay)
    mapf.printPaths(stdout);

  fprintf(stderr, "lazy-cbs ; %s ; %s ; %d ; %s ; %.02lf ; ", map_fname.c_str(), agents_fname.c_str(), al.size(),
    okay ? "done" : "timeout", 1000.0 * (std::clock() - start) / CLOCKS_PER_SEC);
  mapf.printStats(stderr);
  fprintf(stderr, "\n");
}
