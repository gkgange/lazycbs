#ifndef GEAS_MAPF__SOLVER_H
#define GEAS_MAPF__SOLVER_H
// ECBS includes
//#include "map_loader.h"
//#include "agents_loader.h"

// geas includes
#include <geas/solver/solver.h>
#include <geas/utils/bitset.h>

#include <lazycbs/pf/pf.hh>
#include <lazycbs/pf/sipp.hh>

#include <lazycbs/mapf/instance.h>
#include <lazycbs/mapf/coordinator.h>
#include <lazycbs/mapf/agent-pf.h>

namespace mapf {

class MAPF_Solver {
 public:
  struct SolveAborted { };

  struct penalty {
    geas::pid_t p;
    geas::pval_t lb;
  };
  
  struct target_key {
    target_key(int _target, int _visitor)
      : target(_target), visitor(_visitor) { }
    int target;
    int visitor;
  };
  struct target_key_hasher {
    size_t operator()(const target_key& k) const {
      size_t h(5331);
      h = ((h<<5) + k.target)^h;
      h = ((h<<5) + k.visitor)^h;
      return h;
    }
  };
  struct target_key_eq {
    bool operator()(const target_key& x, const target_key& y) const {
      return x.target == y.target && x.visitor == y.visitor;
    }
  };
 
  struct cons_key {
    int timestamp;
    pf::Move move;
    int loc;
  };
  struct cons_key_hasher {
    size_t operator()(const cons_key& k) const {
      size_t h(5331);
      h = ((h<<5) + k.timestamp)^h;
      h = ((h<<5) + k.move)^h;
      h = ((h<<5) + k.loc)^h;
      return h;
    }
  };
  struct cons_key_eq {
    bool operator()(const cons_key& x, const cons_key& y) const {
      return x.timestamp == y.timestamp && x.move == y.move && x.loc == y.loc;
    }
  };
    
  struct barrier_key {
    int agent;
    pf::Move dir; 
    int location;
    int time_at_edge;
  };
  struct barrier_data {
    geas::patom_t act; 
    int start;
    int duration;
  };

  struct barrier_key_hasher {
    size_t operator()(const barrier_key& k) const {
      size_t h(5331);
      h = ((h<<5) + k.agent)^h;
      h = ((h<<5) + (size_t) k.dir)^h;
      h = ((h<<5) + k.location)^h;
      h = ((h<<5) + k.time_at_edge)^h;
      return h;
    }
  };
  struct barrier_key_eq {
    bool operator()(const barrier_key& x, const barrier_key& y) const {
      return x.agent == y.agent && x.dir == y.dir && x.location == y.location && x.time_at_edge == y.time_at_edge;
    }
  };

  enum ConflictType { C_MUTEX, C_BARRIER };
  /*
  struct barrier_info {
    int s_loc; // Start corner
    int e_loc; // Exit corner
  };
  */
  struct barrier_info {
    int st_row;
    int st_col;
    pf::Move dir_h;
    pf::Move dir_v;
    int len_h;
    int len_v;
  };

  struct conflict {
    conflict(void) { }

  conflict(int _timestamp, int _a1, int _a2, pf::Move _move, int _loc)
      : timestamp(_timestamp), type(C_MUTEX)
      , a1(_a1), a2(_a2), p{_move, _loc} { }

  conflict(int _timestamp, int _a1, int _a2, int _strow, int _stcol,
           pf::Move _dx, pf::Move _dy, int _lx, int _ly)
      : timestamp(_timestamp), type(C_BARRIER)
      , a1(_a1), a2(_a2), b({_strow, _stcol, _dx, _dy, _lx, _ly}) { }
    /*
    conflict(int _timestamp, int _a1, int _a2, int h_loc, int v_loc, int r_loc)
      : timestamp(_timestamp), type(C_BARRIER)
      , a1(_a1), a2(_a2), b({h_loc, v_loc, r_loc}) { }
      */

    static conflict barrier(int t, int a1, int a2, std::pair<int, int> loc, pf::Move horiz, pf::Move vert, int dur_h, int dur_v) {
      conflict c(t, a1, a2, loc.first, loc.second, horiz, vert, dur_h, dur_v);
      return c;
    }

    int timestamp;
    ConflictType type;

    int a1;
    int a2;
     
    union {
      struct {
        pf::Move move;
        int loc;
      } p;
      barrier_info b;
    };
  };

  struct cons_data {
    intvar sel; // Selector variable
    btset::bitset attached; // Which agents are already attached?
  };

  MAPF_Solver(const Map& ml, const Agents& al, int cost_ub);

  // Problem information
  /*
  const MapLoader* ml;
  const AgentsLoader* al;
  const EgraphReader* egr;
  const int map_size;
  */

  // Solver engine
  geas::solver s;

  vec< std::pair<int, int> > nav_coords;
  // Indexing so we can do barriers easily.
  vec< vec<int> > row_locs;
  vec< vec<int> > col_locs;

  navigation nav;

  coordinator coord;

  // Single-agent search engines
  vec<Agent_PF*> pathfinders;

  // For conflict checking
  // vec<bool> reservation_table;
  // Now handled by the coordinator.
  vec<int> cmap; // Map at the current time
  vec<int> nmap; // Map at the next time

  // Constraints?
  vec<cons_data> constraints;
  vec< vec<barrier_data> > barriers;
  std::unordered_map<cons_key, int, cons_key_hasher, cons_key_eq> cons_map;
  std::unordered_map<barrier_key, int, barrier_key_hasher, barrier_key_eq> barrier_map;
  std::unordered_map<target_key, int, target_key_hasher, target_key_eq> target_map;
  // conflict new_conflict;
  vec<conflict> new_conflicts;
  p_sparseset agent_set;

  // For unsat-core reasoning
  vec<penalty> penalties;
  std::unordered_map<geas::pid_t, int> penalty_table;
  int cost_lb;
  int cost_ub;

  // How many high-level conflicts have been processed?
  int HL_conflicts;

  inline int row_of(int loc) const { return nav_coords[loc].first; }
  inline int col_of(int loc) const { return nav_coords[loc].second; }

  int maxPathLength(void) const;

  // Search for some feasible plan, given a set of assumptions.
  bool buildPlan(vec<geas::patom_t>& assumps);
  bool minimizeCost();
//   bool minimizeMakespan();
  void printPaths(FILE* f = stdout) const;
  void printStats(FILE* f = stdout) const;
   
  bool runUCIter(void);
  bool checkForConflicts(void);
  // In buildPlan, we also try to apply bypasses.
  bool resolveConflicts(void);
  bool addConflict(void);
  bool processCore(vec<geas::patom_t>& core);
      
  geas::patom_t getBarrier(int ai, int t0, std::pair<int, int> coord, pf::Move dir, int dur);
 
  bool checkBarrierViolated(int ai, int t, int p, int delta, int dur) const;

  std::pair<int, pf::Step*> monotoneSubchainStart(pf::Move dx, pf::Move dy, int ai, int loc, int t) const;
  std::pair<int, pf::Step*> monotoneSubchainEnd(pf::Move dx, pf::Move dy, int ai, int loc, int t) const;
  int monotoneSubchainEnd(pf::Move dy, pf::Move dx, int ai, int t) const;

  ~MAPF_Solver();
};

bool MAPF_MinCost(MAPF_Solver& s);
bool MAPF_MinCost_REC(MAPF_Solver& s);
bool MAPF_MinMakespan(MAPF_Solver& s);
bool MAPF_MaxDeadlines(MAPF_Solver& s);

}

#endif
