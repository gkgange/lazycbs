#include <geas/constraints/builtins.h>

#include <lazycbs/mapf/mapf-solver.h>
#include <lazycbs/pf/pf.hpp>

#include "compute_heuristic.h"

#define DEBUG_UC
// #define MAPF_NO_RECTANGLES
//#define MAPF_USE_TARGETS

namespace mapf {

/*
static std::pair<int, bool*> mapf_get_res_table(MAPF_Solver* m, int excl) {
  return m->retrieve_reservation_table(excl);
}
*/

static int map_size(const MapLoader& ml) { return ml.rows  * ml.cols; }

MAPF_Solver::MAPF_Solver(const MapLoader& ml, const AgentsLoader& al, int UB)
  :
  /* ml(&_ml), al(&_al), egr(&_egr), map_size(ml->rows * ml->cols) */
  /* reservation_table(map_size, false), */ cmap(map_size(ml), -1), nmap(map_size(ml), -1)
  , agent_set(al.num_of_agents)
  , cost_ub(UB)
  , HL_conflicts(0)
  , nav(navigation::of_obstacle_array(ml.cols, ml.rows, ml.get_map(), nav_coords))
  , row_locs(ml.rows), col_locs(ml.cols)
  , coord(s, nav) {
    int num_of_agents = al.num_of_agents;
    // int map_size = ml->rows*ml->cols;
    cost_lb = 0;

    // Assumes nav_coords are created left-to-right, top-to-bottom.
    for(int ii = 0; ii < nav_coords.size(); ++ii) {
      auto p = nav_coords[ii];
      row_locs[p.first].push(ii);
      col_locs[p.second].push(ii);
    }

    for(int ai = 0; ai < num_of_agents; ++ai) {
      int init_loc = _nav::find_coord(nav_coords, al.initial_locations[ai].first, al.initial_locations[ai].second);
      int goal_loc = _nav::find_coord(nav_coords, al.goal_locations[ai].first, al.goal_locations[ai].second);

      geas::intvar cv(s.new_intvar(0, UB));
      Agent_PF* pf(new Agent_PF(s.data, coord, cv, init_loc, goal_loc));
      pathfinders.push(pf);

      cost_lb += pf->pathCost();
      penalty_table.insert(std::make_pair(cv.p, penalties.size()));
      penalties.push(penalty { cv.p, geas::from_int(pf->pathCost()) });
    }
}

bool apply_penalties(MAPF_Solver& mf) {
   for(MAPF_Solver::penalty& p : mf.penalties)  {
    geas::patom_t at(geas::le_atom(p.p, p.lb));
    if(!mf.s.assume(at))
      return false;
    if(mf.s.is_aborted())
      throw MAPF_Solver::SolveAborted { };
  }
  return true;
}

static int Move_dx[] = { 1, -1, 0, 0, 0 };
static int Move_dy[] = { 0, 0, 1, -1, 0 };

void log_conflict(MAPF_Solver& mapf) {
  for(auto new_conflict : mapf.new_conflicts) {
    int a1(new_conflict.a1);
    int a2(new_conflict.a2);
    int t(new_conflict.timestamp);

    if(new_conflict.type == MAPF_Solver::C_BARRIER) {
      auto b = new_conflict.b;
      fprintf(stderr, "%%%% Adding rectangle: [%d, (%d, %d) |- (%d, %d), %d, %d]\n",
        t, b.st_row, b.st_col,
              b.st_row + Move_dy[b.dir_v] * (b.len_v-1),
              b.st_col + Move_dx[b.dir_h] * (b.len_h-1),
           a1, a2);
    } else {
      assert(new_conflict.a1 != new_conflict.a2);
      const Agent_PF* pf1 = mapf.pathfinders[new_conflict.a1];
      const Agent_PF* pf2 = mapf.pathfinders[new_conflict.a2];
      fprintf(stderr, "%%%% Adding conflict: [%d, (%d, %d), %s, %d, %d | (%d, %d -> %d, %d) | (%d, %d -> %d, %d) ]\n", new_conflict.timestamp, mapf.row_of(new_conflict.p.loc), mapf.col_of(new_conflict.p.loc), pf::move_str(new_conflict.p.move), new_conflict.a1, new_conflict.a2,
              mapf.row_of(pf1->start_pos), mapf.col_of(pf1->start_pos),
              mapf.row_of(pf1->goal_pos), mapf.col_of(pf1->goal_pos),
              mapf.row_of(pf2->start_pos), mapf.col_of(pf2->start_pos),
              mapf.row_of(pf2->goal_pos), mapf.col_of(pf2->goal_pos));
    }
  }
}

bool MAPF_Solver::buildPlan(vec<geas::patom_t>& assumps) {
  s.clear_assumptions();  
  
  // Apply the assumptions.
  for(geas::patom_t at : assumps) {
    if(!s.assume(at))
      return false;
    if(s.is_aborted())
      throw MAPF_Solver::SolveAborted { };
  }

retry:
  switch(s.solve()) {
    // We've proven the current subproblem is infeasible.
    case geas::solver::UNSAT:
      return false;
    // Candidate optimal solution. Check for conflicts.
    case geas::solver::SAT:
      // First, finesse the plan to avoid any remaining conflicts.
      if(!resolveConflicts()) {
        s.restart();
#ifdef DEBUG_UC
        log_conflict(*this);
#endif
        if(!addConflict())
          return false;
        goto retry;
      }
      // If we succeeded, done.
      return true;
    case geas::solver::UNKNOWN:
      throw SolveAborted { };
  }
  // Should be unreachable
  GEAS_ERROR;
}


bool MAPF_Solver::minimizeCost(void) {
  s.clear_assumptions();

  // Set up the penalties, and lower bound.
  cost_lb = 0;
  for(Agent_PF* p : pathfinders) {
    cost_lb += p->pathCost();
  }
#ifdef DEBUG_UC
  fprintf(stderr, "%%%% Initial bound: %d\n", cost_lb);
#endif
    
  vec<geas::patom_t> core;
  if(!apply_penalties(*this))
    return false;
  while(checkForConflicts()) {
#ifdef DEBUG_UC
    log_conflict(*this);
#endif
    // s.restart();
    if(!addConflict())
      return false;
    
    // s.clear_assumptions();
    while(!runUCIter()) {
      s.get_conflict(core);
      s.clear_assumptions();
      s.restart();
      /*
      if(!processCore(core))
        return false;
        */
      cost_lb += processCore(core);
      apply_penalties(*this);
#ifdef DEBUG_UC
      fprintf(stderr, "%%%% Found core of size (%d), current lower bound %d\n", core.size(), cost_lb);
#endif
    }
  }
  return true;
}

void MAPF_Solver::printStats(FILE* f) const {
  int LL_num_generated = 0;
  int LL_num_expanded = coord.sipp_pf.LL_expanded;
  int LL_executions = coord.sipp_pf.LL_searches;
  for(Agent_PF* p : pathfinders) {
    /*
    LL_num_generated += p->num_generated;
    LL_num_expanded += p->num_expanded;
    LL_executions += p->num_executions;
    */
  }
  fprintf(f, "%d ; %d ; %d ; %d ; %d ; %d", cost_lb, s.data->stats.conflicts, LL_num_expanded, LL_num_generated, HL_conflicts, LL_executions);
  // std::cout << cost_lb << " ; " << s.data->stats.conflicts << " ; " << LL_num_expanded << " ; " << LL_num_generated << " ; " << HL_conflicts << " ; " << LL_executions;
}

void MAPF_Solver::printPaths(FILE* f) const {
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    assert(ai == pathfinders[ai]->agent_id);
    int loc = coord.plans[ai].origin;
    fprintf(f, "Agent %d: (%d, %d)", ai, row_of(loc)-1, col_of(loc)-1);
    for(pf::Step step : coord.get_path(ai)) {
      // fprintf(f, " %d", loc);
      loc = coord.nav.delta[loc][step.second];
      fprintf(f, " %d:(%d,%d)", step.first, row_of(loc)-1, col_of(loc)-1);
    }
    fprintf(f, "\n");
  }
}
// Returns maximum length
/*
int MAPF_Solver::extractPaths(void) {
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    paths[ai].clear();
    pathfinders[ai]->extractPath(paths[ai]);
  }
  return 0;
}
*/

int MAPF_Solver::maxPathLength(void) const {
  int len = 0;
  for(Agent_PF* p : pathfinders)
    len = std::max(len, static_cast<int>(std::ceil(p->pathCost())));
  return len;
}

// Exactly as for the normal MAPF solver, check the current
// 'incumbent' for conflicts.
// FIXME: This is wasteful; use a sparse representation.
/*
inline int agentPosition(Agent_PF* p, int t) {
  const vec<int>& P(p->getPath());
  return (t < P.size()) ? P[t] : P.last();
}

inline void clear_map(MAPF_Solver* s, vec<int>& map, int t) {
  for(Agent_PF* p : s->pathfinders) {
    map[agentPosition(p, t)] = -1;
  }
}
*/

/*
void MAPF_Solver::printMonotoneSubchain(int dy, int dx, int ai, int t) {
  const vec<int>& P(pathfinders[ai]->getPath());

  char dx(0);
  char dy(0);
  
  int p(P[t]);
  int r(p / ml->cols);
  int c(p % ml->cols);

  int rp(r);
  int cp(c);
  // Run back in time.
  int s = t-1;
  for(; s >= 0; --s) {
    int rs(P[s] / ml->cols);
    int cs(P[s] % ml->cols);
    
    if(abs(dy - (rs - rp)) > 1)
      break;
    if(abs(dx - (cs - cp)) > 1)
      break; 
    if(rs != rp)
      dy = (rs - rp);
    if(cs != cp)
      dy = (rs - rp);
    rp = rs;
    cp = cs;
  }
  ++s;

  dx = 0;
  dy = 0;
  rp = r;
  cp = c;
  for(++t; t < P.size(); ++t) {
    int rs(P[t] / ml->cols);
    int cs(P[t] % ml->cols);

    if(abs(dy - (rp - rs)) > 1)
      break;
    if(abs(dx - (cp - cs)) > 1)
      break; 

    if(rs != rp)
      dy = (rp - rs);
    if(cs != cp)
      dy = (rp - rs);
    rp = rs;
    cp = cs;
  }

  fprintf(stderr, "%% {{%d [%d] :", ai, s); 
  for(int p = s; p < t; ++p) {
    fprintf(stderr, " (%d, %d)", P[p] / ml->cols, P[p] % ml->cols);
  }
  fprintf(stderr, "\n");
}
*/
// Returns <loc, it> where loc is the location *before* step it.
std::pair<int, pf::Step*> MAPF_Solver::monotoneSubchainStart(pf::Move dx, pf::Move dy, int ai, int loc, int t) const {
  const pf::Path& path(coord.get_path(ai));

  unsigned dir_mask = (1 << dx) | (1 << dy);

  auto b = path.begin();
  auto it = pf::find_step(path, t);
  assert(it->first == t);

  loc = coord.nav.inv[loc][it->second];
  while(b != it) {
    auto pred = it-1;
    if(pred->first != t-1 ||
       !(dir_mask & (1 << pred->second)))
      break;
    loc = coord.nav.inv[loc][pred->second];
    it = pred;
    --t;
  }
  return std::make_pair(loc, it);
}

// Returns <loc, it>, where loc is the location *after* step it.
std::pair<int, pf::Step*> MAPF_Solver::monotoneSubchainEnd(pf::Move dy, pf::Move dx, int ai, int loc, int t) const {
  const pf::Path& path(coord.get_path(ai));

  unsigned dir_mask = (1 << dx) | (1 << dy);

  auto it = pf::find_step(path, t);
  auto en = path.end();
  assert(it->first == t);

  auto succ = it+1;
  while(succ != en) {
    if(succ->first != t+1 ||
       !(dir_mask & (1 << succ->second)))
      return std::make_pair(loc, it);
    loc = coord.nav.delta[loc][succ->second];
    it = succ;
    succ = it+1;
    ++t;
  }
  return std::make_pair(loc, it);
}

bool MAPF_Solver::resolveConflicts(void) {
  // FIXME
  #if 0
  int pMax = maxPathLength();
  
  vec<bool> conflicting(pathfinders.size(), false);

  vec<int> agent_loc;
  vec<pf::Step*> path_iter;
  vec<pf::Step*> path_end;
  int Tnext = INT_MAX;

  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    int loc = pathfinders[ai]->start_pos;
    const pf::Path& path(pathfinders[ai]->getPath());
    agent_loc.push(loc);
    path_iter.push(path.begin());
    path_end.push(path.end());
    Tnext = std::max(Tnext, path.begin()->first);
    
    assert(nmap[loc] < 0); // Shouldn't be any conflicts at t0.
    nmap[loc] = ai;
  }

  // Run through the candidate plan, identify agents with conflicts.
  while(Tnext < INT_MAX) {
    int t = Tnext;
    Tnext = INT_MAX;
    std::swap(cmap, nmap);

    for(int ai = 0; ai < pathfinders.size(); ++ai) {
      int loc = agentPosition(pathfinders[ai], t);
      if(nmap[loc] >= 0) {
        // Edge conflict between agents ai and nmap[loc].
        conflicting[ai] = true;
        conflicting[nmap[loc]] = true;
      } else {
        nmap[loc] = ai;
      }
      if(cmap[loc] > 0 && cmap[loc] != ai) {
        // Get the new location of the agent we're replacing.
        int rloc = agentPosition(pathfinders[cmap[loc]], t);
        if(cmap[rloc] == ai) {
          // Edge conflict between agents ai and cmap[loc].
          conflicting[ai] = true;
          conflicting[cmap[loc]] = true; 
        }
      }
    }
    // Now we zero out the previous cmap.
    clear_map(this, cmap, t-1);
  }
  clear_map(this, nmap, pMax);
  
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    if(conflicting[ai]) {
      // Try re-routing agent ai, in the hopes of getting a better plan.
      pathfinders[ai]->find_bypass();
    }
  }
  #endif
  return !checkForConflicts();
}

template<class T>
void clear_map(const T& agents, const vec<int>& loc, vec<int>& map) {
  for(int ai : agents)
    map[loc[ai]] = -1;
}

void clear_map(const vec<int>& loc, vec<int>& map) {
  for(int ii = 0; ii < loc.size(); ++ii)
    map[loc[ii]] = -1;
}

template<class V>
struct ptr_rev_iter {
  ptr_rev_iter(V* _ptr) : ptr(_ptr) { }

  V* ptr;

  V& operator*(void) { return *ptr; }
  const V& operator*(void) const { return *ptr; }
  ptr_rev_iter& operator++(void) { --ptr; return *this; }
  bool operator!=(const ptr_rev_iter& o) { return ptr != o.ptr; }
};
template<class V>
ptr_rev_iter<V> ptr_rev(V* ptr) { return ptr_rev_iter<V>(ptr); }

template<class It, class F>
vec<Agent_PF::constraint> extract_barrier(const MAPF_Solver& m, int t0, int c0, int len, It b, It e, F loc_coord) {
  int tEnd = t0 + len;

  vec<Agent_PF::constraint> csts;
  
  for(; b != e; ++b) {
    int loc = *b;
    int c = loc_coord(loc);
    int t = t0 + (c - c0);
    if(t >= tEnd)
      break;

    csts.push(Agent_PF::constraint(pf::M_WAIT, loc, t));
  }
  return csts;
}

vec<Agent_PF::constraint> barrier_constraints(const MAPF_Solver& m, int t0, std::pair<int, int> coord, pf::Move dir, int len) {
  const auto& coords(m.nav_coords);

  switch(dir) {
  case pf::M_RIGHT:
    {
      const vec<int>& row(m.row_locs[coord.first]);
    return extract_barrier(m, t0, coord.second, len,
                           std::lower_bound(row.begin(), row.end(), coord.second,
                                            [&coords](int loc, int col) { return coords[loc].second < col; }),
                           row.end(),
                           [&coords](int loc) { return coords[loc].second; });
    }
  case pf::M_LEFT:
    {
      const vec<int>& row(m.row_locs[coord.first]);
      auto it = std::upper_bound(row.begin(), row.end(), coord.second,
                                 [&coords](int col, int loc) { return col < coords[loc].second; });
      return extract_barrier(m, t0, -coord.second, len,
                             ptr_rev(it-1),
                             ptr_rev(row.begin()-1),
                             [&coords](int loc) { return -coords[loc].second; });
    }
  case pf::M_DOWN:
    {
      const vec<int>& col(m.col_locs[coord.second]);
    return extract_barrier(m, t0, coord.first, len,
                           std::lower_bound(col.begin(), col.end(), coord.first,
                                            [&coords](int loc, int row) { return coords[loc].first < row; }),
                           col.end(),
                           [&coords](int loc) { return coords[loc].first; });
  }
  case pf::M_UP:
    {
      const vec<int>& col(m.col_locs[coord.second]);
      auto it = std::upper_bound(col.begin(), col.end(), coord.first,
                                 [&coords](int row, int loc) { return row < coords[loc].first; });
      return extract_barrier(m, t0, -coord.first, len,
                             ptr_rev(it-1),
                             ptr_rev(col.begin()-1),
                             [&coords](int loc) { return -coords[loc].first; });
    }
  default:
    assert(0);
    return vec<Agent_PF::constraint>();
  }
}

void print_agent_path(MAPF_Solver& m, int ai) {
  int loc = m.coord.plans[ai].origin;
  fprintf(stdout, "Agent %d: (%d, %d)", ai, m.row_of(loc), m.col_of(loc));
  
  for(pf::Step step : m.coord.get_path(ai)) {
    // fprintf(f, " %d", loc);
    loc = m.coord.nav.delta[loc][step.second];
    fprintf(stdout, " %d:(%d,%d)", step.first, m.row_of(loc), m.col_of(loc));
  }
  fprintf(stdout, "\n");
}

bool MAPF_Solver::checkForConflicts(void) {
  coord.reset();
  int pMax = maxPathLength();
  
  vec<int> prev_loc;
  vec<int> curr_loc;

  vec<pf::Step*> path_it;
  vec<pf::Step*> path_en;

  int Tnext = INT_MAX;
  agent_set.sz = 0;
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    const pf::Path& path(coord.get_path(ai));
    auto it = path.begin();
    auto en = path.end();
    path_it.push(it);
    path_en.push(en);

    int loc = coord.plans[ai].origin;
    curr_loc.push(loc);
    prev_loc.push(loc);

    if(!coord.is_active(ai))
      continue;
    agent_set.insert(ai);
    if(it != en)
      Tnext = std::min(Tnext, it->first);
    assert(nmap[loc] < 0); // Shouldn't be any conflicts at t0.
    nmap[loc] = ai;
  }

  // All agents are interesting.
  // agent_set.sz = pathfinders.size();

  while(Tnext < INT_MAX) {
    int t = Tnext;
    Tnext = INT_MAX;

    std::swap(prev_loc, curr_loc);
    std::swap(cmap, nmap);

    // for(int ai = 0; ai < pathfinders.size(); ++ai) {
    for(int ai : agent_set.rev()) {
      int loc;
      if(path_it[ai] != path_en[ai] && path_it[ai]->first == t) {
        loc = nav.delta[prev_loc[ai]][path_it[ai]->second];
        ++path_it[ai];
      } else {
        loc = prev_loc[ai];
      }
      if(path_it[ai] != path_en[ai])
        Tnext = std::min(Tnext, path_it[ai]->first);
      curr_loc[ai] = loc;
      
      if(nmap[loc] >= 0) {
        int aj(nmap[loc]);
        assert(aj != ai);

        pf::Move m_ai = pf::NUM_MOVES;
        pf::Move m_aj = pf::NUM_MOVES;

        /*
        printf("%% Vertex conflict: %d x %d at time %d\n", ai, aj, t);
        print_agent_path(*this, ai);
        print_agent_path(*this, aj);
        */
        if(path_it[ai] == path_en[ai] && t > pathfinders[ai]->cost.lb(s.data->ctx())) {
          fprintf(stderr, "%% Agent %d crosses target %d [%d -> %d]\n", aj, ai, pathfinders[ai]->cost.lb(s.data->ctx()), t);
        } else if(path_it[aj] == path_en[aj] && t > pathfinders[aj]->cost.lb(s.data->ctx())) {
          fprintf(stderr, "%% Agent %d crosses target %d [%d -> %d]\n", ai, aj, pathfinders[aj]->cost.lb(s.data->ctx()), t);
        }
#ifdef MAPF_USE_TARGETS
        if((path_it[ai] == path_en[ai] && t > pathfinders[ai]->cost.lb(s.data->ctx()))  ||
           (path_it[aj] == path_en[aj] && t > pathfinders[aj]->cost.lb(s.data->ctx()))) {
          if(path_it[ai] != path_en[ai])
            std::swap(ai, aj);
          assert(path_it[ai] == path_en[ai] && t > pathfinders[ai]->cost.lb(s.data->ctx()));
          //pathfinders[aj]->debug_print_path(coord.get_path(aj));
          new_conflicts.push(conflict::barrier(t, ai, aj,
                                              std::make_pair(0, 0),
                                               pf::M_WAIT, pf::M_WAIT,
                                              0, 0));
          goto conflict_handled;
        }
#endif
        // Pretty ugly case checks.
        if(path_it[ai] == coord.get_path(ai).begin() || path_it[aj] == coord.get_path(aj).begin())
          goto conflict_is_not_rectangle;
        if( (path_it[ai]-1)->first != t || (path_it[aj]-1)->first != t )
          goto conflict_is_not_rectangle;

        m_ai = (path_it[ai]-1)->second;
        m_aj = (path_it[aj]-1)->second;
        if( (pf::move_axis(m_ai)|pf::move_axis(m_aj)) != pf::Ax_BOTH )
          goto conflict_is_not_rectangle;

        if(pf::move_axis(m_ai) != pf::Ax_HORIZ) {
          std::swap(ai, aj);
          std::swap(m_ai, m_aj);
        }
        assert(pf::move_axis(m_ai) == pf::Ax_HORIZ);

        {
        // Now we need to find the rectangle boundaries.
        std::pair<int, pf::Step*> sH(monotoneSubchainStart(m_ai, m_aj, ai, loc, t));
        std::pair<int, pf::Step*> sV(monotoneSubchainStart(m_ai, m_aj, aj, loc, t));

        // If there is overhang, adjust the locations.
        while(true) {
          if(Move_dx[m_ai] * (col_of(sV.first) - col_of(sH.first)) < 0) {
            assert(sV.second->first < t);
            sV.first = coord.nav.delta[sV.first][sV.second->second];
            sV.second++;
            continue;
          }
          if(Move_dy[m_aj] * (row_of(sH.first) - row_of(sV.first)) < 0) {
            assert(sH.second->first < t);
            sH.first = coord.nav.delta[sH.first][sH.second->second];
            sH.second++;
            continue;
          }
          break;
        }

        // Now do the same for the end positions.
        std::pair<int, pf::Step*> eH(monotoneSubchainEnd(m_ai, m_aj, ai, loc, t));
        std::pair<int, pf::Step*> eV(monotoneSubchainEnd(m_ai, m_aj, aj, loc, t));
        while(true) {
          if(Move_dx[m_ai] * (col_of(eH.first) - col_of(eV.first)) < 0) {
            assert(eV.second->first >= t);
            eV.first = coord.nav.inv[eV.first][eV.second->second];
            eV.second--;
            continue;
          }
          if(Move_dy[m_aj] * (row_of(eV.first) - row_of(eH.first)) < 0) {
            assert(eH.second->first >= t);
            eH.first = coord.nav.inv[eH.first][eH.second->second];
            eH.second--;
            continue;
          }
          break;
        }
        // Now we've found the biggest feasible rectangles.
        int dur_H = 1 + abs(col_of(eV.first) - col_of(sV.first));
        int dur_V = 1 + abs(row_of(eH.first) - row_of(sH.first));
        if(dur_H > 1 && dur_V > 1) {
          int t0 = t - abs(row_of(loc) - row_of(sH.first)) - abs(col_of(loc) - col_of(sV.first));
          new_conflicts.push(conflict::barrier(t0, ai, aj,
                                              std::make_pair(row_of(sH.first), col_of(sV.first)),
                                              m_ai, m_aj,
                                              dur_H, dur_V));
          goto conflict_handled;
        }
        }
        // Fall through
        /*
        // Already occupied.
        int dy1 = row_of(loc) - row_of(prev_loc[ai]);
        int dx1 = col_of(loc) - col_of(prev_loc[ai]);
        int dy2 = row_of(loc) - row_of(prev_loc[aj]);
        int dx2 = col_of(loc) - col_of(prev_loc[aj]);
#ifdef MAPF_NO_RECTANGLES
        goto fallback;
#endif
        if(dx1 != dx2 && dy1 != dy2) {
          // This is a rectangle conflict
          int dy(dy1 + dy2);
          int dx(dx1 + dx2);
          
          // Make sure ai is the horizontal agent.
          if(dx2)
            std::swap(ai, aj);

          // Find the start positions
          // FIXME==============
          int stH(monotoneSubchainStart(dy, dx, ai, t));
          int stV(monotoneSubchainStart(dy, dx, aj, t));
          
          int sH(agentPosition(pathfinders[ai], stH));
          int sV(agentPosition(pathfinders[aj], stV));

          // If there is overhang, adjust the locations.
          while(true) {
            if(dx * (col_of(sV) - col_of(sH)) < 0) {
              ++stV;     
              sV = agentPosition(pathfinders[aj], stV);
              continue;
            }
            if(dy * (row_of(sH) - row_of(sV)) < 0) {
              ++stH;
              sH = agentPosition(pathfinders[ai], stH); 
              continue;
            }
            break;
          }

          int etH(monotoneSubchainEnd(dy, dx, ai, t));
          int etV(monotoneSubchainEnd(dy, dx, aj, t));

          int eH(agentPosition(pathfinders[ai], etH));
          int eV(agentPosition(pathfinders[aj], etV));

          while(true) {
            if(dx * (col_of(eH) - col_of(eV)) < 0) {
              --etV;
              eV = agentPosition(pathfinders[aj], etV);
              continue;
            }
            if(dy * (row_of(eV) - row_of(eH)) < 0) {
              --etH;
              eH = agentPosition(pathfinders[ai], etH);
              continue;
            }
            break;
          }
          assert(stH <= t);
          assert(stV <= t);
          assert(t <= etH);
          assert(t <= etV);
          assert(dy * (row_of(sH) - row_of(sV)) >= 0);
          assert(dx * (col_of(sV) - col_of(sH)) >= 0);
          assert(dy * (row_of(eV) - row_of(eH)) >= 0);
          assert(dx * (col_of(eH) - col_of(eV)) >= 0);
           
          int locS(ml->linearize_coordinate(row_of(sV), col_of(sH)));
          int locE(ml->linearize_coordinate(row_of(eV), col_of(eH)));
          int t0(stH - abs(row_of(sH) - row_of(locS)));
          assert(t0 == stV - abs(col_of(sV) - col_of(locS)));
          new_conflicts.push(conflict::barrier(t0, ai, aj, locS, locE));
        } else {
#ifdef MAPF_NO_RECTANGLES
        fallback:
#endif
          new_conflicts.push(conflict(t, ai, nmap[loc], loc, -1));
        }
        */
conflict_is_not_rectangle:
        // FIXME: Re-introduce rectangles/barriers.
        new_conflicts.push(conflict(t, ai, aj, pf::M_WAIT, loc));

conflict_handled:
        /* /
        clear_map(agent_set, prev_loc, cmap);
        clear_map(agent_set, curr_loc, nmap);
        */
        /* */
        cmap[prev_loc[ai]] = -1;
        nmap[loc] = aj;
        /* */
        agent_set.remove(ai);
        // return true;
        continue;
      }
      nmap[loc] = ai;
    }

    for(int ai : agent_set.rev()) {
      // Now check the edge constraints.
      int loc = curr_loc[ai];
      if(cmap[loc] > 0 && cmap[loc] != ai) {
        // Get the new location of the agent we're replacing.
        int rloc = curr_loc[cmap[loc]];
        if(cmap[rloc] == ai) {
          // Edge conflict
          // new_conflicts.push(conflict(t-1, ai, cmap[loc], loc, rloc));
          new_conflicts.push(conflict(t, ai, cmap[loc], coord.nav.move_dir(rloc, loc), loc));
          /* /
          clear_map(agent_set, prev_loc, cmap);
          clear_map(agent_set, curr_loc, nmap);
          / */
          cmap[prev_loc[ai]] = -1;
          nmap[curr_loc[ai]] = -1;
          /* */
          agent_set.remove(ai);
          // return true;
          continue;
        }
      }
    }
    // Now we zero out the previous cmap.
    clear_map(agent_set, prev_loc, cmap);
  }
  clear_map(agent_set, curr_loc, nmap);

  return new_conflicts.size() > 0;
}

bool MAPF_Solver::checkBarrierViolated(int ai, int t, int p, int delta, int dur) const {
  assert(t >= 0);
  /*
  const vec<int>& P(pathfinders[ai]->getPath());
  for(int dt = 0; dt < dur; ++dt, ++t) {
    if(P[t] == p)
      return true;
    p += delta;
  }
  */
  assert(0);
  return false;
}

//enum BarrierDir { UP = 0, LEFT = 1, DOWN = 2, RIGHT = 3 };

geas::patom_t MAPF_Solver::getBarrier(int ai, int t, std::pair<int, int> coord, pf::Move dir, int dur) {
  // TODO: If this barrier is trivially violated, just return F.
  int origin = pathfinders[ai]->start_pos;
  if(t == 0 && coord.first == row_of(origin) && coord.second == col_of(origin))
    return geas::at_False;

  // TODO: Reintroduce approximate
  int t0 = t - Move_dy[dir] * coord.first - Move_dx[dir] * coord.second;
  int sel_coord = pf::move_axis(dir) == pf::Ax_HORIZ ? coord.first : coord.second;
  barrier_key k { ai, dir, sel_coord, t0 };
  auto it(barrier_map.find(k));
  int idx;
  if(it != barrier_map.end()) {
    idx = (*it).second;
  } else {
    idx = barriers.size();
    barriers.push();
    barrier_map.insert(std::make_pair(k, idx));
  }

  // Check if this barrier has already been generated.
  vec<barrier_data>& candidates(barriers[idx]);
  for(const barrier_data& b : candidates) {
    if(b.start == t && b.duration == dur) {
      // fprintf(stderr, "%%%% HIT!\n");
      return b.act;
    }
  }

  // If not, we'll create it. 
  geas::patom_t act(s.new_boolvar());
  // Set up entailment relationships.
          /*
#ifdef BARRIER_ENTAIL
  int end = t + dur;
  for(const barrier_data& b : candidates) {
    if(b.start <= t && end <= b.start + b.duration) {
      // b subsumes this.
      // fprintf(stderr, "%% Found super-barrier\n");
      add_clause(s.data, ~b.act, act);
    }
    if(t <= b.start && b.start + b.duration <= end) {
      // fprintf(stderr, "%% Found sub-barrier\n");
      add_clause(s.data, act, ~b.act);
    }
  }
#endif
          */
  vec<Agent_PF::constraint> csts = barrier_constraints(*this, t, coord, dir, dur);
  pathfinders[ai]->register_barrier(act, csts); 
  assert(csts.size() > 0);
  candidates.push(barrier_data { act, t, dur });
  return act;

  /*
  int delta = ml.cols * barrier_dy[dir] + barrier_dx[dir];
  assert(t >= 0);
  if(t == 0 && p == pathfinders[ai]->engine.start_location)
    return geas::at_False;

  // Project the direction back in the appropriate direction, to see what set of barriers we're in.
  int p_ident;
  int t0;
  switch(dir) {
    case UP:
      p_ident = col_of(p);  
      t0 = t - row_of(p);
      break;
    case LEFT:
      p_ident = row_of(p);
      t0 = t - col_of(p);
      break;
    case DOWN:
      p_ident = col_of(p);
      t0 = t - row_of(p);
      break;
    case RIGHT:
    default:
      p_ident = row_of(p);
      t0 = t - col_of(p);
      break;
  }
  barrier_key k { ai, dir, p_ident, t0 };
  auto it(barrier_map.find(k));
  int idx;
  if(it != barrier_map.end()) {
    idx = (*it).second;
  } else {
    idx = barriers.size();
    barriers.push();
    barrier_map.insert(std::make_pair(k, idx));
  }
  
  // Check if this barrier has already been generated.
  vec<barrier_data>& candidates(barriers[idx]);
  for(const barrier_data& b : candidates) {
    if(b.start == t && b.duration == dur) {
      // fprintf(stderr, "%%%% HIT!\n");
      return b.act;
    }
  }
  // If not, we'll create it. 
  geas::patom_t act(s.new_boolvar());
  // Set up entailment relationships.
#ifdef BARRIER_ENTAIL
  int end = t + dur;
  for(const barrier_data& b : candidates) {
    if(b.start <= t && end <= b.start + b.duration) {
      // b subsumes this.
      // fprintf(stderr, "%% Found super-barrier\n");
      add_clause(s.data, ~b.act, act);
    }
    if(t <= b.start && b.start + b.duration <= end) {
      // fprintf(stderr, "%% Found sub-barrier\n");
      add_clause(s.data, act, ~b.act);
    }
  }
#endif
  pathfinders[ai]->register_barrier(act, t, p, delta, dur); 
  candidates.push(barrier_data { act, t, dur });
  return act;
  */
}

bool MAPF_Solver::addConflict(void) {
  HL_conflicts++;
  for(auto new_conflict : new_conflicts) {
    if(new_conflict.type == C_BARRIER) {
      const barrier_info& b = new_conflict.b;
#ifdef MAPF_USE_TARGETS
      if(b.dir_h == pf::M_WAIT) { // Actually a target conflict.
        int ai = new_conflict.a1;
        int aj = new_conflict.a2;
          target_key k(ai, aj);
          auto it = target_map.find(k);
          if(it != target_map.end()) {
            pathfinders[aj]->tighten_target(it->second, new_conflict.timestamp);
          } else {
            int loc = pathfinders[ai]->goal_pos;
            int idx = pathfinders[aj]->register_target(loc, pathfinders[ai]->cost,
                                                       new_conflict.timestamp);
            target_map.insert(std::make_pair(k, idx));
          }
          continue;
      }
#endif

      // Check the four barriers.
      vec<geas::clause_elt> atoms;
      atoms.push(getBarrier(new_conflict.a2, new_conflict.timestamp,
                                                          std::make_pair(b.st_row, b.st_col),
                                                          b.dir_h, b.len_h));
      atoms.push(getBarrier(new_conflict.a2, new_conflict.timestamp + b.len_v - 1,
                            std::make_pair(b.st_row + Move_dy[b.dir_v] * (b.len_v-1), b.st_col),
                                                          b.dir_h, b.len_h));
      atoms.push(getBarrier(new_conflict.a1, new_conflict.timestamp,
                                                          std::make_pair(b.st_row, b.st_col),
                                                          b.dir_v, b.len_v));
      atoms.push(getBarrier(new_conflict.a1, new_conflict.timestamp + b.len_h - 1,
                            std::make_pair(b.st_row, b.st_col + Move_dx[b.dir_h] * (b.len_h-1)),
                                                          b.dir_v, b.len_v));
      add_clause(*s.data, atoms);
#if 0
      int aH(new_conflict.a1);
      int aV(new_conflict.a2);
      int p_s(new_conflict.b.s_loc);
      int p_e(new_conflict.b.e_loc);

      vec<geas::clause_elt> barrier_atoms;

      // Entry barrier starts at top-left
      int s_time(new_conflict.timestamp);
      // fprintf(stderr, "%% Adding rectangle [%d] (%d, %d) -> (%d, %d)\n", s_time, row_of(p_s), col_of(p_s), row_of(p_e), col_of(p_e));
      int dt(std::min(0, s_time));
      int h_dur(1 + abs(row_of(p_e) - row_of(p_s)));
      int h_delta(row_of(p_s) < row_of(p_e) ? ml->cols : -ml->cols);
      
      // assert(checkBarrierViolated(aH, s_time - dt, p_s - dt*h_delta, h_delta, h_dur + dt));
      BarrierDir dH(row_of(p_s) < row_of(p_e) ? DOWN : UP);
      if(s_time > 0 || pathfinders[aH]->engine.start_location != p_s - dt*h_delta) {
        barrier_atoms.push(getBarrier(aH, dH, s_time - dt, p_s - dt*h_delta, h_dur + dt));
      }

      int eh_start(ml->linearize_coordinate(row_of(p_s), col_of(p_e)));
      int eh_time(s_time + abs(col_of(p_e) - col_of(p_s)));
      barrier_atoms.push(getBarrier(aH, dH, eh_time - dt, eh_start - dt*h_delta, h_dur+dt));

      int v_dur(1 + abs(col_of(p_e) - col_of(p_s)));
      int v_delta(col_of(p_s) < col_of(p_e) ? 1 : -1);
      BarrierDir dV(col_of(p_s) < col_of(p_e) ? RIGHT : LEFT);

      if(s_time > 0 || pathfinders[aV]->engine.start_location != p_s - dt*v_delta) {
        barrier_atoms.push(getBarrier(aV, dV, s_time - dt, p_s - dt*v_delta, v_dur+dt));
      }

      int ev_start(ml->linearize_coordinate(row_of(p_e), col_of(p_s)));
      int ev_time(s_time + abs(row_of(p_e) - row_of(p_s)));
      barrier_atoms.push(getBarrier(aV, dV, ev_time - dt, ev_start - dt*v_delta, v_dur+dt));

      // One of the barriers must be active
      add_clause(*s.data, barrier_atoms);

      /*
      if(new_conflict.timestamp == 0) {
        patom_t sel(s.new_boolvar());
        // Set up the horizontal exit barrier
        int h_start(ml->linearize_coordinate(row_of(p_h), col_of(p_e)));
        int h_time(abs(col_of(p_h) - col_of(p_e)));
        int h_dur(1 + abs(row_of(p_h) - row_of(p_e)));
        int h_delta(row_of(p_h) < row_of(p_e) ? ml->cols : -ml->cols);
      
        pathfinders[new_conflict.a1]->register_barrier(sel, h_time, h_start, h_delta, h_dur);

        // And repeat the same for the vertical exit barrier
        int v_start(ml->linearize_coordinate(row_of(p_e), col_of(p_v)));
        int v_time(abs(row_of(p_v) - row_of(p_e)));
        int v_dur(1 + abs(col_of(p_v) - col_of(p_e)));
        int v_delta(col_of(p_h) < col_of(p_e) ? 1 : -1);
        
        pathfinders[new_conflict.a2]->register_barrier(~sel, v_time, v_start, v_delta, v_dur);
      } else {
        // Need the entry and exit barriers
        // FIXME: Build a table of barriers, so we can re-use them between conflicts.
        patom_t s1(s.new_boolvar());
        patom_t e1(s.new_boolvar());
        patom_t s2(s.new_boolvar());
        patom_t e2(s.new_boolvar());

        // Entry barrier starts at top-left
        int p_s(ml->linearize_coordinate(row_of(p_v), col_of(p_h)));
        int s_time(new_conflict.timestamp - abs(row_of(p_s) - row_of(p_h)));
        int dt(std::min(0, s_time));

        int h_dur(1 + abs(row_of(p_e) - row_of(p_s)));
        int h_delta(row_of(p_s) < row_of(p_e) ? ml->cols : -ml->cols);

        pathfinders[new_conflict.a1]->register_barrier(s1, s_time - dt, p_s - dt*h_delta, h_delta, h_dur - dt);

        int eh_start(ml->linearize_coordinate(row_of(p_s), col_of(p_e)));
        int eh_time(s_time + abs(col_of(p_e) - col_of(p_s)));

        pathfinders[new_conflict.a1]->register_barrier(e1, eh_time - dt, eh_start - dt*h_delta, h_delta, h_dur - dt);

        int v_dur(1 + abs(col_of(p_e) - col_of(p_s)));
        int v_delta(col_of(p_s) < col_of(p_e) ? 1 : -1);

        pathfinders[new_conflict.a2]->register_barrier(s2, s_time - dt, p_s - dt*v_delta, v_delta, v_dur - dt);

        int ev_start(ml->linearize_coordinate(row_of(p_e), col_of(p_s)));
        int ev_time(s_time + abs(row_of(p_e) - row_of(p_s)));

        pathfinders[new_conflict.a2]->register_barrier(e2, ev_time - dt, ev_start - dt*v_delta, v_delta, v_dur - dt);

        // One of the barriers must be active
        add_clause(s.data, s1, e1, s2, e2);
      }
        */
#endif
    } else {
      int loc = new_conflict.p.loc;
      pf::Move move = new_conflict.p.move;
      // Normalize. This should work fine for
      // M_WAIT (vertex) constraints too.
      if(loc > coord.nav.inv[loc][move]) {
        loc = coord.nav.inv[loc][move];
        move = pf::move_inv(move);
      }
        
      cons_key k { new_conflict.timestamp, move, loc };
      auto it(cons_map.find(k));
       
      int idx;
      if(it != cons_map.end()) {
        idx = (*it).second;
      } else {
        idx = constraints.size();
        cons_map.insert(std::make_pair(k, idx));
        constraints.push(cons_data { s.new_intvar(0, pathfinders.size()-1), btset::bitset(pathfinders.size()) });
      }

      int a1(new_conflict.a1);
      int a2(new_conflict.a2);
      cons_data& c(constraints[idx]);
      if(!c.attached.elem(a1)) {
        patom_t at(c.sel != a1);
        while(s.level() > 0 && at.lb(s.data->ctx()))
          s.backtrack();
        pathfinders[a1]->register_obstacle(at, k.timestamp, k.move, k.loc);
        if(k.loc == pathfinders[a1]->goal_pos)
          add_clause(s.data, ~at, pathfinders[a1]->cost > k.timestamp);
        c.attached.insert(a1);
      }
      if(!c.attached.elem(a2)) {
        patom_t at(c.sel != a2);
        while(s.level() > 0 && at.lb(s.data->ctx()))
          s.backtrack();
        pathfinders[a2]->register_obstacle(at, k.timestamp, k.move, k.loc);
        if(k.loc == pathfinders[a2]->goal_pos)
          add_clause(s.data, ~at, pathfinders[a2]->cost > k.timestamp);
        c.attached.insert(a2);
      }
      // FIXME: Abstract properly
      //s.data->confl.pred_saved[c.sel.p>>1].val = geas::from_int((rand() % 2 ? a1 : a2));
    }
  }
  new_conflicts.clear();
  return true;
}

bool MAPF_Solver::processCore(vec<geas::patom_t>& core) {
  // If the core is empty, we're unsatisfiable.
  // Shouldn't usually happen, since everyone can just wait at the
  // start state.
  if(core.size() == 0)
    return false;

  // Collect info for each element of the core.
  vec<int> idxs;
  uint64_t Dmin = UINT64_MAX;

  for(geas::patom_t c : core) {
    // Every assumption should be in the table.
    int c_idx = (*penalty_table.find(c.pid)).second;
    assert(c.val > penalties[c_idx].lb);
    uint64_t c_delta = c.val - penalties[c_idx].lb;
    Dmin = std::min(Dmin, c_delta);
    idxs.push(c_idx);
  }
  // We can increase the lower bound by Dmin.
  // Introduce the new penalty terms, and increase the existing bounds by Dmin.
  vec<int> coeffs(idxs.size(), 1);
  for(uint64_t d = 1; d <= Dmin; ++d) {
    vec<geas::patom_t> slice;
    for(int ci : idxs) {
      slice.push(geas::ge_atom(penalties[ci].p, penalties[ci].lb + d));
    }
    // New penalty var.
    intvar p(s.new_intvar(0, slice.size()-1));
    geas::bool_linear_ge(s.data, geas::at_True, p, coeffs, slice, -1);
    penalty_table.insert(std::make_pair(p.p, penalties.size()));
    penalties.push(penalty { p.p, geas::from_int(0) });
  }

  //cost_lb += Dmin;
  for(int ci : idxs) {
    penalties[ci].lb += Dmin;
  }
  return Dmin;
}

bool MAPF_Solver::runUCIter(void) {
  switch(s.solve()) {
    // No solution, we've got a new core.
    case geas::solver::UNSAT:
      return false;
    // Candidate optimal solution. Check for conflicts
    case geas::solver::SAT:  
        return true;
    case geas::solver::UNKNOWN:
      throw SolveAborted { };
  }
  return false;
}

MAPF_Solver::~MAPF_Solver(void) {

}

bool MAPF_MinCost(MAPF_Solver& mapf) {
  // Reset any existing assumptions
  mapf.s.clear_assumptions(); 

  vec<MAPF_Solver::penalty> penalties;
  std::unordered_map<geas::pid_t, int> penalty_table;

  int cost_lb(0); 
  for(Agent_PF* p : mapf.pathfinders) {
    cost_lb += p->pathCost();
    geas::pid_t id(p->cost.p);
    penalty_table.insert(std::make_pair(id, penalties.size()));
    penalties.push(MAPF_Solver::penalty { id, geas::from_int(p->pathCost()) });
  }
#ifdef DEBUG_UC
  fprintf(stderr, "%%%% Initial bound: %d\n", cost_lb);
#endif
 
  vec<geas::patom_t> assumps;
  for(MAPF_Solver::penalty& p : penalties)
    assumps.push(geas::le_atom(p.p, p.lb));

  vec<geas::patom_t> core;
  while(!mapf.buildPlan(assumps)) {
    core.clear();
    mapf.s.get_conflict(core);
    mapf.s.clear_assumptions();
    if(core.size() == 0)
      return false;
    // Look at the core elements.
    vec<int> idxs;
    uint64_t Dmin = UINT64_MAX;

    for(geas::patom_t c : core) {
      // Every assumption should be in the table.
      int c_idx = (*penalty_table.find(c.pid)).second;
      assert(c.val > penalties[c_idx].lb);
      uint64_t c_delta = c.val - penalties[c_idx].lb;
      Dmin = std::min(Dmin, c_delta);
      idxs.push(c_idx);
    }
    // We can increase the lower bound by Dmin.
    // Introduce the new penalty terms, and increase the existing bounds by Dmin.
    vec<int> coeffs(idxs.size(), 1);
    for(uint64_t d = 1; d <= Dmin; ++d) {
      vec<geas::patom_t> slice;
      for(int ci : idxs) {
        slice.push(geas::ge_atom(penalties[ci].p, penalties[ci].lb + d));
      }
      // New penalty var.
      intvar p(mapf.s.new_intvar(0, slice.size()-1));
      geas::bool_linear_ge(mapf.s.data, geas::at_True, p, coeffs, slice, -1);
      penalty_table.insert(std::make_pair(p.p, penalties.size()));
      penalties.push(MAPF_Solver::penalty { p.p, geas::from_int(0) });
    }
    // Update the existing penalties and bounds
    for(int ci : idxs) {
      penalties[ci].lb += Dmin;
    }
    cost_lb += Dmin;
    mapf.cost_lb = cost_lb;
#ifdef DEBUG_UC
      fprintf(stderr, "%%%% Found core of size (%d), current lower bound %d\n", core.size(), cost_lb);
#endif
    
    // Now set up the updated assumptions
    assumps.clear();
    for(MAPF_Solver::penalty& p : penalties)
      assumps.push(geas::le_atom(p.p, p.lb));
  }
  // assert(!mapf.checkForConflicts());
  return true;
}

struct subproblem {
  subproblem(geas::intvar _cost, int _lb)
    : cost(_cost), lb(_lb) { }
  geas::intvar cost;
  int lb;
  vec<coordinator::agent_cell> agents;
};

struct pending {
  pending(vec<int>& _core_members, int _lb)
    : core_members(_core_members), lb(_lb) { }
  vec<int> core_members;
  int lb;
};

struct opt_state {
  std::unordered_map<geas::pid_t, int> cost_map;

  vec<subproblem> subproblems;
  vec<pending> delayed;
};

int process_subproblem(MAPF_Solver& m, opt_state& state, const vec<int>& core_members, int lb_0) {
  int lb = lb_0;
  geas::intvar cost = m.s.new_intvar(lb, m.cost_ub);
  m.coord.clear_active();

  m.s.restart();
  vec<int> cs;
  vec<geas::intvar> xs;
  cs.push(-1);
  xs.push(cost);
  for(int sub_idx : core_members) {
    const subproblem& sub(state.subproblems[sub_idx]);
    cs.push(1);
    xs.push(sub.cost);
    m.coord.make_active(sub.agents.begin(), sub.agents.end());
  }
  geas::linear_le(m.s.data, cs, xs, 0);

  vec<geas::patom_t> assumps;
  vec<geas::patom_t> core;
  assumps.push(cost <= lb);
  while(!m.buildPlan(assumps)) {
    core.clear();
    m.s.get_conflict(core);
    m.s.clear_assumptions();
    // Globally infeasible (somehow, probably should be unreachable).
    assert(core.size() == 1);
    assert(core[0].pid == cost.p);

    lb = cost.lb_of_pval(core[0].val);
    assumps[0] = cost <= lb;
  }

  int idx = state.subproblems.size();
  state.subproblems.push(subproblem(cost, lb));
  m.coord.dump_active(state.subproblems.last().agents);
  state.cost_map.insert(std::make_pair(cost.p, idx));
  return lb - lb_0;
}

int process_delayed(MAPF_Solver& m, opt_state& state) {
  int delta = 0;
  for(const pending& p : state.delayed)
    delta += process_subproblem(m, state, p.core_members, p.lb);
  state.delayed.clear();
  return delta;
}

void fill_assumps(MAPF_Solver& m, const opt_state& state, vec<geas::patom_t>& assumps) {
  assumps.clear();
  for(auto p : state.cost_map) {
    const subproblem& sub(state.subproblems[p.second]);
    m.coord.make_active(sub.agents.begin(), sub.agents.end());
    assumps.push(sub.cost <= sub.lb);
  }
}

int process_core(MAPF_Solver& m, opt_state& state, const vec<geas::patom_t>& core) {
  assert(core.size() > 0);
  if(core.size() == 1) {
    int s_id = state.cost_map.find(core[0].pid)->second;
    int s_lb = state.subproblems[s_id].cost.lb_of_pval(core[0].val);
    int delta = state.subproblems[s_id].lb - s_lb;
    state.subproblems[s_id].lb = s_lb;
    return delta;
  } else {
    int lb = 0;
    int delta = INT_MAX;
    vec<int> subproblems;
    for(geas::patom_t at : core) {
      auto it = state.cost_map.find(at.pid);
      int s_id = it->second;
      state.cost_map.erase(it);
      const subproblem& sub(state.subproblems[s_id]);
      m.coord.rem_active(sub.agents.begin(), sub.agents.end());

      int s_lb = sub.cost.lb_of_pval(at.val);
      lb += sub.lb;
      delta = std::min(delta, s_lb - sub.lb);
      subproblems.push(s_id);
    }
    state.delayed.push(pending(subproblems, lb + delta));
    return delta;
  }
}

bool MAPF_MinCost_REC(MAPF_Solver& mapf) {
  int lb = 0;
  opt_state state;
  const auto& ctx = mapf.s.data->ctx();
  for(Agent_PF* p : mapf.pathfinders) {
    int p_lb = p->cost.lb(ctx);
    lb += p_lb;
    int s_id = state.subproblems.size();
    state.subproblems.push(subproblem(p->cost, p_lb));
    state.subproblems[s_id].agents.push(coordinator::agent_cell(p->agent_id));
    state.cost_map.insert(std::make_pair(p->cost.p, s_id));
  }

  mapf.cost_lb = lb;
#ifdef DEBUG_UC
  fprintf(stderr, "%%%% Initial bound: %d\n", lb);
#endif

  vec<geas::patom_t> assumps;
  vec<geas::patom_t> core;

 restart_iter:
  fill_assumps(mapf, state, assumps);
  if(!mapf.buildPlan(assumps)) {
    // At least one core.
    do {
      core.clear();
      mapf.s.get_conflict(core);
      lb += process_core(mapf, state, core);
#ifdef DEBUG_UC
      fprintf(stderr, "%%%% Found core of size (%d), current lower bound %d\n", core.size(), lb);
#endif
      mapf.cost_lb = lb;
      mapf.s.clear_assumptions();
      fill_assumps(mapf, state, assumps);
    } while(!mapf.buildPlan(assumps));
    mapf.s.restart();
    lb += process_delayed(mapf, state);
    mapf.cost_lb = lb;
#ifdef DEBUG_UC
    fprintf(stderr, "%%%% Tightened cores, current lower bound %d\n", lb);
#endif
    goto restart_iter;
  }
  return true;
}

bool MAPF_MinMakespan(MAPF_Solver& mapf) {
  mapf.s.clear_assumptions();
  int makespan_lb(0);
  std::unordered_map<geas::pid_t, unsigned int> agent_table;
  unsigned int ai = 0;
  for(Agent_PF* p : mapf.pathfinders) {
    makespan_lb = std::max(makespan_lb, (int) p->pathCost());
    agent_table.insert(std::make_pair(p->cost.p, ai));
    ++ai;
  }

  vec<geas::patom_t> assumps;

  for(Agent_PF* p : mapf.pathfinders)
    assumps.push(p->cost <= makespan_lb);
  
  vec<geas::patom_t> core;
  mapf.cost_lb = makespan_lb;
  while(!mapf.buildPlan(assumps)) {
    core.clear();
    mapf.s.get_conflict(core);
    mapf.s.clear_assumptions();
    // Globally infeasible (somehow, probably should be unreachable).
    if(core.size() == 0)
      return false;
    // Otherwise, extract the new makespan from the core. 
    int new_makespan(0);
    for(geas::patom_t at : core) {
      // Turn the core atoms back into bounds.
      auto it(agent_table.find(at.pid));
      assert(it != agent_table.end());
      const geas::intvar& x(mapf.pathfinders[(*it).second]->cost);
      new_makespan = std::max(new_makespan, x.lb_of_pval(at.val));
    }
    assert(new_makespan > makespan_lb);
    makespan_lb = mapf.cost_lb = new_makespan;
    assumps.clear();
    for(Agent_PF* p : mapf.pathfinders)
      assumps.push(p->cost <= makespan_lb);
  }

  return true;
}

bool MAPF_MaxDeadlines(MAPF_Solver& mapf, vec<unsigned int>& deadlines) {
  assert(mapf.pathfinders.size() == deadlines.size());
  mapf.s.clear_assumptions();
  int cost_lb(0);
  unsigned int ai = 0;
  // In the result, even values are thresholds,
  // odd values are counts.
  std::unordered_map<geas::pid_t, int> penalty_table;
  vec<bool> enforced(deadlines.size(), true);
  vec<MAPF_Solver::penalty> penalties;

  for(Agent_PF* p : mapf.pathfinders) {
    // If the deadline is infeasible, just add it to the set of penalties.
    if(p->pathCost() > deadlines[ai]) {
      cost_lb++;
      enforced[ai] = false;
    } else {
      geas::pid_t id(p->cost.p);
      penalty_table.insert(std::make_pair(id, ai<<1));
    }
  }

  // Initially, we only have thresholds.
  vec<geas::patom_t> assumps;
  for(int ai : irange(mapf.pathfinders.size())) {
    if(enforced[ai]) {
      const intvar& x(mapf.pathfinders[ai]->cost);
      assumps.push(x <= deadlines[ai]);
    }
  }

  vec<geas::patom_t> core;
  while(!mapf.buildPlan(assumps)) {
    core.clear();
    mapf.s.get_conflict(core);
    mapf.s.clear_assumptions();  
    if(core.size() == 0)
      return false;
    // Look at the core elements.
    vec<int> idxs;
    uint64_t Dmin = UINT64_MAX;

    for(geas::patom_t c : core) {
      // Every assumption should be in the table.
      int c_tag = (*penalty_table.find(c.pid)).second;
      idxs.push(c_tag);
      if(!(c_tag & 1)) { // First bit is zero; original deadline.
        int ai(c_tag>>1);  
        // If a deadline is in the core, we can relax by at most one step. 
        Dmin = 1;
        enforced[ai] = 0;
      } else {
        int p_idx(c_tag>>1);
        assert(c.val > penalties[p_idx].lb);
        uint64_t c_delta = c.val - penalties[p_idx].lb;
        Dmin = std::min(Dmin, c_delta);
      }
    }
    // We can increase the lower bound by Dmin.
    // Introduce the new penalty terms, and increase the existing bounds by Dmin.
    vec<int> coeffs(idxs.size(), 1);
    for(uint64_t d = 1; d <= Dmin; ++d) {
      vec<geas::patom_t> slice;
      for(int c_tag : idxs) {
        if(!(c_tag & 1)) {
          int ai(c_tag>>1);
          slice.push(mapf.pathfinders[ai]->cost > deadlines[ai]); 
        } else {
          int ci(c_tag>>1);
          slice.push(geas::ge_atom(penalties[ci].p, penalties[ci].lb + d));
          penalties[ci].lb += Dmin;
        }
      }
      // New penalty var.
      intvar p(mapf.s.new_intvar(0, slice.size()-1));
      geas::bool_linear_ge(mapf.s.data, geas::at_True, p, coeffs, slice, -1);
      penalty_table.insert(std::make_pair(p.p, penalties.size()));
      penalties.push(MAPF_Solver::penalty { p.p, geas::from_int(0) });
    }
    // Update the total cost.
    cost_lb += Dmin;
    
    // Now set up the updated assumptions
    assumps.clear();
    int ai = 0;
    for(Agent_PF* p : mapf.pathfinders) {
      if(enforced[ai])
        assumps.push(p->cost <= deadlines[ai]);
      ++ai;
    }
    for(MAPF_Solver::penalty& p : penalties)
      assumps.push(geas::le_atom(p.p, p.lb)); 
  }
  return true;
}

}
