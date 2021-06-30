#include <geas/constraints/builtins.h>

#include <lazycbs/mapf/mapf-solver.h>
#include <lazycbs/pf/pf.hpp>

#include "compute_heuristic.h"

#define DEBUG_UC
// #define MAPF_NO_RECTANGLES

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
  , coord(s, nav) {
    int num_of_agents = al.num_of_agents;
    // int map_size = ml->rows*ml->cols;
    cost_lb = 0;

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

void log_conflict(MAPF_Solver& mapf) {
  for(auto new_conflict : mapf.new_conflicts) {
    int a1(new_conflict.a1);
    int a2(new_conflict.a2);
    int t(new_conflict.timestamp);

    if(new_conflict.type == MAPF_Solver::C_BARRIER) {
      fprintf(stderr, "%%%% Adding rectangle: [%d, (%d, %d) |- (%d, %d), %d, %d]\n",
        t, mapf.row_of(new_conflict.b.s_loc), mapf.col_of(new_conflict.b.s_loc),
           mapf.row_of(new_conflict.b.e_loc), mapf.col_of(new_conflict.b.e_loc),
           a1, a2);
    } else {
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
  int LL_num_expanded = 0;
  int LL_executions = 0;
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
int MAPF_Solver::monotoneSubchainStart(int dy, int dx, int ai, int t) const {
  assert(0); // FIXME
  /*
  int p(agentPosition(pathfinders[ai], t));
  for(--t; t >= 0; --t) {
    int q(agentPosition(pathfinders[ai], t));
    if(p == q)
      return t+1;
    if(col_of(p) != col_of(q) && col_of(p) - col_of(q) != dx)
      return t+1;
    if(row_of(p) != row_of(q) && row_of(p) - row_of(q) != dy)
      return t+1;
    p = q; 
  }
  */
  return 0;
}

int MAPF_Solver::monotoneSubchainEnd(int dy, int dx, int ai, int t) const {
  assert(0); // FIXME
  /*
  int p(agentPosition(pathfinders[ai], t));
  int tMax(pathfinders[ai]->getPath().size());
  for(++t; t < tMax; ++t) {
    int q(agentPosition(pathfinders[ai], t));
    if(p == q)
      return t-1;
    if(col_of(p) != col_of(q) && col_of(q) - col_of(p) != dx)
      return t-1;
    if(row_of(p) != row_of(q) && row_of(q) - row_of(p) != dy)
      return t-1;
    p = q; 
  }
  return tMax-1;
  */
  return INT_MAX;
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

// Multiple-conflict handling
bool MAPF_Solver::checkForConflicts(void) {
  int pMax = maxPathLength();
  
  vec<int> prev_loc;
  vec<int> curr_loc;

  vec<pf::Step*> path_it;
  vec<pf::Step*> path_en;

  int Tnext = INT_MAX;
  for(int ai = 0; ai < pathfinders.size(); ++ai) {
    int loc = coord.plans[ai].origin;
    const pf::Path& path(coord.get_path(ai));

    assert(nmap[loc] < 0); // Shouldn't be any conflicts at t0.
    curr_loc.push(loc);
    prev_loc.push(loc);
    nmap[loc] = ai;

    auto it = path.begin();
    auto en = path.end();
    if(it != en)
      Tnext = std::min(Tnext, it->first);
    path_it.push(it);
    path_en.push(en);
  }

  // All agents are interesting.
  agent_set.sz = pathfinders.size();

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
        /*
        // Already occupied.
        int aj(nmap[loc]);
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
        // FIXME: Re-introduce rectangles/barriers.
        new_conflicts.push(conflict(t, ai, nmap[loc], pf::M_WAIT, loc));

        clear_map(agent_set, prev_loc, cmap);
        clear_map(agent_set, curr_loc, nmap);
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
          new_conflicts.push(conflict(t, ai, cmap[loc], coord.nav.move_dir(loc, rloc), loc));
          clear_map(agent_set, prev_loc, cmap);
          clear_map(agent_set, curr_loc, nmap);
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
static int barrier_dx[4] = { 0, -1, 0, 1 };
static int barrier_dy[4] = { -1, 0, 1, 0 };

geas::patom_t MAPF_Solver::getBarrier(int ai, BarrierDir dir, int t, int p, int dur) {
  // If this barrier is trivially violated, just return F.
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
  assert(0);
  return at_Undef;
}

bool MAPF_Solver::addConflict(void) {
  HL_conflicts++;
  for(auto new_conflict : new_conflicts) {
    if(new_conflict.type == C_BARRIER) {
      assert(0);
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
      if(loc > coord.nav.delta[loc][move]) {
        loc = coord.nav.delta[loc][move];
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
        c.attached.insert(a1);
      }
      if(!c.attached.elem(a2)) {
        patom_t at(c.sel != a2);
        while(s.level() > 0 && at.lb(s.data->ctx()))
          s.backtrack();
        pathfinders[a2]->register_obstacle(at, k.timestamp, k.move, k.loc);
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
