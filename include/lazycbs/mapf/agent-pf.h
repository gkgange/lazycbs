#ifndef GEAS_AGENT__PF_H
#define GEAS_AGENT__PF_H
#include <utility>
#include <functional>
#include "single_agent_ecbs.h"

#include <geas/engine/propagator.h>
#include <geas/engine/propagator_ext.h>
#include <geas/vars/intvar.h>

#include <geas/mtl/bool-set.h>

#include <lazycbs/mapf/coordinator.h>

#define MAPF_BETTER_EXPLANATIONS
namespace mapf {

using namespace geas;

class Agent_PF : public propagator, public prop_inst<Agent_PF> {
  // An obstacle, with the activating atom.
  enum ObstacleType { O_MUTEX, O_BARRIER };
  /*
  struct barrier_info {
    int pos; // Barrier position at the start time
    pf::Move dir; // Difference between successive barrier positions
    int duration; // How long does the barrier hold?
  };
  */
  // Generalized version.
 public:
  struct constraint {
    constraint(pf::Move _move, int _loc, int _time)
    : move(_move), loc(_loc), time(_time) { }

    pf::Move move;
    int loc;
    int time;
  };
 protected:

  struct barrier_info {
    barrier_info(geas::patom_t _at) : at(_at) { }
    geas::patom_t at;
    vec<constraint> constraints;
  };
  
  struct gen_barrier_info {
    vec<constraint> constraints;
  };
  
  struct mutex_info {
    int loc;
    pf::Move move;
  };
  struct obstacle_info {
  obstacle_info(patom_t _at, int _timestep, int loc, pf::Move move)
  : at(_at), timestep(_timestep), tag(O_MUTEX), p({ loc, move }) { }
    /*
  obstacle_info(patom_t _at, int _timestep, int loc, pf::Move move, int duration)
      : at(_at), timestep(_timestep), tag(O_BARRIER), b({ loc, move, duration}) { }
    */
    obstacle_info(patom_t _at, int _bidx)
    : at(_at), timestep(-1), tag(O_BARRIER), b(_bidx) { }
    
    patom_t at;
    int timestep;
    ObstacleType tag;
    
    union { 
      // barrier_info b;
      int b;
      mutex_info p;
    };
  };

  // For tracking explanations
  struct reason_cell {
    reason_cell(void)
    {
      for(int ii = 0; ii < pf::NUM_MOVES; ++ii)
        r[ii] = at_Undef;
    }
    geas::patom_t& operator[](pf::Move m) { return r[m]; }

    geas::patom_t r[pf::NUM_MOVES];
  };
  typedef uint64_triemap<uint64_t, reason_cell, UIntOps> reason_map;

  // A (previous) incumbent.
  /*
  struct path_info {
    double cost;
    vec<int> path;
  };
  */

  // In both cases, only wake up if the new obstacle would invalidate
  // the incumbent.
  watch_result wake_cost(int _xi) {
    // Only wake up if the incumbent is invalidated.
    if(ub(cost) < pathCost()) {
      queue_prop();
    }
    return Wt_Keep;
  }

  watch_result wake_obstacle(int ci) {
    apply_obstacle(ci);
    if(incumbent_conflicts_with(ci)) {
      queue_prop();
    }
    return Wt_Keep;
  }

  watch_result wake_barrier(int bi) {
    apply_gen_barrier(bi);
    if(incumbent_conflicts_with(barriers[bi].constraints))
      queue_prop();
    return Wt_Keep;
  }

  void expl_length(int tl, pval_t p, vec<clause_elt>& expl) {
    // For now, just collect the set of constraints that were active.
    obstacle_atoms(tl, expl);
    /* // FIXME
#ifndef MAPF_BETTER_EXPLANATIONS
    obstacle_atoms(tl, expl);
#else
    vec<int> ex_csts;
    extract_lb_explanation(tl, cost.lb_of_pval(p), ex_csts);
    for(int c : ex_csts)
      EX_PUSH(expl, ~obstacles[c].at);
#endif
    */
  }

  void obstacle_atoms(int tl, vec<clause_elt>& expl) {
    for(int si = 0; si < tl; ++si) {
      // EX_PUSH(expl, ~obstacles[obs_stack[si]].at);
      constraint c = obs_stack[si];
      geas::patom_t at = (*reasons[c.loc].find(c.time)).value[c.move];
      EX_PUSH(expl, ~at);
    }
  }

  void restore_stack(void) {
    // Iteration order doesn't matter, because the multiset of pops
    // is the same.
    for(int si = obs_tl; si < obs_stack.size(); ++si) {
      for(int si = obs_tl; si < obs_stack.size(); ++si) {
        const constraint& c(obs_stack[si]);
        sctx.release(c.move, c.loc, c.time);
      }
    }
    obs_stack.shrink(obs_stack.size() - obs_tl);
  }

  void apply_gen_barrier(int bidx) {
    const barrier_info& b(barriers[bidx]);

    for(constraint c : b.constraints) {
      forbid(c.move, c.loc, c.time, b.at);
      if(c.move != pf::M_WAIT)
        forbid(pf::move_inv(c.move), coord.nav.inv[c.loc][c.move], c.time, b.at);
    }
  }

  void forbid(pf::Move move, int loc, int time, geas::patom_t reason) {
    // Check whether constraint is already posted.
    if(sctx.is_allowed(move, loc, time)) {
      sctx.forbid(move, loc, time);
      auto it = reasons[loc].find(time);
      (*it).value[move] = reason;
      obs_stack.push(constraint(move, loc, time));
    }
  }

  void apply_obstacle(int ci) {
    // Make sure we're starting from a consistent state.
    if(obs_tl < obs_stack.size())
      restore_stack();
    assert(obs_tl == obs_stack.size());
    
    // Then apply the new obstacle.
    const obstacle_info& o(obstacles[ci]);

    /*
    obs_stack.push(ci);
    set(obs_tl, obs_stack.size());
    */
    if(o.tag == O_BARRIER) {
      //apply_barrier(o.timestep, o.b);
      assert(0);
      //apply_gen_barrier(o.b);
    } else {
      forbid(o.p.move, o.p.loc, o.timestep, o.at);
      if(o.p.move != pf::M_WAIT)
        forbid(pf::move_inv(o.p.move), coord.nav.inv[o.p.loc][o.p.move], o.timestep, o.at);
    }
    set(obs_tl, obs_stack.size());
  }

  // The first step not before t.
  static int get_path_index(const pf::Path& path, int t) {
    auto it = std::lower_bound(path.begin(), path.end(),
                               t, [](pf::Step s, int t) { return s.first < t; });
    return it - path.begin();
  }

 
  bool incumbent_conflicts_with(int ci) {
    update_incumbent();

    const obstacle_info& o(obstacles[ci]);
    // const vec<int>& path(history[hist_pos].path);
    if(o.tag == O_BARRIER) {
      /*
      int p = o.b.pos;
      for(int t = o.timestep; t < o.timestep + o.b.duration; ++t) {
        int ap = (t < path.size()) ? path[t] : path.last();
        if(ap == p)
          return true;
        p += o.b.delta;
      }
      */
      assert(0); // TODO
      return false;
    } else {
      const pf::Path& path(coord.get_path(agent_id));
      int idx = get_path_index(path, o.timestep);

      pf::Step step(idx < path.size() ? path[idx] : std::make_pair(INT_MAX, pf::M_WAIT));
      int loc = coord.nav.delta[incumbent[idx]][step.second];

      // First, check vertex constraints.
      if(o.p.move == pf::M_WAIT && loc == o.p.loc)
        return true;

      // Edge constraints only trigger at the moment of transition.
      if(step.first != o.timestep)
        return false;

      // Check forward direction. Started somewhere, moved
      // in constraint direction, landed at constraint location.
      if(path[idx].second == o.p.move && loc == o.p.loc)
        return true;
      
      // Otherwise, we started from the constraint location,
      // and moved in the inverse direction.
      if(path[idx].second == pf::move_inv(o.p.move)
         && incumbent[idx] == o.p.loc)
        return true;
      return false;
    }
  }
  bool incumbent_conflicts_with(const vec<constraint>& constraints) {
    update_incumbent();
    const pf::Path& path(coord.get_path(agent_id));

    for(const constraint& c : constraints) {
      int idx = get_path_index(path, c.time);
      pf::Step step(idx < path.size() ? path[idx] : std::make_pair(INT_MAX, pf::M_WAIT));
      int loc = coord.nav.delta[incumbent[idx]][step.second];

      // First, check vertex constraints.
      if(c.move == pf::M_WAIT && loc == c.loc)
        return true;

      // Edge constraints only trigger at the moment of transition.
      if(step.first != c.time)
        return false;

      // Check forward direction. Started somewhere, moved
      // in constraint direction, landed at constraint location.
      if(path[idx].second == c.move && loc == c.loc)
        return true;
      
      // Otherwise, we started from the constraint location,
      // and moved in the inverse direction.
      if(path[idx].second == pf::move_inv(c.move)
         && incumbent[idx] == c.loc)
        return true;
      return false;
    }
  }


public:
 Agent_PF(solver_data* s, coordinator& _coord, intvar _cost, int start_location, int goal_location)
    : propagator(s)
    , coord(_coord)
    // , active_obstacles(map_size)
    , start_pos(start_location), goal_pos(goal_location)
    , cost(_cost), obs_tl(0) //, hist_pos(0)
    , has_bypass(false)
    , incumbent_path_num(0)
    , sctx(coord.nav.size())
    , f_heur(coord.nav.fwd_heuristic(goal_location))
    , r_heur(coord.nav.rev_heuristic(start_location))
    , reasons(coord.nav.size())
    {
    cost.attach(E_UB, watch<&P::wake_cost>(0, Wt_IDEM));
    int pathC = coord.sipp_pf.search(start_pos, goal_pos,
                                     sctx, f_heur, coord.res_table.reserved);
    if(pathC == INT_MAX)
      throw RootFail();
    agent_id = coord.add_agent(start_pos, coord.sipp_pf.path);

    if(cost.lb(s) < pathC) {
      if(!set_lb(cost, pathC, reason()))
        throw RootFail();
    }
  }

  ~Agent_PF(void) {
    delete[] f_heur;
    delete[] r_heur;
  }

  bool check_sat(ctx_t& ctx);
  bool check_unsat(ctx_t& ctx) { return !check_sat(ctx); };


  void update_incumbent(void) {
    if(incumbent_path_num != coord.plans[agent_id].path_num) {
      incumbent_path_num = coord.plans[agent_id].path_num;

      incumbent.clear();
      const auto& path(coord.get_path(agent_id));
      int loc = start_pos;

      for(pf::Step step : path) {
        incumbent.push(loc);
        loc = coord.nav.delta[loc][step.second];
      }
      assert(loc == goal_pos);
      incumbent.push(loc);
    }
  }

  void make_reason_cell(int loc, int timestep) {
    auto it = reasons[loc].find(timestep);
    if(it == reasons[loc].end())
      reasons[loc].add(timestep, reason_cell());
  }

  int register_obstacle(patom_t at, int timestep, pf::Move move, int loc) {
    int ci(obstacles.size());
    // Add the new constraint to the pool
    obstacles.push(obstacle_info(at, timestep, loc, move));
    make_reason_cell(loc, timestep);
    if(move != pf::M_WAIT)
      make_reason_cell(coord.nav.inv[loc][move], timestep);

    // And make sure the propagator wakes up when becomes set.
    attach(s, at, watch<&P::wake_obstacle>(ci, Wt_IDEM));
    return ci;
  }

  int register_barrier(patom_t at, const vec<constraint>& constraints) {
    int bi(barriers.size()); 
    barriers.push(barrier_info(at));
    for(auto c : constraints)
      barriers[bi].constraints.push(c);
    /*
    // Add the new constraint to the pool
    barriers.push(obstacle_info(at, timestep, loc, move, duration));
    // And make sure the propagator wakes up when becomes set.
    attach(s, at, watch<&P::wake_obstacle>(ci, Wt_IDEM));
    */
    return bi;
  }

  bool propagate(vec<clause_elt>& confl) {
    has_bypass = 0;
    // assert(obs_stack.size() == obs_tl);
    assert(obs_tl <= obs_stack.size());
    if(obs_tl < obs_stack.size())
      restore_stack();

    coord.hide_agent(agent_id);
    int pathC = coord.sipp_pf.search(start_pos, goal_pos,
                                     sctx, f_heur, coord.res_table.reserved);
    if(pathC == INT_MAX) {
      coord.restore_agent(agent_id);

      // Failed to produce any path
      // TODO: Explanation
      obstacle_atoms(obs_stack.size(), confl);
      return false;
    }

    // Otherwise, update the lower bound.
    if(lb(cost) < pathC) {
      if(!set_lb(cost, pathC, expl<&P::expl_length>(obs_stack.size()))) {
        coord.restore_agent(agent_id);
        return false;
      }
    }
    coord.restore_agent_with(agent_id, coord.sipp_pf.path);
    return true;
  }

  bool find_bypass(void) {
    assert(0);
    #if 0
    if(coord.sipp_pf.search_upto(start_loc, goal_loc, ub(cost),
                                 sctx, coord.res_table)) {
      // Make sure the bypass is reset after backtracking
      if(!has_bypass)
        s->persist.bt_flags.push(&has_bypass);
      has_bypass = true;
    
      // TODO
      /*
      const std::vector<int>& path(engine.path);
      bypass_path.clear();
      for(int p : path)
        bypass_path.push(p);
      */
      return true;
    }
    #endif
    return false;
  }

  // double pathCost(void) const { return has_bypass ? bypass_path.size()-1 : history[hist_pos].cost; }
  inline double pathCost(void) const { return pf::path_cost(coord.get_path(agent_id)); }

  int agent_id;
  
  int start_pos;
  int goal_pos;

  intvar cost;

  vec<obstacle_info> obstacles; // The registered obstacles
  vec<barrier_info> barriers;
  vec<constraint> obs_stack;           // The history of pushed obstacles
  Tint obs_tl;                  // How big _should_ obs_stack be?

  // History is now managed by the coordinator. Bypasses are currently still here,
  // but that's likely to change.
  pf::Path bypass_path;
  char has_bypass;

  vec<int> incumbent;
  int incumbent_path_num;

  // We keep sctx separate per agent, to reduce the number of intervals
  // we consider.
  coordinator& coord;
  sipp_ctx sctx; 
  int* f_heur;
  int* r_heur;

  // Needs to be kept synchronized with sctx.
  vec<reason_map> reasons;
};

}
#endif
