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
  struct barrier_info {
    int pos; // Barrier position at the start time
    pf::Move dir; // Difference between successive barrier positions
    int duration; // How long does the barrier hold?
  };
  struct mutex_info {
    int loc;
    pf::Move move;
  };
  struct obstacle_info {
  obstacle_info(patom_t _at, int _timestep, int loc, pf::Move move)
  : at(_at), timestep(_timestep), tag(O_MUTEX), p({ loc, move }) { }
  obstacle_info(patom_t _at, int _timestep, int loc, pf::Move move, int duration)
      : at(_at), timestep(_timestep), tag(O_BARRIER), b({ loc, move, duration}) { }
    patom_t at;
    int timestep;
    ObstacleType tag;
    
    union { 
      barrier_info b;
      mutex_info p;
    };
  };

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
      EX_PUSH(expl, ~obstacles[obs_stack[si]].at);
    }
  }

  // Restore the active constraints to the state we _should_ be in now,
  // looking at obs_tl.
  void revert_barrier(int t, const barrier_info& b) {
    assert(0);
    /*
    int p = b.pos;
    while(t < end) {
      // Ugh. Need to make sure we don't double-release
      // a location.
      sctx.release(pf::M_VERT, p, t);
      int pred = p;
      p = coord.nav.delta[p][b.move];
      t += coord.nav.dist(pred, p);
    }
    */
  }

  void restore_stack(void) {
    // Iteration order doesn't matter, because the multiset of pops
    // is the same.
    for(int si = obs_tl; si < obs_stack.size(); ++si) {
      const obstacle_info& o(obstacles[obs_stack[si]]);
      if(o.tag == O_BARRIER) {
        revert_barrier(o.timestep, o.b);
      } else {
        sctx.release(o.p.move, o.p.loc, o.timestep);
        if(o.p.move != pf::M_WAIT) {
          sctx.release(pf::move_inv(o.p.move),
                       coord.nav.inv[o.p.loc][o.p.move], o.timestep);
        }
      }
    }
    obs_stack.shrink(obs_stack.size() - obs_tl);
  }

  void apply_barrier(int t, const barrier_info& b) {
    assert(0);
#if 0
    int p = b.pos;
    int end = t + b.duration;
    while(t < end) {
      sctx.release(pf::M_VERT, p, t);
    }
    for(; t < end; ++t) {
#if 0
      active_obstacles[t].push_back(std::make_pair(p, -1));
#else
      active_obstacles[p].push_back(std::make_pair(t, -1));
#endif
      p += b.delta;
    }
#endif
  }

  void apply_obstacle(int ci) {
    // Make sure we're starting from a consistent state.
    if(obs_tl < obs_stack.size())
      restore_stack();
    assert(obs_tl == obs_stack.size());
    
    // Then apply the new obstacle.
    const obstacle_info& o(obstacles[ci]);

    obs_stack.push(ci);
    set(obs_tl, obs_stack.size());
    if(o.tag == O_BARRIER) {
      apply_barrier(o.timestep, o.b);
    } else {
      sctx.forbid(o.p.move, o.p.loc, o.timestep);
      if(o.p.move != pf::M_WAIT)
        sctx.forbid(pf::move_inv(o.p.move), coord.nav.inv[o.p.loc][o.p.move], o.timestep);
    }
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

  int register_obstacle(patom_t at, int timestep, pf::Move move, int loc) {
    int ci(obstacles.size());
    // Add the new constraint to the pool
    obstacles.push(obstacle_info(at, timestep, loc, move));
    // And make sure the propagator wakes up when becomes set.
    attach(s, at, watch<&P::wake_obstacle>(ci, Wt_IDEM));
    return ci;
  }

  int register_barrier(patom_t at, int timestep, int loc, pf::Move move, int duration) {
    int ci(obstacles.size()); 
    // Add the new constraint to the pool
    obstacles.push(obstacle_info(at, timestep, loc, move, duration));
    // And make sure the propagator wakes up when becomes set.
    attach(s, at, watch<&P::wake_obstacle>(ci, Wt_IDEM));
    return ci;
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
  vec<int> obs_stack;           // The history of pushed obstacles
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
};

}
#endif
