#ifndef GEAS_AGENT__PF_H
#define GEAS_AGENT__PF_H
#include <iostream>
#include <utility>
#include <functional>

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

  struct target_info {
    int loc;
    geas::intvar time;
    int threshold;
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
  inline bool is_active(void) const { return coord.is_active(agent_id); }

  watch_result wake_cost(int _xi) {
    // Only wake up if the incumbent is invalidated.
    if(!is_active())
      return Wt_Keep;
    coord.reset();
    if(ub(cost) < pathCost()) {
      queue_prop();
    }
    return Wt_Keep;
  }

  watch_result wake_cost_lb(int _xi) {
    if(!is_active())
      return Wt_Keep;
    coord.reset();
    if(lb(cost) > pathCost()) {
      apply_delay(lb(cost));
      queue_prop();
    }
    return Wt_Keep;
  }

  watch_result wake_obstacle(int ci) {
    if(!is_active())
      return Wt_Keep;
    apply_obstacle(ci);
    if(incumbent_conflicts_with(ci)) {
      queue_prop();
    }
    return Wt_Keep;
  }

  watch_result wake_barrier(int bi) {
    if(!is_active())
      return Wt_Keep;
    if(obs_tl < obs_stack.size())
      restore_stack();
    assert(obs_tl == obs_stack.size());
    apply_gen_barrier(bi);
    if(incumbent_conflicts_with<>(barriers[bi].constraints))
      queue_prop();
    return Wt_Keep;
  }

  watch_result wake_target(int ti) {
    if(!is_active())
      return Wt_Keep;
    const target_info& t(targets[ti]);
    if(ub(t.time) <= t.threshold) {
      if(obs_tl < obs_stack.size())
        restore_stack();
      assert(ub(t.time) < sctx[t.loc].Tend);
      obs_stack.push(constraint(pf::NUM_MOVES, ti, sctx[t.loc].Tend));
      set(obs_tl, obs_stack.size());
      sctx[t.loc].Tend = ub(t.time);
      queue_prop();
    }
    return Wt_Keep;
  }
  
  void expl_length_bound(int lb, vec<clause_elt>& expl) {
    vec<sipp_explainer::cst> ex_csts;
    coord.sipp_ex.explain(start_pos, goal_pos, lb,
                          sctx, f_heur, r_heur, ex_csts);
    for(auto c : ex_csts) {
      if(c.move == sipp_explainer::M_LOCK) {
        // Target constraint
        int ti = target_map.find(c.loc)->second;
        if(c.time > targets[ti].threshold)
          targets[ti].threshold = c.time;
        EX_PUSH(expl, targets[ti].time > c.time);
      } else {
        geas::patom_t at = (*reasons[c.loc].find(c.time)).value[(pf::Move) c.move];
        EX_PUSH(expl, ~at);
      }
    }
  }
  void expl_length(int tl, pval_t p, vec<clause_elt>& expl) {
    // For now, just collect the set of constraints that were active.
#ifdef MAPF_BETTER_EXPLANATIONS
    // Temporarily reset the obs_stack.
    set_stack_to(tl);
    expl_length_bound(cost.lb_of_pval(p)-1, expl);
    /*
    vec<sipp_explainer::cst> ex_csts;
    // Temporarily reset the obs_stack.
    set_stack_to(tl);
    coord.sipp_ex.explain(start_pos, goal_pos, cost.lb_of_pval(p)-1,
                          sctx, f_heur, r_heur, ex_csts);
    for(auto c : ex_csts) {
      if(c.move == sipp_explainer::M_LOCK) {
        // Target constraint
        int ti = target_map.find(c.loc)->second;
        EX_PUSH(expl, targets[ti].time > c.time);
      } else {
        geas::patom_t at = (*reasons[c.loc].find(c.time)).value[(pf::Move) c.move];
        EX_PUSH(expl, ~at);
      }
    }
    */
#else
    obstacle_atoms(tl, expl);
#endif
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

 public:
  void debug_print_path(const pf::Path& path) const {
    int loc = start_pos;
    std::cout << agent_id << ":" << loc;
    for(pf::Step s : path) {
      loc = coord.nav.delta[loc][s.second];
      std::cout << "," << s.first << "=>" << loc;
    }
    std::cout << std::endl;
  }
  
 protected:
  void obstacle_atoms(int tl, vec<clause_elt>& expl) {
    for(int si = 0; si < tl; ++si) {
      // EX_PUSH(expl, ~obstacles[obs_stack[si]].at);
      constraint c = obs_stack[si];
      if(c.move == pf::NUM_MOVES) {
        // Either target or delay constraint.
        if(c.loc < 0) {
          // Delay constraint
          EX_PUSH(expl, cost < lb(cost));
        } else {
          const target_info& t(targets[c.loc]);
          // FIXME: Track the relevant threshold instead.
          EX_PUSH(expl, t.time > ub(t.time));
        }
      } else {
        geas::patom_t at = (*reasons[c.loc].find(c.time)).value[c.move];
        EX_PUSH(expl, ~at);
      }
    }
  }

  void set_stack_to(int tl) {
    for(int si = tl; si < obs_stack.size(); ++si) {
      const constraint& c(obs_stack[si]);
      if(c.move == pf::NUM_MOVES) {
        if(c.loc < 0) {
          // Goal delay constraint
          if(c.time >= goal_min)
            continue;
          auto it = sctx[goal_pos].reach(goal_min);
          assert(it->start == goal_min);
          assert(goal_min == 0 || it->constraints & pf::C_DELAY);

          it->constraints &= ~pf::C_DELAY;
          if(c.time > 0) {
            it = sctx[goal_pos].reach(c.time);
            assert(it->start == c.time);
            it->constraints |= pf::C_DELAY;
          }
          goal_min = c.time;
        } else {
          // Target constraint. Reset to the old Tend.
          int loc = targets[c.loc].loc;
          if(sctx[loc].Tend < c.time)
            sctx[loc].Tend = c.time;
        }
      } else {
        sctx.release(c.move, c.loc, c.time);
      }
    }
  }
  void restore_stack(void) {
    // Iteration order doesn't matter, because the multiset of pops
    // is the same.
    set_stack_to(obs_tl);
    obs_stack.shrink(obs_stack.size() - obs_tl);
  }

  void apply_gen_barrier(int bidx) {
    const barrier_info& b(barriers[bidx]);

    for(constraint c : b.constraints) {
      forbid(c.move, c.loc, c.time, b.at);
      if(c.move != pf::M_WAIT)
        forbid(pf::move_inv(c.move), coord.nav.inv[c.loc][c.move], c.time, b.at);
    }
    set(obs_tl, obs_stack.size());
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

  void apply_delay(int t) {
    // Make sure we're starting from a consistent state.
    if(obs_tl < obs_stack.size())
      restore_stack();
    if(goal_min < t) {
      obs_stack.push(constraint(pf::NUM_MOVES, -1, goal_min));
      if(goal_min > 0) {
        auto it = sctx[goal_pos].reach(goal_min);
        assert(it->start == goal_min);
        it->constraints &= ~pf::C_DELAY;
      }
      goal_min = t;
      set(obs_tl, obs_stack.size());
      auto& itv = sctx._create(goal_pos, t);
      itv.constraints |= pf::C_DELAY;
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
      apply_gen_barrier(o.b);
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
      int loc = incumbent[idx];
      if(step.first == o.timestep)
        loc = coord.nav.delta[loc][step.second];

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

  template<bool DEBUG = false>
  bool incumbent_conflicts_with(const vec<constraint>& constraints) {
    update_incumbent();
    const pf::Path& path(coord.get_path(agent_id));

    for(int ci = 0; ci < constraints.size(); ++ci) {
      const constraint& c = constraints[ci];
      
      int idx = get_path_index(path, c.time);
      pf::Step step(idx < path.size() ? path[idx] : std::make_pair(INT_MAX, pf::M_WAIT));

      int loc = incumbent[idx];
      if(step.first == c.time)
        loc = coord.nav.delta[loc][step.second];

      // First, check vertex constraints.
      if(c.move == pf::M_WAIT && loc == c.loc) {
        if(DEBUG)
          fprintf(stderr, "%% Conflicting vertex [%d].\n", ci);
        return true;
      }

      // Edge constraints only trigger at the moment of transition.
      if(step.first != c.time)
        continue;

      // Check forward direction. Started somewhere, moved
      // in constraint direction, landed at constraint location.
      if(path[idx].second == c.move && loc == c.loc) {
        if(DEBUG)
          fprintf(stderr, "%% Conflicting edge [%d].\n", ci);
        return true;
      }
      
      // Otherwise, we started from the constraint location,
      // and moved in the inverse direction.
      if(path[idx].second == pf::move_inv(c.move)
         && incumbent[idx] == c.loc) {
        if(DEBUG)
          fprintf(stderr, "%% Conflicting dual edge [%d].\n", ci);
        return true;
      }
    }
    return false;
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
    , goal_min(0)
    {
    cost.attach(E_UB, watch<&P::wake_cost>(0, Wt_IDEM));
    //cost.attach(E_LB, watch<&P::wake_cost_lb>(0, Wt_IDEM));
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
    coord.reset();
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

  int register_target(int loc, geas::intvar time, int threshold) {
    int ti = targets.size();
    targets.push(target_info { loc, time, threshold });
    target_map.insert(std::make_pair(loc, ti));
    time.attach(E_UB, watch<&P::wake_target>(ti, Wt_IDEM));
    return ti;
  }
  void tighten_target(int ti, int threshold) {
    if(threshold <= targets[ti].threshold) {
      debug_print_path(coord.get_path(agent_id));
    }
    assert(threshold > targets[ti].threshold);
    targets[ti].threshold = threshold;
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
    for(auto c : constraints) {
      make_reason_cell(c.loc, c.time);
      barriers[bi].constraints.push(c);
    }
    attach(s, at, watch<&P::wake_barrier>(bi, Wt_IDEM));
    return bi;
    /*
    // Add the new constraint to the pool
    barriers.push(obstacle_info(at, timestep, loc, move, duration));
    // And make sure the propagator wakes up when becomes set.
    attach(s, at, watch<&P::wake_obstacle>(ci, Wt_IDEM));
    return ci;
    */
  }

  bool propagate(vec<clause_elt>& confl) {
    has_bypass = 0;
    // assert(obs_stack.size() == obs_tl);
    assert(obs_tl <= obs_stack.size());
    if(obs_tl < obs_stack.size())
      restore_stack();

    /*
    if(agent_id==49)
      debug_print_path(coord.get_path(agent_id));
    */
    int c_lb = lb(cost);
    coord.hide_agent(agent_id);
    int pathC = coord.sipp_pf.search(start_pos, goal_pos,
                                     sctx, f_heur, coord.res_table.reserved);
    /*
    if(goal_min > 0)
      fprintf(stderr, "%% Agent %d arrived %d, earliest %d.\n", agent_id, pathC, goal_min);
    */
    if(pathC < c_lb) {
      fprintf(stderr, "%% Uh oh: c_lb(%d), pathC(%d).\n", c_lb, pathC);
      coord.sipp_pf.path.push(pf::Step(c_lb, pf::M_WAIT));
      pathC = c_lb;
    }
    if(pathC == INT_MAX) {
      coord.restore_agent(agent_id);

      // Failed to produce any path
      // TODO: Explanation
      fprintf(stderr, "%% PING!!\n");
      obstacle_atoms(obs_stack.size(), confl);
      return false;
    }
    //if(agent_id==49)
    //  debug_print_path(coord.sipp_pf.path);

    // Otherwise, update the lower bound.
    if(lb(cost) < pathC) {
      if(!set_lb(cost, pathC, expl<&P::expl_length>(obs_stack.size(), expl_thunk::Ex_BTPRED))) {
        coord.restore_agent(agent_id);
        return false;
      }
    }
    coord.restore_agent_with(agent_id, coord.sipp_pf.path);
    // assert(!incumbent_conflicts_with<true>(obs_stack));
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

  int goal_min;

  vec<obstacle_info> obstacles; // The registered obstacles, and other kinds of constraints
  vec<barrier_info> barriers;
  vec<target_info> targets;
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
  std::unordered_map<int, int> target_map;
};

}
#endif
