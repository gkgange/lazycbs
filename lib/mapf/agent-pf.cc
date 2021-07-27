#include <lazycbs/mapf/agent-pf.h>
// #define EXPLAIN_LATEST

namespace mapf {

void _apply_cst(const navigation& nav, sipp_ctx& ctx, int loc, int time, pf::Move m) {
  ctx._create(loc, time).constraints |= 1 << m;
  if(m != pf::M_WAIT)
    ctx._create(nav.inv[loc][m], time).constraints |= 1 << pf::move_inv(m);
}

bool Agent_PF::check_sat(ctx_t& ctx) {
  // Create a fresh SIPP context, apply
  // any constraints which are enforced.
  sipp_ctx check_ctx(coord.nav.size());
  
  for(const obstacle_info& o : obstacles) {
    if(o.at.lb(ctx)) {
      // Barriers are now a separate thing.
      assert(o.tag == O_MUTEX);
      _apply_cst(coord.nav, check_ctx, o.p.loc, o.timestep, o.p.move);
    }
  }

  for(const barrier_info& b : barriers) {
    if(!b.at.lb(ctx))
      continue;
    for(constraint c : b.constraints)
      _apply_cst(coord.nav, check_ctx, c.loc, c.time, c.move);
  }

  for(const target_info& t : targets) {
    /*
    if(t.time.ub(ctx) > t.threshold)
      continue;
    */
    check_ctx[t.loc].Tend = t.time.ub(ctx);
  }

  int Tlb = cost.lb(ctx);
  check_ctx._create(goal_pos, Tlb).constraints |= pf::C_DELAY;

  int Tub = cost.ub(ctx);
  int pathC = coord.sipp_pf.search(start_pos, goal_pos,
                                   check_ctx, f_heur, coord.res_table.reserved);
  return pathC <= Tub;
}


}
