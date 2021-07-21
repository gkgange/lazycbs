#ifndef LAZYCBS__COORDINATOR__H
#define LAZYCBS__COORDINATOR__H

#include <geas/utils/bitops.h>
#include <geas/solver/solver.h>
#include <geas/engine/propagator.h>
#include <lazycbs/pf/pf.hh>
#include <lazycbs/pf/sipp.hh>

namespace mapf {

// The coordinator is responsible for tracking
// incumbent paths and updating the reservation table.
class coordinator {
 public:
  struct agent_cell {
    agent_cell(int _block, uint32_t _mask)
    : block(_block), mask(_mask) { }
    agent_cell(int ai)
    : block(geas::B32::block(ai)), mask(geas::B32::bit(ai)) { }

    int block;
    uint32_t mask;
  };
  
 coordinator(geas::solver& _s, const navigation& _nav)
   : s(_s), nav(_nav), res_table(nav.size())
    , sipp_pf(nav), sipp_ex(nav)
    , changes_tl(0) { }

  struct history {
    history(int _origin, const pf::Path& init_path)
    : origin(_origin), path_num(1) {
      paths.push(init_path);
    }

    // The time of the final step.
    int cost(void) const {
      pf::Step last_step = paths.last().last();
      return last_step.first;
    }

    int path_num;
    int origin;
    vec<pf::Path> paths;
  };

  inline bool is_active(int agent) const {
    return active_agents[geas::B32::block(agent)] & geas::B32::bit(agent);
  }

  int get_origin(int agent) const { return plans[agent].origin; }
  const pf::Path& get_path(int agent) const { return plans[agent].paths.last(); }

  int add_agent(int origin, const pf::Path& initial_plan);
  void update_plan(int agent, const pf::Path& new_plan);

  // Manipulating the reservation table.
  void hide_agent(int agent);
  void restore_agent(int agent);
  void restore_agent_with(int agent, const pf::Path& new_plan);

  void make_active(const agent_cell* b, const agent_cell* e);
  void rem_active(const agent_cell* b, const agent_cell* e);
  void clear_active(void);
  void dump_active(vec<agent_cell>& out);
  
  geas::solver& s;

  vec<uint32_t> active_agents;
  vec<int> active_words;
  
  const navigation& nav;
  reservation::table res_table;

  sipp_pathfinder sipp_pf;
  sipp_explainer sipp_ex;

  // Restore consistency.
  void reset(void);
  
  vec<history> plans;
  vec<int> changes;
  geas::Tint changes_tl;
};

};

#endif
