#include <lazycbs/mapf/coordinator.h>

namespace mapf {

void coordinator::reset(void) {
  // Ordering doesn't matter, so we just run left to right.
  for(int ii = changes_tl; ii < changes.size(); ++ii) {
    int ai = changes[ii];
    int origin = plans[ai].origin;
    vec<pf::Path>& hist(plans[ai].paths);
    res_table.release(nav, origin, hist.last());
    hist.pop();
    res_table.reserve(nav, origin, hist.last());
    plans[ai].path_num++;
  }
  changes.shrink(changes.size() - changes_tl);
}

int coordinator::add_agent(int origin, const pf::Path& initial_plan) {
  int ai = plans.size();
  plans.push(history(origin, initial_plan));
  res_table.reserve(nav, origin, initial_plan);
  return ai;
}

void coordinator::hide_agent(int agent) {
  res_table.release(nav, plans[agent].origin, plans[agent].paths.last());

}
void coordinator::restore_agent(int agent) {
  res_table.reserve(nav, plans[agent].origin, plans[agent].paths.last());
}

void coordinator::restore_agent_with(int agent, const pf::Path& new_plan) {
  plans[agent].path_num++;
  vec<pf::Path>& hist(plans[agent].paths);
  res_table.reserve(nav, plans[agent].origin, new_plan);
  if(pf::path_cost(hist.last()) < pf::path_cost(new_plan)) {
    hist.push();
    changes.push(agent);
    changes_tl.set(s.data->persist, changes.size());
  }
  new_plan.copyTo(hist.last());
}


void coordinator::update_plan(int agent, const pf::Path& new_plan) {
  hide_agent(agent);
  restore_agent_with(agent, new_plan);
};

}
