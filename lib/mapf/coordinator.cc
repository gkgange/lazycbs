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
  int block = geas::B32::block(ai);

  // Every agent starts off active.
  if(active_agents.size() <= block)
    active_agents.push(0);
  if(!active_agents[block])
    active_words.push(block);
  active_agents[block] |= geas::B32::bit(ai);

  return ai;
}

void coordinator::clear_active(void) {
  for(int w : active_words)
    active_agents[w] = 0;
  active_words.clear();
}

void coordinator::make_active(const agent_cell* b, const agent_cell* e) {
  for(; b != e; ++b) {
    if(!active_agents[b->block])
      active_words.push(b->block);
    active_agents[b->block] |= b->mask;
  }
}
void coordinator::rem_active(const agent_cell* b, const agent_cell* e) {
  for(; b != e; ++b) {
    active_agents[b->block] &= ~b->mask;
  }
}

void coordinator::dump_active(vec<agent_cell>& out) {
  for(int w : active_words) {
    if(!active_agents[w])
      continue;
    out.push(agent_cell(w, active_agents[w]));
    active_agents[w] = 0;
  }
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
