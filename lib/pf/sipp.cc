#include <cstdio>
#include <algorithm>
#include <lazycbs/pf/sipp.hh>

namespace mapf {

// Find the index just after the last Vertex blockage.
int find_goal_index(sipp_loc& loc) {
  int idx = loc.size()-2;
  while(idx > 0 && !loc[idx].may_wait())
    --idx;
  return idx;
}

sipp_loc::sipp_loc(void)
  : Tend(INT_MAX), timestamp(-1) {
  i.push(sipp_interval(0));
  i.push(sipp_interval(INT_MAX, pf::C_VERT));
}

sipp_ctx::sipp_ctx(const navigation& nav)
  : ptr(new sipp_loc[nav.size()])
{ }

// Possible improvement: keep a residual marker as
// a the initial mid.
sipp_interval* sipp_loc::reach(unsigned t) const {
  auto b(begin());
  auto en(end());
  while(b != en) {
    auto mid = b + (en - b)/2;
    if(mid->start > t) {
      en = b;
    } else if( (mid+1)->start <= t) {
      b = mid+1;
    } else {
      return mid;
    }
  }
  // This should never be reachable.
  assert(0);
}

unsigned sipp_pathfinder::search(int origin, int goal, sipp_loc* ctx, int* heur) {
  // Set up parameters
  heap.clear();
  state = ctx;
  heap.env.ctx = ctx;
  heap.env.heur = heur;
  timestamp++;

  // Find the goal index.
  int goal_idx = find_goal_index(get(goal));

  sipp_loc& origin_loc(get(origin));
  origin_loc[0].reach = origin_loc[0].next_ex = 0;
  origin_loc[0].pred = pf::M_WAIT;
  heap.insert(IntId(origin, 0));

  while(!heap.empty()) {
    IntId curr = heap.top();
    heap.pop();
    // Don't need to use get(), because curr.loc should already
    // be initialized.
    sipp_loc& curr_loc(state[curr.loc]);
    sipp_interval& curr_itv(curr_loc[curr.idx]);
    // fprintf(stderr, "%% Popping: %d : %d (%d).\n", curr.loc, curr.idx, curr_itv.reach);

    // Check for goal location.
    // We do it here, in case we might arrive at the goal
    // along one interval, then later by an earlier one.
    if(curr.loc == goal && curr.idx >= goal_idx)
      return curr_itv.reach;

    unsigned t = curr_itv.next_ex + 1;
    unsigned max_ex = 1 + std::min(curr_loc[curr.idx+1].start, curr_loc.Tend);
    unsigned next_ex = max_ex;
    for(auto p : nav.successors(curr.loc)) {
      sipp_loc& succ(get(p.second));
      if(succ.Tend <= t)
        continue;
      sipp_interval* succ_it(succ.reach(t));
      // Expand only the first open adjacent sibling,
      // if any.
      do {
        unsigned succ_t = t;
        unsigned Tmax = std::min(max_ex, succ.Tend);
        if(!succ_it->is_allowed(p.first)) {
          for(++succ_it; succ_it->start < Tmax; ++succ_it) {
            if(!succ_it->is_allowed(p.first))
              continue;
            succ_t = succ_it->start;
            goto found_open_succ;
          }
          break;
        }
      found_open_succ:
        if(t < succ_it->reach) {
          succ_it->next_ex = succ_t;
          succ_it->pred = p.first;
          IntId succ_id(p.second, succ_it - succ.begin());
          if(succ_it->reach < INT_MAX) { // Already in heap
            heap.decrease(succ_id);
          } else {
            heap.insert(succ_id);
          }
          succ_it->reach = succ_t;
        }
        if( (succ_it+1)->start < INT_MAX )
          next_ex = std::min(next_ex, (succ_it+1)->start - 1);
      } while(0);
    }

    // Deal with wait successors.
    if(next_ex < max_ex) {
      // Partial expansion.
      curr_itv.next_ex = next_ex;
      heap.insert(curr);
    } else {
      // We _should_ still be the top.
      sipp_interval& wait(curr_loc[curr.idx+1]);
      if(wait.reach < curr_loc.Tend
         && wait.start < wait.reach
         && !wait.may_wait()) {
        IntId wait_id(curr.loc, curr.idx+1);
        wait.pred = pf::M_WAIT;
        if(wait.reach < INT_MAX) { // Already in the heap
          wait.reach = wait.next_ex = wait.start;
          heap.decrease(wait_id);
        } else {
          wait.reach = wait.next_ex = wait.start;
          heap.insert(wait_id);
        }
      }
    }
  }
  return UINT_MAX;
}

};

