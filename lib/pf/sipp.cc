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
  : Tend(INT_MAX-1), timestamp(-1) {
  i.push(sipp_interval(0));
  i.push(sipp_interval(INT_MAX-1, pf::C_VERT));
}

sipp_ctx::sipp_ctx(int size)
  : ptr(new sipp_loc[size]), timestamp(0)
{ }


void sipp_ctx::forbid(pf::Move m, unsigned loc, int time) {
  sipp_interval& itv(_create(loc, time));
  itv.constraints |= (1 << m);
}
void sipp_ctx::lock(unsigned loc, int time) {
  ptr[loc].Tend = time;
}
void sipp_ctx::unlock(unsigned loc) {
  ptr[loc].Tend = INT_MAX;
}

bool sipp_ctx::release(pf::Move m, unsigned loc, int time) {
  auto it = ptr[loc].reach(time);
  assert(it->start == time);
  it->constraints &= ~(1 << m);
  if(it->constraints)
    return true;

  // Maybe delete this node.
  return false;
}

template<class T>
void shuffle_right(vec<T>& vec, int idx, int count) {
  int pos = vec.size()-1;
  T pad = vec.last();
  for(int ii = 0; ii < count; ++ii)
    vec.push(pad);
  int dest = vec.size()-1;

  for(; pos >= idx; --pos, --dest)
    vec[dest] = vec[pos];
}

sipp_interval& sipp_ctx::_create(unsigned loc, int time) {
  // Surely there's a better way than this.
  auto it = ptr[loc].reach(time);
  int idx = it - ptr[loc].begin();
  if(it->start == time) {
    if( (it+1)->start == time+1 ) {
      return *it;
    } else {
      assert( (it+1)->start > time+1 );
      shuffle_right(ptr[loc].i, idx+1, 1);
      ptr[loc][idx+1] = sipp_interval(time+1);
      return ptr[loc][idx];
    }
  } else {
    if( (it+1)->start == time+1) {
      shuffle_right(ptr[loc].i, idx+1, 1);
      ptr[loc][idx+1] = sipp_interval(time);
      return ptr[loc][idx+1];
    } else {
      shuffle_right(ptr[loc].i, idx+1, 2);
      ptr[loc][idx+1] = sipp_interval(time);
      ptr[loc][idx+2] = sipp_interval(time+1);
      return ptr[loc][idx+1];
    }
  }
}

// Possible improvement: keep a residual marker as
// a the initial mid.
sipp_interval* sipp_loc::reach(int t) const {
  auto b(begin());
  auto en(end());
  while(b != en) {
    auto mid = b + (en - b)/2;
    if(mid->start > t) {
      en = mid;
    } else if( (mid+1)->start <= t) {
      b = mid+1;
    } else {
      return mid;
    }
  }
  // This should never be reachable.
  assert(0);
}

int sipp_pathfinder::search(int origin, int goal, sipp_ctx& ctx, int* heur,
                            const constraints& reserved) {
  ++LL_searches;

  // Set up parameters
  heap.clear();
  state = ctx.ptr;
  heap.env.ctx = ctx.ptr;
  heap.env.heur = heur;
  timestamp = ++ctx.timestamp;

  // Find the goal index.
  int goal_idx = find_goal_index(get(goal));

  sipp_loc& origin_loc(get(origin));
  origin_loc[0].reach = origin_loc[0].next_ex = 0;
  origin_loc[0].pred = pf::M_WAIT;
  origin_loc[0].second = 0;
  heap.insert(IntId(origin, 0));

  while(!heap.empty()) {
    IntId curr = heap.top();
    heap.pop();
    ++LL_expanded;

    // Don't need to use get(), because curr.loc should already
    // be initialized.
    sipp_loc& curr_loc(state[curr.loc]);
    sipp_interval& curr_itv(curr_loc[curr.idx]);
    // fprintf(stderr, "%% Popping: %d : %d (%d).\n", curr.loc, curr.idx, curr_itv.reach);

    // Check for goal location.
    // We do it here, in case we might arrive at the goal
    // along one interval, then later by an earlier one.
    if(curr.loc == goal && curr.idx >= goal_idx) {
      // Reconstruct the corresponding path
      path.clear();

      int path_loc = curr.loc;
      sipp_interval* path_step(&curr_itv);
      while(path_step->reach > 0) {
        path.push(std::make_pair(path_step->reach, path_step->pred));
        path_loc = nav.inv[path_loc].dest[path_step->pred];
        path_step = state[path_loc].reach(path_step->reach-1);
      }
      std::reverse(path.begin(), path.end());
      return curr_itv.reach;
    }

    int t = curr_itv.next_ex + 1;
    int max_ex = 1 + std::min(curr_loc[curr.idx+1].start, curr_loc.Tend);
    int next_ex = max_ex;
    unsigned char second = curr_itv.second;
    for(auto p : nav.successors(curr.loc)) {
      sipp_loc& succ(get(p.second));
      if(succ.Tend <= t)
        continue;
      sipp_interval* succ_it(succ.reach(t));
      // Expand only the first open adjacent sibling,
      // if any.
      do {
        int succ_t = t;
        int Tmax = std::min(max_ex, succ.Tend);
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
        unsigned char p_second = std::min(127, second + reserved.score(p.first, p.second, succ_t));
        if(succ_t < succ_it->reach
           || (succ_t == succ_it->reach && p_second < succ_it->second)) {
          succ_it->reach = succ_it->next_ex = succ_t;
          succ_it->second = p_second;
          succ_it->pred = p.first;
          IntId succ_id(p.second, succ_it - succ.begin());
          heap.insert_or_decrease(succ_id);
        }
        if( (succ_it+1)->start < INT_MAX-1 )
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
      // FIXME: Deal with conflicts on waits.
      sipp_interval& wait(curr_loc[curr.idx+1]);
      unsigned char w_second = std::min(127, second + reserved.score(pf::M_WAIT, curr.loc, wait.start));
      if(wait.reach < curr_loc.Tend
         && (wait.start < wait.reach || w_second < wait.second)
         && !wait.may_wait()) {
        IntId wait_id(curr.loc, curr.idx+1);
        wait.pred = pf::M_WAIT;
        wait.reach = wait.next_ex = wait.start;
        wait.second = w_second;
        heap.insert_or_decrease(wait_id);
      }
    }
  }
  return INT_MAX;
}

// UGH. This is slightly awkward. For this we really do want to take into account
// conflicts which cross an interval.
#if 0
bool sipp_pathfinder::search_upto(int origin, int goal, int cost_ub, sipp_ctx& ctx, int* heur,
                            const constraints& reserved) {
  ++LL_searches;

  // Set up parameters
  heap.clear();
  state = ctx.ptr;
  heap.env.ctx = ctx.ptr;
  heap.env.heur = heur;
  timestamp = ++ctx.timestamp;

  // Find the goal index.
  int goal_idx = find_goal_index(get(goal));
  if(get(goal)[goal_idx].start > cost_ub)
    return false;

  assert(0 && "TODO: Implement sipp_pathfinder::search_upto");
  return false;
}
#endif

void sipp_explainer::explain(int origin, int goal, int Tub, sipp_ctx& state,
                             int* heur, int* r_heur, vec<cst>& out_expl) {
  ctx = state.ptr;
  timestamp = ++state.timestamp;
  
  mark_heap.clear();
  mark_heap.env.ctx = state.ptr;

  // Find the starting time.
  sipp_loc& goal_loc(get(goal));
  sipp_interval* goal_itv(goal_loc.reach(Tub));

  goal_itv->next_ex = Tub;
  mark_heap.insert(IntId(goal, goal_itv - goal_loc.begin()));
  
  while(!mark_heap.empty()) {
    IntId curr(mark_heap.top());
    mark_heap.pop();

    sipp_loc& curr_loc(state[curr.loc]);
    sipp_interval&  curr_itv(curr_loc[curr.idx]);
    // We have to process predecessors of each interval separately
    // anyway, so we'll just process one interval at a time.
    // TODO: Deal with r_heur.
    if(! (curr_itv.constraints & pf::C_VERT) ) {
      int t_start = curr_itv.start;
      int t_reach = std::min(curr_itv.next_ex, curr_loc.Tend-1);

      if(curr.idx > 0 && curr_loc[curr.idx-1].next_ex < t_start-1) {
        curr_loc[curr.idx-1].next_ex = t_start-1;
        mark_heap.insert_or_decrease(IntId(curr.loc, curr.idx-1));
      }
      // Now look at any predecessors over the range [t_start-1, t_reach-1].
      for(auto p : nav.predecessors(curr.loc)) {
        if(!curr_itv.is_allowed(p.first))
          continue;
        sipp_loc& pred_loc(get(p.second));
        // Process the predecessor.
        int Tmin = std::max(r_heur[p.second], t_start-1);
        int pred_reach = t_reach-1;
        if(Tmin > pred_reach)
          continue;

        auto pred_it = pred_loc.reach(pred_reach);
        while(Tmin < pred_it->start+1) {
          if(pred_reach > pred_it->next_ex) {
            pred_it->next_ex = pred_reach;
            mark_heap.insert_or_decrease(IntId(p.second, pred_it - pred_loc.begin()));
          }
          pred_reach = pred_it->start-1;
          --pred_it;
        }
        // Final interval.
        if(pred_reach > pred_it->next_ex) {
          pred_it->next_ex = pred_reach;
          mark_heap.insert_or_decrease(IntId(p.second, pred_it - pred_loc.begin()));
        }
      }
    }
  }

  collect_heap.clear();
  collect_heap.env.ctx = state.ptr;

  get(origin)[0].reach = 0;
  collect_heap.insert(IntId(origin, 0));

  while(!collect_heap.empty()) {
    IntId curr(collect_heap.top());
    collect_heap.pop();

    sipp_loc& curr_loc(state[curr.loc]);
    int t_start = curr_loc[curr.idx].reach;
    int t_end = t_start-1;
    int Tmax = Tub - heur[curr.loc];
    auto curr_it(curr_loc.begin() + curr.idx);
    auto curr_en(curr_loc.end());
    // Are we already infeasible?
    if(t_start + heur[curr.loc] > Tub)
      continue;

    int idx = curr.idx;
    while(curr_it != curr_en) {
      if(t_start <= curr_it->next_ex) {
        // We would have a feasible transition here, so either a
        // vertex constraint or target constraint is necessary.
        if(curr_loc.Tend <= t_start) {
          // Prefer target constraints.
          out_expl.push(cst(curr.loc, t_start, M_LOCK));
        } else {
          assert(curr_it->constraints & pf::C_VERT);
          out_expl.push(cst(curr.loc, t_start, pf::M_WAIT));
        }
        break;
      }
      ++curr_it;
      ++idx;
      curr_it->reach = curr_it->start;
      t_end = curr_it->start-1;
      // Every [curr] interval we hit is at its minimal
      // time, so process it now.
      if(collect_heap.in_heap(IntId(curr.loc, idx)))
        collect_heap.remove(IntId(curr.loc, idx));
    }
    if(t_end < t_start)
      continue;
    // We've identified the feasible range for curr, now
    // check the reachable successors.
    for(auto p : nav.successors(curr.loc)) {
      sipp_loc& succ_loc(get(p.second));
      int t_succ = t_start+1;
      auto succ_it(succ_loc.reach(t_succ));
      while(succ_it->start < t_end) {
        if(t_succ < succ_it->reach) {
          // We only want to take an edge conflict if we can't
          // handle it with a vertex or target constraint.
          if(t_succ < succ_it->next_ex
             && t_succ < succ_loc.Tend
             && !(succ_it->constraints & pf::C_VERT)) {
            assert(succ_it->constraints & (1 << p.first));
            out_expl.push(cst(p.second, t_succ, p.first));
          } else {
            succ_it->reach = t_succ;
            collect_heap.insert_or_decrease(IntId(p.second, succ_it - succ_loc.begin()));
          }
        }
        ++succ_it;
        t_succ = succ_it->start;
      }
    }
  }
}

};

