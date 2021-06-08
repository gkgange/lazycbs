#ifndef LAZYCBS__PF__HPP
#define LAZYCBS__PF__HPP

#include <geas/mtl/Vec.h>
#include <geas/mtl/p-sparse-set.h>
// #include <lazycbs/support/graph.h>
//#include <lazycbs/support/trieset.h>
#include <lazycbs/utils/intrusive-heap.h>

#include <lazycbs/pf/pf.hh>

namespace mapf {

int nav_push(navigation& n) {
  int idx = n.succ.size();

  n.succ.push();
  n.pred.push();
  n.delta.push();
  n.delta.last().dest[pf::M_WAIT] = idx;
  n.inv.push();
  n.inv.last().dest[pf::M_WAIT] = idx;
  return idx;
}
void nav_add(navigation& n, int src, pf::Move m, int dest) {
  n.succ[src].push(std::make_pair(m, dest));
  n.pred[dest].push(std::make_pair(m, src));
  n.delta[src].dest[m] = dest;
  n.inv[dest].dest[m] = src;
}

template<class T>
navigation navigation::of_obstacle_array(int width, int height, const T& array) {
  vec<int> prev_row(width);
  vec<int> curr_row(width);

  navigation nav;
  if(!array[0])
    curr_row[0] = nav_push(nav);

  for(int c = 1; c < width; ++c) {
    if(!array[c]) {
      curr_row[c] = nav_push(nav);
      if(!array[c-1]) {
        // Add the transitions.
        nav_add(nav, curr_row[c-1], pf::M_RIGHT, curr_row[c]);
        nav_add(nav, curr_row[c], pf::M_LEFT, curr_row[c-1]);
      }
    }
  }

  int prev_idx = 0;
  int idx = width;
  for(int r = 1; r < height; ++r) {
    std::swap(curr_row, prev_row);

    if(!array[idx]) {
      curr_row[0] = nav_push(nav);
      if(!array[prev_idx]) {
        nav_add(nav, prev_row[0], pf::M_DOWN, curr_row[0]);
        nav_add(nav, curr_row[0], pf::M_UP, prev_row[0]);
      }
    }
    ++idx, ++prev_idx;
    for(int c = 1; c < width; ++c, ++idx, ++prev_idx) {
      if(!array[idx]) {
        curr_row[c] = nav_push(nav);
        if(!array[idx-1]) {
          nav_add(nav, curr_row[c-1], pf::M_RIGHT, curr_row[c]);
          nav_add(nav, curr_row[c], pf::M_LEFT, curr_row[c-1]);
        }
        if(!array[prev_idx]) {
          nav_add(nav, prev_row[c], pf::M_DOWN, curr_row[c]);
          nav_add(nav, curr_row[c], pf::M_UP, prev_row[c]);
        }
      }
    }
  }

  return nav;
}

#if 0
template<class Ctx, class Reason>
Interval* PF<Ctx, Reason>::Loc::reach(unsigned t) const {
  // Binary search.
  Interval* low(begin());
  Interval* high(end()-1);
  while(low < high) {
    Interval* mid(low + (high - low + 1)/2);
    if(mid->start <= t) {
      low = mid;
    } else {
      high = mid-1;
    }
  }
  return low;
}

template<class Ctx, class Reason>
bool PF<Ctx,Reason>::search(void) {
  Loc& goal(get(goal_loc));
  unsigned goal_idx(goal.size()-1);
  // Find the earliest interval where we can wait forever.
  assert(! (goal[goal_idx-1].constraints & C_VERT) );
  while(goal_idx > 0) {
    if(goal[goal_idx-1].constraints & C_VERT)
      break;
    --goal_idx;
  }
  
  enqueue(IntId(source_loc, 0));
  while(!heap.empty()) {
    IntId curr(heap.top()); heap.pop();

    // Check if we're at the goal.
    if(curr.loc == goal && curr.idx >= goal_idx)
      return true;

    // Otherwise, look at the possible successors.
    Interval* i_curr(get(curr.loc).begin() + curr.idx);
    unsigned t(i_curr->next_ex);
    unsigned t_close( (i_curr+1)->start );
    unsigned t_new(UINT_MAX);

    // Look at the possible wait successor.
    for(auto p : Ctx::successors(c.loc)) {
      // p ~ (constraints, loc)
      Loc& succ(get(p.second));
      auto it(succ.reach(t+1));
      // reach finds a lower bound.
      assert(it->start <= t);
      if(! (p.first & it->constraints) ) {
        if(t < it->reach) {
          // Better arrival time at this interval.
          it->reach = t;
          enqueue(p.second, it - succ.begin());
        }
        t_new = std::min(t_new, (it+1)->start);
      } else {
        auto en(succ.end());
        for(++it; it != en; ++it) {
          if(! (p.first & it->constraints) ) {
            // Needed to wait until the next interval.
            if(it->start < it->reach) {
              it->reach = it->start;
              enqueue(p.second, it - succ.begin());
            }
            t_new = std::min(t_new, (it+1)->start);
          }
        }
      }
    }
    // Only queue the wait successor if we won't need to
    // wait for other successors.
    if(t_new < t_close) {
      i_curr->next_ex = t_new;
      enqueue(curr.loc, curr.idx);
    } else {
      // Finished expanding this interval.
      i_curr->next_ex = INT_MAX;
      ++i_curr;
      if(! (i_curr->constraints & C_VERT)
         && i_curr->start < i_curr->reach) {
        i_curr->reach = i_curr->start;
        enqueue(curr.loc, curr.idx+1);
      }
    }
  }
  // Unreachable.
  return false;
}

// Explanation follows the typical pattern; flood-fill
// backward to mark reverse reachability, then forward to collect the reason,
// maintaining separation between the forward and reverse reachable
// segments.
// Abuses next_ex to record the latest feasible arrival time.
template<class Ctx, class Reason>
void PF<Ctx,Reason>::mark_reach_pred(unsigned t) {
  while(!heap.empty()) {
    IntId curr(heap.top()); heap.pop();
    Loc& loc(get(curr.loc));

    int t_early = loc[curr.idx].start-1;
    int t_late = loc[curr.idx].reach-1;
    // If we can get to the target from some point in this interval,
    // and were allowed to wait, we can definitely get to the target from
    // the end of the previous interval.
    if(curr.idx > 0 && !(get(curr)[curr.idx-1].constraints & C_VERT)) {
      enqueue_rev(curr.loc, curr.idx-1, t_early);
    }
    for(auto p : Ctx::predecessors(curr.loc)) {
      Loc& pred(get(p.first));
      // Work backward from the end of this interval (so we can
      // easily deal with wait slides).
      auto p_it = pred.reach(t_late);
      while(t_early <= p_it->start) {
        int t_next = (p_it+1)->start;
        if(p_it->constraints & p.second) {
          t_late = p_it->start;
          --p_it;
          continue;
        }
        // We can reach curr if we leave pred at time t_late.
        if(t_late <= p_it->next_ex)
          break;
        // Now mark any wait-reachable things we haven't already
        // reached.
        p_it->next_ex = t_late;
        t_late = p_it->start;
        --p_it;
        while(t_early <= p_it->start) {
          if(p_it->constraints & C_VERT)
            break;
          if(t_late <= p_it->next_ex)
            break;
          // This interval is allowed to slide
          // through to the next.
          p_it->next_ex = t_late;
          enqueue_rev(p.first, idx, t_late);
          t_late = p_it->start;
          --p_it;
        }
      }
      assert(pred[idx].start < t_late);
      if(! (p.first & pred[idx].constraints) ) {
        enqueue_rev(p.first, idx, t_late);
      }
    }
  }
}

template<class Ctx, class Reason, class Expl>
void PF<Ctx,Reason>::collect_expl(unsigned t, Expl& expl) {
  // Now, work forward from the start location.
  // Initial location shouldn't be able to reach the goal.
  assert(get(origin)[0].next_ex < 0);
  enqueue(origin, 0);

  while(!heap.empty()) {
    IntId curr(heap.top()); heap.pop();

    Loc& loc(get(curr.loc));
    auto it(loc.begin() + curr.idx);
    auto en(loc.end());

    for(; it != en; ++it) {
      if(it->reach <= it->next_ex) {
        // If we were allowed to traverse this interval,
        // we would have a path to the goal.
        assert(it->constraints & C_VERT);
        expl.push(Ctx::reason(it, C_VERT));
        break;
      }
      // Otherwise, we have to look at the adjacent nodes.
      unsigned t_step = it->reach+1;
      unsigned t_end = (it+1)->start;
      for(auto p : Ctx::successors(curr.loc)) {
        Loc& succ(get(p.second));
        // We don't need to expand if we could never
        // have reached here in time.
        if(t_step + succ.heur > t)
          continue;
        auto succ_it(succ.reach(t_succ));
        unsigned t_succ = t_step;
        for(; t_succ < t_end; ++suc_it, t_succ = succ_it->start) {
          if(t_succ < succ_it->reach) {
            if(t_succ < succ_it->next_ex
               && !(succ_it->constraints & C_VERT)) {
              // Need to use the edge constraint.
              assert(succ_it->constraints & p.first);
              expl.push(Ctx::reason(succ_it, succ_it->constraints & p.first));
            }
          }
          ++succ_it;
          t_succ = succ_it->start;
        }
      }
    }

    if(ex_marked(get(curr)[curr.idx]+1)) {
      // If we expanded this node, and the successor is marked,
      // we need 
      expl.push(reason(curr.loc, curr.idx, C_VERT));
    }
  }
}
template<class Ctx, class Reason, class Expl>
void PF<Ctx,Reason>::explain<Expl>(unsigned t, Expl& expl) {

};
#endif

}
#endif
