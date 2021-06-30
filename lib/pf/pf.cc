#include <algorithm>
#include <climits>
#include <lazycbs/pf/pf.hh>

namespace mapf {

static pf::Move move_inv[] = {
  pf::M_LEFT,
  pf::M_RIGHT,
  pf::M_UP,
  pf::M_DOWN,
  pf::M_WAIT,
  pf::NUM_MOVES
};

void constraints::forbid(pf::Move m, int loc, unsigned t) {
  auto& mask(masks[loc]);
  auto it = mask.find(t);
  if(it) {
    (*it).value |= (1 << m);
  } else {
    mask.add(t, 1 << m);
  }
}

bool constraints::release(pf::Move m, int loc, unsigned t) {
  auto& mask(masks[loc]);
  auto it = mask.find(t);
  assert(it);
  char bits = (*it).value & ~(1 << m);
  if(bits) {
    (*it).value = bits;
    return true;
  } else {
    // (*it).value = 0; // FIXME
    mask.rem(t); 
    return false;
  }
}

void constraints::lock(int loc, unsigned t) {
  lock_time[loc] = t;
}

int* navigation::fwd_heuristic(int dest) const {
  int* heur = new int[succ.size()];
  vec<int> queue;
  for(int l = 0; l < succ.size(); ++l)
    heur[l] = INT_MAX;

  heur[dest] = 0;
  queue.push(dest);

  for(int ii = 0; ii < queue.size(); ++ii) {
    int loc = queue[ii];
    int pred_dist = heur[loc]+1;
    for(auto p : predecessors(loc)) {
      if(pred_dist < heur[p.second]) {
        heur[p.second] = pred_dist;
        queue.push(p.second);
      }
    }
  }
  return heur;
}

int* navigation::rev_heuristic(int origin) const {
  int* heur = new int[succ.size()];
  vec<int> queue;
  for(int l = 0; l < succ.size(); ++l)
    heur[l] = INT_MAX;

  heur[origin] = 0;
  queue.push(origin);

  for(int ii = 0; ii < queue.size(); ++ii) {
    int loc = queue[ii];
    int succ_dist = heur[loc]+1;
    for(auto p : successors(loc)) {
      if(succ_dist < heur[p.second]) {
        heur[p.second] = succ_dist;
        queue.push(p.second);
      }
    }
  }
  return heur;
}

pf::Move navigation::move_dir(int src, int dest) const {
  for(auto p : successors(src)) {
    if(p.second == dest)
      return p.first;
  }
  assert(src == dest);
  return pf::M_WAIT;
}

namespace reservation {
table::table(int map_sz)
  : res_counts(map_sz)
  , reserved(map_sz)
{

}

void table::_dec_move(int loc, int t, pf::Move m) {
  auto& cell = (*res_counts[loc].find(t)).value;
  if(!--cell.counts[m]) {
    // Clear a bit.
    if(!reserved.release(m, loc, t))
      res_counts[loc].rem(t);
  }
}

void table::_inc_move(int loc, int t, pf::Move m) {
  auto cell_it = res_counts[loc].find(t);
  if(cell_it == res_counts[loc].end()) {
    res_cell c;
    c[m]++;
    res_counts[loc].add(t, c);
    reserved.forbid(m, loc, t);
  } else {
    if(! (*cell_it).value[m]++ ) {
      reserved.forbid(m, loc, t);
    }
  }
}

// Used for temporarily suppressing an agent's plan during pathfinding.
void table::release(const navigation& nav, int origin, const pf::Path& path) {
  int prev = origin;
  for(pf::Step step : path) {
    int t = step.first;
    int curr = nav.delta[prev][step.second];
    // Both c_cell and p_cell should exist.
    _dec_move(curr, t, pf::M_WAIT);
    if(prev != curr)
      _dec_move(prev, t, move_inv[step.second]);
    prev = curr;
  }
}

void table::reserve(const navigation& nav, int origin, const pf::Path& path) {
  int prev = origin;
  for(pf::Step step : path) {
    int t = step.first;
    int curr = nav.delta[prev][step.second];
    // Both c_cell and p_cell should exist.
    _inc_move(curr, t, pf::M_WAIT);
    if(prev != curr)
      _inc_move(prev, t, move_inv[step.second]);
    prev = curr;
  }
}

}

/*
void explainer::mark(int tMax, const constraints& state, constraints& transitions) {
  int t = tMax;
  current.insert(goal);

  while(!current.empty()) {
    for(int loc : current) {
      if(!state.is_allowed(M_WAIT, loc, t)) {
        transitions.forbid(M_WAIT, loc, t);
        continue;
      }

      for(auto p : predecessors(loc)) {
        if(t <= rev_heur[p.second])
          continue;
        if(state.is_allowed(p.first, loc, t)) {
          pred.insert(p.second);
        } else {
          transitions.forbid(p.first, loc, t);
        }
      }
    }

    --t;
    current.clear();
    std::swap(current, pred);
  }
}

}

void explainer::collect(int tMax, constraints& transitions, vec<cst> reason) {
  int t = 0;
  current.insert(origin);

  while(!current.empty()) {
    for(int loc : current) {
      if(!transitions.is_allowed(M_WAIT, loc, t)) {
        reason.push(cst(loc, t, M_WAIT));
        continue;
      }
      // Otherwise, expand the children
      for(auto p : successors(loc)) {
        if(t + fwd_heur[p.second] >= tMax)
          continue;
        if(transitions.is_allowed(p.first, p.second, t+1)) {
          pred.insert(p.second);
        } else {
          reason.push(cst(p.second, t+1, p.first));
        }
      }
    }
  }
}
*/

simple_pathfinder::cell_ref simple_pathfinder::get(int loc, unsigned t) {
  cell_map& m(cells[loc]);
  int base = t & ~(CELL_SIZE-1);
  unsigned off = t & (CELL_SIZE-1);
  auto it = m.find(base);
  if(!it) {
    it = m.add(base, cell(loc, base, timestamp));
  } else {
    (*it).value.reset(timestamp);
  }
  return cell_ref(&(*it).value, off);
}

template<class Heap>
void add_or_decrease(Heap& h, simple_pathfinder::cell_ref ref) {
  if(h.in_heap(ref))
    h.decrease(ref);
  else
    h.insert(ref);
}

int simple_pathfinder::search(int origin, int goal, int* heur,
                              const constraints& csts, const constraints& reservation) {
  // Set up transient state.
  ++timestamp;
  heap.env.heur = heur;

  unsigned goal_time = 0;
  for(auto ref : csts.masks[goal]) {
    if(ref.value & pf::C_VERT) {
      assert(ref.key >= goal_time);
      goal_time = ref.key+1;
    }
  }

  cell_ref origin_cell(get(origin, 0));
  origin_cell.secondary() = 0;
  heap.insert(origin_cell);

  while(!heap.empty()) {
    cell_ref curr = heap.top();
    heap.pop();
    unsigned loc = curr.loc();
    unsigned t = curr.time();

    unsigned char confl_cost = curr.secondary();

    for(auto p : successors(loc)) {
      if(csts.is_allowed(p.first, p.second, t+1)) {
        cell_ref succ = get(p.second, t+1);
        unsigned char score_inc = reservation.score(p.first, p.second, t+1);

        if(succ.loc() == goal && t+1 >= goal_time)
          return t+1;
        if(confl_cost + score_inc < succ.secondary()) {
          succ.secondary() = std::min(confl_cost + score_inc, secondary_max());
          if(!heap.in_heap(succ))
            heap.insert(succ);
          else
            heap.decrease(succ);
        }
      }
    }
    if(csts.is_allowed(pf::M_WAIT, loc, t+1)) { 
      // Don't need to check goal condition, because
      // the goal is always blocked immediately before release time.
      cell_ref wait = get(loc, t+1);
      unsigned char wait_inc = reservation.score(pf::M_WAIT, loc, t+1);
      if(confl_cost + wait_inc < wait.secondary()) {
        wait.secondary() = std::min(confl_cost + wait_inc, secondary_max());
        if(!heap.in_heap(wait))
          heap.insert(wait);
        else
          heap.decrease(wait);
      }
    }
  }
  return INT_MAX;
}

}
