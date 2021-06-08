#ifndef LAZYCBS__SIPP__HH
#define LAZYCBS__SIPP__HH
#include <lazycbs/pf/pf.hh>

namespace mapf {

// Alternate SIPP-esque pathfinder.
struct sipp_interval {
  sipp_interval(unsigned _start, unsigned char _constraints = 0)
    : start(_start), constraints(_constraints) { }

  // Persistent data
  inline bool is_allowed(pf::Move m) const {
    return ! (constraints & ( (1 << m) | pf::C_VERT));
  }
  inline bool may_wait(void) const {
    return !(constraints & pf::C_VERT);
  }
  unsigned start; // When does the interval start?
  unsigned char constraints; // What constraints are active?

  pf::Move pred;

  // Transient state
  unsigned reach; // What time have could we reach this interval?
  unsigned next_ex; // What time will we next expand from?

  unsigned tag; // Intrusive heap info
};

struct sipp_loc {
  sipp_loc(void);
  vec<sipp_interval> i;
  unsigned Tend;

  unsigned timestamp;

  unsigned size(void) const { return i.size(); }
  sipp_interval& operator[](int idx) { return i[idx]; }

  sipp_interval* begin(void) const { return i.begin(); }
  sipp_interval* reach(unsigned t) const; // Last interval not-after t.
  // Final interval is always <INT_MAX, C_VERT>, don't include it during iteration.
  // We use INT_MAX (instead of UINT_MAX) as an ugly fix for boundary conditions.
  sipp_interval* end(void) const { return i.end()-1; }

  void reset(unsigned current_time) {
    for(sipp_interval& iv : i)
      iv.reach = INT_MAX;
    timestamp = current_time;
  }
};

struct sipp_ctx {
  sipp_ctx(const navigation& nav);
  ~sipp_ctx(void) { delete[] ptr; }

  sipp_loc* ptr;
};

struct sipp_pathfinder {
  struct IntId {
    IntId(unsigned _loc, unsigned _idx)
      : loc(_loc), idx(_idx) { }
    bool operator==(const IntId& o) const {return loc == o.loc && idx == o.idx; }
    unsigned loc;
    unsigned idx;
  };

  // Operations for the Step heap.
  struct sipp_env {
    sipp_env(void)
      : ctx(nullptr) { }

    sipp_loc* ctx;
    int* heur;

    sipp_interval& get(IntId s) const { return ctx[s.loc].i[s.idx]; }
    unsigned int G(IntId s) const { return heur[s.loc]; }
    unsigned int H(IntId s) const { return G(s) + get(s).next_ex; }

    // Currently ignoring the reservation table.
    bool lt(IntId s, IntId t) const {
      if(H(s) != H(t))
        return H(s) < H(t);
      return G(s) < G(t);
    }
    unsigned pos(IntId s) const { return get(s).tag; }
    unsigned pos(IntId s, unsigned t) { return get(s).tag = t; }
  };

  sipp_pathfinder(const navigation& _nav)
    : nav(_nav), heap(sipp_env()), timestamp(0) { }

  inline sipp_loc& get(unsigned loc) const {
    if(timestamp != state[loc].timestamp)
      state[loc].reset(timestamp);
    return state[loc];
  }
  /*
  inline sipp_interval& get(unsigned loc, unsigned idx) const { return state[loc].i[idx]; }
  inline sipp_interval& get(IntId c) const { return state[c.loc].i[c.idx]; }
  */

  unsigned search(int origin, int goal, sipp_loc* state, int* heur);

  // Parameters
  const navigation& nav;
  IntrusiveHeap<IntId, sipp_env> heap;

  // Transient state
  sipp_loc* state;

  unsigned timestamp;
};

class sipp_explainer {
  struct cst {
    cst(int _loc, unsigned _time, pf::Move _move)
      : loc(_loc), time(_time), move(_move) { }
    int loc;
    unsigned time : 24;
    unsigned move : 8;
  };
};

};

#endif
