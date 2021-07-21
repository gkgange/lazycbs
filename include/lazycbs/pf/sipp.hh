#ifndef LAZYCBS__SIPP__HH
#define LAZYCBS__SIPP__HH
#include <lazycbs/pf/pf.hh>

namespace mapf {

// Alternate SIPP-esque pathfinder.
struct sipp_interval {
  sipp_interval(int _start, unsigned char _constraints = 0)
    : start(_start), constraints(_constraints), tag(0) { }

  // Persistent data
  inline bool is_allowed(pf::Move m) const {
    return ! (constraints & ( (1 << m) | pf::C_VERT));
  }
  inline bool may_wait(void) const {
    return !(constraints & (pf::C_VERT | pf::C_DELAY));
  }
  int start; // When does the interval start?
  unsigned char constraints; // What constraints are active?

  pf::Move pred;

  // Transient state
  unsigned char second;
  
  int reach; // What time have could we reach this interval?
  int next_ex; // What time will we next expand from?

  unsigned tag; // Intrusive heap info
};

struct sipp_loc {
  sipp_loc(void);

  vec<sipp_interval> i;
  int Tend;

  int timestamp;

  int size(void) const { return i.size(); }
  sipp_interval& operator[](int idx) { return i[idx]; }

  sipp_interval* begin(void) const { return i.begin(); }
  sipp_interval* reach(int t) const; // Last interval not-after t.
  // Final interval is always <INT_MAX, C_VERT>, don't include it during iteration.
  // We use INT_MAX (instead of UINT_MAX) as an ugly fix for boundary conditions.
  sipp_interval* end(void) const { return i.end()-1; }

  void reset(int current_time) {
    for(sipp_interval& iv : i) {
      iv.reach = INT_MAX-1;
      iv.next_ex = INT_MIN+1;
    }
    timestamp = current_time;
  }
};

struct sipp_ctx {
  sipp_ctx(int size);
  ~sipp_ctx(void) { delete[] ptr; }


  void forbid(pf::Move m, unsigned loc, int time);
  void lock(unsigned loc, int time);
  void unlock(unsigned loc);
  bool release(pf::Move m, unsigned loc, int time);

  sipp_interval& _create(unsigned loc, int time);

  sipp_loc& operator[](int c) { return ptr[c]; }
  const sipp_loc& operator[](int c) const { return ptr[c]; }
  
  bool is_allowed(pf::Move m, unsigned loc, unsigned time) const {
    auto itv = ptr[loc].reach(time);
    return itv->is_allowed(m);
  }

  sipp_loc* ptr;
  int timestamp;
};

struct IntId {
  IntId(unsigned _loc, int _idx)
    : loc(_loc), idx(_idx) { }
  bool operator==(const IntId& o) const {return loc == o.loc && idx == o.idx; }
  unsigned loc;
  int idx;
};

struct sipp_pathfinder {

  // Operations for the Step heap.
  struct sipp_env {
    sipp_env(void)
      : ctx(nullptr) { }

    sipp_loc* ctx;
    int* heur;

    sipp_interval& get(IntId s) const { return ctx[s.loc].i[s.idx]; }
    int G(IntId s) const { return heur[s.loc]; }
    int H(IntId s) const { return G(s) + get(s).next_ex; }

    // Currently ignoring the reservation table.
    bool lt(IntId s, IntId t) const {
      if(H(s) != H(t))
        return H(s) < H(t);
      if(get(s).second != get(t).second)
        return get(s).second < get(t).second;
      return G(s) < G(t);
    }
    unsigned pos(IntId s) const { return get(s).tag; }
    unsigned pos(IntId s, unsigned t) { return get(s).tag = t; }
  };

  sipp_pathfinder(const navigation& _nav)
    : nav(_nav), heap(sipp_env()), timestamp(0)
    , LL_searches(0), LL_generated(0), LL_expanded(0) { }

  inline sipp_loc& get(unsigned loc) const {
    if(timestamp != state[loc].timestamp)
      state[loc].reset(timestamp);
    return state[loc];
  }

  int search(int origin, int goal, sipp_ctx& state, int* heur,
             const constraints& reserved);
  int search_upto(int origin, int goal, int ub, sipp_ctx& state, int* heur,
             const constraints& reserved);

  // Parameters
  const navigation& nav;
  IntrusiveHeap<IntId, sipp_env> heap;

  // Recording the incumbent path in terms of
  // (timestep, location).
  pf::Path path;

  // Transient state
  sipp_loc* state;

  int timestamp;

  // Stats tracking
  int LL_searches;
  int LL_generated;
  int LL_expanded;
};

class sipp_explainer {
public:
  struct cst {
    cst(int _loc, int _time, unsigned _move)
      : loc(_loc), time(_time), move(_move) { }
    unsigned loc;
    int time;
    unsigned move;
  };
  struct mark_env {
    mark_env(void)
      : ctx(nullptr) { }

    sipp_loc* ctx;

    sipp_interval& get(IntId s) const { return ctx[s.loc].i[s.idx]; }
    int H(IntId s) const { return get(s).next_ex; }

    // Currently ignoring the reservation table.
    bool lt(IntId s, IntId t) const {return H(s) > H(t); }
    unsigned pos(IntId s) const { return get(s).tag; }
    unsigned pos(IntId s, unsigned t) { return get(s).tag = t; }
  };
  struct collect_env {
    collect_env(void)
      : ctx(nullptr) { }

    sipp_loc* ctx;

    sipp_interval& get(IntId s) const { return ctx[s.loc].i[s.idx]; }
    int H(IntId s) const { return get(s).reach; }

    // Currently ignoring the reservation table.
    bool lt(IntId s, IntId t) const {return H(s) < H(t); }
    unsigned pos(IntId s) const { return get(s).tag; }
    unsigned pos(IntId s, unsigned t) { return get(s).tag = t; }
  };

  enum { M_LOCK = pf::NUM_MOVES };

  sipp_explainer(const navigation& _nav)
    : nav(_nav), ctx(nullptr), timestamp(0)
    , mark_heap(mark_env()), collect_heap(collect_env()) { }

  inline sipp_loc& get(unsigned loc) const {
    if(timestamp != ctx[loc].timestamp)
      ctx[loc].reset(timestamp);
    return ctx[loc];
  }

  void explain(int origin, int goal, int Tub, sipp_ctx& state, int* heur, int* r_heur,
               vec<cst>& out_cst); 

  const navigation& nav;
  
  sipp_loc* ctx;
  int timestamp;
  IntrusiveHeap<IntId, mark_env> mark_heap;
  IntrusiveHeap<IntId, collect_env> collect_heap;
};

};

#endif
