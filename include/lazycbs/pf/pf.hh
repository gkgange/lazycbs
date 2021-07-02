#ifndef LAZYCBS__PF__HH
#define LAZYCBS__PF__HH
#include <cstdint>
#include <climits>
#include <geas/mtl/Vec.h>
// #include <lazycbs/support/graph.h>
#include <geas/mtl/int-triemap.h>
#include <lazycbs/utils/intrusive-heap.h>

namespace mapf {

struct pf {
  enum Move {
    M_RIGHT = 0,
    M_LEFT = 1,
    M_DOWN = 2,
    M_UP = 3,
    M_WAIT = 4,

    NUM_MOVES
  };

  enum Axis {
    Ax_NONE = 0,
    Ax_HORIZ = 1,
    Ax_VERT = 2,
    Ax_BOTH = 3
  };

  enum Constraint {
    // Edge constraints
    C_NONE = 0,
    C_RIGHT = 1, C_LEFT = 2, C_DOWN = 4, C_UP = 8,
    C_VERT = 16
  };

  typedef std::pair<int, Move> Step;
  typedef vec<Step> Path;

  static int path_cost(const Path& path) { return path.last().first; }

  static Step* find_step(const Path& path, int t) {
    return std::lower_bound(path.begin(), path.end(),
                               t, [](Step s, int t) { return s.first < t; });
  }

  /*
  static const Move move_inv[] = {
    M_LEFT,
    M_RIGHT,
    M_UP,
    M_DOWN,
    M_WAIT,
    NUM_MOVES
  };
  */
  inline static Move move_inv(Move m) {
    static const Move _move_inv[] = {
        M_LEFT,
        M_RIGHT,
        M_UP,
        M_DOWN,
        M_WAIT,
        NUM_MOVES
    };
    return _move_inv[m];
  }
  inline static Axis move_axis(Move m) {
    static const Axis _move_axis[] = {
      Ax_HORIZ,
      Ax_HORIZ,
      Ax_VERT,
      Ax_VERT,
      Ax_NONE,
      Ax_NONE
    };
    return _move_axis[m];
  }

  inline static const char* move_str(Move m) {
    static const char* _move_str[] = {
      "RIGHT",
      "LEFT",
      "DOWN",
      "UP",
      "WAIT",
      "<NUM_MOVES>"
    };
    return _move_str[m];
  }
};

struct constraints {
  typedef uint64_triemap<uint64_t, unsigned char, UIntOps> mask_map;
  constraints(int size)
    : masks(size), lock_time(size, UINT_MAX) { }

  // Location -> (Time -> Constraints)
  inline bool is_allowed(pf::Move m, int loc, unsigned time) const;
  inline unsigned char score(pf::Move m, int loc, unsigned time) const;

  void forbid(pf::Move m, int loc, unsigned time);
  void lock(int loc, unsigned time);
  // Precondition: (m, loc, t) must currently be constrained.
  // Returns if (loc, t) is still constrained by something else.
  bool release(pf::Move m, int loc, unsigned time);
  
  vec<mask_map> masks;
  vec<unsigned> lock_time;
};

// The important one. Am I allowed to move *into* loc *by* a
// move m at time t.
// We do it this way, so we can 
inline bool constraints::is_allowed(pf::Move m, int loc, unsigned time) const {
  if(lock_time[loc] <= time)
    return false;
  
  auto& map = masks[loc];
  auto it = map.find(time);
  if(it == map.end())
    return true;
  return ! ( (*it).value & ((1 << m)|pf::C_VERT) );
}

inline unsigned char constraints::score(pf::Move m, int loc, unsigned time) const {
  auto& map = masks[loc];
  auto it = map.find(time);
  if(it == map.end())
    return 0;
  // One for vertex conflict, additional for edge conflict.
  return __builtin_popcount((*it).value & ((1 << m) | pf::C_VERT));
}

struct navigation {
  typedef vec< std::pair<pf::Move, int> > adj_list;
  struct transition {
    int operator[](pf::Move m) const { return dest[m]; }
    int dest[pf::NUM_MOVES];
  };


  template<class T>
  static navigation of_obstacle_array(int width, int height, const T& blocked,
                                      vec<std::pair<int, int> >& out_coord);

  int size(void) const { return succ.size(); }
  
  int* fwd_heuristic (int dest) const;
  int* rev_heuristic (int src) const;

  const adj_list& successors(int loc) const { return succ[loc]; }
  const adj_list& predecessors(int loc) const { return pred[loc]; }

  pf::Move move_dir(int pred, int succ) const;

  int nearest_location(int row, int col, pf::Move move);
  
  vec<adj_list> succ;
  vec<adj_list> pred;
  vec<transition> delta; // loc -> move -> succ(loc, move)
  vec<transition> inv; // loc -> move -> pred(loc, move)
};

namespace reservation {

struct table {
  table(int size);

  inline bool is_allowed(pf::Move m, int loc, unsigned time);
  inline unsigned char score(pf::Move m, int loc, unsigned time);

  void reserve(const navigation& nav, int origin, const pf::Path& path);
  void release(const navigation& nav, int origin, const pf::Path& path);

  // Used for temporarily suppressing agents during planning.
  void _dec_move(int loc, int t, pf::Move m);
  void _inc_move(int loc, int t, pf::Move m);
  
  // Ignore implementation details this really shouldn't be in a header.
  struct res_cell {
    res_cell(void)
      : total(0)
    {
      for(int ii = 0; ii < pf::NUM_MOVES; ++ii)
        counts[ii] = 0;
    }
    unsigned& operator[](pf::Move m) { return counts[m]; }

    unsigned counts[pf::NUM_MOVES];
    unsigned total;
  };
  typedef uint64_triemap<uint64_t, res_cell, UIntOps> res_count_map;

  // TODO: Replace these with less awful structures.
  vec<res_count_map> res_counts;
  constraints reserved;
};

inline bool table::is_allowed(pf::Move m, int loc, unsigned time) {
  return reserved.is_allowed(m, loc, time);
}

inline unsigned char table::score(pf::Move m, int loc, unsigned time) {
  return reserved.score(m, loc, time);
}

};

class explainer {
  struct cst {
    cst(int _loc, unsigned _time, pf::Move _move)
      : loc(_loc), time(_time), move(_move) { }
    int loc;
    unsigned time : 24;
    unsigned move : 8;
  };
};

// Standard non-SIPP pathfinder.
// ============================
// Shared pathfinder, re-used between different agents.
struct simple_pathfinder {
  // How do we represent nodes?
  enum { CELL_BITS = 5 };
  enum { CELL_SIZE = 1 << CELL_BITS };
  static constexpr int secondary_max(void) { return (1<<7)-1; }

  // A range of 32 time-steps on the same location.
  // Costs a little extra memory, but saves us a lot of allocations.
  struct cell {
    cell(unsigned _loc, unsigned _base, unsigned _timestamp)
      : loc(_loc), base(_base)
      , timestamp(_timestamp) {
      memset(tag, 0, sizeof(unsigned) * CELL_SIZE);
      memset(secondary, -1, sizeof(unsigned char) * CELL_SIZE);
    }
    inline void reset(unsigned time) {
      if(time != timestamp) {
        memset(tag, 0, sizeof(unsigned) * CELL_SIZE);
        memset(secondary, -1, sizeof(unsigned char) * CELL_SIZE);
        timestamp = time;
      }
    }

    unsigned loc;
    unsigned base;

    unsigned timestamp;

    unsigned tag[CELL_SIZE]; // Cross-references
    unsigned char secondary[CELL_SIZE];
    pf::Move pred[CELL_SIZE];
  };
  typedef uint64_triemap<uint64_t, cell, UIntOps> cell_map;

  // Think about how to make this more efficient.
  struct cell_ref {
    cell_ref(cell* _ptr, unsigned char _off)
      : ptr(_ptr), off(_off) { }

    bool operator==(const cell_ref& o) const {return ptr == o.ptr && off == o.off; }

    cell* ptr;
    unsigned char off;

    inline int loc(void) const { return ptr->loc; }
    inline int time(void) const { return ptr->base + off; }
    inline unsigned char& secondary(void) { return ptr->secondary[off]; }
    inline const unsigned char& secondary(void) const { return ptr->secondary[off]; }
    inline pf::Move& pred(void) {  return ptr->pred[off]; }
    inline const pf::Move& pred(void) const {  return ptr->pred[off]; }
  };

  struct cell_env {
    bool lt(const cell_ref& x, const cell_ref& y) const {
      int x_dist = x.time() + heur[x.loc()];
      int y_dist = y.time() + heur[y.loc()];
      if(x_dist == y_dist) {
        return x.secondary() < y.secondary();
      }
      return x_dist < y_dist;
    }
    unsigned pos(const cell_ref& x) { return x.ptr->tag[x.off]; }
    void pos(const cell_ref& x, unsigned tag) { x.ptr->tag[x.off] = tag; }

    int* heur;
  };

  inline const navigation::adj_list& successors(int loc) const { return nav.successors(loc); }
  inline const navigation::adj_list& predecessors(int loc) const { return nav.predecessors(loc); }

  cell_ref get(int loc, unsigned t);

  // Shared map/search environment.
  simple_pathfinder(const navigation& _nav)
    : nav(_nav), cells(nav.size()), heap(cell_env()), timestamp(0) { }

  const navigation& nav;
  vec<cell_map> cells;
  IntrusiveHeap<cell_ref, cell_env> heap;

  int timestamp;

  int search(int origin, int goal, int* heur,
             const constraints& csts, const constraints& res);
};

}

#endif
