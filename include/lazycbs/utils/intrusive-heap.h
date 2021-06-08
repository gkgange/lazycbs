#ifndef LAZYCBS__INTRUSIVE_HEAP__H
#define LAZYCBS__INTRUSIVE_HEAP__H
#include <geas/mtl/Vec.h>

// Usual array-based heap, but allowing the current heap index to be stored
// elsewhere.
// Env must implement three functions:
// lt : T -> T -> bool (the comparator)
// pos : T -> unsigned int (returns the last saved index, if any)
// pos : T -> unsigned int -> void (sets a new saved index)
template<class T, class Env>
class IntrusiveHeap {
 public:
  Env env;

  static unsigned int parent(unsigned int x) { return (x-1)>>1; }
  static unsigned int left(unsigned int x) { return (x<<1)+1; }
 protected:
  void percolateUp(unsigned int pos, T elt) {
    while(pos) {
      unsigned int r(parent(pos));
      if(!env.lt(elt, elts[r]))
        break;

      elts[pos] = elts[r];
      env.pos(elts[r], pos);
      pos = r;
    }
    elts[pos] = elt;
    env.pos(elts[pos], pos);
  }

  void percolateDown(unsigned int pos, T elt) {
    unsigned int c(left(pos));
    while(c < (unsigned) elts.size()) {
      c += (c+1 < (unsigned) elts.size() && env.lt(elts[c+1], elts[c]));
      T elt_c(elts[c]);
      if(!env.lt(elt_c, elt))
        break;

      elts[pos] = elt_c;
      env.pos(elt_c, pos);
      pos = c;
      c = left(pos);
    }
    env.pos(elt, pos);
    elts[pos] = elt;
  }

public:
  IntrusiveHeap(Env _env) : env(_env) { }
  
  bool empty(void) const { return elts.size() == 0; }
  void clear(void) { elts.clear(); }
  bool in_heap(const T& elt) { return env.pos(elt) < (unsigned) elts.size() && elts[env.pos(elt)] == elt; }

  void remove(const T& elt) {
    assert(in_heap(elt));
    unsigned int pos(env.pos(elt));
    T rep(elts.last());
    elts.pop();
    if(pos != elts.size()) {
      if(pos == 0 || env.lt(elts[parent(pos)], rep))
        percolateDown(pos, rep);
      else
        percolateUp(pos, rep);
    }
  }

  void push_raw(const T& elt) { elts.push(elt); }
  void heapify(void) {
    if(elts.size() <= 1) return;
    unsigned int l(elts.size()-1);
    unsigned int c(parent(l));
    
    // Set up the cross-references
    for(; l > c; --l)
      env.pos(elts[l], l);

    // Now fill in the rest.
    while(true) {
      percolateDown(c, elts[c]);
      if(!c) break;
      --c;
    }
  }

  void insert(const T& elt) {
    assert(!in_heap(elt));
    unsigned int pos(elts.size());
    elts.push(elt);
    percolateUp(pos, elt);
  }
  void decrease(const T& elt) {
    assert(in_heap(elt));
    percolateUp(env.pos(elt), elt);
  }
  void increase(const T& elt) {
    assert(in_heap(elt));
    percolateDown(env.pos(elt), elt);
  }
  T top(void) { assert(!empty()); return elts[0]; }
  void pop(void) {
    assert(!empty());
    T last(elts.last());
    elts.pop();

    if(elts.size() > 0)
      percolateDown(0, last);
  }
  void replace_top(const T& elt) {
    percolateDown(0, elt);
  }
  
  vec<T> elts;
};

#endif
