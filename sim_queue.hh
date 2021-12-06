#ifndef __sim_queue__
#define __sim_queue__

#include <cstdint>
#include <cassert>


template <typename T>
class sim_queue {
public:
  class funcobj {
  public:
    virtual bool operator()(T e) = 0;

  };
private:
  uint64_t len = 0, len2 = 0;
  uint64_t n_free = 0;
  T *data = nullptr;
  uint64_t read_idx = 0, write_idx = 0;
  uint64_t push_(T v) {
    uint64_t slot_id = write_idx & (len-1);
    data[slot_id]=v;
    write_idx = (write_idx+1) & (len2-1);
    return slot_id;
  }
  T pop_() {
    T v = data[read_idx & (len-1)];
    read_idx = (read_idx+1) & (len2-1);
    return v;
  }
  T peek_() const {
    T v = data[read_idx & (len-1)];
    return v;
  }
public:
  sim_queue(uint64_t len = 1) : len(len), len2(2*len) {
    bool pow2 = ((len-1)&len)==0;
    assert(pow2);
    data = new T[len];
    n_free = len;
    memset(data, 0, sizeof(T)*len);
  }
  void resize(uint64_t len) {
    delete [] data;
    this->len = len;
    len2 = 2*len;
    data = new T[len];
    memset(data, 0, sizeof(T)*len);
    read_idx = write_idx = 0;
    n_free = len;
  }
  ~sim_queue() {
    delete [] data;
  }
  bool empty() const {
    return read_idx==write_idx;
  }
  bool full() const {
    const uint64_t m_rd_idx = read_idx & (len-1);
    const uint64_t m_wr_idx = write_idx & (len-1);
    return (m_rd_idx==m_wr_idx) && (read_idx != write_idx);
  }
  uint64_t size() const {
    return (write_idx > read_idx) ? (write_idx-read_idx) : ((len2-write_idx)+read_idx);
  }
  int64_t free_size() const {
    return n_free;
  }
  void clear() {
    //memset(data, 0, sizeof(T)*len);
    n_free = len;
    read_idx = write_idx = 0;
  }
  uint64_t push(T v) {
    assert(!full());
    --n_free;
    return push_(v);
  }
  T pop() {
    assert(!empty());
    ++n_free;
    return pop_();
  }
  T peek() const {
    assert(!empty());
    return peek_();
  }
  T peek_next_pop() const {
    uint64_t next_rd = (read_idx+1) & (len2-1);
    return data[next_rd & (len-1)];
  }
  size_t capacity() const {
    return len;
  }
  uint64_t get_read_idx() const {
    return read_idx & (len-1);
  }
  uint64_t get_write_idx() const {
    return write_idx & (len-1);
  }
  T &at(size_t idx) {
    return data[idx];
  }
};


#endif
