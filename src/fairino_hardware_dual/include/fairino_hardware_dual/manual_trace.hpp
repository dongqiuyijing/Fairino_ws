#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <thread>
#include <time.h>
#include <unistd.h>

namespace fairino_hardware_dual {
inline int64_t trace_now() noexcept {
  timespec ts{};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return int64_t(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}
// Fixed-size POD. Producers never wait: overlapping lifecycle/control calls
// are counted as trace loss, not allowed to race or delay robot control.
struct ManualTraceRecord {
  int64_t begin{}, end{};
  uint64_t rx{};
  int kind{}, mode{}, code{}, reason{};
  double vx{}, age{}, period{}, cmd_t{};
  std::array<double, 6> pose{};
};
template<size_t N> class TraceQueue {
  static_assert(std::atomic<size_t>::is_always_lock_free);
  std::array<ManualTraceRecord, N> records_{};
  std::atomic<size_t> head_{0}, tail_{0}, dropped_{0};
  std::atomic_flag producer_ = ATOMIC_FLAG_INIT;
public:
  bool push(const ManualTraceRecord & r) noexcept {
    if (producer_.test_and_set(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed); return false;
    }
    auto h = head_.load(std::memory_order_relaxed);
    auto next = (h + 1) % N;
    if (next == tail_.load(std::memory_order_acquire)) {
      dropped_.fetch_add(1, std::memory_order_relaxed);
      producer_.clear(std::memory_order_release); return false;
    }
    records_[h] = r;
    head_.store(next, std::memory_order_release);
    producer_.clear(std::memory_order_release); return true;
  }
  bool pop(ManualTraceRecord & r) noexcept {
    auto t = tail_.load(std::memory_order_relaxed);
    if (t == head_.load(std::memory_order_acquire)) return false;
    r = records_[t]; tail_.store((t + 1) % N, std::memory_order_release); return true;
  }
  size_t dropped() const noexcept { return dropped_.load(); }
};
class ManualTrace {
  TraceQueue<8192> control_, input_;
  std::atomic<bool> stop_{false};
  std::thread worker_;
  std::ofstream out_;
  bool enabled_{false};
  uint64_t total_{0}, nonzero_{0}, zero_{0};
  void drain(TraceQueue<8192> & q) {
    ManualTraceRecord r;
    while (q.pop(r)) {
      if (r.kind == 2) { ++total_; if (r.pose[0] != 0.0) ++nonzero_; else ++zero_; }
      out_ << r.begin << ',' << r.end << ',' << r.kind << ',' << r.rx << ','
           << r.mode << ',' << r.code << ',' << r.reason << ',' << r.vx << ','
           << r.age << ',' << r.period << ',' << r.cmd_t;
      for (double v : r.pose) out_ << ',' << v;
      out_ << '\n';
    }
  }
  void summary() {
    out_ << "# health," << trace_now() << ',' << control_.dropped() << ','
         << input_.dropped() << ',' << total_ << ',' << nonzero_ << ',' << zero_ << '\n';
    out_.flush();
  }
public:
  void start(const std::string & ip) {
    if (enabled_) return;
    const char * dir = std::getenv("FR3_TRACE_DIR");
    if (!dir || !*dir) return;
    out_.open(std::string(dir) + "/hardware_" + ip + "_" +
              std::to_string(getpid()) + "_" + std::to_string(trace_now()) + ".csv");
    if (!out_) throw std::runtime_error("Cannot open FR3_TRACE_DIR hardware trace");
    out_ << std::setprecision(17)
         << "# schema=1; clock=CLOCK_MONOTONIC; ip=" << ip
         << "; kind:1=rx,2=cart_return,3=joint_read,4=start_return,5=end_return,6=servoj,7=write_cycle,8=cart_call,9=start_call,10=end_call\n"
         << "# cart constants:pos_gain=1/1/1/1/1/1;exaxis=0/0/0/0;acc=0;vel=60;filterT=0;gain=0\n"
         << "# reason:0=nonzero,1=input_zero,2=unstamped,3=watchdog,4=inhibited,5=gripper,6=boundary\n"
         << "begin_ns,end_ns,kind,rx_seq,mode,ret,reason,vx_mm_s,age_ms,period_s,cmdT_s,p0,p1,p2,p3,p4,p5\n";
    enabled_ = true;
    worker_ = std::thread([this] {
      while (!stop_.load()) {
        drain(control_); drain(input_); summary();
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      drain(control_); drain(input_); summary();
    });
  }
  ~ManualTrace() {
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
  }
  bool enabled() const noexcept { return enabled_; }
  void control(const ManualTraceRecord & r) noexcept { if (enabled_) control_.push(r); }
  void input(const ManualTraceRecord & r) noexcept { if (enabled_) input_.push(r); }
};
}  // namespace fairino_hardware_dual
