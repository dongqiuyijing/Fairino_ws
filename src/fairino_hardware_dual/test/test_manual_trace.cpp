#include <gtest/gtest.h>
#include <filesystem>
#include "fairino_hardware_dual/manual_trace.hpp"
#include "fairino_hardware_dual/manual_cartesian.hpp"
using namespace fairino_hardware_dual;
TEST(ManualTrace, CurrentExperimentalCommandPreserved) {
  auto plan = plan_manual_cartesian_write(0.01004, 10.0, 38.0, true, false);
  EXPECT_EQ(plan.cart.mode, 2);
  EXPECT_DOUBLE_EQ(plan.cart.cmd_t_s, 0.01004);
  EXPECT_DOUBLE_EQ(plan.cart.x_mm, 10.0 * 0.01004);
}
TEST(ManualTrace, CapacityAndOrder) {
  TraceQueue<4> q;
  ManualTraceRecord r;
  for (int i=0; i<3; ++i) { r.rx=i; ASSERT_TRUE(q.push(r)); }
  EXPECT_FALSE(q.push(r)); EXPECT_EQ(q.dropped(), 1u);
  for (int i=0; i<3; ++i) { ASSERT_TRUE(q.pop(r)); EXPECT_EQ(r.rx, i); }
  EXPECT_FALSE(q.pop(r));
}
TEST(ManualTrace, ConcurrentProducerConsumer) {
  TraceQueue<1024> q;
  std::atomic<bool> done{false};
  std::thread writer([&] {
    for (uint64_t i=0; i<100000; ++i) {
      ManualTraceRecord r; r.rx=i; r.pose[0]=double(i);
      q.push(r);
    }
    done.store(true);
  });
  ManualTraceRecord r;
  uint64_t count=0, previous=0;
  while (!done.load()) {
    if (q.pop(r)) {
      if (count) { EXPECT_GT(r.rx, previous); }
      EXPECT_EQ(r.pose[0], double(r.rx)); previous=r.rx; ++count;
    }
  }
  writer.join();
  while(q.pop(r)) { if(count) { EXPECT_GT(r.rx, previous); } previous=r.rx; ++count; }
  EXPECT_EQ(count + q.dropped(), 100000u);
}
TEST(ManualTrace, WriterFlushesCountsAndParameters) {
  char directory[] = "/tmp/fr3-trace-test-XXXXXX";
  ASSERT_NE(mkdtemp(directory), nullptr);
  const char* old = std::getenv("FR3_TRACE_DIR");
  const std::string saved = old ? old : "";
  setenv("FR3_TRACE_DIR", directory, 1);
  {
    ManualTrace trace; trace.start("offline");
    ManualTraceRecord r; r.kind=2; r.pose[0]=0.08; r.mode=2; r.cmd_t=0.008;
    trace.control(r); r.pose[0]=0; trace.control(r);
  }
  if (old) setenv("FR3_TRACE_DIR", saved.c_str(), 1); else unsetenv("FR3_TRACE_DIR");
  for (const auto& entry : std::filesystem::directory_iterator(directory)) {
    std::ifstream input(entry.path());
    const std::string content((std::istreambuf_iterator<char>(input)), {});
    EXPECT_NE(content.find("# health,"), std::string::npos);
    EXPECT_NE(content.find(",0,0,2,1,1\n"), std::string::npos);
    EXPECT_NE(content.find("exaxis=0/0/0/0"), std::string::npos);
    std::filesystem::remove(entry.path());
  }
  std::filesystem::remove(directory);
}
