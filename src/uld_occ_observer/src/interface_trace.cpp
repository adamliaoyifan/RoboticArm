#include "uld_occ_observer/interface_trace.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <unistd.h>
#include <sys/syscall.h>

namespace uld_occ_observer {
namespace {
constexpr std::uint32_t kCapacity = 65536;

struct Slot {
  std::atomic<std::uint32_t> ready{0};
  std::int64_t mono_ns{};
  std::int64_t thread_cpu_ns{};
  int pid{};
  int tid{};
  std::uint64_t seq{};
  char event[48]{};
  char request_id[96]{};
  char status[96]{};
  char phase[16]{};
};

Slot g_slots[kCapacity];
std::atomic<std::uint32_t> g_write{0};
std::atomic<std::uint64_t> g_dropped{0};
std::atomic<std::uint64_t> g_seq{1};
std::atomic<int> g_started{0};
std::mutex g_emit_mu;
std::string g_run_id = "unset";
FILE* g_file = nullptr;
std::thread g_flush;
std::atomic<int> g_stop{0};
std::uint32_t g_read = 0;
std::mutex g_flush_mutex;
thread_local std::string g_bound_request;

std::int64_t clock_ns(clockid_t clock) {
  timespec ts{};
  clock_gettime(clock, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
}

void copy_field(char* dest, std::size_t size, const char* value) {
  if (!value) value = "";
  std::snprintf(dest, size, "%s", value);
}

void json_escape(FILE* file, const char* value) {
  for (const unsigned char* cursor = reinterpret_cast<const unsigned char*>(value); *cursor; ++cursor) {
    if (*cursor == '"' || *cursor == '\\') std::fputc('\\', file);
    if (*cursor >= 32) std::fputc(*cursor, file);
  }
}

void write_slot(const Slot& slot) {
  std::fprintf(g_file,
               "{\"schema_version\":1,\"run_id\":\"");
  json_escape(g_file, g_run_id.c_str());
  std::fprintf(g_file, "\",\"phase\":\"");
  json_escape(g_file, slot.phase);
  std::fprintf(g_file, "\",\"event_seq\":%llu,\"event\":",
               static_cast<unsigned long long>(slot.seq));
  std::fputc('"', g_file);
  json_escape(g_file, slot.event);
  std::fprintf(g_file, "\",\"mono_ns\":%lld,\"pid\":%d,\"tid\":%d,\"thread_cpu_ns\":%lld,\"request_id\":",
               static_cast<long long>(slot.mono_ns), slot.pid, slot.tid,
               static_cast<long long>(slot.thread_cpu_ns));
  std::fputc('"', g_file);
  json_escape(g_file, slot.request_id);
  std::fprintf(g_file, "\",\"chain_id\":null,\"input_id\":null,\"status\":\"");
  json_escape(g_file, slot.status);
  std::fprintf(g_file, "\"}\n");
}

void drain() {
  if (!g_file) return;
  const std::uint32_t claimed = g_write.load(std::memory_order_acquire);
  while (g_read < claimed && g_read < kCapacity) {
    if (g_slots[g_read].ready.load(std::memory_order_acquire) == 0) return;
    write_slot(g_slots[g_read]);
    ++g_read;
  }
  std::fflush(g_file);
}

void flush_loop() {
  while (g_stop.load(std::memory_order_acquire) == 0) {
    drain();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  drain();
}
}  // namespace

std::atomic<int> interface_trace_flag{0};

void interface_trace_bind_request(const char* request_id) {
  g_bound_request = request_id ? request_id : "";
}

void interface_trace_event_slow(const char* event, const char* request_id,
                                const char* status, const char* phase) {
  // This mutex is the trace ring, not a product lock. Sequence and the
  // monotonic stamp are taken together so a later sequence is not stamped
  // earlier by another thread.
  std::lock_guard<std::mutex> guard(g_emit_mu);
  const std::uint32_t pos = g_write.fetch_add(1, std::memory_order_relaxed);
  if (pos >= kCapacity) {
    g_dropped.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  Slot& slot = g_slots[pos];
  slot.mono_ns = clock_ns(CLOCK_MONOTONIC);
  slot.thread_cpu_ns = clock_ns(CLOCK_THREAD_CPUTIME_ID);
  slot.pid = static_cast<int>(::getpid());
  slot.tid = static_cast<int>(::syscall(SYS_gettid));
  slot.seq = g_seq.fetch_add(1, std::memory_order_relaxed);
  copy_field(slot.event, sizeof slot.event, event);
  const char* bound = request_id;
  if (bound == nullptr || bound[0] == '\0') bound = g_bound_request.c_str();
  copy_field(slot.request_id, sizeof slot.request_id, bound);
  copy_field(slot.status, sizeof slot.status, status);
  copy_field(slot.phase, sizeof slot.phase, phase);
  slot.ready.store(1, std::memory_order_release);
}

void interface_trace_configure_from_env() {
  if (g_started.exchange(1) != 0) return;
  const char* enabled = std::getenv("ULD_OCC_INTERFACE_TRACE");
  if (!enabled || std::strcmp(enabled, "1") != 0) return;
  const char* path = std::getenv("ULD_OCC_TRACE_PATH");
  const char* run = std::getenv("ULD_OCC_TRACE_RUN_ID");
  if (!path || !path[0]) return;
  if (run && run[0]) g_run_id = run;
  g_file = std::fopen(path, "w");
  if (!g_file) return;
  g_flush = std::thread(flush_loop);
  interface_trace_flag.store(1, std::memory_order_release);
}

void interface_trace_flush() {
  std::lock_guard<std::mutex> guard(g_flush_mutex);
  if (!g_file) return;
  interface_trace_flag.store(0, std::memory_order_release);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (std::chrono::steady_clock::now() < deadline) {
    const std::uint32_t claimed = std::min(g_write.load(std::memory_order_acquire), kCapacity);
    bool ready = true;
    for (std::uint32_t index = 0; index < claimed; ++index) {
      if (g_slots[index].ready.load(std::memory_order_acquire) == 0) ready = false;
    }
    if (ready) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (g_flush.joinable()) {
    g_stop.store(1, std::memory_order_release);
    g_flush.join();
  }
  drain();
  const auto dropped = g_dropped.load(std::memory_order_relaxed);
  const auto seq = g_seq.fetch_add(1, std::memory_order_relaxed);
  std::fprintf(g_file,
               "{\"schema_version\":1,\"run_id\":\"");
  json_escape(g_file, g_run_id.c_str());
  std::fprintf(g_file,
               "\",\"phase\":\"capture\",\"event_seq\":%llu,\"event\":\"trace_footer\","
               "\"mono_ns\":%lld,\"pid\":%d,\"tid\":%d,\"thread_cpu_ns\":%lld,"
               "\"request_id\":null,\"chain_id\":null,\"input_id\":null,\"status\":\"%llu\"}\n",
               static_cast<unsigned long long>(seq),
               static_cast<long long>(clock_ns(CLOCK_MONOTONIC)),
               static_cast<int>(::getpid()),
               static_cast<int>(::syscall(SYS_gettid)),
               static_cast<long long>(clock_ns(CLOCK_THREAD_CPUTIME_ID)),
               static_cast<unsigned long long>(dropped));
  std::fflush(g_file);
  std::fclose(g_file);
  g_file = nullptr;
}
}  // namespace uld_occ_observer
