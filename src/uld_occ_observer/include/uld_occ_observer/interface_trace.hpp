#pragma once
#include <atomic>
#include <cstdint>

namespace uld_occ_observer {
// 0 = off. The inline check is one relaxed load so the off arm does not take
// the product locks or write the ring.
extern std::atomic<int> interface_trace_flag;

void interface_trace_configure_from_env();
void interface_trace_flush();
// Fills an empty request id on later events from this thread. The off arm of
// interface_trace_event does not read it.
void interface_trace_bind_request(const char* request_id);
void interface_trace_event_slow(const char* event, const char* request_id,
                                const char* status, const char* phase);

inline void interface_trace_event(const char* event, const char* request_id,
                                  const char* status, const char* phase) {
  if (interface_trace_flag.load(std::memory_order_relaxed) == 0) return;
  interface_trace_event_slow(event, request_id ? request_id : "",
                             status ? status : "", phase ? phase : "");
}
}  // namespace uld_occ_observer
