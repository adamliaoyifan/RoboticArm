#include "joint_command_capture/recorder.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <random>
#include <sstream>

namespace joint_command_capture
{
namespace
{
Recorder * g_singleton = nullptr;

inline uint64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline uint64_t system_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}
inline size_t pow2_at_least(size_t v)
{
  size_t p = 1;
  while (p < v) {p <<= 1;}
  return p;
}
std::string new_epoch()
{
  std::random_device rd;
  std::mt19937_64 gen(rd());
  char buf[40];
  std::snprintf(buf, sizeof(buf), "%016llx%016llx",
    static_cast<unsigned long long>(gen()),
    static_cast<unsigned long long>(gen()));
  return std::string(buf, 32);
}
std::string json_escape(const std::string & s)
{
  std::string out;
  for (char c : s) {
    if (c == '"' || c == '\\') {
      out += '\\';
      out += c;
    } else if (static_cast<unsigned char>(c) < 0x20) {
      out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}
}  // namespace

// ---------------------------------------------------------------- Config ---
Recorder::Config Recorder::Config::from_env()
{
  Config c;
  const char * env = nullptr;
  if ((env = std::getenv("JOINT_CAPTURE_MODE")) != nullptr) {c.mode = env;}
  if ((env = std::getenv("JOINT_CAPTURE_DIR")) != nullptr) {c.run_dir = env;}
  auto num = [](const char * name, double & out) {
      const char * v = std::getenv(name);
      if (v == nullptr || *v == '\0') {return;}
      char * end = nullptr;
      const double d = std::strtod(v, &end);
      if (end != v && *end == '\0' && d >= 0.0) {out = d;}
    };
  auto mib = [](const char * name, size_t & out) {
      const char * v = std::getenv(name);
      if (v == nullptr || *v == '\0') {return;}
      char * end = nullptr;
      const long long d = std::strtoll(v, &end, 10);
      if (end != v && *end == '\0' && d > 0) {out = static_cast<size_t>(d);}
    };
  num("JOINT_CAPTURE_PRE_S", c.pre_s);
  num("JOINT_CAPTURE_POST_S", c.post_s);
  num("JOINT_CAPTURE_WALL_DEADLINE_S", c.wall_deadline_s);
  mib("JOINT_CAPTURE_RING_WRITE_MIB", c.ring_write_mib);
  mib("JOINT_CAPTURE_RING_READ_MIB", c.ring_read_mib);
  mib("JOINT_CAPTURE_RING_ITER_MIB", c.ring_iter_mib);
  mib("JOINT_CAPTURE_RING_EVENT_MIB", c.ring_event_mib);
  mib("JOINT_CAPTURE_QUEUE_MIB", c.queue_mib);
  mib("JOINT_CAPTURE_FLUSH_MIB", c.flush_mib);
  c.watermark = std::string(JCC_VERSION) + " git:" + JCC_GIT_SHA;
  return c;
}

bool Recorder::Config::valid(std::string & why) const
{
  if (mode != "off" && mode != "armed" && mode != "window") {
    why = "CAPTURE_CONFIG_INVALID_MODE";
    return false;
  }
  if (mode != "off" && run_dir.empty()) {
    why = "CAPTURE_CONFIG_INVALID_DIR";
    return false;
  }
  if (ring_write_mib == 0 || ring_read_mib == 0 || ring_iter_mib == 0 ||
    ring_event_mib == 0 || queue_mib == 0 || flush_mib == 0)
  {
    why = "CAPTURE_CONFIG_INVALID_BUDGET";
    return false;
  }
  return true;
}

// -------------------------------------------------------------- Singleton ---
Recorder & Recorder::instance()
{
  if (g_singleton == nullptr) {g_singleton = new Recorder();}
  return *g_singleton;
}

void Recorder::destroy_for_tests()
{
  delete g_singleton;
  g_singleton = nullptr;
}

// ---------------------------------------------------------------- Ring -----
template<typename T>
void Recorder::Ring<T>::push(const T & rec) noexcept
{
  const uint64_t h = head.load(std::memory_order_relaxed);
  uint64_t t = tail.load(std::memory_order_relaxed);
  const uint64_t cap = mask + 1;
  if (h - t == cap) {                     // full: drop oldest, keep recent past
    tail.fetch_add(1, std::memory_order_release);
    ++dropped;
    ++t;
  }
  slots[h & mask] = rec;
  head.store(h + 1, std::memory_order_release);
}

// ------------------------------------------------------------ Configure ----
bool Recorder::configure(const Config & cfg)
{
  if (configured_) {
    const bool same = cfg_.mode == cfg.mode && cfg_.run_dir == cfg.run_dir &&
      cfg_.post_s == cfg.post_s && cfg_.wall_deadline_s == cfg.wall_deadline_s;
    if (!same) {note_fault("CAPTURE_RECONFIGURE_REFUSED");}
    return same && active_.load(std::memory_order_relaxed);
  }
  std::string why;
  if (!cfg.valid(why)) {
    note_fault(why);
    configured_ = true;                    // sticky refuse, stays OFF
    return false;
  }
  cfg_ = cfg;
  if (cfg_.epoch.empty()) {cfg_.epoch = new_epoch();}
  configured_ = true;
  if (cfg.mode == "off") {return false;}   // OFF: nothing started

  mode_ = (cfg.mode == "window") ? Mode::WINDOW : Mode::ARMED;
  const auto cap_of = [](size_t mib, size_t rec_size) {
      return pow2_at_least((mib * 1024 * 1024) / rec_size + 1);
    };
  w_ring_ = std::make_unique<Ring<PluginWriteRecord>>(
    cap_of(cfg_.ring_write_mib, sizeof(PluginWriteRecord)));
  r_ring_ = std::make_unique<Ring<PluginReadRecord>>(
    cap_of(cfg_.ring_read_mib, sizeof(PluginReadRecord)));
  i_ring_ = std::make_unique<Ring<IterationRecord>>(
    cap_of(cfg_.ring_iter_mib, sizeof(IterationRecord)));
  e_ring_ = std::make_unique<Ring<EventRecord>>(
    cap_of(cfg_.ring_event_mib, sizeof(EventRecord)));

  wall_start_ns_ = now_ns();
  if (!ensure_run_dir()) {
    // Refuse loudly: nothing recorded, no thread; the plugin logs the fault.
    return false;
  }
  active_.store(true, std::memory_order_relaxed);
  flush_thread_ = std::thread([this] {flush_loop();});
  return true;
}

bool Recorder::ensure_run_dir()
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path dir(cfg_.run_dir);
  if (fs::exists(dir, ec)) {
    if (!fs::is_directory(dir, ec) || !fs::is_empty(dir, ec)) {
      note_fault("CAPTURE_DIR_EXISTS");
      return false;
    }
  } else if (!fs::create_directories(dir, ec) || ec) {
    note_fault("CAPTURE_DIR_CREATE_FAILED");
    return false;
  }
  fs::create_directories(dir / "ctl", ec);
  return true;
}

// ------------------------------------------------------------- Producers ---
void Recorder::record_write(const PluginWriteRecord & rec)
{
  if (!active_.load(std::memory_order_relaxed)) {++off_calls_; return;}
  PluginWriteRecord r = rec;
  r.dropped_before = w_ring_->dropped;
  w_stats_.dropped_ring = w_ring_->dropped;
  ++w_stats_.produced;
  w_ring_->push(r);
}

void Recorder::record_read(const PluginReadRecord & rec)
{
  if (!active_.load(std::memory_order_relaxed)) {++off_calls_; return;}
  PluginReadRecord r = rec;
  r.dropped_before = r_ring_->dropped;
  r_stats_.dropped_ring = r_ring_->dropped;
  ++r_stats_.produced;
  r_ring_->push(r);
}

void Recorder::record_iteration(const IterationRecord & rec)
{
  if (!active_.load(std::memory_order_relaxed)) {++off_calls_; return;}
  i_stats_.dropped_ring = i_ring_->dropped;
  ++i_stats_.produced;
  i_ring_->push(rec);
}

void Recorder::record_event(uint64_t external_id, EventKind kind, const char * text)
{
  if (!active_.load(std::memory_order_relaxed)) {++off_calls_; return;}
  std::lock_guard<std::mutex> guard(event_mutex_);
  if (std::find(seen_trigger_ids_.begin(), seen_trigger_ids_.end(), external_id) !=
    seen_trigger_ids_.end())
  {
    return;                                // idempotent external id
  }
  if (seen_trigger_ids_.size() < 4096) {
    seen_trigger_ids_.push_back(external_id);
  } else {
    note_fault("EVENT_REGISTRY_FULL");
    return;
  }
  EventRecord ev{};
  ev.event_id = external_id;
  ev.wall_mono_ns = now_ns();
  ev.sim_time_ns = last_sim_ns_.load(std::memory_order_relaxed);
  ev.kind = static_cast<uint8_t>(kind);
  std::snprintf(ev.text, sizeof(ev.text), "%s", text ? text : "");
  ++e_stats_.produced;
  e_ring_->push(ev);
}

void Recorder::record_event_internal(EventKind kind, const char * text)
{
  if (!active_.load(std::memory_order_relaxed)) {++off_calls_; return;}
  std::lock_guard<std::mutex> guard(event_mutex_);
  EventRecord ev{};
  ev.event_id = kInternalEventIdBit | ++internal_event_counter_;
  ev.wall_mono_ns = now_ns();
  ev.sim_time_ns = last_sim_ns_.load(std::memory_order_relaxed);
  ev.kind = static_cast<uint8_t>(kind);
  std::snprintf(ev.text, sizeof(ev.text), "%s", text ? text : "");
  ++e_stats_.produced;
  e_ring_->push(ev);
}

void Recorder::request_close(const char * reason)
{
  std::lock_guard<std::mutex> guard(event_mutex_);
  if (close_reason_.empty()) {close_reason_ = reason ? reason : "REQUESTED";}
}

std::string Recorder::close_reason() const
{
  std::lock_guard<std::mutex> guard(event_mutex_);
  return close_reason_;
}

// ---------------------------------------------------------------- Flush ----
void Recorder::flush_loop()
{
  record_event_internal(EventKind::CONFIGURED, cfg_.mode.c_str());
  if (mode_ == Mode::WINDOW) {
    open_stream_files();                   // WINDOW streams the whole leg
  }
  uint64_t last_poll_ns = 0;
  while (!closed_.load(std::memory_order_relaxed)) {
    const uint64_t now = now_ns();
    if (now - last_poll_ns >= 100000000ULL) {  // 10 Hz ctl poll (this thread)
      poll_control_dir();
      last_poll_ns = now;
    }
    check_deadlines();
    if (!close_reason().empty()) {break;}
    if (files_open_) {
      drain_ring(*w_ring_, w_cons_, w_stats_, files_[0]);
      drain_ring(*r_ring_, r_cons_, r_stats_, files_[1]);
      drain_ring(*i_ring_, i_cons_, i_stats_, files_[2]);
      {
        std::lock_guard<std::mutex> guard(event_mutex_);
        drain_ring(*e_ring_, e_cons_, e_stats_, files_[3]);
      }
      for (auto & f : files_) {
        f.out.flush();
        if (!f.out) {note_fault("CAPTURE_WRITE_FAILED");}
      }
    }
    if (now - last_health_ns_ >= 1000000000ULL) {
      write_health();
      last_health_ns_ = now;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (files_open_) {                       // final drain
    drain_ring(*w_ring_, w_cons_, w_stats_, files_[0]);
    drain_ring(*r_ring_, r_cons_, r_stats_, files_[1]);
    drain_ring(*i_ring_, i_cons_, i_stats_, files_[2]);
    std::lock_guard<std::mutex> guard(event_mutex_);
    drain_ring(*e_ring_, e_cons_, e_stats_, files_[3]);
  }
  const std::string reason =
    close_reason().empty() ? std::string("FLUSH_LOOP_EXIT") : close_reason();
  close_stream_files(reason.c_str());
  closed_.store(true, std::memory_order_relaxed);
  active_.store(false, std::memory_order_relaxed);
  write_health();
}

bool Recorder::open_stream_files()
{
  namespace fs = std::filesystem;
  if (files_open_ || cfg_.run_dir.empty()) {return files_open_;}
  struct Spec
  {
    const char * name;
    uint32_t kind;
    size_t size;
  };
  const Spec specs[] = {
    {"plugin_write", 1, sizeof(PluginWriteRecord)},
    {"plugin_read", 2, sizeof(PluginReadRecord)},
    {"iteration", 3, sizeof(IterationRecord)},
    {"events", 4, sizeof(EventRecord)},
  };
  for (const auto & s : specs) {
    FileStream fs_file;
    fs_file.path = (fs::path(cfg_.run_dir) / (std::string(s.name) + ".bin")).string();
    fs_file.record_kind = s.kind;
    fs_file.out.open(fs_file.path, std::ios::binary | std::ios::trunc);
    if (!fs_file.out) {
      note_fault("CAPTURE_WRITE_FAILED");
      continue;
    }
    BinFileHeader hdr{};
    std::memcpy(hdr.magic, "JCC1", 4);
    hdr.header_size = sizeof(BinFileHeader);
    hdr.record_size = static_cast<uint32_t>(s.size);
    hdr.record_kind = s.kind;
    hdr.wall_mono_start_ns = wall_start_ns_;
    hdr.wall_system_start_ns = system_now_ns();
    std::snprintf(hdr.watermark, sizeof(hdr.watermark), "%s", cfg_.watermark.c_str());
    std::snprintf(hdr.epoch, sizeof(hdr.epoch), "%s", cfg_.epoch.c_str());
    fs_file.out.write(reinterpret_cast<const char *>(&hdr), sizeof(hdr));
    fs_file.bytes_written = sizeof(hdr);
    fs_file.open = true;
    files_.push_back(std::move(fs_file));
  }
  files_open_ = files_.size() == 4 &&
    std::all_of(files_.begin(), files_.end(), [](const FileStream & f) {return f.open;});
  if (!files_open_ && files_.empty()) {note_fault("CAPTURE_WRITE_FAILED");}
  return files_open_;
}

void Recorder::close_stream_files(const char * reason)
{
#pragma pack(push, 1)
  struct Footer
  {
    char magic[4];                         // "JCCF"
    uint32_t footer_size;
    uint64_t records_written;
    uint32_t close_reason_code;            // 0 ok, 1 truncated
  };
  static_assert(sizeof(Footer) == 20, "reader schema expects 20");
#pragma pack(pop)
  for (auto & f : files_) {
    if (!f.open) {continue;}
    Footer foot{};
    std::memcpy(foot.magic, "JCCF", 4);
    foot.footer_size = sizeof(Footer);
    foot.records_written = f.records_written;
    foot.close_reason_code =
      (std::strncmp(reason, "WALL_DEADLINE", 13) == 0) ? 1u : 0u;
    f.out.write(reinterpret_cast<const char *>(&foot), sizeof(foot));
    f.out.flush();
    f.out.close();
    f.open = false;
  }
  write_manifest(reason);
}

template<typename T>
void Recorder::drain_ring(
  Ring<T> & ring, uint64_t & consumer, StreamStats & stats, FileStream & file)
{
  const uint64_t t = ring.tail.load(std::memory_order_acquire);
  if (consumer < t) {consumer = t;}        // fast-forward over overwritten
  const uint64_t h = ring.head.load(std::memory_order_acquire);
  const uint64_t budget = cfg_.flush_mib * 1024 * 1024;
  while (consumer < h) {
    const T & rec = ring.slots[consumer & ring.mask];
    const uint64_t s = rec.sim_time_ns;    // all record kinds carry it first
    uint64_t prev = last_sim_ns_.load(std::memory_order_relaxed);
    while (s > prev &&
      !last_sim_ns_.compare_exchange_weak(prev, s, std::memory_order_relaxed)) {}
    if (!file.open) {
      ++stats.dropped_queue;               // stream unopenable: named loss
      ++consumer;
      continue;
    }
    if (file.bytes_written + sizeof(T) > budget) {
      ++stats.dropped_queue;               // per-file budget exhausted
      ++consumer;
      continue;
    }
    file.out.write(reinterpret_cast<const char *>(&rec), sizeof(T));
    if (!file.out) {
      note_fault("CAPTURE_WRITE_FAILED");
      ++stats.dropped_queue;
      ++consumer;
      continue;
    }
    file.bytes_written += sizeof(T);
    ++file.records_written;
    ++stats.written;
    ++consumer;
  }
  stats.dropped_ring = ring.dropped;
}

void Recorder::poll_control_dir()
{
  namespace fs = std::filesystem;
  std::error_code ec;
  const fs::path ctl = fs::path(cfg_.run_dir) / "ctl";
  if (!fs::exists(ctl, ec)) {return;}
  for (auto it = fs::directory_iterator(ctl, ec);
    it != fs::directory_iterator(); ++it)
  {
    const std::string name = it->path().filename().string();
    if (name.rfind("trigger-", 0) != 0) {continue;}
    if (name.size() > 5 && name.compare(name.size() - 5, 5, ".done") == 0) {continue;}
    const std::string id_str = name.substr(8);
    char * end = nullptr;
    const unsigned long long id = std::strtoull(id_str.c_str(), &end, 10);
    if (end == id_str.c_str()) {continue;} // not a numbered trigger file
    std::string reason = "manual";
    {
      std::ifstream in(it->path());
      std::getline(in, reason);
      if (reason.empty()) {reason = "manual";}
    }
    handle_trigger(id, reason);
    std::error_code ren_ec;
    fs::rename(it->path(), it->path().string() + ".done", ren_ec);
  }
  if (fs::exists(ctl / "stop", ec)) {request_close("CTL_STOP");}
}

void Recorder::handle_trigger(uint64_t external_id, const std::string & reason)
{
  // Every distinct external id leaves an event trace; duplicate ids are
  // filtered idempotently inside record_event.
  record_event(external_id, EventKind::TRIGGER, reason.c_str());
  if (triggered_) {return;}
  triggered_ = true;
  trigger_wall_ns_ = now_ns();
  trigger_sim_ns_ = last_sim_ns_.load(std::memory_order_relaxed);
  if (!files_open_) {open_stream_files();} // ARMED: evidence dump starts here
}

bool Recorder::check_deadlines()
{
  const uint64_t now = now_ns();
  if (now - wall_start_ns_ >=
    static_cast<uint64_t>(cfg_.wall_deadline_s * 1e9))
  {
    record_event_internal(EventKind::TRUNCATED, "WALL_DEADLINE");
    request_close("WALL_DEADLINE");
    return true;
  }
  if (triggered_) {
    const uint64_t sim_now = last_sim_ns_.load(std::memory_order_relaxed);
    if (sim_now - trigger_sim_ns_ >= static_cast<uint64_t>(cfg_.post_s * 1e9)) {
      record_event_internal(EventKind::CLOSED, "POST_WINDOW_COMPLETE");
      request_close("POST_WINDOW_COMPLETE");
      return true;
    }
  }
  return false;
}

void Recorder::note_fault(const std::string & what)
{
  if (faults_.size() < 64 &&
    std::find(faults_.begin(), faults_.end(), what) == faults_.end())
  {
    faults_.push_back(what);
  }
}

void Recorder::write_health()
{
  if (cfg_.run_dir.empty()) {return;}
  namespace fs = std::filesystem;
  const fs::path tmp = fs::path(cfg_.run_dir) / "health.tmp";
  const fs::path dst = fs::path(cfg_.run_dir) / "health.json";
  std::ofstream out(tmp, std::ios::trunc);
  if (!out) {return;}
  out << health_json();
  out.close();
  std::error_code ec;
  fs::rename(tmp, dst, ec);
}

std::string Recorder::health_json() const
{
  std::ostringstream os;
  os << "{\"epoch\":\"" << cfg_.epoch << "\",\"mode\":\"" << cfg_.mode
     << "\",\"watermark\":\"" << json_escape(cfg_.watermark)
     << "\",\"triggered\":" << (triggered_ ? "true" : "false")
     << ",\"closed\":" << (closed_.load() ? "true" : "false")
     << ",\"active\":" << (active_.load() ? "true" : "false")
     << ",\"off_branch_calls\":" << off_calls_.load()
     << ",\"write\":{"
     << "\"produced\":" << w_stats_.produced
     << ",\"dropped_ring\":" << w_stats_.dropped_ring
     << ",\"dropped_queue\":" << w_stats_.dropped_queue
     << ",\"written\":" << w_stats_.written << "}"
     << ",\"read\":{"
     << "\"produced\":" << r_stats_.produced
     << ",\"dropped_ring\":" << r_stats_.dropped_ring
     << ",\"dropped_queue\":" << r_stats_.dropped_queue
     << ",\"written\":" << r_stats_.written << "}"
     << ",\"iter\":{"
     << "\"produced\":" << i_stats_.produced
     << ",\"dropped_ring\":" << i_stats_.dropped_ring
     << ",\"dropped_queue\":" << i_stats_.dropped_queue
     << ",\"written\":" << i_stats_.written << "}"
     << ",\"event\":{"
     << "\"produced\":" << e_stats_.produced
     << ",\"dropped_ring\":" << e_stats_.dropped_ring
     << ",\"dropped_queue\":" << e_stats_.dropped_queue
     << ",\"written\":" << e_stats_.written << "}"
     << ",\"faults\":[";
  for (size_t i = 0; i < faults_.size(); ++i) {
    if (i) {os << ",";}
    os << "\"" << json_escape(faults_[i]) << "\"";
  }
  os << "]}\n";
  return os.str();
}

void Recorder::write_manifest(const char * reason)
{
  if (cfg_.run_dir.empty()) {return;}
  namespace fs = std::filesystem;
  std::ofstream out(fs::path(cfg_.run_dir) / "manifest_recorder.json",
    std::ios::trunc);
  if (!out) {return;}
  out << "{\"epoch\":\"" << cfg_.epoch << "\",\"watermark\":\""
     << json_escape(cfg_.watermark) << "\",\"mode\":\"" << cfg_.mode
     << "\",\"close_reason\":\"" << json_escape(reason)
     << "\",\"pre_s\":" << cfg_.pre_s << ",\"post_s\":" << cfg_.post_s
     << ",\"wall_deadline_s\":" << cfg_.wall_deadline_s
     << ",\"ring_mib\":[" << cfg_.ring_write_mib << "," << cfg_.ring_read_mib
     << "," << cfg_.ring_iter_mib << "," << cfg_.ring_event_mib << "]"
     << ",\"queue_mib\":" << cfg_.queue_mib << ",\"flush_mib\":" << cfg_.flush_mib
     << ",\"record_sizes\":[" << sizeof(PluginWriteRecord)
     << "," << sizeof(PluginReadRecord) << "," << sizeof(IterationRecord)
     << "," << sizeof(EventRecord) << "]"
     << ",\"streams\":{";
  const StreamStats * st[4] = {&w_stats_, &r_stats_, &i_stats_, &e_stats_};
  const char * names[4] = {"plugin_write", "plugin_read", "iteration", "events"};
  for (int i = 0; i < 4; ++i) {
    if (i) {out << ",";}
    out << "\"" << names[i] << "\":{" << "\"produced\":" << st[i]->produced
        << ",\"dropped_ring\":" << st[i]->dropped_ring
        << ",\"dropped_queue\":" << st[i]->dropped_queue
        << ",\"written\":" << st[i]->written << "}";
  }
  out << "},\"faults\":[";
  for (size_t i = 0; i < faults_.size(); ++i) {
    if (i) {out << ",";}
    out << "\"" << json_escape(faults_[i]) << "\"";
  }
  out << "]}\n";
}

Recorder::~Recorder()
{
  if (flush_thread_.joinable()) {
    if (close_reason().empty()) {request_close("DESTRUCTOR");}
    flush_thread_.join();
  }
}

}  // namespace joint_command_capture
