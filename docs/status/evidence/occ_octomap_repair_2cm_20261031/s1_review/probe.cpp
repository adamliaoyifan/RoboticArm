#define main submitted_behavior_main
#include "/tmp/elfin-occ-octomap-repair-2cm-g1/src/uld_occ_observer/test/test_repair_s1_behavior.cpp"
#undef main
int main() {
  const auto cfg = load();
  ObservationStore store(cfg.geometry, cfg.resolution, cfg.depth, cfg.clock_domain, cfg.digest, false);
  store.integrate(observation(cfg, "seed", 1000000000));
  SnapshotRepository repo(store);
  const auto a = repo.acquire(2000000000, 5000000000, true);
  auto held = repo.get(a.handle);
  auto cache = held->surface();
  const auto one = repo.usage();
  const auto b = repo.acquire(2000000000, 5000000000, true);
  const bool same = repo.get(b.handle).get() == held.get();
  const auto two = repo.usage();
  std::cout << "same_snapshot=" << same << " one_cache=" << one.surface_cache_bytes
            << " two_cache=" << two.surface_cache_bytes << "\n";
  expect(same && one.surface_cache_bytes > 0 && two.surface_cache_bytes == one.surface_cache_bytes,
         "same cache must be counted once for two handles");
  store.reset(1);
  repo.release(a.handle); repo.release(b.handle);
  const auto retained = repo.usage();
  const auto readable = held->read_cells({0,0,0}, {2,2,2}, 0,8);
  std::vector<std::shared_ptr<const CellBlock>> blocks;
  held->collect_blocks(blocks);
  std::cout << "retained_blocks=" << blocks.size() << " reported_blocks_bytes=" << retained.shared_block_bytes
            << " retained_cache_bytes=" << held->surface_cache_allocated_bytes()
            << " reported_cache_bytes=" << retained.surface_cache_bytes
            << " readable_cells=" << readable.cells.size() << "\n";
  expect(!blocks.empty() && retained.shared_block_bytes >= blocks.size()*sizeof(CellBlock),
         "released handle with live reader must retain resource attribution");
  expect(retained.surface_cache_bytes >= held->surface_cache_allocated_bytes(),
         "live reader cache must remain attributed after reset and release");
  std::cout << "failures=" << failures << "\n";
  return failures ? 1 : 0;
}
