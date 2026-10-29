#include "block_storage_checks.hpp"
#include "uld_occ_observer/block_storage.hpp"
#include "uld_occ_observer/observer.hpp"
#include "uld_occ_observer/occupancy_grid.hpp"
#include <iostream>
#include <limits>
#include <map>
#include <string>

namespace uld_occ_observer {
namespace {
void require(bool value, const std::string& message) {
  if (!value) throw Error(message);
}
template <class Function>
void require_error(const char* expected, Function&& function) {
  try {
    function();
  } catch (const Error& error) {
    require(error.what() == std::string(expected),
            std::string(expected) + " rejected with " + error.what());
    return;
  }
  throw Error(std::string(expected) + " was not rejected");
}
void compare_resolution_map(double resolution) {
  GeometryDescriptor geometry{"fixture_geometry", "fixture_digest", 0.8, 0.8, 0.0, 1.2, {}};
  ObservationStore store(geometry, resolution, {}, "test_clock", "config");
  const std::vector<std::vector<Vec3>> frames{
      {{0.1, 0.0, 0.8}, {-0.2, 0.1, 0.9}},
      {{0.2, -0.1, 0.85}, {0.0, 0.1, 0.75}},
      {{0.15, 0.0, 1.0}},
  };
  for (std::size_t i = 0; i < frames.size(); ++i) {
    OpticalObservation observation;
    observation.identity = {"sensor", "r" + std::to_string(i), "optical", "test_clock",
                            "fixture_geometry", 1, static_cast<std::int64_t>(1000 * (i + 1))};
    observation.uld_from_optical.stamp_ns = observation.identity.stamp_ns;
    observation.points_m = frames[i];
    require(store.integrate(observation).status == "INTEGRATED", "resolution frame rejected");
  }
  auto grid = OccupancyGrid::fromCheckpointBytes(store.checkpoint().tree_bytes, resolution);
  std::map<CellIndex, CellValue> tree;
  grid.forEachLeaf([&](CellIndex index, float log_odds) {
    tree.emplace(index, CellValue{log_odds, log_odds >= 0.f});
  });
  const auto snapshot = store.snapshot();
  const auto grid_shape = store.domain()->shape();
  std::size_t known = 0;
  std::size_t cursor = 0;
  const auto total = static_cast<std::size_t>(grid_shape.x) * grid_shape.y * grid_shape.z;
  while (cursor < total) {
    const auto page = snapshot->read_cells({0, 0, 0}, grid_shape, cursor, 65536);
    for (const auto& cell : page.cells) {
      const auto found = tree.find(cell.index);
      if (!cell.known) {
        require(found == tree.end(), "tree leaf missing from the contiguous block");
        require(cell.value.log_odds == 0.f && !cell.value.occupied, "unknown cell kept a value");
        continue;
      }
      require(found != tree.end() && cell.active && cell.value.log_odds == found->second.log_odds &&
                  cell.value.occupied == found->second.occupied,
              "contiguous block differs from the tree");
      ++known;
    }
    require(page.next_cursor > cursor, "resolution page stalled");
    cursor = page.next_cursor;
    if (page.done) break;
  }
  require(cursor == total && known == tree.size() && known == store.diagnostics().known_cells && known > 0,
          "resolution " + std::to_string(resolution) + " did not cover every known cell");
}
void require_round_trip(const BlockLayout& layout) {
  const auto shape = layout.grid_shape();
  std::size_t seen = 0;
  for (int z = 0; z < shape.z; ++z) {
    for (int y = 0; y < shape.y; ++y) {
      for (int x = 0; x < shape.x; ++x) {
        const CellIndex index{x, y, z};
        const auto address = layout.locate(index);
        require(address.directory_index < layout.block_count(), "directory slot outside the layout");
        require(layout.cell_index(address.block_index, address.local_offset) == index,
                "legal address did not round-trip");
        require(Snapshot::block_index(index) == address.block_index, "block_index diverged");
        const auto again = layout.locate(layout.cell_index(address.block_index, address.local_offset));
        require(again.directory_index == address.directory_index &&
                    again.local_offset == address.local_offset &&
                    again.block_index == address.block_index,
                "inverse address did not locate the same slot");
        ++seen;
      }
    }
  }
  require(seen == static_cast<std::size_t>(shape.x) * shape.y * shape.z, "round-trip missed a cell");
}
}  // namespace

void BlockStorageChecks::check_addressing() {
  require(kBlockEdge == 8 && kBlockCells == 512, "block edge changed");
  const CellIndex partial{10, 17, 9};
  BlockLayout layout(partial);
  require(layout.block_count() == 2u * 3u * 2u, "partial shape block count");
  const auto origin = layout.locate({0, 0, 0});
  require(origin.block_index == CellIndex{0, 0, 0} && origin.local_offset == 0 &&
              origin.directory_index == 0,
          "origin address");
  const auto edge = layout.locate({7, 0, 0});
  require(edge.block_index == CellIndex{0, 0, 0} && edge.local_offset == 7 &&
              edge.directory_index == origin.directory_index,
          "last cell of the first block");
  const auto next = layout.locate({8, 0, 0});
  require(next.block_index == CellIndex{1, 0, 0} && next.local_offset == 0 &&
              next.directory_index != origin.directory_index,
          "first cell of the next block");
  const auto last = layout.locate({9, 16, 8});
  require(last.block_index == CellIndex{1, 2, 1} && last.local_offset == 1 + 8 * 0 + 64 * 0,
          "last in-grid cell");
  const auto mixed = layout.locate({8, 8, 8});
  require(mixed.block_index == CellIndex{1, 1, 1} && mixed.local_offset == 0 &&
              mixed.directory_index == (1u * 3u + 1u) * 2u + 1u,
          "directory formula");
  require_round_trip(layout);
  require_round_trip(BlockLayout({8, 8, 8}));
  require_round_trip(BlockLayout({7, 1, 9}));
  require_round_trip(BlockLayout({1, 1, 1}));

  require_error("CELL_INDEX_OUT_OF_RANGE", [&] { layout.locate({-1, 0, 0}); });
  require_error("CELL_INDEX_OUT_OF_RANGE", [&] { layout.locate({0, -1, 0}); });
  require_error("CELL_INDEX_OUT_OF_RANGE", [&] { layout.locate({0, 0, -1}); });
  require_error("CELL_INDEX_OUT_OF_RANGE", [&] { layout.locate({10, 0, 0}); });
  require_error("CELL_INDEX_OUT_OF_RANGE", [&] { layout.locate({0, 17, 0}); });
  require_error("CELL_INDEX_OUT_OF_RANGE", [&] { layout.locate({0, 0, 9}); });
  require_error("CELL_INDEX_OUT_OF_RANGE", [&] { Snapshot::block_index({-1, 8, 8}); });
  require_error("INVALID_BLOCK_ADDRESS", [&] { layout.cell_index({1, 0, 0}, 2); });
  require_error("INVALID_BLOCK_ADDRESS", [&] { layout.cell_index({0, 0, 0}, 512); });
  require_error("INVALID_BLOCK_ADDRESS", [&] { layout.cell_index({-1, 0, 0}, 0); });
  require_error("INVALID_BLOCK_ADDRESS", [&] { layout.cell_index({2, 0, 0}, 0); });
  require_error("INVALID_BLOCK_ADDRESS", [&] { layout.cell_index({0, 3, 0}, 0); });
  require_error("INVALID_BLOCK_ADDRESS", [&] { layout.cell_index({0, 0, 2}, 0); });
  require_error("INVALID_BLOCK_LAYOUT", [] { BlockLayout({0, 8, 8}); });
  require_error("INVALID_BLOCK_LAYOUT", [] { BlockLayout({8, -1, 8}); });
  require_error("BLOCK_INDEX_OVERFLOW", [] {
    BlockLayout({std::numeric_limits<int>::max(), std::numeric_limits<int>::max(),
                 std::numeric_limits<int>::max()});
  });
  std::cout << "PASS block addressing sizeof(CellBlock)=" << sizeof(CellBlock) << "\n";
}

void BlockStorageChecks::check_batch_lifecycle() {
  const BlockLayout layout({24, 16, 16});
  const CellIndex first{0, 0, 0};
  const CellIndex second{1, 0, 0};
  const CellIndex other{8, 0, 0};
  const CellIndex later{0, 8, 0};
  BlockRegistry registry;
  auto original = std::make_shared<CellBlock>();
  const auto put = [&](CellBlock& block, CellIndex index, CellValue value) {
    block.write(layout.locate(index).local_offset, value);
  };
  const auto got = [&](const CellBlock& block, CellIndex index) {
    CellValue value{99.f, true};
    require(block.read(layout.locate(index).local_offset, value), "stored cell missing at " +
            std::to_string(index.x) + "," + std::to_string(index.y) + "," + std::to_string(index.z));
    return value;
  };
  put(*original, first, CellValue{1.5f, true});
  put(*original, second, CellValue{-0.25f, false});
  registry[Snapshot::block_index(first)] = original;
  const auto original_block = Snapshot::block_index(first);
  require(original.use_count() == 2, "fixture block is already shared");

  OccupancyDiagnostic off(ProfileMode::Off);
  {
    BlockUpdateBatch abandoned(layout, registry, off);
    abandoned.write(other, CellValue{0.5f, true});
    require(registry.size() == 1, "uncommitted write published a block");
    require(original->known_count() == 2 && got(*original, first).log_odds == 1.5f,
            "abandoned batch mutated the old block");
  }
  require(registry.size() == 1 && registry[original_block].get() == original.get(),
          "abandoned batch kept a published block");
  require(off.report().copied_blocks == 0 && off.report().copied_cells == 0,
          "Off recorded a block copy");

  OccupancyDiagnostic stages(ProfileMode::Stages);
  BlockRegistry published;
  {
    BlockUpdateBatch batch(layout, registry, stages);
    batch.write(first, CellValue{3.0f, true});
    batch.write(other, CellValue{0.5f, true});
    batch.write(second, CellValue{-0.25f, false});
    batch.write(other, CellValue{0.75f, true});
    batch.write(later, CellValue{0.1f, false});
    require(batch.touched_blocks() == 3, "interleaved writes created extra blocks");
    require(stages.report().copied_blocks == 1 && stages.report().copied_cells == 2,
            "an existing block was copied more than once");
    require(registry.size() == 1 && original->known_count() == 2 && got(*original, first).log_odds == 1.5f,
            "writes changed the source registry");
    require(original.use_count() == 2, "source block was updated in place");
    batch.commit(published);
    require_error("BLOCK_BATCH_CLOSED", [&] { batch.write(first, CellValue{9.0f, true}); });
    require_error("BLOCK_BATCH_CLOSED", [&] { batch.commit(published); });
  }
  require(registry.size() == 1 && got(*original, first).log_odds == 1.5f && original->known_count() == 2,
          "commit replaced the source registry");
  require(published.size() == 3, "commit missed a touched block");
  const auto& copied = *published[original_block];
  const auto copied_first = got(copied, first);
  const auto copied_second = got(copied, second);
  require(copied.known_count() == 2 && copied_first.log_odds == 3.0f && copied_first.occupied &&
              copied_second.log_odds == -0.25f && !copied_second.occupied,
          "slots in one block overwrote each other");
  require(got(*published[Snapshot::block_index(other)], other).log_odds == 0.75f,
          "a repeated write to another block did not keep the later value");
  require(published[Snapshot::block_index(later)]->known_count() == 1, "a new block kept a foreign slot");
  require(original.use_count() == 2, "commit aliased the immutable source block");
  std::cout << "PASS block batch lifecycle\n";
}

void BlockStorageChecks::check_representation() {
  CellBlock block;
  require(block.write(0, CellValue{0.f, false}), "zero log-odds was treated as unknown");
  require(!block.write(0, CellValue{-0.5f, false}), "a repeated write increased the known count");
  require(block.known_count() == 1, "known count changed on an update");
  CellValue stale{4.f, true};
  require(!block.read(1, stale) && stale.log_odds == 0.f && !stale.occupied,
          "reading an unknown slot left the previous value");
  CellValue known;
  require(block.read(0, known) && known.log_odds == -0.5f && !known.occupied,
          "a known free cell with updated log-odds was lost");
  std::size_t visits = 0;
  block.for_each_known([&](std::uint16_t offset, const CellValue& value) {
    ++visits;
    require(offset == 0 && value.log_odds == -0.5f, "known scan visited the wrong slot");
  });
  require(visits == 1, "known scan did not skip empty words");
  require_error("INVALID_BLOCK_OFFSET", [&] { block.write(kBlockCells, CellValue{}); });

  const BlockLayout layout({20, 12, 10});
  const CellIndex repeat{1, 2, 3};
  const CellIndex neighbor{2, 2, 3};
  const CellIndex far{9, 8, 8};
  std::map<CellIndex, CellValue> oracle;
  BlockRegistry registry;
  OccupancyDiagnostic off(ProfileMode::Off);
  {
    BlockUpdateBatch batch(layout, registry, off);
    const auto put = [&](CellIndex index, CellValue value) {
      batch.write(index, value);
      oracle[index] = value;
    };
    put({0, 0, 0}, CellValue{0.f, false});
    put(repeat, CellValue{0.2f, true});
    put(neighbor, CellValue{-1.f, false});
    put(repeat, CellValue{0.8f, true});
    put(far, CellValue{0.3f, true});
    batch.commit(registry);
  }
  std::map<CellIndex, CellValue> seen;
  std::size_t known_slots = 0;
  for (const auto& entry : registry) {
    entry.second->for_each_known([&](std::uint16_t offset, const CellValue& value) {
      const auto index = layout.cell_index(entry.first, offset);
      require(seen.emplace(index, value).second, "one known cell was stored twice");
      ++known_slots;
    });
  }
  require(known_slots == oracle.size(), "known slots differ from the reference map");
  for (const auto& item : oracle) {
    const auto found = seen.find(item.first);
    require(found != seen.end() && found->second.log_odds == item.second.log_odds &&
                found->second.occupied == item.second.occupied,
            "a known cell differs from the reference map");
  }

  GeometryDescriptor chamfer_box{"chamfer", "digest", 1.0, 1.0, 0.0, 1.0, Chamfer{0.0, 0.2, 0.4}};
  auto domain = std::make_shared<const UldDomain>(chamfer_box, 0.05);
  const BlockLayout chamfer_layout(domain->shape());
  CellIndex inactive{-1, -1, -1};
  CellIndex active_cell{-1, -1, -1};
  const auto shape = domain->shape();
  for (int z = 0; z < shape.z && (inactive.x < 0 || active_cell.x < 0); ++z) {
    for (int y = 0; y < shape.y && (inactive.x < 0 || active_cell.x < 0); ++y) {
      for (int x = 0; x < shape.x && (inactive.x < 0 || active_cell.x < 0); ++x) {
        const CellIndex index{x, y, z};
        if (!domain->active(index) && inactive.x < 0) inactive = index;
        if (domain->active(index) && active_cell.x < 0) active_cell = index;
      }
    }
  }
  require(inactive.x >= 0 && active_cell.x >= 0, "chamfer fixture has no inactive cell");
  BlockRegistry chamfer_blocks;
  const auto keep = [&](CellIndex index, CellValue value) {
    const auto address = chamfer_layout.locate(index);
    auto block_ptr = std::const_pointer_cast<CellBlock>(chamfer_blocks[address.block_index]);
    if (!block_ptr) {
      block_ptr = std::make_shared<CellBlock>();
      chamfer_blocks[address.block_index] = block_ptr;
    }
    block_ptr->write(address.local_offset, value);
  };
  keep(inactive, CellValue{0.8f, true});
  keep(active_cell, CellValue{0.2f, true});
  Snapshot chamfer_snapshot{{}, domain, chamfer_blocks};
  require(!chamfer_snapshot.cell(inactive).known, "a chamfer-exterior cell was returned as known");
  require(chamfer_snapshot.cell(active_cell).known, "an active chamfer cell was dropped");
  SurfaceProfile profile;
  const auto columns = chamfer_snapshot.surface(&profile);
  require(profile.scanned_cells == chamfer_snapshot.known_entry_count() && profile.scanned_cells == 2,
          "surface scanned empty slots or skipped a known cell");
  std::size_t column_known = 0;
  for (const auto& column : *columns) column_known += column.known_count;
  require(column_known == 1, "a chamfer-exterior cell entered a surface column");
  const auto active_column = static_cast<std::size_t>(active_cell.y) * shape.x + active_cell.x;
  require((*columns)[active_column].has_occupied, "active cell missed the surface column");

  compare_resolution_map(0.01);
  compare_resolution_map(0.02);
  compare_resolution_map(0.05);
  std::cout << "PASS contiguous block sizeof(CellBlock)=" << sizeof(CellBlock) << "\n";
}
}  // namespace uld_occ_observer
