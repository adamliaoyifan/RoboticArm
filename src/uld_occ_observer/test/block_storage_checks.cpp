#include "block_storage_checks.hpp"
#include "uld_occ_observer/block_storage.hpp"
#include <iostream>
#include <limits>
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
  (*original)[first] = CellValue{1.5f, true};
  (*original)[second] = CellValue{-0.25f, false};
  registry[Snapshot::block_index(first)] = original;
  const auto original_block = Snapshot::block_index(first);
  require(original.use_count() == 2, "fixture block is already shared");

  OccupancyDiagnostic off(ProfileMode::Off);
  {
    BlockUpdateBatch abandoned(layout, registry, off);
    abandoned.write(other, CellValue{0.5f, true});
    require(registry.size() == 1, "uncommitted write published a block");
    require(original->size() == 2 && (*original)[first].log_odds == 1.5f, "abandoned batch mutated the old block");
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
    require(registry.size() == 1 && original->size() == 2 && (*original)[first].log_odds == 1.5f,
            "writes changed the source registry");
    require(original.use_count() == 2, "source block was updated in place");
    batch.commit(published);
    require_error("BLOCK_BATCH_CLOSED", [&] { batch.write(first, CellValue{9.0f, true}); });
    require_error("BLOCK_BATCH_CLOSED", [&] { batch.commit(published); });
  }
  require(registry.size() == 1 && (*original)[first].log_odds == 1.5f && original->size() == 2,
          "commit replaced the source registry");
  require(published.size() == 3, "commit missed a touched block");
  const auto& copied = *published[original_block];
  require(copied.size() == 2 && copied.at(first).log_odds == 3.0f && copied.at(first).occupied &&
              copied.at(second).log_odds == -0.25f && !copied.at(second).occupied,
          "slots in one block overwrote each other");
  require(published[Snapshot::block_index(other)]->at(other).log_odds == 0.75f,
          "a repeated write to another block did not keep the later value");
  require(published[Snapshot::block_index(later)]->size() == 1, "a new block kept a foreign slot");
  require(original.use_count() == 2, "commit aliased the immutable source block");
  std::cout << "PASS block batch lifecycle\n";
}
}  // namespace uld_occ_observer
