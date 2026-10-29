#pragma once
#include "uld_occ_observer/snapshot.hpp"
#include <cstddef>
#include <cstdint>
#include <vector>

namespace uld_occ_observer {
// Fixed block edge. Block coordinates, directory slots, and local offsets share
// these values. There is no second edge length in Snapshot::block_index.
inline constexpr int kBlockEdge = 8;
inline constexpr int kBlockCells = kBlockEdge * kBlockEdge * kBlockEdge;

struct BlockAddress {
  CellIndex block_index;
  std::size_t directory_index{};
  std::uint16_t local_offset{};
};

// Grid-shape addressing. locate rejects negatives and cells outside the grid
// before any unsigned conversion. cell_index rejects invalid blocks, offsets,
// and padding slots in a partial edge block.
class BlockLayout {
 public:
  explicit BlockLayout(CellIndex grid_shape);
  BlockAddress locate(CellIndex index) const;
  CellIndex cell_index(CellIndex block, std::uint16_t offset) const;
  std::size_t block_count() const { return block_count_; }
  CellIndex grid_shape() const { return grid_shape_; }
  static CellIndex block_coordinate(CellIndex index);

 private:
  std::size_t directory_index_of(CellIndex block) const;
  CellIndex grid_shape_;
  std::uint64_t blocks_x_{}, blocks_y_{}, blocks_z_{};
  std::size_t block_count_{};
};

// One frame of block writes. The directory holds non-owning addresses of blocks
// owned by touched_. Commit publishes shared_ptr<const CellBlock>. A later write
// or commit is rejected. Destroying an uncommitted batch releases only its
// temporary blocks. Old registry blocks are copied, never updated in place.
class BlockUpdateBatch {
 public:
  BlockUpdateBatch(const BlockLayout& layout, const BlockRegistry& registry,
                   OccupancyDiagnostic& diagnostic);
  BlockUpdateBatch(const BlockUpdateBatch&) = delete;
  BlockUpdateBatch& operator=(const BlockUpdateBatch&) = delete;
  void write(CellIndex index, CellValue value);
  void commit(BlockRegistry& registry);
  std::size_t touched_blocks() const { return touched_.size(); }

 private:
  BlockLayout layout_;
  const BlockRegistry* registry_;
  OccupancyDiagnostic* diagnostic_;
  std::vector<CellBlock*> directory_;
  std::vector<std::shared_ptr<CellBlock>> touched_;
  std::vector<CellIndex> touched_blocks_;
  bool committed_{false};
};
}  // namespace uld_occ_observer
