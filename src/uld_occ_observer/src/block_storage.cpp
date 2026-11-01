#include "uld_occ_observer/block_storage.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include <limits>

namespace uld_occ_observer {
namespace {
std::uint64_t checked_mul(std::uint64_t left, std::uint64_t right) {
  if (left != 0 && right > std::numeric_limits<std::uint64_t>::max() / left)
    throw Error("BLOCK_INDEX_OVERFLOW");
  return left * right;
}
std::uint64_t checked_add(std::uint64_t left, std::uint64_t right) {
  if (right > std::numeric_limits<std::uint64_t>::max() - left)
    throw Error("BLOCK_INDEX_OVERFLOW");
  return left + right;
}
std::uint64_t blocks_on_axis(int extent) {
  if (extent <= 0) throw Error("INVALID_BLOCK_LAYOUT");
  return static_cast<std::uint64_t>(
      (static_cast<std::int64_t>(extent) + (kBlockEdge - 1)) / kBlockEdge);
}
}  // namespace

CellIndex BlockLayout::block_coordinate(CellIndex index) {
  if (index.x < 0 || index.y < 0 || index.z < 0) throw Error("CELL_INDEX_OUT_OF_RANGE");
  return {index.x / kBlockEdge, index.y / kBlockEdge, index.z / kBlockEdge};
}

BlockLayout::BlockLayout(CellIndex grid_shape) : grid_shape_(grid_shape) {
  blocks_x_ = blocks_on_axis(grid_shape.x);
  blocks_y_ = blocks_on_axis(grid_shape.y);
  blocks_z_ = blocks_on_axis(grid_shape.z);
  const auto count = checked_mul(checked_mul(blocks_x_, blocks_y_), blocks_z_);
  if (count > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
    throw Error("BLOCK_INDEX_OVERFLOW");
  block_count_ = static_cast<std::size_t>(count);
}

std::size_t BlockLayout::directory_index_of(CellIndex block) const {
  if (block.x < 0 || block.y < 0 || block.z < 0) throw Error("INVALID_BLOCK_ADDRESS");
  const auto bx = static_cast<std::uint64_t>(block.x);
  const auto by = static_cast<std::uint64_t>(block.y);
  const auto bz = static_cast<std::uint64_t>(block.z);
  if (bx >= blocks_x_ || by >= blocks_y_ || bz >= blocks_z_) throw Error("INVALID_BLOCK_ADDRESS");
  const auto row = checked_add(checked_mul(bz, blocks_y_), by);
  const auto index = checked_add(checked_mul(row, blocks_x_), bx);
  if (index >= block_count_) throw Error("BLOCK_INDEX_OVERFLOW");
  return static_cast<std::size_t>(index);
}

BlockAddress BlockLayout::locate(CellIndex index) const {
  if (index.x < 0 || index.y < 0 || index.z < 0 || index.x >= grid_shape_.x ||
      index.y >= grid_shape_.y || index.z >= grid_shape_.z)
    throw Error("CELL_INDEX_OUT_OF_RANGE");
  const auto block = block_coordinate(index);
  const auto lx = static_cast<std::int64_t>(index.x) -
                  static_cast<std::int64_t>(block.x) * kBlockEdge;
  const auto ly = static_cast<std::int64_t>(index.y) -
                  static_cast<std::int64_t>(block.y) * kBlockEdge;
  const auto lz = static_cast<std::int64_t>(index.z) -
                  static_cast<std::int64_t>(block.z) * kBlockEdge;
  if (lx < 0 || ly < 0 || lz < 0 || lx >= kBlockEdge || ly >= kBlockEdge || lz >= kBlockEdge)
    throw Error("CELL_INDEX_OUT_OF_RANGE");
  const auto packed = lx + static_cast<std::int64_t>(kBlockEdge) * ly +
                      static_cast<std::int64_t>(kBlockEdge * kBlockEdge) * lz;
  BlockAddress address;
  address.block_index = block;
  address.directory_index = directory_index_of(block);
  address.local_offset = static_cast<std::uint16_t>(packed);
  return address;
}

CellIndex BlockLayout::cell_index(CellIndex block, std::uint16_t offset) const {
  if (offset >= kBlockCells) throw Error("INVALID_BLOCK_ADDRESS");
  directory_index_of(block);
  const int lx = offset % kBlockEdge;
  const int ly = (offset / kBlockEdge) % kBlockEdge;
  const int lz = offset / (kBlockEdge * kBlockEdge);
  const auto x = static_cast<std::int64_t>(block.x) * kBlockEdge + lx;
  const auto y = static_cast<std::int64_t>(block.y) * kBlockEdge + ly;
  const auto z = static_cast<std::int64_t>(block.z) * kBlockEdge + lz;
  if (x < 0 || y < 0 || z < 0 || x >= grid_shape_.x || y >= grid_shape_.y || z >= grid_shape_.z)
    throw Error("INVALID_BLOCK_ADDRESS");
  return {static_cast<int>(x), static_cast<int>(y), static_cast<int>(z)};
}

BlockUpdateBatch::BlockUpdateBatch(const BlockLayout& layout, const BlockRegistry& registry,
                                   OccupancyDiagnostic& diagnostic)
    : layout_(layout), registry_(&registry), diagnostic_(&diagnostic) {
  directory_.assign(layout_.block_count(), nullptr);
}

void BlockUpdateBatch::write(CellIndex index, CellValue value) {
  if (committed_) throw Error("BLOCK_BATCH_CLOSED");
  const auto address = layout_.locate(index);
  auto* slot = directory_[address.directory_index];
  if (slot == nullptr) {
    std::shared_ptr<CellBlock> created;
    const auto existing = registry_->find(address.block_index);
    if (existing == registry_->end()) {
      created = std::make_shared<CellBlock>();
    } else {
      diagnostic_->block_copied(existing->second->known_count());
      created = std::make_shared<CellBlock>(*existing->second);
    }
    // Allocation-lifecycle accounting: every block object allocation (with
    // its make_shared control block) raises the class total at the site.
    AllocationTotals::instance().add(
        "block_allocations", sizeof(CellBlock) + kControlBlockBytes);
    touched_.push_back(std::move(created));
    touched_blocks_.push_back(address.block_index);
    slot = touched_.back().get();
    directory_[address.directory_index] = slot;
  }
  slot->write(address.local_offset, value);
}

void BlockUpdateBatch::commit(BlockRegistry& registry) {
  if (committed_) throw Error("BLOCK_BATCH_CLOSED");
  committed_ = true;
  for (std::size_t i = 0; i < touched_.size(); ++i) {
    // A brand-new key allocates a registry node; replacing an existing key
    // reuses its node and allocates nothing.
    if (registry.find(touched_blocks_[i]) == registry.end())
      AllocationTotals::instance().add(
          "block_directory_nodes", map_node_bytes<BlockRegistry::value_type>());
    registry[touched_blocks_[i]] = touched_[i];
  }
}
}  // namespace uld_occ_observer
