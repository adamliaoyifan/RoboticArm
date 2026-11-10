#include "query_content_digest.hpp"
#include "uld_occ_observer/observer.hpp"
#include <octomap/OcTree.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <iomanip>
#include <locale>
#include <sstream>

namespace uld_occ_observer {
namespace {
void write_surface(std::ostream& out, const char* name, const std::vector<SurfaceColumn>* surface) {
  if (!surface) {
    out << name << " absent\n";
    return;
  }
  out << name << ' ' << surface->size() << '\n';
  for (const auto& column : *surface) {
    out << column.x << ' ' << column.y << ' ' << column.active_count << ' '
        << column.known_count << ' ' << (column.has_occupied ? 1 : 0) << ' ';
    if (column.has_occupied) out << column.top_z;
    else out << '-';
    out << '\n';
  }
}
}  // namespace

std::string canonical_probability_digest(const std::string& tree_bytes) {
  std::istringstream input(tree_bytes, std::ios::binary);
  std::unique_ptr<octomap::AbstractOcTree> raw(octomap::AbstractOcTree::read(input));
  auto* tree = dynamic_cast<octomap::OcTree*>(raw.get());
  if (!tree || !input || !std::isfinite(tree->getResolution()) || tree->getResolution() <= 0)
    throw std::runtime_error("INVALID_PROBABILITY_TREE");
  // Validate before prune: normalization must not hide stale inner occupancy.
  for (auto it = tree->begin_tree(); it != tree->end_tree(); ++it) {
    if (!std::isfinite(it->getLogOdds()))
      throw std::runtime_error("NONFINITE_PROBABILITY_NODE");
    if (!tree->nodeHasChildren(&*it)) continue;
    float maximum = -std::numeric_limits<float>::infinity();
    for (unsigned child = 0; child < 8; ++child) {
      if (!tree->nodeChildExists(&*it, child)) continue;
      const float value = tree->getNodeChild(&*it, child)->getLogOdds();
      if (!std::isfinite(value)) throw std::runtime_error("NONFINITE_PROBABILITY_NODE");
      maximum = std::max(maximum, value);
    }
    if (it->getLogOdds() != maximum)
      throw std::runtime_error("INCONSISTENT_PROBABILITY_PARENT");
  }
  tree->prune();
  std::ostringstream output(std::ios::binary);
  if (!tree->write(output)) throw std::runtime_error("PROBABILITY_TREE_WRITE_FAILED");
  return sha256(output.str());
}

void check_probability_digest() {
  const auto bytes = [](const octomap::OcTree& tree) {
    std::ostringstream out(std::ios::binary);
    if (!tree.write(out)) throw std::runtime_error("DIGEST_TEST_WRITE_FAILED");
    return out.str();
  };
  const auto require = [](bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
  };
  octomap::OcTree tree(0.01);
  for (unsigned x = 0; x < 2; ++x)
    for (unsigned y = 0; y < 2; ++y)
      for (unsigned z = 0; z < 2; ++z)
        tree.setNodeValue(octomap::OcTreeKey(32768+x, 32768+y, 32768+z), 0.5f);
  tree.prune();
  const auto pruned = bytes(tree);
  tree.expand();
  const auto expanded = bytes(tree);
  require(pruned != expanded, "DIGEST_TEST_REQUIRES_DIFFERENT_STRUCTURES");
  require(canonical_probability_digest(pruned) == canonical_probability_digest(expanded),
          "DIGEST_EXPANSION_CHANGED_PROBABILITY");
  tree.setNodeValue(octomap::OcTreeKey(32768, 32768, 32768), 0.6f);
  require(canonical_probability_digest(pruned) != canonical_probability_digest(bytes(tree)),
          "DIGEST_MISSED_PROBABILITY_CHANGE");
  octomap::OcTree zero(0.01);
  const auto unknown = canonical_probability_digest(bytes(zero));
  zero.setNodeValue(octomap::OcTreeKey(32768, 32768, 32768), 0.0f);
  require(unknown != canonical_probability_digest(bytes(zero)), "DIGEST_MERGED_UNKNOWN_ZERO");
  zero.getRoot()->setLogOdds(0.25f);
  bool rejected = false;
  try { canonical_probability_digest(bytes(zero)); }
  catch (const std::runtime_error&) { rejected = true; }
  require(rejected, "DIGEST_ACCEPTED_INCONSISTENT_PARENT");
}

std::string query_content_sha256(const CellPage& roi,
                                 const std::vector<SurfaceColumn>* surface_first,
                                 const std::vector<SurfaceColumn>* surface_second) {
  std::ostringstream out;
  out.imbue(std::locale::classic());
  out << std::setprecision(17);
  out << "roi " << roi.cells.size() << '\n';
  for (const auto& cell : roi.cells) {
    out << cell.index.x << ' ' << cell.index.y << ' ' << cell.index.z << ' '
        << (cell.active ? 1 : 0) << ' ' << (cell.known ? 1 : 0) << ' '
        << (cell.value.occupied ? 1 : 0) << ' ';
    if (cell.known) out << static_cast<double>(cell.value.log_odds);
    else out << '-';
    out << '\n';
  }
  write_surface(out, "surface_first", surface_first);
  write_surface(out, "surface_second", surface_second);
  return sha256(out.str());
}
}  // namespace uld_occ_observer
