#include "query_content_digest.hpp"
#include "uld_occ_observer/observer.hpp"
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
