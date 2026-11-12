#pragma once
#include "uld_occ_observer/snapshot.hpp"

namespace uld_occ_observer {
// Builds source blocks only: a declaration must not bias sensor log odds.
BlockRegistry initial_empty_blocks(const UldDomain& domain, std::size_t& known);
bool declaration_only(const SnapshotIdentity& identity);
std::int64_t evidence_stamp(const SnapshotIdentity& identity);
}
