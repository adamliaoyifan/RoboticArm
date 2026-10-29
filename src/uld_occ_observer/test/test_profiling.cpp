#include "uld_occ_observer/observer.hpp"
#include "fusion_profile_checks.hpp"
#include "block_storage_checks.hpp"
#include "uld_occ_observer/block_storage.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <thread>

using namespace uld_occ_observer;
void require(bool value, const char* message) { if (!value) throw Error(message); }
void store_cell(const UldDomain& domain, CellBlock& block, CellIndex index, CellValue value) {
  BlockLayout layout(domain.shape());
  block.write(layout.locate(index).local_offset, value);
}
OpticalObservation observation(std::string id, std::int64_t stamp) {
  OpticalObservation out;
  out.identity = {"test",std::move(id),"optical","test_clock","fixture_geometry",1,stamp};
  out.uld_from_optical.stamp_ns = stamp;
  out.points_m = {{0,0,.6},{0,0,.6},{.8,0,.8},{0,0,.4},
                 {std::numeric_limits<double>::quiet_NaN(),0,1}};
  return out;
}
void check_profile(const UpdateReceipt& r) {
  const auto& p = r.profile;
  const auto nested = [&](ProfileStage s) { return p.milliseconds[static_cast<std::size_t>(s)]; };
  if (!p.detailed()) {
    require(nested(ProfileStage::RayGeneration)==0 && nested(ProfileStage::RayFilterDedup)==0 &&
                nested(ProfileStage::TreeUpdate)==0 && nested(ProfileStage::BlockMaintenance)==0,
            "nested spans ran outside Detailed");
  }
  if (!p.enabled()) {
    for (auto value : p.milliseconds) require(value==0,"disabled timing nonzero");
    require(p.rays_attempted==0,"disabled counter nonzero"); return;
  }
  double exclusive = 0;
  for (std::size_t i=0;i<kProfileStages;++i) {
    require(p.milliseconds[i]>=0 && std::isfinite(p.milliseconds[i]),"invalid duration");
    if (i<kExclusiveStages) exclusive+=p.milliseconds[i];
  }
  require(exclusive<=r.total_ms+.1 && p.residual_ms>=-.1,"exclusive spans exceed total");
  const auto duration = [&](ProfileStage s) {return p.milliseconds[static_cast<std::size_t>(s)];};
  require(duration(ProfileStage::RayGeneration)+duration(ProfileStage::RayFilterDedup)<=duration(ProfileStage::RayPreparation)+.1,"ray nested spans exceed parent");
  require(duration(ProfileStage::TreeUpdate)+duration(ProfileStage::BlockMaintenance)<=duration(ProfileStage::Fusion)+.1,"fusion nested spans exceed parent");
  if (r.status!="INTEGRATED") return;
  require(p.rays_attempted==r.accepted_points,"ray count mismatch");
  require(p.ray_keys_generated==p.active_ray_key_visits+p.inactive_ray_key_visits,"key accounting mismatch");
  require(p.rays_with_active_keys+p.rays_without_active_keys==p.rays_attempted,"ray classification mismatch");
  require(p.active_ray_key_visits==p.free_insert_duplicates+p.free_unique_before_hit_removal,"dedup accounting mismatch");
  require(p.copied_blocks<=r.rebuilt_blocks,"too many block copies");
  require(p.free_unique_before_hit_removal>=r.free_keys,"hit removal grew free set");
}
void check_surface_matches_cell_scan() {
  GeometryDescriptor box{"box","digest",0.8,0.8,0,1.2,{}};
  auto domain = std::make_shared<const UldDomain>(box, 0.05);
  const CellIndex low{2,2,1};
  const CellIndex high{2,2,4};
  auto both = std::make_shared<CellBlock>();
  store_cell(*domain, *both, low, CellValue{0.8f, true});
  store_cell(*domain, *both, high, CellValue{0.8f, true});
  BlockRegistry blocks;
  blocks[Snapshot::block_index(low)] = both;
  Snapshot populated{{}, domain, blocks};
  SurfaceProfile profile;
  const auto columns = populated.surface(&profile);
  const auto shape = domain->shape();
  std::vector<SurfaceColumn> oracle;
  for (int y = 0; y < shape.y; ++y) {
    for (int x = 0; x < shape.x; ++x) {
      SurfaceColumn column;
      column.x = x; column.y = y;
      for (int z = 0; z < shape.z; ++z) {
        const auto sample = populated.cell({x,y,z});
        column.active_count += sample.active;
        column.known_count += sample.known;
        if (sample.known && sample.value.occupied) {
          column.has_occupied = true;
          column.top_z = std::min(domain->descriptor().ceiling_z,
              domain->origin().z + (z+1)*domain->resolution());
        }
      }
      oracle.push_back(column);
    }
  }
  require(columns->size()==oracle.size() && profile.scanned_cells==populated.known_entry_count() &&
              profile.scanned_cells==2, "surface shape");
  require(profile.domain_init_collected && profile.domain_init_performed, "populated surface skipped init observation");
  const auto product = static_cast<std::uint64_t>(shape.x)*static_cast<std::uint64_t>(shape.y)*shape.z;
  require(profile.domain_init_scanned_cells==product, "domain init did not visit every geometry cell");
  require(profile.domain_init_ms>0 && profile.total_ms+0.05>=profile.domain_init_ms &&
              profile.build_ms+0.05>=profile.domain_init_ms,
          "first surface total excluded domain initialization");
  for (std::size_t i = 0; i < oracle.size(); ++i) {
    const auto& got = (*columns)[i];
    const auto& expect = oracle[i];
    require(got.x==expect.x && got.y==expect.y &&
                got.active_count==expect.active_count && got.known_count==expect.known_count &&
                got.has_occupied==expect.has_occupied && got.top_z==expect.top_z,
            "block surface differed from a full cell scan");
  }
  // A constructed snapshot omits a stored cell. This checks column lowering, not a production remove API.
  auto lower = std::make_shared<CellBlock>();
  store_cell(*domain, *lower, low, CellValue{0.8f, true});
  BlockRegistry only_low;
  only_low[Snapshot::block_index(low)] = lower;
  Snapshot removed{{}, domain, only_low};
  const auto dropped = removed.surface();
  const auto column = static_cast<std::size_t>(low.y) * shape.x + low.x;
  require((*dropped)[column].has_occupied &&
              (*dropped)[column].top_z == domain->origin().z + (low.z+1)*domain->resolution() &&
              (*dropped)[column].top_z < (*columns)[column].top_z,
          "removing the higher occupied cell kept the old top");
  GeometryDescriptor chamfer_box{"chamfer","digest",1.0,1.0,0.0,1.0,Chamfer{0.0,0.2,0.4}};
  auto chamfer = std::make_shared<const UldDomain>(chamfer_box, 0.05);
  Snapshot empty{{}, chamfer, {}};
  const auto built = empty.surface();
  const auto chamfer_shape = chamfer->shape();
  for (int y = 0; y < chamfer_shape.y; ++y) {
    for (int x = 0; x < chamfer_shape.x; ++x) {
      std::size_t active = 0;
      for (int z = 0; z < chamfer_shape.z; ++z) active += empty.cell({x,y,z}).active;
      require((*built)[static_cast<std::size_t>(y)*chamfer_shape.x+x].active_count==active &&
                  (*built)[static_cast<std::size_t>(y)*chamfer_shape.x+x].known_count==0,
              "chamfer column active count drifted");
    }
  }
  auto known_free = std::make_shared<CellBlock>();
  const CellIndex free_index{1, 1, 2};
  store_cell(*domain, *known_free, free_index, CellValue{-0.4f, false});
  BlockRegistry free_blocks;
  free_blocks[Snapshot::block_index(free_index)] = known_free;
  Snapshot free_only{{}, domain, free_blocks};
  const auto free_columns = free_only.surface();
  const auto free_column = static_cast<std::size_t>(free_index.y) * shape.x + free_index.x;
  require((*free_columns)[free_column].known_count==1 && !(*free_columns)[free_column].has_occupied &&
              (*free_columns)[free_column].top_z==0,
          "known free cell became occupied");
  require((*free_columns)[0].known_count==0 && !(*free_columns)[0].has_occupied, "empty column invented occupancy");
  auto lowered = std::make_shared<CellBlock>();
  store_cell(*domain, *lowered, low, CellValue{0.8f, true});
  store_cell(*domain, *lowered, high, CellValue{0.2f, false});
  BlockRegistry lowered_blocks;
  lowered_blocks[Snapshot::block_index(low)] = lowered;
  Snapshot became_free{{}, domain, lowered_blocks};
  const auto lowered_columns = became_free.surface();
  require((*lowered_columns)[column].known_count==2 && (*lowered_columns)[column].has_occupied &&
              (*lowered_columns)[column].top_z==domain->origin().z+(low.z+1)*domain->resolution() &&
              (*lowered_columns)[column].top_z<domain->origin().z+(high.z+1)*domain->resolution(),
          "a higher free cell kept the old occupied top");
  GeometryDescriptor capped{"ceiling", "digest", 0.4, 0.4, 0.0, 0.99, {}};
  auto capped_domain = std::make_shared<const UldDomain>(capped, 0.05);
  const auto capped_shape = capped_domain->shape();
  const CellIndex top_cell{capped_shape.x/2, capped_shape.y/2, capped_shape.z-1};
  require(capped_domain->active(top_cell), "ceiling fixture has no active top cell");
  const double raw_top = capped_domain->origin().z+(top_cell.z+1)*capped_domain->resolution();
  require(raw_top>capped.ceiling_z, "ceiling fixture does not truncate");
  auto top_block = std::make_shared<CellBlock>();
  store_cell(*capped_domain, *top_block, top_cell, CellValue{0.8f, true});
  BlockRegistry top_blocks;
  top_blocks[Snapshot::block_index(top_cell)] = top_block;
  Snapshot capped_snapshot{{}, capped_domain, top_blocks};
  const auto capped_columns = capped_snapshot.surface();
  const auto capped_index = static_cast<std::size_t>(top_cell.y)*capped_shape.x+top_cell.x;
  require((*capped_columns)[capped_index].has_occupied &&
              (*capped_columns)[capped_index].top_z==capped.ceiling_z &&
              (*capped_columns)[capped_index].top_z<raw_top,
          "occupied top was not truncated at the ceiling");
}

void check_domain_init_once() {
  GeometryDescriptor box{"init", "digest", 1.0, 1.0, 0.0, 1.0, {}};
  auto domain = std::make_shared<const UldDomain>(box, 0.05);
  const auto shape = domain->shape();
  const auto product = static_cast<std::uint64_t>(shape.x)*shape.y*shape.z;
  SurfaceProfile first, second_snapshot, cache;
  Snapshot populated{{}, domain, {}};
  populated.surface(&first);
  require(first.domain_init_performed && first.domain_init_scanned_cells==product && first.scanned_cells==0,
          "empty first surface did not record a geometry scan");
  Snapshot rebuilt{{}, domain, {}};
  rebuilt.surface(&second_snapshot);
  require(second_snapshot.domain_init_collected && !second_snapshot.domain_init_performed &&
              second_snapshot.domain_init_scanned_cells==0 && second_snapshot.domain_init_ms==0 &&
              second_snapshot.scanned_cells==0 && !second_snapshot.cache_hit,
          "a new snapshot repeated domain initialization");
  rebuilt.surface(&cache);
  require(cache.cache_hit && cache.scanned_cells==0 && cache.domain_init_scanned_cells==0 &&
              !cache.domain_init_performed,
          "cache hit scanned blocks or geometry");
  auto fresh = std::make_shared<const UldDomain>(box, 0.05);
  SurfaceProfile again;
  Snapshot other{{}, fresh, {}};
  other.surface(&again);
  require(again.domain_init_performed && again.domain_init_scanned_cells==product,
          "a new domain reused the previous initialization");
  auto off_domain = std::make_shared<const UldDomain>(box, 0.05);
  Snapshot off_snapshot{{}, off_domain, {}};
  off_snapshot.surface(nullptr);
  SurfaceProfile after_off;
  Snapshot after{{}, off_domain, {}};
  after.surface(&after_off);
  require(after_off.domain_init_collected && !after_off.domain_init_performed &&
              after_off.domain_init_scanned_cells==0,
          "an unprofiled first query fabricated a geometry scan");
  SurfaceProfile blank;
  require(!blank.domain_init_collected, "a default profile claims domain init was collected");
  auto shared = std::make_shared<const UldDomain>(box, 0.05);
  Snapshot left{{}, shared, {}};
  Snapshot right{{}, shared, {}};
  SurfaceProfile left_profile, right_profile;
  std::thread first_call([&] { left.surface(&left_profile); });
  std::thread second_call([&] { right.surface(&right_profile); });
  first_call.join();
  second_call.join();
  require(int(left_profile.domain_init_performed)+int(right_profile.domain_init_performed)==1,
          "concurrent first queries did not record exactly one initialization");
  const auto& ran = left_profile.domain_init_performed ? left_profile : right_profile;
  const auto& waited = left_profile.domain_init_performed ? right_profile : left_profile;
  require(ran.domain_init_scanned_cells==product && waited.domain_init_scanned_cells==0,
          "a waiting caller reported the geometry scan");
  const auto left_columns = left.surface();
  const auto right_columns = right.surface();
  require(left_columns->size()==right_columns->size(), "concurrent surfaces differed in shape");
  for (std::size_t i = 0; i < left_columns->size(); ++i)
    require((*left_columns)[i].active_count==(*right_columns)[i].active_count, "concurrent active counts differed");
}

void check_retained_snapshot() {
  GeometryDescriptor geometry{"fixture_geometry", "fixture_digest", .8, .8, 0, 1.2, {}};
  ObservationStore store(geometry, .05, {}, "test_clock", "config");
  require(store.integrate(observation("kept", 1000)).status=="INTEGRATED", "seed frame rejected");
  const auto old_snapshot = store.snapshot();
  const auto old_columns = *old_snapshot->surface();
  auto later = observation("later", 2000);
  later.points_m = {{0.2, 0.05, 0.95}, {-0.1, 0.0, 0.7}};
  require(store.integrate(later).status=="INTEGRATED", "second frame rejected");
  const auto updated = store.snapshot();
  require(updated->identity().map_revision>old_snapshot->identity().map_revision, "map revision did not advance");
  const auto retained = *old_snapshot->surface();
  require(retained.size()==old_columns.size(), "retained snapshot changed shape");
  for (std::size_t i = 0; i < retained.size(); ++i) {
    require(retained[i].x==old_columns[i].x && retained[i].y==old_columns[i].y &&
                retained[i].active_count==old_columns[i].active_count &&
                retained[i].known_count==old_columns[i].known_count &&
                retained[i].has_occupied==old_columns[i].has_occupied &&
                retained[i].top_z==old_columns[i].top_z,
            "an old snapshot changed after a later update");
  }
  const auto fresh = *updated->surface();
  std::size_t old_known = 0, new_known = 0;
  for (const auto& column : old_columns) old_known += column.known_count;
  for (const auto& column : fresh) new_known += column.known_count;
  require(new_known!=old_known, "the updated snapshot repeated the old columns");
}

int main() {
  try {
    GeometryDescriptor geometry{"fixture_geometry","fixture_digest",.8,.8,0,1.2,{}};
    ObservationStore off(geometry,.05,{},"test_clock","config");
    ObservationStore stages(geometry,.05,{},"test_clock","config");
    ObservationStore detailed(geometry,.05,{},"test_clock","config");
    std::vector<OpticalObservation> cases;
    cases.push_back(observation("valid",1000));
    auto empty=observation("empty",2000); empty.points_m={{0,0,.1}}; cases.push_back(empty);
    cases.push_back(cases.front());
    auto conflict=cases.front(); conflict.points_m.push_back({.1,0,1}); cases.push_back(conflict);
    cases.push_back(observation("old",500));
    auto bad_tf=observation("tf",3000); bad_tf.uld_from_optical.stamp_ns=1; cases.push_back(bad_tf);
    auto exterior=observation("exterior",4000); exterior.uld_from_optical.translation={3,0,0}; cases.push_back(exterior);
    for (int i=0;i<32;++i) cases.push_back(observation("sat"+std::to_string(i),5000+i));
    std::shared_ptr<const Snapshot> old_snapshot;
    for (std::size_t i=0;i<cases.size();++i) {
      const auto a=off.integrate(cases[i]);
      const auto b=stages.integrate(cases[i],ProfileMode::Stages);
      const auto c=detailed.integrate(cases[i],ProfileMode::Detailed);
      check_profile(a); check_profile(b); check_profile(c);
      require(a.status==b.status && b.status==c.status,"mode changed status");
      require(a.identity.map_revision==b.identity.map_revision && b.identity.map_revision==c.identity.map_revision,"mode changed map revision");
      require(a.identity.observed_revision==b.identity.observed_revision && b.identity.observed_revision==c.identity.observed_revision,"mode changed observed revision");
      require(a.free_keys==b.free_keys && b.free_keys==c.free_keys && a.hit_keys==c.hit_keys,"mode changed keys");
      require(a.changed_keys==b.changed_keys && b.changed_keys==c.changed_keys,"mode changed probability count");
      const auto bytes=off.checkpoint().tree_bytes;
      require(bytes==stages.checkpoint().tree_bytes && bytes==detailed.checkpoint().tree_bytes,"mode changed full probability tree");
      if (i==0) {
        require(c.profile.inactive_ray_key_visits>0 && c.profile.free_insert_duplicates>0,"fixture missed exterior/duplicate paths");
        require(c.changed_keys>0 && c.free_keys>0,"release cleared the scan before apply");
        require(c.profile.milliseconds[static_cast<std::size_t>(ProfileStage::RayFilterDedup)]>0,
                "detailed nested ray span did not record");
        require(b.profile.milliseconds[static_cast<std::size_t>(ProfileStage::RayFilterDedup)]==0,
                "stages timed a detailed nested span");
        old_snapshot=detailed.snapshot();
      }
      if (i==6) require(c.profile.rays_without_active_keys==c.accepted_points,"exterior-ray fixture misclassified");
      if (i==cases.size()-1) require(c.changed_keys==0 && c.rebuilt_blocks==0,"fixture not saturated");
    }
    SurfaceProfile first,second;
    auto snapshot=detailed.snapshot();
    const auto surface=snapshot->surface(&first);
    const auto again=snapshot->surface(&second);
    require(!first.cache_hit && first.scanned_cells>0,"first surface not measured");
    require(second.cache_hit && second.scanned_cells==0 && surface==again,"cache hit changed output or scanned");
    require(first.build_ms<=first.total_ms+.1 && second.build_ms==0,"surface spans inconsistent");
    OccupancyDiagnostic surface_session(ProfileMode::Off);
    surface_session.record_surface(first);
    require(surface_session.report().surface_recorded &&
                surface_session.report().surface.scanned_cells==first.scanned_cells,
            "surface sample was not recorded");
    require(old_snapshot->identity().map_revision<=snapshot->identity().map_revision,"old snapshot changed");
    require(first.domain_init_collected && second.domain_init_scanned_cells==0 && second.scanned_cells==0,
            "cache hit retained a scan count");
    check_surface_matches_cell_scan();
    check_domain_init_once();
    check_retained_snapshot();
    BlockStorageChecks storage_checks;
    storage_checks.check_addressing();
    storage_checks.check_batch_lifecycle();
    storage_checks.check_representation();
    FusionProfileChecks fusion_checks;
    fusion_checks.check_phase_lifecycle();
    fusion_checks.check_modes_and_equivalence();
    fusion_checks.check_sequence_and_failure();
    fusion_checks.check_directory_readback();
    fusion_checks.check_apply_batch();
    std::cout << "PASS profiling semantic equivalence: " << cases.size() << " cases x3 modes; counters/timings/cache paths\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 3; }
}
