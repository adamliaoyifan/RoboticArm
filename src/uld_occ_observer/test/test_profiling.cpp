#include "uld_occ_observer/observer.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

using namespace uld_occ_observer;
void require(bool value, const char* message) { if (!value) throw Error(message); }
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
  if (!p.detailed()) {
    require(p.domain_check_ms==0 && p.free_insert_ms==0 && p.occupied_insert_ms==0 &&
                p.hit_erase_ms==0 && p.release_free_ms==0 && p.release_occupied_ms==0,
            "split timers ran outside Detailed");
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
  (*both)[low] = CellValue{0.8f, true};
  (*both)[high] = CellValue{0.8f, true};
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
  require(columns->size()==oracle.size() && profile.scanned_cells==2, "surface shape");
  for (std::size_t i = 0; i < oracle.size(); ++i) {
    const auto& got = (*columns)[i];
    const auto& expect = oracle[i];
    require(got.active_count==expect.active_count && got.known_count==expect.known_count &&
                got.has_occupied==expect.has_occupied && got.top_z==expect.top_z,
            "block surface differed from a full cell scan");
  }
  auto lower = std::make_shared<CellBlock>();
  (*lower)[low] = CellValue{0.8f, true};
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
        require(c.profile.domain_check_ms>0 && c.profile.free_insert_ms>0 && c.profile.hit_erase_ms>0,
                "detailed split timers did not record");
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
    check_surface_matches_cell_scan();
    std::cout << "PASS profiling semantic equivalence: " << cases.size() << " cases x3 modes; counters/timings/cache paths\n";
    return 0;
  } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 3; }
}
