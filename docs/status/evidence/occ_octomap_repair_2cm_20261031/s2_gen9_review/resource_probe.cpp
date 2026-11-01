#include "uld_occ_observer/configuration.hpp"
#include "uld_occ_observer/snapshot_repository.hpp"
#include "uld_occ_observer/resource_usage.hpp"
#include <iostream>
using namespace uld_occ_observer;
int main() {
  auto& totals=AllocationTotals::instance();
  totals.reset_for_test();
  {
    auto first=std::make_shared<Snapshot>(SnapshotIdentity{},nullptr,BlockRegistry{});
    const auto before=totals.live("surface_caches");
    { auto second=first->with_identity(SnapshotIdentity{});
      std::cout<<"shared_cache same="<<(first->surface_cache()==second->surface_cache())
               <<" before="<<before<<" during="<<totals.live("surface_caches")<<"\n";
    }
  }
  std::cout<<"shared_cache_after_last_owner="<<totals.live("surface_caches")<<"\n";
  totals.reset_for_test();
  const auto cfg=load_configuration(
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/geometry_cuboid.yaml",
    "/home/adamliao/work/elfin_humble_ws/docs/status/evidence/occ_o6_o7_20261029/run01/fixtures/config_cuboid_r0p05.yaml");
  ObservationStore store(cfg.geometry,cfg.resolution,cfg.depth,cfg.clock_domain,cfg.digest,false);
  OpticalObservation obs;
  obs.identity.sensor_id="cam0"; obs.identity.observation_id="audit";
  obs.identity.optical_frame="d555_color_optical_frame";
  obs.identity.clock_domain=cfg.clock_domain;obs.identity.geometry_hash=cfg.geometry.geometry_hash;
  obs.identity.epoch=1;obs.identity.stamp_ns=1000000000;
  obs.uld_from_optical.rotation={0,0,1,-1,0,0,0,-1,0};
  obs.uld_from_optical.translation={-1.15,0,1.20};obs.uld_from_optical.stamp_ns=1000000000;
  obs.points_m.push_back({0,0,1.0});
  std::cout<<"integrate="<<store.integrate(obs).status<<"\n";
  SnapshotRepository repo(store);
  auto lease=repo.acquire(2000000000,5000000000,true);
  auto snapshot=repo.get(lease.handle);
  auto columns=snapshot->surface();
  auto bytes=columns->capacity()*sizeof(SurfaceColumn);
  repo.release(lease.handle); store.reset(1); snapshot.reset();
  auto usage=repo.usage();
  std::cout<<"external_surface_columns="<<columns->size()<<" retained_buffer_bytes="<<bytes
           <<" reported_cache="<<usage.cache_bytes<<" tracker_live="<<totals.live("surface_caches")<<"\n";
}
