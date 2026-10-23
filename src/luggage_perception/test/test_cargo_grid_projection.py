"""Vectorized projection compared with the legacy scalar semantics."""
import random
import pytest
from luggage_perception.cargo_volume_mapper import CargoVolumeMapper


@pytest.mark.parametrize('resolution',[.01,.02,.05])
def test_random_mixed_columns_match_scalar_oracle(resolution):
    m=CargoVolumeMapper((.2,.15,.3),(0.,0.,.15),0.,resolution)
    rng=random.Random(193)
    for i in range(len(m._grid)):
        m._grid[i]=rng.randrange(3);m._source[i]=rng.randrange(3)
        m._labels[i]=rng.randrange(4);m._active[i]=rng.randrange(2)
    s=m.surface_map_2d()
    for ix in range(m.nx):
        for iy in range(m.ny):
            ids=[m._index(ix,iy,z) for z in range(m.nz) if m._active[m._index(ix,iy,z)]]
            occ=[i for i in ids if m._grid[i]==2]
            unknown=any(m._grid[i]==0 for i in ids)
            labels={}
            for i in occ:
                if m._labels[i]>0:labels[m._labels[i]]=labels.get(m._labels[i],0)+1
            expected_state='occupied' if occ else ('unknown' if unknown else 'free')
            height=min((occ[-1]//(m.nx*m.ny)+1)*resolution,m.inner_h) if occ else 0.
            confidence=('geometry' if m._source[occ[-1]]==2 else 'sensor') if occ else ('none' if unknown else 'sensor')
            assert s['state'][ix][iy]==expected_state
            assert s['height'][ix][iy]==height
            assert s['confidence'][ix][iy]==confidence
            assert s['semantic_label'][ix][iy]==(max(labels,key=labels.get) if labels else 0)
            assert s['known_ratio'][ix][iy]==(sum(m._grid[i]!=0 for i in ids)/len(ids) if ids else 0.)
