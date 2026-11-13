"""Constructed calibrated scenes and lossless boundary capture, never calibration."""
from dataclasses import asdict, fields, is_dataclass
import hashlib
import json
import os
from pathlib import Path
import re
from types import SimpleNamespace

import numpy as np
import pytest

from luggage_perception.observed_suction_domain import ObservedSuctionDomain
from luggage_perception.pickup_view_association import (
    BufferedPickupRgb, PickupAssociationInputs, ReportedPickupCandidate,
)
from luggage_perception.pickup_view_evidence import PickupViewAcquisition, PickupViewEvidence
from luggage_perception.stationary_pickup_association import (
    PickupCandidateCorrespondence, StationaryPickupAssociation, StationaryPickupPolicy,
    associate_stationary_pickup,
)


SETTLED_NS = 2_000_000_000


def fixture_policy(**changes):
    values = dict(depth_residual_m=.005, rgb_l2_difference=20., bbox_margin_pixels=0.,
                  min_paired_points=6, min_fraction=.8, max_samples=256,
                  provenance='constructed-16x16-fixture-only; not sensor calibration')
    return StationaryPickupPolicy(**{**values, **changes})


def candidate(*, bbox=(0, 0, 16, 16), held=False):
    return ReportedPickupCandidate(0, 'constructed box', .9, bbox, held)


def planar_view(*, newer=False, camera_x=None, depth=None, rgb=None, region=None,
                candidates=None, selected=0, with_rgb=True, domain_changes=None):
    stamp = 3_000_000_000 if newer else 1_000_000_000
    identity = 'new-detection' if newer else 'old-detection'
    domain = ObservedSuctionDomain(
        box_id=identity, instance_id=identity, generation=8 if newer else 2,
        stamp_ns=stamp, frame_id='world', contact_model_version=1,
        contact_model_hash='constructed-contact-model', box_xyz_world=(0., 0., .9),
        box_quaternion_xyzw=(0., 0., 0., 1.), box_wdh=(.5, .5, .2),
        u_axis_world=(1., 0., 0.), v_axis_world=(0., 1., 0.), normal_world=(0., 0., 1.),
        plane_offset_world_m=-1., origin_uv_top_m=(-.25, -.25), cell_size_m=.05,
        shape=(10, 10), support=bytes([1])*100,
    )
    if domain_changes:
        from dataclasses import replace
        domain = replace(domain, **domain_changes)
    camera_x = (.05 if newer else 0.) if camera_x is None else camera_x
    matrix = np.diag([1., -1., -1., 1.]); matrix[:3, 3] = (camera_x, 0., 2.)
    yy, xx = np.indices((16, 16))
    # Texture follows WORLD geometry, so translated cameras see shifted colours.
    if rgb is None:
        world_x = (xx-7.5)/40.+camera_x
        world_y = -(yy-7.5)/40.
        rgb = np.stack((128+world_x*100, 128+world_y*100, np.full_like(world_x, 80)), axis=-1)
        rgb = np.rint(rgb).clip(0, 255).astype(np.uint8)
    inputs = None
    if with_rgb:
        inputs = PickupAssociationInputs(
            BufferedPickupRgb.capture(rgb, stamp_ns=domain.stamp_ns,
                optical_frame_id='fixture_optical', encoding='rgb8'),
            tuple(candidates or (candidate(),)), selected, 17 if newer else 4,
            domain.generation, domain.instance_id, 'constructed-fixture',
        )
    acquisition = PickupViewAcquisition.capture(
        np.full((16, 16), 1000, np.uint16) if depth is None else depth,
        np.ones((16, 16), bool) if region is None else region,
        SimpleNamespace(fx=40., fy=40., cx=7.5, cy=7.5), matrix,
        stamp_ns=domain.stamp_ns, optical_frame_id='fixture_optical',
        instance_id=domain.instance_id, generation=domain.generation,
        region_source='detection_bbox', association_inputs=inputs,
    )
    return domain, acquisition.bind(domain)


def planar_case(**new_options):
    return (*planar_view(), *planar_view(newer=True, **new_options))


def encode(value):
    if is_dataclass(value):
        return dict(__type__=type(value).__name__, fields={f.name: encode(getattr(value, f.name))
                                                        for f in fields(value)})
    if isinstance(value, bytes):
        return dict(__bytes_hex__=value.hex())
    if isinstance(value, (tuple, list)):
        return [encode(v) for v in value]
    if isinstance(value, dict):
        return {str(k): encode(v) for k, v in value.items()}
    return value


def decode(value):
    """Lossless replay for captured domain/view/policy/association dataclasses."""
    if isinstance(value, list):
        return tuple(decode(v) for v in value)
    if isinstance(value, dict):
        if '__bytes_hex__' in value:
            return bytes.fromhex(value['__bytes_hex__'])
        if '__type__' in value:
            classes = (ObservedSuctionDomain, BufferedPickupRgb, PickupAssociationInputs,
                       ReportedPickupCandidate, PickupViewAcquisition, PickupViewEvidence,
                       StationaryPickupPolicy, PickupCandidateCorrespondence, StationaryPickupAssociation)
            cls = next(c for c in classes if c.__name__ == value['__type__'])
            return cls(**{k: decode(v) for k, v in value['fields'].items()})
        return {k: decode(v) for k, v in value.items()}
    return value


class CaseArchive:
    def __init__(self, directory):
        self.directory, self.counter = directory, 0
        self.ledger = None
        directory.mkdir(parents=True, exist_ok=False)

    def save(self, name, value):
        (self.directory/(name+'.json')).write_text(json.dumps(encode(value), indent=2, allow_nan=False)+'\n')

    def associate(self, case, **changes):
        self.counter += 1
        prefix = f'association_{self.counter:02d}'
        arguments = dict(settled_stamp_ns=SETTLED_NS, stationary=True, policy=fixture_policy())
        arguments.update(changes)
        self.save(prefix+'_inputs', dict(case=case, arguments=arguments))
        arrays = {}
        for name, domain, view in (('before', *case[:2]), ('after', *case[2:])):
            a = view.acquisition
            arrays[name+'_depth_mm'] = np.frombuffer(a.depth_mm, dtype=a.depth_dtype).reshape(a.shape)
            arrays[name+'_region'] = np.frombuffer(a.instance_region, dtype=bool).reshape(a.shape)
            arrays[name+'_support'] = domain.cells()
            if a.association_inputs:
                arrays[name+'_rgb'] = a.association_inputs.rgb.array()
        np.savez_compressed(self.directory/(prefix+'_raw.npz'), **arrays)
        try:
            result = associate_stationary_pickup(*case, **arguments)
        except Exception as error:
            self.save(prefix+'_exception', dict(type=type(error).__name__, message=str(error)))
            raise
        self.save(prefix+'_output', result)
        return result

    def finish(self):
        # Runs even when an assertion fails before its explicit after-snapshot.
        if self.ledger is not None:
            path, clock = self.ledger
            self.save('ledger_final', dict(clock_wall_s=clock[0],
                      durable=json.loads(path.read_text())))
        records = [dict(file=p.name, bytes=p.stat().st_size,
                        sha256=hashlib.sha256(p.read_bytes()).hexdigest())
                   for p in sorted(self.directory.iterdir()) if p.is_file()]
        self.save('manifest', dict(records=records, replay_possible=True,
                                  association_samples=self.counter))


@pytest.fixture
def case_archive(request, tmp_path):
    root = Path(os.environ.get('PICKUP_CONTINUATION_EVIDENCE', str(tmp_path)))
    archive = CaseArchive(root/re.sub(r'[^\w.-]', '_', request.node.name))
    try:
        yield archive
    finally:
        archive.finish()
