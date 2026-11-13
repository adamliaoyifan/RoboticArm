"""Measured static-structure association with explicit validity coverage.

A structure label suppresses only its occupied endpoint update. It never removes
that ray, moves its endpoint, frees the terminal cell or changes the static ULD.
Calibration comes from an explicitly identified empty structure patch. Its
observed residual/range/spatial coverage is not a hardware accuracy guarantee.
"""
import hashlib
import json
import numpy as np
from scipy.spatial import cKDTree


def _features(points, neighbors):
    distances, indices = cKDTree(points).query(points,k=neighbors)
    patches = points[indices]
    centered = patches-patches.mean(axis=1,keepdims=True)
    values, vectors = np.linalg.eigh(np.einsum('nki,nkj->nij',centered,centered))
    return distances[:,-1], vectors[:,:,0], values[:,0]/np.maximum(values[:,1],1e-24)


def calibrate_structure(points, *, normal, offset_m, geometry_hash, sensor_frame,
                        sensor_mount_id, source_id, origins, neighbors=12):
    points, origins = np.asarray(points,dtype=float), np.asarray(origins,dtype=float)
    normal = np.asarray(normal,dtype=float)
    if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 4*neighbors
            or origins.shape != points.shape or not np.isfinite(np.r_[points.ravel(),origins.ravel()]).all()
            or normal.shape != (3,) or abs(np.linalg.norm(normal)-1) > 1e-8
            or not all((geometry_hash,sensor_frame,sensor_mount_id,source_id))):
        raise ValueError('MEASURED_EMPTY_STRUCTURE_CALIBRATION_REQUIRED')
    residual = points@normal+float(offset_m)
    radius, normals, flatness = _features(points,neighbors)
    angular = np.abs(normals@normal)
    ranges = np.linalg.norm(points-origins,axis=1)
    incidence = np.abs(np.einsum('ni,i->n',(points-origins)/ranges[:,None],normal))
    if not np.isfinite(ranges).all() or min(ranges) <= 0:
        raise ValueError('VALID_CALIBRATION_RAYS_REQUIRED')
    record = dict(schema=1,geometry_hash=geometry_hash,sensor_frame=sensor_frame,
        sensor_mount_id=sensor_mount_id,source_id=source_id,normal=normal.tolist(),offset_m=float(offset_m),
        sample_count=len(points),neighbors=int(neighbors),bounds_min=points.min(axis=0).tolist(),
        bounds_max=points.max(axis=0).tolist(),residual_min_m=float(residual.min()),
        residual_max_m=float(residual.max()),max_neighbor_radius_m=float(radius.max()),
        min_normal_cosine=float(angular.min()),max_flatness=float(flatness.max()),
        range_m=[float(ranges.min()),float(ranges.max())],
        incidence=[float(incidence.min()),float(incidence.max())],
        uncertainty_scope='empirical_coverage_only')
    record['calibration_id']=hashlib.sha256(json.dumps(record,sort_keys=True).encode()).hexdigest()
    return record


class StructureAssociation:
    def __init__(self, record, *, geometry_hash, sensor_frame, sensor_mount_id):
        record = dict(record)
        digest = record.pop('calibration_id')
        if (record.get('schema') != 1 or not all((geometry_hash,sensor_frame,sensor_mount_id))
                or (record['geometry_hash'],record['sensor_frame'],record['sensor_mount_id']) !=
                   (geometry_hash,sensor_frame,sensor_mount_id)
                or hashlib.sha256(json.dumps(record,sort_keys=True).encode()).hexdigest() != digest):
            raise ValueError('STRUCTURE_CALIBRATION_BINDING_MISMATCH')
        self.record, self.calibration_id = record,digest

    def classify(self, points, origin):
        points, origin = np.asarray(points,dtype=float), np.asarray(origin,dtype=float)
        labels = np.zeros(len(points),dtype=np.uint8)
        valid = np.isfinite(points).all(axis=1)
        q = points[valid]
        r = self.record
        if len(q) < r['neighbors']: return labels
        radius,normals,flatness = _features(q,r['neighbors'])
        normal = np.asarray(r['normal'])
        residual = q@normal+r['offset_m']
        ranges = np.linalg.norm(q-origin,axis=1)
        incidence = np.abs(((q-origin)/np.maximum(ranges[:,None],1e-24))@normal)
        # Numerical tolerance only; never a fixed physical deletion band.
        eps = 1e-12
        inside = (q >= np.asarray(r['bounds_min'])-eps) & (q <= np.asarray(r['bounds_max'])+eps)
        match = (inside.all(axis=1) & (residual >= r['residual_min_m']-eps)
            & (residual <= r['residual_max_m']+eps)
            & (radius <= r['max_neighbor_radius_m']+eps)
            & (np.abs(normals@normal) >= r['min_normal_cosine']-eps)
            & (flatness <= r['max_flatness']+eps)
            & (ranges >= r['range_m'][0]-eps) & (ranges <= r['range_m'][1]+eps)
            & (incidence >= r['incidence'][0]-eps) & (incidence <= r['incidence'][1]+eps))
        labels[np.flatnonzero(valid)] = match.astype(np.uint8)
        return labels
