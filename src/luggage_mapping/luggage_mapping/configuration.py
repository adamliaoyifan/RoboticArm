"""Pin one effective map-owner configuration for all integrated consumers."""
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path

import yaml

from luggage_description.cargo_grid_parameters import resolve_resolution
from luggage_description.container_geometry import descriptor_from_scene_config
from luggage_description.scene_tf_config_utils import load_scene_tf_config
from luggage_mapping.observer import ObserverConfiguration


@dataclass(frozen=True)
class MapOwnerConfiguration:
    backend: str
    resolution_m: float
    geometry_file: str = ''
    observer_config_file: str = ''


def _write_immutable(path, content):
    try:
        with path.open('xb') as stream:
            stream.write(content)
    except FileExistsError:
        if path.read_bytes() != content:
            raise ValueError('MAP_OWNER_ARTIFACT_CONTENT_MISMATCH')


def owner_configuration(*, backend, scene, resolution_override='',
                        observer_config='', artifact_root=None):
    if backend not in ('observer', 'grid_g'):
        raise ValueError('occupancy_backend must be observer or grid_g')
    resolution = resolve_resolution(resolution_override)
    if backend == 'grid_g':
        return MapOwnerConfiguration(backend, resolution)
    descriptor = descriptor_from_scene_config(load_scene_tf_config(scene)).descriptor()
    canonical = json.dumps(descriptor, sort_keys=True, separators=(',', ':'), allow_nan=False)
    geometry = yaml.safe_dump(dict(schema='uld_geometry_export_v1', descriptor_json=canonical,
        descriptor_sha256=hashlib.sha256(canonical.encode()).hexdigest()), sort_keys=True).encode()
    configuration = yaml.safe_load(Path(observer_config).read_bytes())
    configuration['resolution_m'] = resolution
    encoded = yaml.safe_dump(configuration, sort_keys=True).encode()
    identity = hashlib.sha256(geometry+b'\n'+encoded).hexdigest()
    root = (Path(artifact_root) if artifact_root is not None else
            Path(os.environ.get('ROS_HOME', Path.home()/'.ros'))/'luggage_mapping')
    folder = root/identity
    folder.mkdir(parents=True, exist_ok=True)
    geometry_path, config_path = folder/'geometry.yaml', folder/'observer.yaml'
    _write_immutable(geometry_path, geometry)
    _write_immutable(config_path, encoded)
    # Validate the very bytes both the native owner and readers will consume.
    admitted = ObserverConfiguration.load(geometry_path, config_path)
    if admitted.resolution_m != resolution:
        raise ValueError('MAP_OWNER_RESOLUTION_MISMATCH')
    return MapOwnerConfiguration(backend, resolution, str(geometry_path), str(config_path))
