"""Measured-pose updates to a private source-mesh collision scene.

Nominal planning retains its immutable snapshot scene. Only this physical
observer adapter follows lower-cargo poses; it never edits the simulator.
"""
import numpy as np
from full_arm import FullArmScene


class ObservedArmScene(FullArmScene):
    def update_world_transform(self,name,transform):
        if name not in {body.name for body in self.snapshot.occupied}:
            raise ValueError('OBSERVED_UPDATE_NOT_LOWER_CARGO:'+name)
        transform=np.asarray(transform,float)
        if transform.shape!=(4,4) or not np.isfinite(transform).all():
            raise ValueError('INVALID_OBSERVED_WORLD_TRANSFORM')
        mesh,_=self.world_parts[name]
        self.world.set_transform(name,transform)
        self.world_managers[name].set_transform(name,transform)
        self.world_parts[name]=(mesh,transform)
        self.world_bounds[name]=self.transformed_bounds(mesh,transform)
