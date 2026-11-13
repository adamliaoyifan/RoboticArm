"""Attachment-owned immutable geometry shared by capacity and motion consumers.

Only ``freeze_geometry`` applies uncertainty. Consumers transform the frozen
geometry; they never pass conservative dimensions back into the constructor.
All transforms map the named child frame into its parent frame.
"""
from dataclasses import dataclass

from ._base import ContractViolation, digest_of, finite_tuple, positive_dimensions, unit_quaternion
from .attachment import compose_transform, invert_transform
from .payload_envelope import PayloadEnvelope, build_envelope


@dataclass(frozen=True)
class AttachmentGeometry:
    measured_dimensions_wdh: tuple
    t_tool_box_translation: tuple
    t_tool_box_quaternion_xyzw: tuple
    conservative: PayloadEnvelope

    def __post_init__(self):
        for name, value in (
                ('measured_dimensions_wdh', positive_dimensions(self.measured_dimensions_wdh, 'measured_dimensions')),
                ('t_tool_box_translation', finite_tuple(self.t_tool_box_translation, 3, 'tool_box_translation')),
                ('t_tool_box_quaternion_xyzw', unit_quaternion(self.t_tool_box_quaternion_xyzw, 'tool_box_quaternion'))):
            object.__setattr__(self, name, value)
        expected = digest_of(self.measured_dimensions_wdh, self.t_tool_box_translation,
                             self.t_tool_box_quaternion_xyzw)
        if self.conservative.nominal_digest != expected:
            raise ContractViolation('ATTACHMENT_GEOMETRY_BINDING_MISMATCH')

    def to_dict(self):
        return dict(measured_dimensions_wdh=self.measured_dimensions_wdh,
                    t_tool_box_translation=self.t_tool_box_translation,
                    t_tool_box_quaternion_xyzw=self.t_tool_box_quaternion_xyzw,
                    conservative=self.conservative.to_dict(), digest=self.digest())

    def digest(self):
        return digest_of('attachment-geometry-v1', self.measured_dimensions_wdh,
                         self.t_tool_box_translation,
                         self.t_tool_box_quaternion_xyzw,
                         self.conservative.digest())

    def conservative_pose_at_box(self, position, quaternion):
        """Return envelope pose in the frame containing the nominal box pose."""
        box_tool = invert_transform(self.t_tool_box_translation,
                                    self.t_tool_box_quaternion_xyzw)
        frame_tool = compose_transform(position, quaternion, *box_tool)
        return compose_transform(*frame_tool,
                                 self.conservative.t_tcp_box_translation,
                                 self.conservative.t_tcp_box_quaternion_xyzw)

    def box_pose_at_conservative(self, position, quaternion):
        """Convert a solver's conservative-box target to the measured-box target.

        In particular, downward-only height growth shifts the envelope centre;
        treating that centre as the nominal centre would move the contact face.
        """
        envelope_tool = invert_transform(
            self.conservative.t_tcp_box_translation,
            self.conservative.t_tcp_box_quaternion_xyzw)
        frame_tool = compose_transform(position, quaternion, *envelope_tool)
        return compose_transform(*frame_tool, self.t_tool_box_translation,
                                 self.t_tool_box_quaternion_xyzw)


def freeze_geometry(measured_dimensions, tool_box_translation,
                    tool_box_quaternion, limits):
    """Snapshot measured input once and construct its conservative geometry."""
    dimensions = positive_dimensions(measured_dimensions, 'measured_dimensions')
    translation = finite_tuple(tool_box_translation, 3, 'tool_box_translation')
    quaternion = unit_quaternion(tool_box_quaternion, 'tool_box_quaternion')
    conservative = build_envelope(dimensions, translation, quaternion, limits)
    return AttachmentGeometry(dimensions, translation, quaternion, conservative)
