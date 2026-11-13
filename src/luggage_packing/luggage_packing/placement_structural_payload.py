"""Payload semantics for structural placement search (G3 integration).

Two different objects live in one request and they must not be mixed
(user item 1, 2026-10-25):

- the CONSERVATIVE ENVELOPE owns the hard gates and the window
  enumeration (a slot must physically fit the envelope);
- the MEASURED (nominal) BOX owns the same-height adjacency comparison:
  the snug term compares the neighbour's measured top against the
  candidate's PREDICTED NOMINAL top (the measured box's top face after
  the envelope is placed), never the envelope top. Downward-only
  envelope growth shifts the envelope centre; using the envelope top
  here is the "ranking-object mismatch" this module exists to prevent.

``StructuralPayload.from_attachment`` derives both from the frozen
``luggage_planning.planning_contract.AttachmentGeometry`` (single snapshot, deterministic,
digest-bound). Flat-pack candidates keep identity rotation, so the
nominal-centre offset in the container frame equals the tool-frame
offset ``t_tool_box_translation - conservative.t_tcp_box_translation``.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class StructuralPayload:
    """Envelope-for-gates / nominal-for-adjacency payload contract."""

    conservative_size_wdh: tuple  # (l, w, h) of the envelope; gates+windows
    measured_dimensions_wdh: tuple  # nominal box; adjacency top only
    nominal_center_offset: tuple  # nominal centre - envelope centre
    attachment_digest: str

    def __post_init__(self):
        object.__setattr__(self, "conservative_size_wdh", tuple(
            float(v) for v in self.conservative_size_wdh))
        object.__setattr__(self, "measured_dimensions_wdh", tuple(
            float(v) for v in self.measured_dimensions_wdh))
        object.__setattr__(self, "nominal_center_offset", tuple(
            float(v) for v in self.nominal_center_offset))
        object.__setattr__(self, "attachment_digest",
                           str(self.attachment_digest))

    @classmethod
    def from_sizes(cls, conservative_size_wdh, measured_dimensions_wdh,
                   attachment_digest=""):
        """Envelope + nominal sizes with a zero centre offset (the
        symmetric-growth case; the digest still binds the pair)."""
        return cls(tuple(conservative_size_wdh),
                   tuple(measured_dimensions_wdh), (0.0, 0.0, 0.0),
                   attachment_digest)

    @classmethod
    def from_attachment(cls, geometry):
        """Derive both objects from one frozen AttachmentGeometry."""
        envelope = geometry.conservative
        offset = tuple(
            float(geometry.t_tool_box_translation[i])
            - float(envelope.t_tcp_box_translation[i])
            for i in range(3))
        return cls(
            tuple(float(v) for v in envelope.dimensions_wdh),
            tuple(float(v) for v in geometry.measured_dimensions_wdh),
            offset,
            str(geometry.digest()))

    @property
    def envelope(self):
        """The size the hard gates consume."""
        return self.conservative_size_wdh

    def predicted_nominal_top(self, center_container):
        """Top-face height of the measured box once the envelope centre
        rests at ``center_container`` (flat pack, identity rotation)."""
        centre_z = float(center_container[2]) + self.nominal_center_offset[2]
        return centre_z + 0.5 * float(self.measured_dimensions_wdh[2])
