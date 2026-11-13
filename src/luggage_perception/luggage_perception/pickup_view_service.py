"""Read-only exact-source camera evidence port; no latest-frame substitutions."""
import numpy as np

from luggage_perception.pickup_view_evidence import (
    DEPTH_ENCODINGS, PickupViewAcquisition, PickupViewEvidence,
)
from luggage_perception.ros_suction_domain import require_matching_detection
from luggage_perception.pickup_view_association import (
    BufferedPickupRgb, PickupAssociationInputs, ReportedPickupCandidate,
)


def view_to_message(view):
    from luggage_msgs.msg import PickupViewEvidenceV2, YoloBox
    result=PickupViewEvidenceV2()
    a=view.acquisition
    result.header.frame_id=a.optical_frame_id
    result.header.stamp.sec,result.header.stamp.nanosec=divmod(a.stamp_ns,10**9)
    result.box_id,result.instance_id,result.generation=view.box_id,a.instance_id,a.generation
    result.domain_sha256,result.evidence_sha256=view.domain_sha256,view.digest()
    result.intrinsics_fx_fy_cx_cy=list(a.intrinsics_fx_fy_cx_cy)
    result.world_from_optical=list(a.world_from_optical)
    result.region_source=a.region_source
    for image,data,encoding,dtype in (
            (result.depth_mm,a.depth_mm,DEPTH_ENCODINGS[np.dtype(a.depth_dtype).name],np.dtype(a.depth_dtype)),
            (result.instance_region,a.instance_region,'mono8',np.dtype('u1'))):
        image.header=result.header
        image.height,image.width=a.shape
        image.encoding=encoding
        image.is_bigendian=dtype.byteorder=='>'
        image.step=a.shape[1]*dtype.itemsize
        image.data=data
    result.association_inputs_available = a.association_inputs is not None
    result.association_inputs_reason = a.association_reason
    result.selected_candidate_index = -1
    if a.association_inputs is not None:
        inputs = a.association_inputs
        rgb = result.rgb
        rgb.header = result.header
        rgb.height, rgb.width = a.shape
        rgb.encoding, rgb.step, rgb.data = 'rgb8', a.shape[1]*3, inputs.rgb.rgb8
        pool = result.candidate_pool
        pool.header = result.header
        pool.image_height, pool.image_width = a.shape
        pool.instance_id, pool.generation = inputs.instance_id, inputs.generation
        pool.frame_seq, pool.backend = inputs.frame_seq, inputs.backend
        pool.detections = [YoloBox(label=c.label, prompt=c.prompt, confidence=c.confidence,
                                  bbox=list(c.bbox), held=c.held) for c in inputs.candidates]
        result.selected_candidate_index = inputs.selected_index
    return result


def view_from_message(message, domain):
    """Validate layout, exact domain/source identity and every received byte."""
    depth,region=message.depth_mm,message.instance_region
    encoding_to_dtype={value:np.dtype(key) for key,value in DEPTH_ENCODINGS.items()}
    if (message.header!=depth.header or message.header!=region.header
            or depth.encoding not in encoding_to_dtype or region.encoding!='mono8'
            or (depth.height,depth.width)!=(region.height,region.width)
            or region.is_bigendian or region.step!=region.width
            or message.domain_sha256!=domain.digest() or message.box_id!=domain.box_id):
        raise ValueError('PICKUP_VIEW_WIRE_IDENTITY_OR_LAYOUT_MISMATCH')
    dtype=encoding_to_dtype[depth.encoding].newbyteorder('>' if depth.is_bigendian else '<')
    if depth.step!=depth.width*dtype.itemsize:
        raise ValueError('PICKUP_VIEW_WIRE_ROW_STEP_MISMATCH')
    inputs = _association_from_message(message)
    a=PickupViewAcquisition(int(message.header.stamp.sec)*10**9+int(message.header.stamp.nanosec),
        message.header.frame_id,message.instance_id,int(message.generation),
        (int(depth.height),int(depth.width)),dtype.str,bytes(depth.data),bytes(region.data),
        message.region_source,tuple(message.intrinsics_fx_fy_cx_cy),tuple(message.world_from_optical),
        inputs,message.association_inputs_reason)
    if a.association_reason != message.association_inputs_reason:
        raise ValueError('PICKUP_ASSOCIATION_WIRE_REASON_MISMATCH')
    view=a.bind(domain)
    if view.digest()!=message.evidence_sha256:
        raise ValueError('PICKUP_VIEW_WIRE_EVIDENCE_DIGEST_MISMATCH')
    return view


def _association_from_message(message):
    from luggage_msgs.msg import YoloDetections
    from sensor_msgs.msg import Image
    rgb, pool = message.rgb, message.candidate_pool
    if not message.association_inputs_available:
        if rgb != Image() or pool != YoloDetections() or message.selected_candidate_index != -1:
            raise ValueError('PICKUP_ASSOCIATION_UNAVAILABLE_WITH_PAYLOAD')
        return None
    if (rgb.header != message.header or pool.header != message.header
            or rgb.encoding != 'rgb8' or rgb.is_bigendian or rgb.step != rgb.width*3
            or (rgb.height,rgb.width) != (message.depth_mm.height,message.depth_mm.width)
            or (pool.image_height,pool.image_width) != (rgb.height,rgb.width)):
        raise ValueError('PICKUP_ASSOCIATION_WIRE_LAYOUT_OR_SOURCE_MISMATCH')
    frame = BufferedPickupRgb(int(rgb.header.stamp.sec)*10**9+int(rgb.header.stamp.nanosec),
        rgb.header.frame_id,(int(rgb.height),int(rgb.width),3),bytes(rgb.data))
    candidates = tuple(ReportedPickupCandidate(int(c.label),c.prompt,float(c.confidence),
        tuple(int(v) for v in c.bbox),bool(c.held)) for c in pool.detections)
    return PickupAssociationInputs(frame,candidates,int(message.selected_candidate_index),
        int(pool.frame_seq),int(pool.generation),pool.instance_id,pool.backend)


class PickupViewService:
    def __init__(self, *, context_lookup):
        self.context_lookup=context_lookup

    def get(self, request, response):
        from luggage_msgs.msg import PickupViewEvidenceV2
        response.success,response.reason,response.view=False,'PICKUP_VIEW_UNAVAILABLE',PickupViewEvidenceV2()
        try:
            if not request.domain_sha256:
                raise ValueError('EXPLICIT_PICKUP_ACQUISITION_REQUIRED')
            context=self.context_lookup(request.domain_sha256)
            if context is None or context.domain_sha256!=request.domain_sha256:
                raise ValueError('PICKUP_ACQUISITION_UNAVAILABLE_OR_EXPIRED')
            require_matching_detection(context.domain,request.detection,generation=int(request.generation),
                contact_model_hash=context.domain.contact_model_hash)
            view=context.pickup_view
            if type(view) is not PickupViewEvidence:
                raise ValueError(context.pickup_view_reason)
            result=view_to_message(view)
            # Revoke/expiry may race with copying a multi-megabyte image.
            current=self.context_lookup(request.domain_sha256)
            if (current is None or current.pickup_view is None
                    or current.pickup_view.digest()!=view.digest()):
                raise ValueError('PICKUP_VIEW_SOURCE_CHANGED_DURING_READ')
            response.view=result
            response.success,response.reason=True,'PICKUP_VIEW_EVIDENCE_AVAILABLE'
        except (AttributeError,TypeError,ValueError,OverflowError) as error:
            response.reason='PICKUP_VIEW_REFUSED:'+type(error).__name__+':'+str(error)
        return response
