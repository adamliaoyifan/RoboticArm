"""Read-only ROS boundary for exact, frozen cargo measurement partitions."""
import hashlib
import json
import math
import threading

import numpy as np

from luggage_perception.cargo_observation_partition import (
    AcquisitionKey, partition_aligned_observation,
)


SOURCE_KINDS = ('rgb','depth','mask','calibration')


def source_sha256(message):
    """Hash complete typed fields; DDS alignment padding is not source identity."""
    def record(value):
        if hasattr(value,'get_fields_and_field_types'):
            fields = {}
            for name,field_type in value.get_fields_and_field_types().items():
                item = getattr(value,name)
                if name == 'data':
                    payload = bytes(item)
                    fields[name] = dict(length=len(payload),sha256=hashlib.sha256(payload).hexdigest())
                elif field_type.startswith(('uint','int')) and '[' not in field_type:
                    fields[name] = int(item)
                elif field_type in ('float','double'):
                    fields[name] = float(item)
                elif field_type == 'boolean':
                    fields[name] = bool(item)
                else:
                    fields[name] = record(item)
            return dict(type=value.__class__.__name__,fields=fields)
        if isinstance(value,(str,int,float,bool)) or value is None:
            return value
        return [record(item) for item in value]
    return hashlib.sha256(json.dumps(record(message),sort_keys=True,
        separators=(',',':'),allow_nan=False).encode()).hexdigest()


def acquisition_key(message):
    stamp = message.header.stamp
    if not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError('INVALID_ACQUISITION_KEY')
    return AcquisitionKey(int(stamp.sec)*1_000_000_000+int(stamp.nanosec),
                          message.header.frame_id)


class CargoObservationBuffer:
    """Bounded complete serialized messages; conflicting acquisitions are toxic.

    Eviction advances a watermark so delayed messages cannot reconstruct an
    evicted/conflicting frame and lose its earlier conflict evidence.
    """
    def __init__(self, *, max_frames, max_bytes, max_pixels, max_age_ns):
        if any(type(v) is not int or v <= 0
               for v in (max_frames,max_bytes,max_pixels,max_age_ns)):
            raise ValueError('POSITIVE_OBSERVATION_RESOURCE_LIMITS_REQUIRED')
        self.max_frames,self.max_bytes = max_frames,max_bytes
        self.max_pixels,self.max_age_ns = max_pixels,max_age_ns
        self._frames,self._conflicts = {},set()
        self._bytes,self._evicted_through = 0,0
        self._lock = threading.Lock()

    def add(self, kind, message):
        from rclpy.serialization import serialize_message, deserialize_message
        if kind not in SOURCE_KINDS:
            raise ValueError('UNKNOWN_OBSERVATION_SOURCE')
        key = acquisition_key(message)
        if (message.width <= 0 or message.height <= 0
                or message.width*message.height > self.max_pixels):
            raise ValueError('OBSERVATION_SIZE_LIMIT')
        if hasattr(message,'data') and len(message.data) > self.max_bytes:
            raise ValueError('OBSERVATION_BYTE_LIMIT')
        payload = serialize_message(message)
        fingerprint = source_sha256(deserialize_message(payload,type(message)))
        if len(payload) > self.max_bytes:
            raise ValueError('OBSERVATION_BYTE_LIMIT')
        with self._lock:
            stamp = key.stamp_ns
            if stamp <= self._evicted_through:
                raise ValueError('OBSERVATION_EVICTED')
            frame = self._frames.setdefault(stamp,{})
            if kind in frame:
                if frame[kind][1] != fingerprint:
                    self._conflicts.add(stamp)
                return False
            frame[kind] = (payload,fingerprint)
            self._bytes += len(payload)
            while len(self._frames) > self.max_frames or self._bytes > self.max_bytes:
                oldest = min(self._frames)
                self._bytes -= sum(len(v[0]) for v in self._frames.pop(oldest).values())
                self._conflicts.discard(oldest)
                self._evicted_through = max(self._evicted_through,oldest)
            return stamp in self._frames

    def exact(self, stamp_ns, now_ns):
        if type(stamp_ns) is not int or stamp_ns <= 0:
            raise ValueError('EXACT_POSITIVE_ACQUISITION_REQUIRED')
        if not 0 <= now_ns-stamp_ns <= self.max_age_ns:
            raise ValueError('OBSERVATION_EXPIRED_OR_FUTURE')
        with self._lock:
            if stamp_ns <= self._evicted_through:
                raise ValueError('OBSERVATION_EVICTED')
            if stamp_ns in self._conflicts:
                raise ValueError('CONFLICTING_ACQUISITION')
            frame = self._frames.get(stamp_ns,{})
            if set(frame) != set(SOURCE_KINDS):
                raise ValueError('EXACT_OBSERVATION_INCOMPLETE')
            return {kind:entry[0] for kind,entry in frame.items()}


def _image(message, encoding, dtype, channels=1):
    """Decode declared rows strictly, including padding and endianness."""
    if message.encoding not in encoding:
        raise ValueError('UNSUPPORTED_OBSERVATION_ENCODING')
    width,height,step = message.width,message.height,message.step
    dt = np.dtype(dtype).newbyteorder('>' if message.is_bigendian else '<')
    if (width <= 0 or height <= 0 or step < width*channels*dt.itemsize
            or len(message.data) != height*step):
        raise ValueError('MALFORMED_OBSERVATION_IMAGE')
    shape = (height,width,channels) if channels != 1 else (height,width)
    strides = (step,channels*dt.itemsize,dt.itemsize) if channels != 1 else (step,dt.itemsize)
    return np.ndarray(shape,dtype=dt,buffer=bytes(message.data),strides=strides)


def observation_message(*, request_id, sensor_id, payloads, transform, stride, max_pixels):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Image, CameraInfo
    from luggage_msgs.msg import CargoObservationPartitionV2
    from luggage_perception.ros_message_adapters import cloud_msg_from_points
    sources = [deserialize_message(payloads[k],CameraInfo if k == 'calibration' else Image)
               for k in SOURCE_KINDS]
    rgb,depth,mask,info = sources
    colour = _image(rgb,('rgb8','bgr8'),'u1',3)
    if depth.encoding == '16UC1':
        metres = _image(depth,('16UC1',),'u2').astype(np.float32)*np.float32(.001)
    else:
        metres = _image(depth,('32FC1',),'f4').astype(np.float32)
    labels = _image(mask,('mono16','16UC1'),'u2')
    if (depth.width,depth.height) != (info.width,info.height):
        raise ValueError('CALIBRATION_SIZE_MISMATCH')
    if (info.binning_x > 1 or info.binning_y > 1 or info.roi.x_offset or info.roi.y_offset
            or info.roi.width or info.roi.height):
        raise ValueError('CROPPED_CALIBRATION_UNSUPPORTED')
    if (transform.header.frame_id != 'container_link'
            or transform.child_frame_id != depth.header.frame_id):
        raise ValueError('ACQUISITION_TF_FRAME_MISMATCH')
    q = transform.transform.rotation
    x,y,z,w = q.x,q.y,q.z,q.w
    if (not all(math.isfinite(v) for v in (x,y,z,w))
            or not math.isclose(x*x+y*y+z*z+w*w,1.,abs_tol=1e-7)):
        raise ValueError('ACQUISITION_TF_QUATERNION_INVALID')
    t = transform.transform.translation
    matrix = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),t.x],
                       [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),t.y],
                       [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y),t.z],
                       [0,0,0,1]],dtype=np.float64)
    tf_key = AcquisitionKey(acquisition_key(transform).stamp_ns,transform.child_frame_id)
    hashes = tuple(source_sha256(message) for message in sources)+(source_sha256(transform),)
    observation_id = 'rgbd:'+hashlib.sha256(''.join(hashes).encode()).hexdigest()
    partition = partition_aligned_observation(sensor_id=sensor_id,observation_id=observation_id,
        keys=tuple(acquisition_key(m) for m in sources)+(tf_key,),rgb=colour,
        depth_m=metres,instance_mask=labels,intrinsics=np.array(info.k).reshape(3,3),
        distortion=info.d,container_from_optical=matrix,stride=stride,max_pixels=max_pixels,
        source_sha256=hashes)
    result = CargoObservationPartitionV2()
    result.header,result.request_id,result.sensor_id = depth.header,request_id,sensor_id
    result.observation_id,result.sampling_stride = observation_id,stride
    result.binding_sha256,result.source_sha256 = partition.binding_sha256,hashes
    result.rgb,result.depth,result.instance_mask,result.camera_info = sources
    result.container_from_optical = transform
    result.cloud = cloud_msg_from_points(partition.points(),depth.header.stamp,depth.header.frame_id)
    result.source_pixel_indices = partition.pixels().tolist()
    result.frame_instance_labels = partition.labels().tolist()
    return result


class CargoObservationService:
    """Inject acquisition-time TF/clock; expose no planning or mutation ports."""
    def __init__(self, *, buffer, sensor_id, stride, now_ns, lookup_tf, recorder=None):
        if not isinstance(sensor_id,str) or not sensor_id.strip() or type(stride) is not int or stride<=0:
            raise ValueError('EXPLICIT_OBSERVATION_SOURCE_AND_STRIDE_REQUIRED')
        self.buffer,self.sensor_id,self.stride = buffer,sensor_id,stride
        self.now_ns,self.lookup_tf,self.recorder = now_ns,lookup_tf,recorder

    def handle(self, request, response):
        from rclpy.serialization import deserialize_message
        from sensor_msgs.msg import Image, CameraInfo
        from luggage_msgs.msg import CargoObservationPartitionV2
        response.valid = False
        response.observation = CargoObservationPartitionV2()
        if self.recorder:
            self.recorder('request',request)
        try:
            if not request.request_id.strip():
                raise ValueError('REQUEST_ID_REQUIRED')
            payloads = self.buffer.exact(int(request.acquisition_stamp_ns),self.now_ns())
            if self.recorder:
                for kind in SOURCE_KINDS:
                    self.recorder('source_'+kind,deserialize_message(payloads[kind],
                        CameraInfo if kind == 'calibration' else Image))
            depth = deserialize_message(payloads['depth'],Image)
            transform = self.lookup_tf(depth.header.frame_id,int(request.acquisition_stamp_ns))
            if self.recorder:
                self.recorder('acquisition_tf',transform)
            result = observation_message(request_id=request.request_id,sensor_id=self.sensor_id,
                payloads=payloads,transform=transform,stride=self.stride,max_pixels=self.buffer.max_pixels)
            # Recheck age/conflicts/eviction after potentially blocking TF and decoding.
            self.buffer.exact(int(request.acquisition_stamp_ns),self.now_ns())
            response.observation,response.valid,response.reason = result,True,'MEASURED_FRAME_PARTITION'
        except Exception as error:
            response.valid,response.reason = False,str(error) or type(error).__name__
        if self.recorder:
            self.recorder('response',response)
        return response
