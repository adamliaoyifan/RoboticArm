"""Simulation-only actuator client, never a planning or perception input."""
import threading


class SimPhysicalReleaseClient:
    def __init__(self,node,group,follow_lock):
        from luggage_msgs.srv import ReleaseSimBox
        self.service=ReleaseSimBox
        self.follow_lock=follow_lock
        self.client=node.create_client(ReleaseSimBox,'/pickup_box_spawner/release_sim_box',
                                       callback_group=group)

    def __call__(self,model,generation):
        # The backend has already disabled future ticks. Finish any currently
        # running SetPose before deleting/recreating that physical model name.
        with self.follow_lock:
            return self._release(model,generation)

    def _release(self,model,generation):
        if not model or type(generation) is not int or generation<1:
            return False,'PHYSICAL_RELEASE_IDENTITY_REQUIRED'
        if not self.client.wait_for_service(timeout_sec=2.):
            return False,'PHYSICAL_RELEASE_SERVICE_UNAVAILABLE'
        done=threading.Event()
        future=self.client.call_async(self.service.Request(model_name=model,expected_generation=generation))
        future.add_done_callback(lambda value:done.set())
        if not done.wait(12.):return False,'PHYSICAL_RELEASE_RESPONSE_UNRESOLVED'
        response=future.result()
        if response is None:return False,'PHYSICAL_RELEASE_RESPONSE_MISSING'
        return response.success,response.message
