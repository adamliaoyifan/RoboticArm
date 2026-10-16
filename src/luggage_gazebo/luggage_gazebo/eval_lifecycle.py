"""Orderly shutdown of an evaluation driver's ROS and dedicated TF workers."""


def close_driver_workers(driver, executor, spinner):
    """Stop callbacks before destroying the nodes they access.

    The caller owns the rclpy context and shuts it down after this returns.
    TransformListener also stops its executor in __del__; stopping and joining
    it here prevents that cleanup from racing with interpreter finalization.
    """
    executor.shutdown()
    spinner.join()
    listener = getattr(driver, '_tf_listener', None)
    tf_executor = getattr(listener, 'executor', None)
    if tf_executor is not None:
        tf_executor.shutdown()
    tf_thread = getattr(listener, 'dedicated_listener_thread', None)
    if tf_thread is not None:
        tf_thread.join()
    executor.remove_node(driver)
    driver.destroy_node()
    tf_node = getattr(driver, '_tf_node', None)
    if tf_node is not None:
        tf_node.destroy_node()
