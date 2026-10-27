"""Serialize stock D430I users; never start a second device owner or send motion."""

def release_competing_camera(target, get_state, transition, guard, emit):
    if target not in ('camera', 'camera_align'):
        raise ValueError('Unknown stock camera owner')
    other = 'camera_align' if target == 'camera' else 'camera'
    guard()
    # Check BOTH services before changing anything. A hung/transitioning node
    # needs recovery, not another configure request or a second camera process.
    states = {name: get_state(name) for name in (target, other)}
    if any(value not in (1, 2, 3) for value in states.values()):
        raise RuntimeError('Camera lifecycle is transitioning; no algorithm request sent')
    if states[other] == 3:
        guard()
        transition(other, 4)
        if get_state(other) != 2:
            raise RuntimeError('Competing camera did not deactivate; no algorithm request sent')
        states[other] = 2
    if states[other] == 2:
        guard()
        transition(other, 2)
    if get_state(other) != 1:
        raise RuntimeError('Competing camera did not release device; no algorithm request sent')
    guard()
    emit(event='CAMERA_OWNER_PREPARED', owner=target, released=other)


def prepare_stock_camera(target, node, namespace, guard, emit):
    from lifecycle_msgs.srv import GetState, ChangeState
    import rclpy

    def call(name, service, kind, request):
        client = node.create_client(kind, namespace+'/camera/'+name+'/'+service)
        try:
            guard()
            if not client.wait_for_service(timeout_sec=2):
                raise RuntimeError('Camera service unavailable: '+name+'/'+service)
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=6)
            if not future.done() or future.result() is None:
                raise RuntimeError('Camera service unresponsive: '+name+'/'+service+
                                   '; recover stock camera before retry; no algorithm request sent')
            guard()
            return future.result()
        finally:
            node.destroy_client(client)

    def state(name):
        return call(name, 'get_state', GetState, GetState.Request()).current_state.id

    def transition(name, value):
        request = ChangeState.Request()
        request.transition.id = value
        if not call(name, 'change_state', ChangeState, request).success:
            raise RuntimeError('Camera release rejected: '+name)

    release_competing_camera(target, state, transition, guard, emit)
