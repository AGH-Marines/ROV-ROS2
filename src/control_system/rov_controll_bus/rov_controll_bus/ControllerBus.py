#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from ds4_driver_msgs.msg import Status

from rov_mfsmc.ModelFreeSlidingControl import ModelFreeSlidingControl
from rov_passthrough_control.PassthroughControl import PassthroughControl
from rov_controll_bus.Controller import Controller

from typing import List, Dict, Tuple


class ControllerBus(Node):
    def __init__(self) -> None:
        super().__init__('rov_controller_bus', allow_undeclared_parameters=False)
        self.cb_group = ReentrantCallbackGroup()

        # ---- params (minimal) ----
        self.declare_parameter('ds4_next_button', 'button_dpad_right')
        self.ds4_next_button = str(self.get_parameter('ds4_next_button').value)

        self._param_prefix = 'c'
        self._param_list = [('version', None), ('init', False)]

        # ---- controllers & queue ----
        self.available_controlers: List[Controller] = self.get_available_controlers()
        self.parameters = self.load_config()                       # dict keyed by c0, c1, ...
        self.controller_queue: List[Controller] = self.get_controller_queue()  # first item is active

        if not self.controller_queue:
            self.get_logger().fatal('No controllers matched config. Exiting.')
            raise SystemExit(1)

        # ---- joystick ----
        self.create_subscription(Status, 'status', self.cb_ds4, 10, callback_group=self.cb_group)
        self._last_press_ms: Dict[str, int] = {}

        # ---- auto-start first controller after spin begins ----
        self._init_timer = self.create_timer(0.1, self._start_active_once, callback_group=self.cb_group)

        self.get_logger().info(f"ControllerBus ready. queue={[c.controller_name for c in self.controller_queue]}")

    # ---------- joystick ----------
    def cb_ds4(self, msg: Status) -> None:
        if getattr(msg, self.ds4_next_button, False) and self._debounced(self.ds4_next_button):
            self.get_logger().info(f'Next controller requested')
            ok = self.switch_to_next()
            if not ok:
                self.get_logger().warn('Switch failed; keeping current controller')

    def _debounced(self, key: str, window_ms: int = 250) -> bool:
        now = self._now_ms()
        last = self._last_press_ms.get(key, 0)
        if now - last >= window_ms:
            self._last_press_ms[key] = now
            return True
        return False

    def _now_ms(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1_000_000)

    # ---------- controller setup ----------
    def get_available_controlers(self) -> List[Controller]:
        return [ModelFreeSlidingControl(), PassthroughControl()]

    def load_config(self) -> Dict[str, dict]:
        out: Dict[str, dict] = {}
        i = 0
        while True:
            name_key = f'{self._param_prefix}{i}'
            self.declare_parameter(name_key, None)
            name = self.get_parameter(name_key).value
            if not isinstance(name, str):
                break
            entry = {'name': name, 'index': i}
            for p, default in self._param_list:
                pk = f'{self._param_prefix}{i}_{p}'
                self.declare_parameter(pk, None)
                val = self.get_parameter(pk).value
                if not (isinstance(val, str) or isinstance(val, bool)):
                    val = default
                entry[p] = val
            out[name_key] = entry
            i += 1
        return out

    def get_controller_queue(self) -> List[Controller]:
        queue: List[Controller] = []
        for key, conf in self.parameters.items():
            for c in self.available_controlers:
                if c.controller_name == conf['name'] or c.controller_short_name == conf['name']:
                    (queue.insert if conf.get('init', False) else queue.append)(0 if conf.get('init', False) else len(queue), c)
        # If no explicit init, keep discovered order
        return queue

    # ---------- always-on behavior ----------
    def _start_active_once(self):
        self.destroy_timer(self._init_timer)
        ok, msg = self._call_trigger_sync(f'/controllers/{self.controller_queue[0].controller_name}/run')
        if ok:
            self.get_logger().info(f"Controller '{self.controller_queue[0].controller_name}' running")
        else:
            self.get_logger().warn(f"Initial start failed: {msg}")

    def switch_to_next(self) -> bool:
        if len(self.controller_queue) < 2:
            self.get_logger().warn('Only one controller available; cannot switch')
            return False

        current = self.controller_queue[0]
        nxt = self.controller_queue[1]

        # Stop current (best-effort)
        ok_stop, msg_stop = self._call_trigger_sync(f'/controllers/{current.controller_name}/stop')
        if not ok_stop:
            self.get_logger().warn(f"Stop '{current.controller_name}' failed: {msg_stop} (continuing)")

        # Start next (must succeed to rotate)
        ok_run, msg_run = self._call_trigger_sync(f'/controllers/{nxt.controller_name}/run')
        if not ok_run:
            self.get_logger().error(f"Run '{nxt.controller_name}' failed: {msg_run}")
            # Try to re-run current to keep 'always running'
            ok_back, msg_back = self._call_trigger_sync(f'/controllers/{current.controller_name}/run')
            if not ok_back:
                self.get_logger().error(f"Fallback run '{current.controller_name}' failed: {msg_back}")
            return False

        # rotate only if run succeeded
        self.controller_queue.pop(0)
        self.controller_queue.append(current)
        self.get_logger().info(f"Switched to '{nxt.controller_name}'")
        return True

    # ---------- Trigger helper ----------
    def _call_trigger_sync(self, service_name: str) -> Tuple[bool, str]:
        client = self.create_client(Trigger, service_name, callback_group=self.cb_group)
        if not client.wait_for_service(timeout_sec=1.0):
            return False, 'service not available'
        fut = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=2.0)
        if not fut.done():
            return False, 'timeout'
        res = fut.result()
        return (bool(res.success), (res.message or '')) if res is not None else (False, 'no response')
