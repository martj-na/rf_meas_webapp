from threading import Thread

from dua_node_py.dua_node import NodeBase
from websocket_bridge.websocket_bridge_utils import AtomicBool
from websocket_bridge.websocket_bridge_class import WebSocketBridge

from std_srvs.srv import SetBool
from rosidl_runtime_py.convert import message_to_ordereddict
import json


class WebSocketBridgeNode(NodeBase):
    """
    WebSocket Bridge node class.
    """

    def __init__(self, node_name: str, verbose: bool) -> None:
        super().__init__(node_name, verbose)

        self.init_atomics()
        self.init_bridge()
        self.init_subscribers()

        if self._autostart:
            self.activate()

        self.get_logger().warn("WebSocket Bridge Node initialized")

    def __del__(self):
        self.cleanup()

    # --------------------------------------------------------------
    # INIT
    # --------------------------------------------------------------

    def init_atomics(self):
        self._running = AtomicBool(initial=False)

    def init_subscribers(self) -> None:
    """
    Create subscribers to all topics listed in self._topics.

    self._topics es: [
        "/zero_span rf_meas_manager_interfaces/msg/ZeroSpan",
        "/channel_power rf_meas_manager_interfaces/msg/ChannelPower",
        "/status std_msgs/msg/String",
    ]
    """
    self._topic_list = []

    topics_raw = getattr(self, "_topics", [])
    if not topics_raw:
        self.get_logger().warn("No topics configured for WebSocket bridge")
        return

    for entry in topics_raw:
        try:
            topic_name, msg_type_str = entry.split(" ", 1)

            pkg, _, msg = msg_type_str.partition("/msg/")
            module = __import__(f"{pkg}.msg", fromlist=[msg])
            msg_cls = getattr(module, msg)

            self.get_logger().info(
                f"[WS BRIDGE] Subscribing to {topic_name} ({msg_type_str})"
            )

            self.dua_create_subscription(
                msg_cls,
                topic_name,
                self.generic_callback,
            )

            self._topic_list.append(topic_name)

        except Exception as e:
            self.get_logger().error(f"Error parsing topic entry '{entry}': {e}")
    def init_service_servers(self):
        self.enable_server = self.dua_create_service_server(
            SetBool,
            '~/enable',
            self.enable_callback
        )

    def init_bridge(self):
        """
        Create the WebSocket server object (not started yet)
        """

        host = self._ws_host
        port = int(self._ws_port)

        self.bridge = WebSocketBridge(host=host, port=port)

        self.get_logger().info(f"WebSocket Bridge created on ws://{host}:{port}")

    # --------------------------------------------------------------
    # ACTIVATION / DEACTIVATION
    # --------------------------------------------------------------

    def activate(self):
        try:
            self._running.store(True)

            # worker thread → avvia il server WebSocket
            self._worker = Thread(target=self.worker_thread_routine, daemon=True)
            self._worker.start()

            self.get_logger().warn("WebSocket Bridge ACTIVATED")
        except Exception as e:
            self.get_logger().error(f"Failed to activate WebSocket Bridge: {e}")
            self._running.store(False)

    def deactivate(self):
        self.get_logger().info("Deactivating WebSocket Bridge")
        self._running.store(False)

        try:
            if hasattr(self, "bridge"):
                self.bridge.stop()  # da implementare nel class WebSocketBridge
        except Exception:
            pass

        if hasattr(self, "_worker") and self._worker.is_alive():
            self._worker.join(timeout=5)

        self.get_logger().warn("WebSocket Bridge DEACTIVATED")

    # --------------------------------------------------------------
    # WORKER THREAD (server WebSocket)
    # --------------------------------------------------------------

    def worker_thread_routine(self):
        self.get_logger().info("WebSocket worker thread started")

        try:
            self.bridge.run_server()  # blocking, inside thread
        except Exception as e:
            self.get_logger().error(f"Fatal error in WS worker: {e}")

        self._running.store(False)
        self.get_logger().info("WebSocket worker thread stopped")

    # --------------------------------------------------------------
    # CALLBACK ROS2
    # --------------------------------------------------------------

    def generic_callback(self, msg):
        """
        Every ROS topic converted to JSON and broadcast via WebSocket.
        """
        try:
            d = message_to_ordereddict(msg)
            json_msg = json.dumps(d)

            self.bridge.broadcast(json_msg)

        except Exception as e:
            self.get_logger().error(f"Error in WebSocket bridge callback: {e}")

    # --------------------------------------------------------------
    # SERVICE: ENABLE/DISABLE
    # --------------------------------------------------------------

    def enable_callback(self, req, resp):
        if req.data:
            self.activate()
        else:
            self.deactivate()

        resp.success = True
        return resp

    # --------------------------------------------------------------

    def cleanup(self):
        self.get_logger().info("WS Bridge cleanup")
        self.deactivate()