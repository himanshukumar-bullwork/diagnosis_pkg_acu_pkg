#!/usr/bin/env python3
import math
import time
import yaml
from collections import deque
from typing import Dict, Any, Tuple, List, Deque, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

# Topic types we sample for rate/age
from geometry_msgs.msg import Twist
from nav2_msgs.msg import Costmap, BehaviorTreeLog
from sensor_msgs.msg import LaserScan, PointCloud2

# TF2
import tf2_ros

# Nav2 actions we probe for readiness
from nav2_msgs.action import NavigateToPose, ComputePathToPose, FollowPath, SmoothPath

# Nav2 action status feeds (for "idle vs active" gating)
from action_msgs.msg import GoalStatusArray

# -------- Remote parameter cache (async, non-blocking) --------
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterType, ParameterValue


def _pv_to_py(pv: ParameterValue):
    t = pv.type
    if t == ParameterType.PARAMETER_NOT_SET: return None
    if t == ParameterType.PARAMETER_STRING:  return pv.string_value
    if t == ParameterType.PARAMETER_BOOL:    return bool(pv.bool_value)
    if t == ParameterType.PARAMETER_INTEGER: return int(pv.integer_value)
    if t == ParameterType.PARAMETER_DOUBLE:  return float(pv.double_value)
    if t == ParameterType.PARAMETER_STRING_ARRAY:  return list(pv.string_array_value)
    if t == ParameterType.PARAMETER_INTEGER_ARRAY: return list(pv.integer_array_value)
    if t == ParameterType.PARAMETER_DOUBLE_ARRAY:  return list(pv.double_array_value)
    if t == ParameterType.PARAMETER_BOOL_ARRAY:    return list(pv.bool_array_value)
    return None


class RemoteParamCache:
    """Caches remote node parameters via /<node>/get_parameters (async)."""
    def __init__(self, node: Node, refresh_s: float = 5.0):
        self.node = node
        self.refresh_s = max(1.0, float(refresh_s))
        self._clients: Dict[str, Any] = {}              # node_fqn -> client
        self._last: Dict[str, Dict[str, Any]] = {}      # node_fqn -> {param: value}
        self._stamp: Dict[str, float] = {}              # node_fqn -> epoch
        self._pending: Dict[str, Tuple[Any, List[str]]] = {}  # node_fqn -> (future, names)
        self._nameset: Dict[str, set] = {}              # node_fqn -> names set

    def ensure(self, node_fqn: str, names: List[str]):
        """Schedule a refresh if stale; merge names to request together."""
        names = [n for n in names if n]
        if not names:
            return
        self._nameset.setdefault(node_fqn, set()).update(names)

        # If a call is already in flight, or cache is fresh, do nothing
        if node_fqn in self._pending and not self._pending[node_fqn][0].done():
            return
        now = time.time()
        if (now - self._stamp.get(node_fqn, 0.0)) < self.refresh_s and node_fqn in self._last:
            return

        client = self._clients.get(node_fqn)
        if client is None:
            client = self.node.create_client(GetParameters, f"{node_fqn}/get_parameters")
            self._clients[node_fqn] = client

        # Non-blocking: if service not up yet, try again next tick
        if not client.wait_for_service(timeout_sec=0.0):
            return

        req = GetParameters.Request()
        req.names = sorted(self._nameset.get(node_fqn, set()))
        if not req.names:
            return

        fut = client.call_async(req)
        self._pending[node_fqn] = (fut, list(req.names))
        self._nameset[node_fqn].clear()

    def poll(self):
        """Harvest completed requests."""
        to_clear: List[str] = []
        for node_fqn, (fut, names) in list(self._pending.items()):
            if not fut.done():
                continue
            to_clear.append(node_fqn)
            try:
                resp = fut.result()
                vals = {}
                # Response has only .values; zip with the request names
                for name, pv in zip(names, resp.values):
                    vals[name] = _pv_to_py(pv)
                self._last[node_fqn] = vals
                self._stamp[node_fqn] = time.time()
            except Exception as e:
                self.node.get_logger().warn(f"[nav2_diag] param fetch failed for {node_fqn}: {e}")
        for n in to_clear:
            self._pending.pop(n, None)

    def get(self, node_fqn: str) -> Dict[str, Any]:
        return self._last.get(node_fqn, {})


# ---------------- Helpers ----------------
def diag_kv(status: DiagnosticStatus, key: str, value: Any):
    status.values.append(KeyValue(key=key, value=str(value)))


class TopicRateMonitor:
    """Track rate and age for a topic. Uses header.stamp if present."""
    def __init__(self, node: Node, topic: str, msg_type, qos: QoSProfile, header_based=True, window=50):
        self.node = node
        self.topic = topic
        self.header_based = header_based
        self.stamps: Deque[rclpy.time.Time] = deque(maxlen=window)
        self.last_age: Optional[float] = None
        self.sub = node.create_subscription(msg_type, topic, self._cb, qos)

    def _cb(self, msg):
        now = self.node.get_clock().now()
        if self.header_based and hasattr(msg, 'header'):
            try:
                stamp = rclpy.time.Time.from_msg(msg.header.stamp)
            except Exception:
                stamp = now
        else:
            stamp = now
        self.stamps.append(stamp)
        self.last_age = (now - stamp).nanoseconds * 1e-9

    def rate_hz(self) -> float:
        if len(self.stamps) < 2:
            return 0.0
        xs = list(self.stamps)
        if len(xs) > 10:
            xs = xs[-10:]
        dt = (xs[-1] - xs[0]).nanoseconds * 1e-9
        if dt <= 0:
            return 0.0
        return (len(xs) - 1) / dt

    def age_s(self) -> float:
        return self.last_age if self.last_age is not None else math.inf


class ActionServerMonitor:
    """Non-blocking readiness probe for a Nav2 action server."""
    def __init__(self, node: Node, action_type, name: str):
        self.client = ActionClient(node, action_type, name)
        self.name = name

    def ready_now(self) -> bool:
        # Non-blocking check; does not stall the diag timer
        return self.client.wait_for_server(timeout_sec=0.0)


class TFMonitor:
    def __init__(self, node: Node):
        self.buf = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.listener = tf2_ros.TransformListener(self.buf, node)

    def age(self, target: str, source: str):
        """Return (age_seconds, ok). If TF missing, (inf, False)."""
        try:
            t = self.buf.lookup_transform(target, source, rclpy.time.Time())
            now = rclpy.time.Time()
            age = (now - rclpy.time.Time.from_msg(t.header.stamp)).nanoseconds * 1e-9
            return max(0.0, age), True
        except Exception:
            return math.inf, False


# --------------- Main diagnostics node ---------------
class Nav2Diagnostics(Node):
    def __init__(self):
        super().__init__('nav2_diagnostics')

        # Required param: path to YAML
        self.declare_parameter('config_file', '')
        cfg_path = self.get_parameter('config_file').get_parameter_value().string_value
        if not cfg_path:
            self.get_logger().fatal('Parameter config_file is required (path to nav2_diag.yaml)')
            raise SystemExit(1)

        with open(cfg_path, 'r') as f:
            self.cfg = yaml.safe_load(f) or {}

        base = self.cfg.get('nav2_diag', {}) or {}
        self.ns = base.get('namespace', '') or ''
        self.components: Dict[str, Dict[str, Any]] = base.get('components', {}) or {}

        # Timing / thresholds (active)
        self.debounce_s = float(base.get('debounce_s', 2.0))
        self.tf_warn = float(base.get('tf_age_warn_s', 0.3))
        self.tf_err = float(base.get('tf_age_error_s', 0.8))
        self.action_wait_warn = float(base.get('action_server_wait_warn_s', 0.5))
        self.action_wait_err = float(base.get('action_server_wait_error_s', 2.0))

        # Idle behavior (optional in YAML)
        idle_cfg = base.get('idle', {}) or {}
        self.gate_cmd_vel = bool(idle_cfg.get('gate_cmd_vel', True))
        self.tf_idle_warn = float(idle_cfg.get('tf_age_warn_s', 2.0))
        self.tf_idle_err = float(idle_cfg.get('tf_age_error_s', 5.0))

        # QoS + publish timer (2 Hz default)
        self.declare_parameter('publish_hz', 2.0)
        hz = float(self.get_parameter('publish_hz').value) or 2.0
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50
        )
        self.pub = self.create_publisher(DiagnosticArray, '/diagnostics', self.qos)
        self.timer = self.create_timer(1.0 / hz, self.tick)

        # Determine if we even need TF
        need_tf = any((c.get('tf_required') for c in self.components.values()))
        self.tfmon = TFMonitor(self) if need_tf else None

        # Monitors
        self.topic_monitors: Dict[Tuple[str, str], TopicRateMonitor] = {}
        self.action_monitors: Dict[str, ActionServerMonitor] = {}
        self._ar_not_ready_since: Dict[str, float] = {}  # comp_name -> epoch seconds

        # Remote parameter cache (for plugin checks)
        self.param_cache = RemoteParamCache(self, refresh_s=5.0)
        # plugin config: comp -> (node_fqn, param_name, required_set)
        self.plugin_cfg: Dict[str, Tuple[str, str, set]] = {}

        # Build monitors from config
        self._setup_monitors_and_plugins()

        # Debounce store per component
        self.last_levels: Dict[str, Tuple[int, float]] = {}  # name -> (level, since_time)

        # Track navigation activity for idle gating
        self._nav_active = False
        # Nav2 action status topics
        self.create_subscription(
            GoalStatusArray, self._ns('/navigate_to_pose/_action/status'),
            self._nav_status_cb, 10
        )
        self.create_subscription(
            GoalStatusArray, self._ns('/follow_path/_action/status'),
            self._ctrl_status_cb, 10
        )

    # ---------- Utils ----------
    def _ns(self, name: str) -> str:
        """Attach namespace if provided in YAML."""
        if not self.ns:
            return name
        if name.startswith('/'):
            return f'{self.ns}{name}'
        return f'{self.ns}/{name}'

    def _node_fqn(self, node_name: str) -> str:
        """Full node name for services like /<node>/get_parameters."""
        base = node_name if node_name.startswith('/') else f'/{node_name}'
        return self._ns(base)

    def _any_active(self, msg: GoalStatusArray) -> bool:
        # ACTIVE-ish includes ACCEPTED(1) and EXECUTING(2)
        return any(s.status in (1, 2) for s in msg.status_list)

    def _nav_status_cb(self, msg: GoalStatusArray):
        self._nav_active = self._any_active(msg)

    def _ctrl_status_cb(self, msg: GoalStatusArray):
        if self._any_active(msg):
            self._nav_active = True

    def _setup_monitors_and_plugins(self):
        # default param names for plugin lists by component
        default_plugin_param = {
            'bt_navigator': 'plugin_lib_names',     # string[]
            'planner_server': 'planner_plugins',    # string[]
            'controller_server': 'controller_plugins',
            'smoother_server': 'smoother_plugins',
        }

        for comp_name, comp in self.components.items():
            # Action servers
            action = comp.get('action_server')
            if action:
                action_full = self._ns(action)
                if comp_name == 'bt_navigator':
                    self.action_monitors[comp_name] = ActionServerMonitor(self, NavigateToPose, action_full)
                elif comp_name == 'planner_server':
                    self.action_monitors[comp_name] = ActionServerMonitor(self, ComputePathToPose, action_full)
                elif comp_name == 'controller_server':
                    self.action_monitors[comp_name] = ActionServerMonitor(self, FollowPath, action_full)
                elif comp_name == 'smoother_server':
                    self.action_monitors[comp_name] = ActionServerMonitor(self, SmoothPath, action_full)

            # Topics
            topics = comp.get('topics', {})
            for key, topic in topics.items():
                tname = self._ns(topic)
                if key in ('costmap', 'costmap_raw'):
                    self.topic_monitors[(comp_name, key)] = TopicRateMonitor(self, tname, Costmap, self.qos, header_based=True)
                elif key == 'cmd_vel':
                    self.topic_monitors[(comp_name, key)] = TopicRateMonitor(self, tname, Twist, self.qos, header_based=False)
                elif key == 'scan':
                    self.topic_monitors[(comp_name, key)] = TopicRateMonitor(self, tname, LaserScan, self.qos, header_based=True)
                elif key == 'rgbd_cloud':
                    self.topic_monitors[(comp_name, key)] = TopicRateMonitor(self, tname, PointCloud2, self.qos, header_based=True)
                elif key == 'behavior_tree_log':
                    self.topic_monitors[(comp_name, key)] = TopicRateMonitor(self, tname, BehaviorTreeLog, self.qos, header_based=True)

            # Plugins (optional per component)
            required = comp.get('plugins_required', []) or []
            if required:
                node_name = comp.get('node_name', comp_name)
                node_fqn = self._node_fqn(node_name)
                param_name = comp.get('plugin_param', default_plugin_param.get(comp_name, ''))
                if param_name:
                    self.plugin_cfg[comp_name] = (node_fqn, param_name, set(required))
                    # schedule parameter polling
                    self.param_cache.ensure(node_fqn, [param_name])

    # ---------- Severity helpers ----------
    @staticmethod
    def _lvl_name(level: int) -> str:
        return {0: 'OK', 1: 'WARN', 2: 'ERROR', 3: 'STALE'}.get(level, 'UNKNOWN')

    def _debounced_level(self, comp: str, new_level: int) -> int:
        now = time.time()
        prev = self.last_levels.get(comp)
        if prev is None or new_level > prev[0]:
            self.last_levels[comp] = (new_level, now)
            return new_level
        prev_level, since = prev
        if new_level < prev_level and (now - since) < self.debounce_s:
            return prev_level
        self.last_levels[comp] = (new_level, now)
        return new_level

    # ---------------- Core tick ----------------
    def tick(self):
        # harvest any parameter responses
        self.param_cache.poll()

        da = DiagnosticArray()
        da.header.stamp = self.get_clock().now().to_msg()
        comp_statuses: Dict[str, DiagnosticStatus] = {}
        nav_active = self._nav_active  # snapshot

        for comp_name, comp in self.components.items():
            status = DiagnosticStatus()
            status.name = comp_name
            status.hardware_id = comp.get('hardware_id', 'nav2')
            level = DiagnosticStatus.OK
            msg = 'OK'

            # 1) Action server readiness (non-blocking, time-based severity)
            if comp.get('action_server') and comp_name in self.action_monitors:
                ready = self.action_monitors[comp_name].ready_now()
                if ready:
                    self._ar_not_ready_since.pop(comp_name, None)
                    diag_kv(status, 'action_server_ok', True)
                else:
                    now_s = time.time()
                    start = self._ar_not_ready_since.get(comp_name)
                    elapsed = 0.0 if start is None else (now_s - start)
                    if start is None:
                        self._ar_not_ready_since[comp_name] = now_s
                    diag_kv(status, 'action_server_ok', False)
                    diag_kv(status, 'action_server_down_for_s', f'{elapsed:.1f}')
                    if elapsed > self.action_wait_err:
                        level = max(level, DiagnosticStatus.ERROR); msg = 'action server unavailable'
                    elif elapsed > self.action_wait_warn:
                        level = max(level, DiagnosticStatus.WARN); msg = 'action server slow to become ready'

            # 2) Topic freshness / rates
            topics = comp.get('topics', {}) or {}
            thr = comp.get('thresholds', {}) or {}

            def bump_from_rate(age, rate, warn_hz=None, err_hz=None, stale_age=None, label='topic'):
                nonlocal level, msg
                if stale_age is not None and age > stale_age:
                    level = max(level, DiagnosticStatus.STALE); msg = f'{label} stale'
                elif err_hz is not None and err_hz > 0 and rate < err_hz:
                    level = max(level, DiagnosticStatus.ERROR); msg = f'{label} rate too low'
                elif warn_hz is not None and warn_hz > 0 and rate < warn_hz:
                    level = max(level, DiagnosticStatus.WARN); msg = f'{label} rate low'

            # costmaps
            for key in ('costmap', 'costmap_raw'):
                if key in topics and (comp_name, key) in self.topic_monitors:
                    mon = self.topic_monitors[(comp_name, key)]
                    age = mon.age_s(); rate = mon.rate_hz()
                    diag_kv(status, f'{key}_age_s', f'{age:.2f}')
                    diag_kv(status, f'{key}_rate_hz', f'{rate:.2f}')
                    bump_from_rate(
                        age=age, rate=rate,
                        warn_hz=thr.get('publish_rate_warn_hz'),
                        err_hz=thr.get('publish_rate_error_hz'),
                        stale_age=thr.get('costmap_age_stale_s'),
                        label=key
                    )

            # cmd_vel — gate by nav activity if configured
            if 'cmd_vel' in topics and (comp_name, 'cmd_vel') in self.topic_monitors:
                mon = self.topic_monitors[(comp_name, 'cmd_vel')]
                age = mon.age_s(); rate = mon.rate_hz()
                diag_kv(status, 'cmd_vel_age_s', f'{age:.2f}')
                diag_kv(status, 'cmd_vel_rate_hz', f'{rate:.1f}')
                if nav_active or not self.gate_cmd_vel:
                    bump_from_rate(
                        age=age, rate=rate,
                        warn_hz=thr.get('cmd_vel_rate_warn_hz'),
                        err_hz=None,
                        stale_age=thr.get('cmd_vel_stale_s'),
                        label='cmd_vel'
                    )

            # sensors & logs
            for key in ('scan', 'rgbd_cloud', 'behavior_tree_log'):
                if key in topics and (comp_name, key) in self.topic_monitors:
                    mon = self.topic_monitors[(comp_name, key)]
                    age = mon.age_s(); rate = mon.rate_hz()
                    diag_kv(status, f'{key}_age_s', f'{age:.2f}')
                    diag_kv(status, f'{key}_rate_hz', f'{rate:.1f}')
                    base_key = key.split("_")[0]
                    bump_from_rate(
                        age=age, rate=rate,
                        warn_hz=thr.get(f'{base_key}_rate_warn_hz'),
                        err_hz=thr.get(f'{base_key}_rate_error_hz'),
                        stale_age=None,
                        label=key
                    )

            # 3) TF checks (looser thresholds when idle)
            if self.tfmon and comp.get('tf_required'):
                for pair in comp.get('tf_required', []) or []:
                    try:
                        tgt, src = [p.strip() for p in pair.split('->')]
                    except Exception:
                        continue
                    age, ok = self.tfmon.age(tgt, src)
                    diag_kv(status, f'tf_{tgt}_{src}_age_s', f'{age:.2f}' if ok else 'missing')

                    tf_warn = self.tf_warn if nav_active else self.tf_idle_warn
                    tf_err  = self.tf_err  if nav_active else self.tf_idle_err

                    if not ok or age > tf_err:
                        level = max(level, DiagnosticStatus.ERROR)
                        msg = 'tf missing' if not ok else 'tf too old'
                    elif age > tf_warn:
                        level = max(level, DiagnosticStatus.WARN)
                        msg = 'tf aging'

            # 4) Plugin checks (if declared in YAML)
            if comp_name in self.plugin_cfg:
                node_fqn, param_name, required = self.plugin_cfg[comp_name]
                # keep requesting occasionally
                self.param_cache.ensure(node_fqn, [param_name])
                vals = self.param_cache.get(node_fqn)
                got = vals.get(param_name, None)
                if got is None:
                    diag_kv(status, 'plugins_param_available', False)
                    diag_kv(status, 'plugins_param_name', param_name)
                    # Don't elevate above WARN while params still loading
                    level = max(level, DiagnosticStatus.WARN)
                    if msg == 'OK': msg = 'waiting for plugin params'
                else:
                    # expect list[str]
                    try:
                        present = set([str(x) for x in (got or [])])
                    except Exception:
                        present = set()
                    missing = sorted(list(required - present))
                    diag_kv(status, 'plugins_param_name', param_name)
                    diag_kv(status, 'plugins_required_count', len(required))
                    diag_kv(status, 'plugins_present_count', len(present))
                    if missing:
                        diag_kv(status, 'plugins_missing', ','.join(missing))
                        # escalate to ERROR – misconfig means node won’t function fully
                        level = max(level, DiagnosticStatus.ERROR)
                        msg = 'required plugins missing'
                        # provide suspect triage
                        diag_kv(status, 'suspect', comp_name)
                        diag_kv(status, 'confidence', 0.95)
                        diag_kv(status, 'evidence', f'missing plugins in {param_name}: {missing}')
                    else:
                        diag_kv(status, 'plugins_ok', True)

            # Debounce improvements
            level = self._debounced_level(comp_name, level)

            status.level = level
            status.message = msg
            comp_statuses[comp_name] = status
            da.status.append(status)

        # 5) Blame rules (optional, first match wins)
        for rule in self.cfg.get('nav2_diag', {}).get('blame_rules', []) or []:
            when = rule.get('when', {}) or {}
            and_tf_missing = rule.get('and_tf_missing', [])
            match = True
            for cname, levels in when.items():
                st = comp_statuses.get(cname)
                if st is None or self._lvl_name(st.level) not in levels:
                    match = False; break
            if match and and_tf_missing:
                for pair in and_tf_missing:
                    try:
                        tgt, src = [p.strip() for p in pair.split('->')]
                    except Exception:
                        match = False; break
                    if self.tfmon:
                        age, ok = self.tfmon.age(tgt, src)
                        if ok:  # expected missing
                            match = False; break
            if match:
                setv = rule.get('set', {}) or {}
                suspect = setv.get('suspect', '')
                confidence = setv.get('confidence', 0.5)
                evidence = setv.get('evidence', '')
                for st in comp_statuses.values():
                    if suspect:
                        diag_kv(st, 'suspect', suspect)
                        diag_kv(st, 'confidence', confidence)
                        if evidence:
                            diag_kv(st, 'evidence', evidence)
                break

        # 6) Overall roll-up
        roll = DiagnosticStatus()
        roll.hardware_id = 'nav2'
        roll.name = 'nav2_overall'
        if not comp_statuses:
            roll.level = DiagnosticStatus.WARN
            roll.message = 'no components configured'
        else:
            worst = max(st.level for st in comp_statuses.values())
            roll.level = worst
            roll.message = self._lvl_name(worst)
        da.status.append(roll)
        self.pub.publish(da)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Diagnostics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
