#!/usr/bin/env python3
"""
explorer.py — Autonomous exploration with obstacle avoidance and return home.

Mode switching via /robot/mode (std_msgs/String):
  "autonomous" — start/resume exploring
  "teleop"     — hand control back, explorer stands by

Manual commands via /explorer/command (std_msgs/String):
  "restart"    — reset mission, re-save home from current pose
  "stop"       — abort, stay in DONE
  "return"     — skip exploration, go home now

Duration params (set before switching to autonomous or calling restart):
  ~explore_seconds  (default 120)
  ~return_timeout   (default 90)
  ~stuck_window     (default 5)
"""

import math
import random
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse


# ── tuneable constants ────────────────────────────────────────────────────────

OBSTACLE_DIST_M   = 0.35
HOME_RADIUS_M     = 0.20
FORWARD_SPEED     = 0.12
RETURN_SPEED      = 0.12
TURN_SPEED        = 0.5
ANGLE_TOLERANCE   = 0.2
STUCK_MOVE_M      = 0.03
BACKUP_SPEED      = -0.10
BACKUP_DURATION_S = 1.2
ESCAPE_TURN_S     = 0.6


# ── helpers ───────────────────────────────────────────────────────────────────

def pose_distance(a, b):
    dx = a.position.x - b.position.x
    dy = a.position.y - b.position.y
    return math.sqrt(dx * dx + dy * dy)

def angle_to_pose(current, target):
    dx = target.position.x - current.position.x
    dy = target.position.y - current.position.y
    return math.atan2(dy, dx)

def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_error(current_yaw, target_angle):
    err = target_angle - current_yaw
    while err >  math.pi: err -= 2.0 * math.pi
    while err < -math.pi: err += 2.0 * math.pi
    return err


# ── explorer ──────────────────────────────────────────────────────────────────

class Explorer:

    STATE_WAIT    = "WAIT"
    STATE_EXPLORE = "EXPLORE"
    STATE_STUCK   = "STUCK"
    STATE_RETURN  = "RETURN"
    STATE_DONE    = "DONE"

    def __init__(self):
        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("/odom",              Odometry, self._odom_cb)
        rospy.Subscriber("/sr04/range",        Range,    self._range_cb)
        rospy.Subscriber("/robot/mode",        String,   self._mode_cb)
        rospy.Subscriber("/explorer/command",  String,   self._command_cb)

        rospy.Service("/explorer/restart", Trigger, self._restart_srv)

        self._active  = False
        self._pose    = None
        self._yaw     = 0.0
        self._blocked = False

        self._reset_mission()

        rospy.loginfo("Explorer: ready — publish 'autonomous' to /robot/mode to start")

    # ── mission state ─────────────────────────────────────────────────────────

    def _reset_mission(self):
        """Wipe mission state and re-read duration params."""
        self._explore_seconds = rospy.get_param("~explore_seconds", 120.0)
        self._return_timeout  = rospy.get_param("~return_timeout",   90.0)
        self._stuck_window    = rospy.get_param("~stuck_window",      5.0)

        self._state            = self.STATE_WAIT
        self._home             = None
        self._start_time       = None
        self._return_start     = None
        self._last_moved_pose  = None
        self._last_moved_time  = None

        rospy.loginfo(
            "Explorer: mission reset (explore=%.0fs return_timeout=%.0fs)",
            self._explore_seconds, self._return_timeout,
        )

    def _save_home(self):
        """Save current pose as home and transition to EXPLORE."""
        if self._pose is None:
            rospy.logwarn("Explorer: cannot save home — no odometry yet")
            return
        self._home            = self._pose
        self._start_time      = rospy.Time.now()
        self._last_moved_pose = self._pose
        self._last_moved_time = rospy.Time.now().to_sec()
        self._state           = self.STATE_EXPLORE
        rospy.loginfo(
            "Explorer: home saved at (%.2f, %.2f) — exploring for %.0fs",
            self._home.position.x, self._home.position.y,
            self._explore_seconds,
        )

    # ── ROS callbacks ─────────────────────────────────────────────────────────

    def _odom_cb(self, msg):
        self._pose = msg.pose.pose
        self._yaw  = yaw_from_quaternion(msg.pose.pose.orientation)

        # first odometry after a reset — auto-save home
        if self._home is None and self._state == self.STATE_WAIT and self._active:
            self._save_home()

    def _range_cb(self, msg):
        self._blocked = (0.0 < msg.range < OBSTACLE_DIST_M)

    def _mode_cb(self, msg):
        mode = msg.data.strip().lower()

        if mode == "autonomous":
            self._active = True
            if self._state in (self.STATE_WAIT, self.STATE_DONE):
                self._reset_mission()
                self._save_home()
            rospy.loginfo("Explorer: autonomous mode — active")

        elif mode == "teleop":
            self._active = False
            self._stop()
            rospy.loginfo("Explorer: teleop mode — standing by")

    def _command_cb(self, msg):
        cmd = msg.data.strip().lower()

        if cmd in ("start", "restart"):
            self._stop()
            self._reset_mission()
            self._save_home()
            self._active = True

        elif cmd == "stop":
            self._active = False
            self._stop()
            self._state = self.STATE_DONE
            rospy.loginfo("Explorer: stopped by command")

        elif cmd == "return":
            if self._state not in (self.STATE_DONE, self.STATE_WAIT):
                self._state        = self.STATE_RETURN
                self._return_start = rospy.Time.now().to_sec()
                rospy.loginfo("Explorer: returning home by command")

    def _restart_srv(self, req):
        self._stop()
        self._reset_mission()
        self._save_home()
        self._active = True
        msg = "Mission restarted" if self._home else "Waiting for odometry"
        return TriggerResponse(success=True, message=msg)

    # ── motion ────────────────────────────────────────────────────────────────

    def _publish(self, linear, angular):
        if not self._active:
            return
        t = Twist()
        t.linear.x  = linear
        t.angular.z = angular
        self._cmd_pub.publish(t)

    def _stop(self):
        # bypass _active so we can always stop
        t = Twist()
        self._cmd_pub.publish(t)

    def _drive_duration(self, linear, angular, duration_s):
        """Blocking drive that keeps MCU watchdog fed and respects mode changes."""
        rate = rospy.Rate(10)
        end  = rospy.Time.now() + rospy.Duration(duration_s)
        while rospy.Time.now() < end and not rospy.is_shutdown():
            if not self._active:
                return
            self._publish(linear, angular)
            rate.sleep()

    # ── stuck detection ───────────────────────────────────────────────────────

    def _update_stuck(self):
        if self._pose is None or self._last_moved_pose is None:
            return False
        dist = pose_distance(self._pose, self._last_moved_pose)
        now  = rospy.Time.now().to_sec()
        if dist > STUCK_MOVE_M:
            self._last_moved_pose = self._pose
            self._last_moved_time = now
            return False
        return (now - self._last_moved_time) > self._stuck_window

    def _reset_stuck_window(self):
        self._last_moved_pose = self._pose
        self._last_moved_time = rospy.Time.now().to_sec()

    # ── state handlers ────────────────────────────────────────────────────────

    def _handle_explore(self):
        elapsed = (rospy.Time.now() - self._start_time).to_sec()

        if elapsed >= self._explore_seconds:
            rospy.loginfo("Explorer: explore time up — returning home")
            self._state        = self.STATE_RETURN
            self._return_start = rospy.Time.now().to_sec()
            self._stop()
            return

        if self._update_stuck():
            rospy.logwarn("Explorer: stuck during exploration")
            self._state = self.STATE_STUCK
            return

        if self._blocked:
            self._stop()         # yield to MCU avoidance
        else:
            self._publish(FORWARD_SPEED, 0.0)

    def _handle_stuck(self):
        rospy.loginfo("Explorer: executing stuck escape")
        self._stop()
        rospy.sleep(0.2)

        self._drive_duration(BACKUP_SPEED, 0.0, BACKUP_DURATION_S)
        direction = random.choice([-1.0, 1.0])
        self._drive_duration(0.0, direction * TURN_SPEED, ESCAPE_TURN_S)
        self._stop()

        self._reset_stuck_window()

        if not self._active:
            return
        self._state = self.STATE_RETURN if self._return_start else self.STATE_EXPLORE
        rospy.loginfo("Explorer: stuck escape done — resuming %s", self._state)

    def _handle_return(self):
        now = rospy.Time.now().to_sec()

        if (now - self._return_start) > self._return_timeout:
            rospy.logwarn("Explorer: return timeout — stopping in place")
            self._stop()
            self._state = self.STATE_DONE
            return

        if pose_distance(self._pose, self._home) < HOME_RADIUS_M:
            rospy.loginfo("Explorer: home reached!")
            self._stop()
            self._state = self.STATE_DONE
            return

        if self._update_stuck():
            rospy.logwarn("Explorer: stuck during return")
            self._state = self.STATE_STUCK
            return

        if self._blocked:
            self._publish(0.0, TURN_SPEED)
            return

        bearing     = angle_to_pose(self._pose, self._home)
        heading_err = angle_error(self._yaw, bearing)

        if abs(heading_err) > ANGLE_TOLERANCE:
            sign = 1.0 if heading_err > 0 else -1.0
            self._publish(0.0, sign * TURN_SPEED)
        else:
            self._publish(RETURN_SPEED, 0.0)

    # ── main loop ─────────────────────────────────────────────────────────────

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self._active:
                pass  # teleop has control

            elif self._state == self.STATE_WAIT:
                pass  # waiting for odometry

            elif self._state == self.STATE_EXPLORE:
                self._handle_explore()

            elif self._state == self.STATE_STUCK:
                self._handle_stuck()

            elif self._state == self.STATE_RETURN:
                self._handle_return()

            elif self._state == self.STATE_DONE:
                self._stop()
                rospy.loginfo_once("Explorer: mission complete — "
                                   "switch to autonomous or call restart to run again")

            rate.sleep()

        self._stop()


if __name__ == "__main__":
    rospy.init_node("explorer")
    try:
        Explorer().run()
    except rospy.ROSInterruptException:
        pass
