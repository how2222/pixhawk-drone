#!/usr/bin/env python3
"""
MAVLink keyboard controller (pygame) for the F450 / Pixhawk PRO quadcopter
==========================================================================
Flies the drone from a laptop over a SiK telemetry radio: no RC transmitter.
Usage:
    python3 mavcontrol.py --port /dev/tty.usbserial-0001 --baudrate 57600

Dependencies:
    pip install pymavlink pygame
"""

import pygame
import threading
import argparse
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("❌ pymavlink is not installed. Run: pip install pymavlink")
    sys.exit(1)


# =============================================================================
# 🎛️  CONFIGURATION
# =============================================================================

KEYBINDINGS = {
    pygame.K_SPACE:  ("SPACE",  "KILL / DISARM",    "disarm_force"),
    pygame.K_a:      ("A",      "ARM",              "arm"),
    pygame.K_t:      ("T",      "TAKEOFF 0.5m",     "takeoff"),
    pygame.K_l:      ("L",      "LAND",             "land"),
    pygame.K_UP:     ("↑",      "THROTTLE +",       "throttle_up"),
    pygame.K_DOWN:   ("↓",      "THROTTLE -",       "throttle_down"),
    pygame.K_LEFT:   ("←",      "YAW LEFT",       "yaw_left"),
    pygame.K_RIGHT:  ("→",      "YAW RIGHT",       "yaw_right"),
    pygame.K_z:      ("Z",      "MODE STABILIZE",   "mode_stabilize"),
    pygame.K_g:      ("G",      "MODE GUIDED",      "mode_guided"),
    pygame.K_h:      ("H",      "MODE ALT_HOLD",    "mode_alt_hold"),
    pygame.K_d:      ("D",      "AHRS ROLL +",      "ahrs_trim_roll_right"),
    pygame.K_q:      ("Q",      "AHRS ROLL -",      "ahrs_trim_roll_left"),
    pygame.K_s:      ("S",      "AHRS PITCH +",     "ahrs_trim_pitch_forward"),
    pygame.K_w:      ("W",      "AHRS PITCH -",     "ahrs_trim_pitch_back"),
    pygame.K_r:      ("R",      "RESET AHRS TRIM",  "ahrs_trim_reset"),
    pygame.K_m:      ("M",      "MONITOR ON/OFF",   "toggle_monitor"),
}

THROTTLE_STEP = 20      # smaller = gentler climb
YAW_STEP      = 5
AHRS_TRIM_STEP = 0.005  # ~0.3° per press (radians)
RC_SEND_HZ    = 10      # RC override send rate (Hz)

# =============================================================================


class MAVController:
    def __init__(self, port, baudrate):
        self.port = port
        self.baudrate = baudrate
        self.mav = None
        self.connected = False
        self.throttle = 1000
        self.current_yaw = 0
        self.ahrs_trim_x = 0.0  # AHRS_TRIM_X (roll, radians)
        self.ahrs_trim_y = 0.0  # AHRS_TRIM_Y (pitch, radians)
        self._rc_loop_running = False
        self._rc_thread = None
        self._monitor_running = False
        self._monitor_thread = None
        self._telemetry = {}

    def connect(self):
        try:
            self.mav = mavutil.mavlink_connection(self.port, baud=self.baudrate)
            self.mav.wait_heartbeat(timeout=10)
            self.connected = True
            return True
        except Exception as e:
            print(f"Connection failed: {e}")
            return False

    # -----------------------------------------------------------------
    # 📊  LIVE TELEMETRY MONITOR
    # -----------------------------------------------------------------
    MONITOR_PARAMS = [
        'AHRS_ORIENT', 'AHRS_ORIENTATION', 'BRD_ORIENT',
        'COMPASS_USE', 'COMPASS_USE2', 'COMPASS_USE3',
        'COMPASS_OFS_X', 'COMPASS_OFS_Y', 'COMPASS_OFS_Z',
        'ATC_RAT_YAW_P', 'ATC_RAT_YAW_I', 'ATC_RAT_YAW_D',
        'ATC_RAT_YAW_FLTD', 'ATC_RAT_YAW_FLTE',
        'MOT_SPIN_ARM', 'MOT_SPIN_MIN',
    ]

    def start_monitor(self):
        if not self.connected:
            return "Not connected"
        self._monitor_running = True
        self._request_data_streams()
        self._fetch_params()
        self._monitor_thread = threading.Thread(target=self._telemetry_recv_loop, daemon=True)
        self._monitor_thread.start()
        return "📊 Monitor ON"

    def stop_monitor(self):
        self._monitor_running = False
        return "📊 Monitor OFF"

    def _request_data_streams(self):
        """Ask the FC to stream ATTITUDE + SERVO_OUTPUT at 4 Hz."""
        for stream_id, rate in [
            (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1, 4),      # ATTITUDE
            (mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS, 4), # SERVO_OUTPUT_RAW
        ]:
            self.mav.mav.request_data_stream_send(
                self.mav.target_system,
                self.mav.target_component,
                stream_id, rate, 1
            )

    def _fetch_params(self):
        """Request all monitored params from the FC."""
        for name in self.MONITOR_PARAMS:
            self.mav.mav.param_request_read_send(
                self.mav.target_system,
                self.mav.target_component,
                name.encode('utf-8'),
                -1
            )
            time.sleep(0.05)

    def _telemetry_recv_loop(self):
        """Background thread: receive MAVLink messages and update _telemetry."""
        while self._monitor_running:
            try:
                msg = self.mav.recv_match(blocking=True, timeout=0.25)
                if msg is None:
                    continue
                mtype = msg.get_type()
                if mtype == 'PARAM_VALUE':
                    self._telemetry[msg.param_id.replace('\x00', '')] = msg.param_value
                elif mtype == 'SERVO_OUTPUT_RAW':
                    self._telemetry['PWM1'] = msg.servo1_raw
                    self._telemetry['PWM2'] = msg.servo2_raw
                    self._telemetry['PWM3'] = msg.servo3_raw
                    self._telemetry['PWM4'] = msg.servo4_raw
                elif mtype == 'ATTITUDE':
                    self._telemetry['yaw_deg'] = round(msg.yaw * 57.2958, 1)
                    self._telemetry['yaw_rate'] = round(msg.yawspeed * 57.2958, 1)
                    self._telemetry['roll_deg'] = round(msg.roll * 57.2958, 1)
                    self._telemetry['pitch_deg'] = round(msg.pitch * 57.2958, 1)
            except Exception:
                pass

    def arm(self):
        if not self.connected: return "Not connected"
        self.mav.arducopter_arm()
        self._start_rc_loop()
        return "✅ ARM sent (RC loop started)"

    def disarm_force(self):
        if not self.connected: return "Not connected"
        self._stop_rc_loop()
        self.throttle = 1000
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 21196, 0, 0, 0, 0, 0
        )
        return "🛑 FORCE DISARM sent (RC loop stopped)"

    def takeoff(self, altitude=0.5):
        if not self.connected: return "Not connected"
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        return f"🚁 TAKEOFF {altitude} m sent"

    def land(self):
        if not self.connected: return "Not connected"
        self.mav.mav.command_long_send(
            self.mav.target_system, self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        return "🛬 LAND sent"

    def throttle_up(self):
        self.throttle = min(2000, self.throttle + THROTTLE_STEP)

        self._send_rc_override()
        return f"⬆️  Throttle: {self.throttle}"

    def throttle_down(self):
        self.throttle = max(1000, self.throttle - THROTTLE_STEP)
        self._send_rc_override()
        return f"⬇️  Throttle: {self.throttle}"

    def yaw_left(self):
        self.current_yaw -= YAW_STEP
        return f"↺  Yaw: {self.current_yaw}°"

    def yaw_right(self):
        self.current_yaw += YAW_STEP
        return f"↻  Yaw: {self.current_yaw}°"

    def ahrs_trim_roll_right(self):
        self.ahrs_trim_x = min(0.1, self.ahrs_trim_x + AHRS_TRIM_STEP)
        self._set_param('AHRS_TRIM_X', self.ahrs_trim_x)
        deg = self.ahrs_trim_x * 57.2958
        return f"➡️  AHRS roll: {deg:+.1f}°"

    def ahrs_trim_roll_left(self):
        self.ahrs_trim_x = max(-0.1, self.ahrs_trim_x - AHRS_TRIM_STEP)
        self._set_param('AHRS_TRIM_X', self.ahrs_trim_x)
        deg = self.ahrs_trim_x * 57.2958
        return f"⬅️  AHRS roll: {deg:+.1f}°"

    def ahrs_trim_pitch_forward(self):
        self.ahrs_trim_y = min(0.1, self.ahrs_trim_y + AHRS_TRIM_STEP)
        self._set_param('AHRS_TRIM_Y', self.ahrs_trim_y)
        deg = self.ahrs_trim_y * 57.2958
        return f"⬆️  AHRS pitch: {deg:+.1f}°"

    def ahrs_trim_pitch_back(self):
        self.ahrs_trim_y = max(-0.1, self.ahrs_trim_y - AHRS_TRIM_STEP)
        self._set_param('AHRS_TRIM_Y', self.ahrs_trim_y)
        deg = self.ahrs_trim_y * 57.2958
        return f"⬇️  AHRS pitch: {deg:+.1f}°"

    def ahrs_trim_reset(self):
        self.ahrs_trim_x = 0.0
        self.ahrs_trim_y = 0.0
        self._set_param('AHRS_TRIM_X', 0.0)
        self._set_param('AHRS_TRIM_Y', 0.0)
        return "🔄 AHRS trim reset (0°/0°)"

    def _set_param(self, name, value):
        if not self.connected:
            return
        self.mav.mav.param_set_send(
            self.mav.target_system,
            self.mav.target_component,
            name.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )

    def _send_rc_override(self):
        if not self.connected:
            return
        self.mav.mav.rc_channels_override_send(
            self.mav.target_system,
            self.mav.target_component,
            0,                # CH1 roll  (0 = FC handles it)
            0,                # CH2 pitch (0 = FC handles it)
            self.throttle,    # CH3 throttle
            0,                # CH4 yaw
            0, 0, 0, 0
        )

    def _start_rc_loop(self):
        if self._rc_loop_running:
            return
        self._rc_loop_running = True
        self._rc_thread = threading.Thread(target=self._rc_loop, daemon=True)
        self._rc_thread.start()

    def _stop_rc_loop(self):
        self._rc_loop_running = False

    def _rc_loop(self):
        """Send RC overrides continuously at RC_SEND_HZ.
        ArduPilot times overrides out after ~1 s without a message."""
        while self._rc_loop_running:
            self._send_rc_override()
            time.sleep(1.0 / RC_SEND_HZ)

    def mode_stabilize(self):
        if not self.connected: return "Not connected"
        self.mav.set_mode("STABILIZE")
        return "🔄 Mode STABILIZE"

    def mode_guided(self):
        if not self.connected: return "Not connected"
        self.mav.set_mode("GUIDED")
        return "🔄 Mode GUIDED"

    def mode_alt_hold(self):
        if not self.connected: return "Not connected"
        self.mav.set_mode("ALT_HOLD")
        return "🔄 Mode ALT_HOLD"

    def toggle_monitor(self):
        if self._monitor_running:
            return self.stop_monitor()
        else:
            return self.start_monitor()

    def run_command(self, cmd_str):
        """Parse and execute a MAVLink command string.
        Supported:
          motortest <motor> <type> <throttle%> <duration_s>
          set <PARAM> <value>
          mode <MODE_NAME>
        """
        if not self.connected:
            return "Not connected"
        parts = cmd_str.strip().split()
        if not parts:
            return ""
        verb = parts[0].lower()

        if verb == 'motortest':
            if len(parts) < 5:
                return "Usage: motortest <motor 1-4> <type 0=pct> <throttle%> <duration_s>"
            try:
                motor    = int(parts[1])
                thr_type = int(parts[2])
                throttle = float(parts[3])
                duration = float(parts[4])
            except ValueError:
                return "Invalid parameters"
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_MOTOR_TEST,
                0,
                motor,       # motor instance (1-based)
                thr_type,    # throttle type (0 = percent)
                throttle,    # throttle value
                duration,    # duration (seconds)
                0, 0, 0
            )
            return f"⚙️  motortest motor={motor} type={thr_type} thr={throttle}% dur={duration}s"

        elif verb == 'set':
            if len(parts) < 3:
                return "Usage: set <PARAM_NAME> <value>"
            param_name = parts[1].upper()
            try:
                value = float(parts[2])
            except ValueError:
                return "Valeur invalide"
            self._set_param(param_name, value)
            # Request read-back so the monitor thread picks it up
            time.sleep(0.3)
            self.mav.mav.param_request_read_send(
                self.mav.target_system,
                self.mav.target_component,
                param_name.encode('utf-8'),
                -1
            )
            # Wait for monitor thread to update telemetry
            time.sleep(0.5)
            actual = self._telemetry.get(param_name, None)
            if actual is not None:
                if isinstance(actual, float) and actual == int(actual):
                    actual = int(actual)
                return f"⚙️  {param_name} = {actual} (reboot required for SERVOx)"
            return f"⚙️  set {param_name} = {value} (sent, no confirmation)"

        elif verb == 'reboot':
            self.mav.mav.command_long_send(
                self.mav.target_system, self.mav.target_component,
                mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
                0, 1, 0, 0, 0, 0, 0, 0
            )
            self.connected = False
            return "🔄 Reboot sent — reconnect after 10 s"

        elif verb == 'mode':
            if len(parts) < 2:
                return "Usage: mode <MODE_NAME>"
            mode_name = parts[1].upper()
            self.mav.set_mode(mode_name)
            return f"🔄 mode {mode_name}"

        else:
            return f"Unknown command: {verb} (motortest/set/mode)"

    def dispatch(self, fn_name):
        fn = getattr(self, fn_name, None)
        if fn:
            return fn()
        return f"Unknown function: {fn_name}"


class MAVControlUI:
    BG         = (13, 13, 13)
    PANEL      = (26, 26, 26)
    ACCENT     = (0, 255, 136)
    KILL_BG    = (80, 20, 20)
    KILL_ACC   = (255, 51, 51)
    TEXT       = (204, 204, 204)
    DIM        = (85, 85, 85)
    FLASH      = (0, 60, 40)
    FLASH_KILL = (100, 0, 0)

    WARN       = (255, 200, 0)
    WIDTH      = 560
    WIDTH_MON  = 920
    HEIGHT     = 580

    def __init__(self, controller):
        self.ctrl = controller
        self.log_msg = "Waiting for connection..."
        self.connected = False
        self.monitor_on = False
        self.flash = {}
        self.cmd_active = False
        self.cmd_text = ""

        pygame.init()
        pygame.display.set_caption("MAVControl")
        self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
        self.clock = pygame.time.Clock()

        self.font_title = pygame.font.SysFont("Menlo", 22, bold=True)
        self.font_key   = pygame.font.SysFont("Menlo", 11)
        self.font_label = pygame.font.SysFont("Menlo", 13, bold=True)
        self.font_log   = pygame.font.SysFont("Menlo", 12)
        self.font_small = pygame.font.SysFont("Menlo", 10)
        self.font_mon   = pygame.font.SysFont("Menlo", 11)
        self.font_mon_h = pygame.font.SysFont("Menlo", 12, bold=True)
        self.font_cmd   = pygame.font.SysFont("Menlo", 13)

        threading.Thread(target=self._connect, daemon=True).start()

    def _connect(self):
        self.log_msg = "⏳ Connecting..."
        if self.ctrl.connect():
            self.connected = True
            self.log_msg = f"✅ Connected to {self.ctrl.port}"
        else:
            self.log_msg = f"❌ Connection failed on {self.ctrl.port}"

    def _execute(self, fn_name, key=None):
        result = self.ctrl.dispatch(fn_name)
        self.log_msg = result
        if key is not None:
            self.flash[key] = time.time()
        if fn_name == 'toggle_monitor':
            self.monitor_on = self.ctrl._monitor_running
            new_w = self.WIDTH_MON if self.monitor_on else self.WIDTH
            self.screen = pygame.display.set_mode((new_w, self.HEIGHT))

    def _draw_button(self, rect, key_label, label, fn_name, keycode):
        is_kill = fn_name == "disarm_force"
        now = time.time()
        flashing = keycode in self.flash and (now - self.flash[keycode]) < 0.15

        bg = (self.FLASH_KILL if is_kill else self.FLASH) if flashing else \
             (self.KILL_BG if is_kill else self.PANEL)

        border_color = self.KILL_ACC if is_kill else (42, 42, 42)
        acc = self.KILL_ACC if is_kill else self.ACCENT

        pygame.draw.rect(self.screen, border_color, rect, border_radius=6)
        inner = pygame.Rect(rect.x + 1, rect.y + 1, rect.w - 2, rect.h - 2)
        pygame.draw.rect(self.screen, bg, inner, border_radius=5)

        key_surf = self.font_key.render(f"[{key_label}]", True, acc)
        self.screen.blit(key_surf, (rect.x + 10, rect.y + 8))

        label_surf = self.font_label.render(label, True, self.TEXT)
        self.screen.blit(label_surf, (rect.x + 10, rect.y + 26))

    def _draw_monitor(self, x, y0):
        """Draw the live telemetry panel on the right side of the window."""
        tel = self.ctrl._telemetry
        y = y0

        # Vertical separator
        pygame.draw.line(self.screen, self.ACCENT, (x - 10, 52), (x - 10, self.HEIGHT - 30), 1)

        self.screen.blit(self.font_mon_h.render("TELEMETRY MONITOR", True, self.ACCENT), (x, y))
        y += 22

        # ── PARAMS ──
        self.screen.blit(self.font_mon_h.render("FC PARAMETERS", True, self.WARN), (x, y))
        y += 16
        for p in self.ctrl.MONITOR_PARAMS:
            val = tel.get(p, '---')
            if isinstance(val, float):
                val = int(val) if val == int(val) else round(val, 4)
            color = self.TEXT
            suffix = ''
            if p == 'AHRS_ORIENT' and val not in (0, '---'):
                color = self.KILL_ACC
                suffix = '  !! MUST BE 0'
            txt = f"  {p:.<32} {val}{suffix}"
            self.screen.blit(self.font_mon.render(txt, True, color), (x, y))
            y += 14

        y += 6
        # ── ATTITUDE ──
        self.screen.blit(self.font_mon_h.render("ATTITUDE", True, self.WARN), (x, y))
        y += 16
        for label, key, unit in [
            ("Yaw",      "yaw_deg",  "deg"),
            ("Yaw rate", "yaw_rate", "deg/s"),
            ("Roll",     "roll_deg", "deg"),
            ("Pitch",    "pitch_deg","deg"),
        ]:
            val = tel.get(key, '---')
            color = self.TEXT
            if key == 'yaw_rate' and isinstance(val, (int, float)) and abs(val) > 10:
                color = self.KILL_ACC
            txt = f"  {label:.<20} {val} {unit}"
            self.screen.blit(self.font_mon.render(txt, True, color), (x, y))
            y += 14

        # Spin warning
        yaw_rate = tel.get('yaw_rate', 0)
        if isinstance(yaw_rate, (int, float)) and abs(yaw_rate) > 10:
            self.screen.blit(self.font_mon_h.render(
                "!! SPIN DETECTED !!", True, self.KILL_ACC), (x + 200, y - 28))

        y += 6
        # ── MOTORS ──
        self.screen.blit(self.font_mon_h.render("MOTOR OUTPUTS (PWM)", True, self.WARN), (x, y))
        y += 16
        # Output → motor mapping as wired on this drone (must match SERVOn_FUNCTION)
        pin_labels = [
            "Out1 M1 front-right (CCW)",
            "Out2 M3 front-left (CW)",
            "Out3 M4 rear-right (CW)",
            "Out4 M2 rear-left (CCW)",
        ]
        pwms = []
        for i, ml in enumerate(pin_labels, 1):
            val = tel.get(f'PWM{i}', '---')
            pwms.append(val)
            txt = f"  {ml:.<28} {val}"
            self.screen.blit(self.font_mon.render(txt, True, self.TEXT), (x, y))
            y += 14

        if all(isinstance(p, int) for p in pwms):
            spread = max(pwms) - min(pwms)
            color = self.KILL_ACC if spread > 150 else self.ACCENT
            txt = f"  {'Max spread':.<28} {spread}"
            self.screen.blit(self.font_mon.render(txt, True, color), (x, y))
            y += 14
            if spread > 150:
                self.screen.blit(self.font_mon_h.render(
                    "!! MOTOR IMBALANCE !!", True, self.KILL_ACC), (x + 200, y - 14))

        # ── COMMAND LINE ──
        y += 14
        pygame.draw.line(self.screen, self.ACCENT, (x, y), (x + 340, y), 1)
        y += 6
        self.screen.blit(self.font_mon_h.render("COMMAND", True, self.WARN), (x, y))
        y += 18
        cmd_w = 340
        cmd_rect = pygame.Rect(x, y, cmd_w, 24)
        border_c = self.ACCENT if self.cmd_active else self.DIM
        pygame.draw.rect(self.screen, border_c, cmd_rect, 1, border_radius=4)
        inner_rect = pygame.Rect(x + 1, y + 1, cmd_w - 2, 22)
        pygame.draw.rect(self.screen, self.PANEL, inner_rect, border_radius=3)
        if self.cmd_text:
            prompt = f"> {self.cmd_text}"
        elif self.cmd_active:
            prompt = "> _"
        else:
            prompt = "[TAB] motortest, set, mode"
        prompt_color = self.TEXT if (self.cmd_text or self.cmd_active) else self.DIM
        self.screen.blit(self.font_cmd.render(prompt, True, prompt_color),
                         (x + 6, y + 4))

    def run(self):
        COLS  = 3
        BTN_W = 160
        BTN_H = 62
        PAD_X = 20
        PAD_Y = 70
        GAP   = 12

        buttons = list(KEYBINDINGS.items())

        running = True
        while running:
            self.screen.fill(self.BG)

            # Header
            t1 = self.font_title.render("MAV", True, self.ACCENT)
            t2 = self.font_title.render("CONTROL", True, self.TEXT)
            self.screen.blit(t1, (PAD_X, 18))
            self.screen.blit(t2, (PAD_X + t1.get_width() + 4, 18))

            # Status
            dot_color   = self.ACCENT if self.connected else self.KILL_ACC
            status_text = "CONNECTED" if self.connected else "DISCONNECTED"
            pygame.draw.circle(self.screen, dot_color, (self.WIDTH - PAD_X - 10, 30), 6)
            st = self.font_key.render(status_text, True, dot_color)
            self.screen.blit(st, (self.WIDTH - PAD_X - 20 - st.get_width(), 24))

            # Separator
            pygame.draw.line(self.screen, self.ACCENT,
                             (PAD_X, 52), (self.WIDTH - PAD_X, 52), 1)

            # Buttons
            for i, (keycode, (key_label, label, fn_name)) in enumerate(buttons):
                row, col = divmod(i, COLS)
                x = PAD_X + col * (BTN_W + GAP)
                y = PAD_Y + row * (BTN_H + GAP)
                self._draw_button(pygame.Rect(x, y, BTN_W, BTN_H),
                                  key_label, label, fn_name, keycode)

            # Log area
            log_y = PAD_Y + ((len(buttons) - 1) // COLS + 1) * (BTN_H + GAP) + 8
            pygame.draw.line(self.screen, self.DIM,
                             (PAD_X, log_y), (self.WIDTH - PAD_X, log_y), 1)
            self.screen.blit(self.font_small.render("LOG", True, self.DIM),
                             (PAD_X, log_y + 8))
            self.screen.blit(self.font_log.render(f"> {self.log_msg}", True, self.ACCENT),
                             (PAD_X, log_y + 22))

            # Telemetry panel (when monitor is active) — right side
            if self.monitor_on:
                self._draw_monitor(self.WIDTH + 10, PAD_Y)

            # Footer
            self.screen.blit(
                self.font_small.render("SPACE = EMERGENCY KILL", True, self.KILL_ACC),
                (PAD_X, self.HEIGHT - 20))

            # Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if self.cmd_active:
                        if event.key == pygame.K_TAB:
                            self.cmd_active = False
                            self.cmd_text = ""
                        elif event.key == pygame.K_ESCAPE:
                            self.cmd_active = False
                            self.cmd_text = ""
                        elif event.key == pygame.K_RETURN:
                            if self.cmd_text.strip():
                                self.log_msg = self.ctrl.run_command(self.cmd_text)
                            self.cmd_text = ""
                            self.cmd_active = False
                        elif event.key == pygame.K_BACKSPACE:
                            self.cmd_text = self.cmd_text[:-1]
                        else:
                            if event.unicode and event.unicode.isprintable():
                                self.cmd_text += event.unicode
                    else:
                        if event.key == pygame.K_TAB and self.monitor_on:
                            self.cmd_active = True
                            self.cmd_text = ""
                        elif event.key == pygame.K_ESCAPE:
                            running = False
                        elif event.key in KEYBINDINGS:
                            _, _, fn_name = KEYBINDINGS[event.key]
                            self._execute(fn_name, key=event.key)

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()


def main():
    parser = argparse.ArgumentParser(description="MAVLink Keyboard Controller")
    parser.add_argument("--port", default="/dev/tty.usbserial-0001")
    parser.add_argument("--baudrate", type=int, default=57600)
    args = parser.parse_args()

    controller = MAVController(args.port, args.baudrate)
    ui = MAVControlUI(controller)
    ui.run()


if __name__ == "__main__":
    main()