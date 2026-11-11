import math
import sys
import time

from pymavlink import mavutil

mav = mavutil.mavlink
SPEED = 0.5
CLIMB = 0.3
YAW_RATE = 15
RATE_HZ = 10
STOP_DISTANCE = 3.0
HALF_WIDTH = 1.0
MIN_ALT = 2.0
STALE_S = 2.0
VELOCITY_MASK = 0b010111000111

KEYS = {
    "w": (1, 0, 0, 0), "s": (-1, 0, 0, 0),
    "d": (0, 1, 0, 0), "a": (0, -1, 0, 0),
    "e": (0, 0, 1, 0), "q": (0, 0, -1, 0),
    "r": (0, 0, 0, 1), "f": (0, 0, 0, -1),
}

HELP = [
    "W/S forward/back   A/D left/right",
    "Q/E yaw left/right R/F up/down",
    "Space stop         Esc quit",
]


def free_distance(sectors, bearing):
    free = math.inf
    for k, d in sectors.items():
        diff = math.radians((k * 45 - bearing + 180) % 360 - 180)
        if abs(diff) < math.pi / 2 and abs(d * math.sin(diff)) < HALF_WIDTH:
            free = min(free, d * math.cos(diff))
    return free


class Drone:
    def __init__(self, conn):
        self.m = mavutil.mavlink_connection(conn, source_system=255, source_component=190)
        self.mode = "UNKNOWN"
        self.armed = False
        self.rel_alt = None
        self.sectors = {}
        self.last_heartbeat = 0.0
        self.last_proximity = 0.0
        while True:
            hb = self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=30)
            if hb is None:
                sys.exit("no heartbeat from the drone")
            if hb.autopilot == mav.MAV_AUTOPILOT_ARDUPILOTMEGA and hb.get_srcComponent() == mav.MAV_COMP_ID_AUTOPILOT1:
                self.m.target_system = hb.get_srcSystem()
                self.m.target_component = hb.get_srcComponent()
                self._handle(hb)
                break
        for msg_id in (mav.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, mav.MAVLINK_MSG_ID_DISTANCE_SENSOR):
            self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                         mav.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, 100000, 0, 0, 0, 0, 0)

    def _handle(self, msg):
        t = msg.get_type()
        now = time.time()
        if t == "HEARTBEAT":
            self.mode = mavutil.mode_string_v10(msg)
            self.armed = bool(msg.base_mode & mav.MAV_MODE_FLAG_SAFETY_ARMED)
            self.last_heartbeat = now
        elif t == "GLOBAL_POSITION_INT":
            self.rel_alt = msg.relative_alt / 1000
        elif t == "DISTANCE_SENSOR" and msg.orientation <= 7:
            valid = 0 < msg.current_distance < msg.max_distance
            self.sectors[msg.orientation] = (msg.current_distance if valid else msg.max_distance) / 100
            self.last_proximity = now

    def poll(self):
        while True:
            msg = self.m.recv_match(blocking=False)
            if msg is None:
                return
            if msg.get_srcSystem() == self.m.target_system and msg.get_srcComponent() == self.m.target_component:
                self._handle(msg)

    @property
    def flying_guided(self):
        return self.mode == "GUIDED" and self.armed and time.time() - self.last_heartbeat < STALE_S

    def send(self, fwd, right, up, yaw_rate):
        self.m.mav.set_position_target_local_ned_send(
            0, self.m.target_system, self.m.target_component,
            mav.MAV_FRAME_BODY_OFFSET_NED, VELOCITY_MASK,
            0, 0, 0, fwd, right, -up, 0, 0, 0, 0, math.radians(yaw_rate))


def command(pressed, drone):
    if "space" in pressed:
        return (0.0, 0.0, 0.0, 0.0), "stopped"
    f = r = y = u = 0
    for key in pressed:
        if key in KEYS:
            df, dr, dy, du = KEYS[key]
            f, r, y, u = f + df, r + dr, y + dy, u + du
    fwd, right = f * SPEED, r * SPEED
    note = ""
    if fwd or right:
        norm = math.hypot(fwd, right) / SPEED
        fwd, right = fwd / norm, right / norm
        if time.time() - drone.last_proximity > STALE_S or not drone.sectors:
            fwd = right = 0.0
            note = "no lidar data yet: horizontal moves off"
        else:
            free = free_distance(drone.sectors, math.degrees(math.atan2(right, fwd)))
            if free < STOP_DISTANCE:
                fwd = right = 0.0
                note = f"blocked: obstacle {free:.1f} m that way"
    up = u * CLIMB
    if up < 0 and (drone.rel_alt is None or drone.rel_alt < MIN_ALT + 0.3):
        up = 0.0
        note = note or f"floor: not going below {MIN_ALT:.0f} m"
    return (fwd, right, up, y * YAW_RATE), note


class PygameKeys:
    NAMES = {"w", "a", "s", "d", "q", "e", "r", "f", "space"}

    def __init__(self):
        import pygame
        self.pg = pygame
        pygame.init()
        self.screen = pygame.display.set_mode((460, 230))
        pygame.display.set_caption("WASD drone control")
        self.font = pygame.font.SysFont(None, 26)

    def read(self):
        pg = self.pg
        quit_now = False
        for event in pg.event.get():
            if event.type == pg.QUIT or (event.type == pg.KEYDOWN and event.key == pg.K_ESCAPE):
                quit_now = True
        state = pg.key.get_pressed()
        pressed = {n for n in self.NAMES if state[pg.key.key_code(n)]}
        return pressed, quit_now

    def show(self, lines):
        self.screen.fill((20, 20, 24))
        for i, line in enumerate(lines):
            self.screen.blit(self.font.render(line, True, (230, 230, 230)), (14, 12 + i * 26))
        self.pg.display.flip()

    def close(self):
        self.pg.quit()


def run(conn, keys):
    drone = Drone(conn)
    period = 1 / RATE_HZ
    sending = False
    try:
        while True:
            start = time.time()
            drone.poll()
            pressed, quit_now = keys.read()
            if quit_now:
                break
            cmd, note = command(pressed, drone)
            if drone.flying_guided:
                drone.send(*cmd)
                sending = True
                status = "SENDING"
            else:
                sending = False
                status = f"not sending: mode {drone.mode}, armed {drone.armed}"
            alt = "?" if drone.rel_alt is None else f"{drone.rel_alt:.1f} m"
            keys.show([status, f"alt {alt}   fwd {cmd[0]:+.1f} right {cmd[1]:+.1f} up {cmd[2]:+.1f} yaw {cmd[3]:+.0f}",
                       note, ""] + HELP)
            time.sleep(max(0.0, period - (time.time() - start)))
    finally:
        if sending and drone.flying_guided:
            drone.send(0, 0, 0, 0)
        keys.close()
        drone.m.close()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit("usage: wasd.py CONN   (e.g. udpin:0.0.0.0:14551)")
    run(sys.argv[1], PygameKeys())
