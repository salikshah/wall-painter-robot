"""Feetech STS3215 half-duplex serial bus driver + dry (no-hardware) backend.

Protocol: Feetech SCS/STS packet protocol at 1 Mbaud (auto-direction USB
adapter, e.g. Waveshare bus-servo adapter). Register addresses follow the
STS3215 memory table; verify against your servo's manual at bring-up.

Two backends with the same API:
  SerialBus  real hardware over pyserial
  DryBus     in-memory servo simulation for dry runs (lag + rate limit +
             quantization); joint loads are injected by the caller.

Run a bus scan on real hardware:  python feetech.py COM3
"""
from pathlib import Path
import struct
import sys
import time

import numpy as np

TICKS_PER_RAD = 4096.0 / (2.0 * np.pi)

REG = {
    'TORQUE_ENABLE': 40,
    'ACC': 41,
    'GOAL_POSITION': 42,
    'MOVING_SPEED': 44,
    'PRESENT_POSITION': 56,
    'PRESENT_SPEED': 58,
    'PRESENT_LOAD': 60,
    'PRESENT_VOLTAGE': 62,
    'PRESENT_TEMP': 63,
}

INSTR_PING = 0x01
INSTR_READ = 0x02
INSTR_WRITE = 0x03
INSTR_SYNC_WRITE = 0x83
INSTR_SYNC_READ = 0x82
BROADCAST = 0xFE


class BusError(RuntimeError):
    pass


def checksum(parts):
    return (~(sum(parts) & 0xFF)) & 0xFF


def packet(servo_id, instr, params):
    ln = len(params) + 2
    body = [servo_id, ln, instr] + list(params)
    return bytes([0xFF, 0xFF] + body + [checksum(body)])


def decode_load(raw):
    mag = raw & 0x03FF
    return -mag if (raw & 0x0400) else mag


def encode_load(value):
    mag = int(min(1000, abs(round(value))))
    return mag | (0x0400 if value < 0 else 0)


class ServoSpec:
    def __init__(self, name, servo_id, sign=1, offset_rad=0.0,
                 load_sign=1, max_torque_nm=2.94):
        self.name = name
        self.servo_id = servo_id
        self.sign = sign
        self.offset_rad = offset_rad
        self.load_sign = load_sign
        self.max_torque_nm = max_torque_nm

    def to_ticks(self, q_rad):
        return int(round((self.offset_rad + self.sign * q_rad) * TICKS_PER_RAD)) % 4096

    def to_rad(self, ticks):
        q = self.sign * (ticks / TICKS_PER_RAD - self.offset_rad)
        return float(q)

    def to_torque(self, raw_load):
        return self.load_sign * decode_load(raw_load) / 1000.0 * self.max_torque_nm

    def from_torque(self, tau_nm):
        units = tau_nm / max(1e-9, self.max_torque_nm) * 1000.0
        return encode_load(self.load_sign * units)


class SerialBus:
    def __init__(self, port, baud=1_000_000, timeout=0.05, retries=3):
        import serial
        self.ser = serial.Serial(port, baudrate=baud, timeout=timeout,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                                 stopbits=serial.STOPBITS_ONE)
        self.retries = retries
        time.sleep(0.05)

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def tick(self, dt):
        pass

    def _exchange(self, pkt, servo_id, n_params):
        last = None
        for _ in range(self.retries):
            try:
                self.ser.reset_input_buffer()
                self.ser.write(pkt)
                self.ser.flush()
                return self._read_response(servo_id, n_params)
            except BusError as e:
                last = e
                time.sleep(0.01)
        raise BusError(f'servo {servo_id}: {last}')

    def _read_response(self, servo_id, n_params):
        hdr = self.ser.read(2)
        if hdr != b'\xff\xff':
            raise BusError('no header')
        head = self.ser.read(3)
        if len(head) != 3:
            raise BusError('short header')
        sid, ln, err = head
        if sid != servo_id:
            raise BusError(f'id mismatch ({sid} != {servo_id})')
        rest = self.ser.read(ln)
        if len(rest) != ln:
            raise BusError('short body')
        params, chk = rest[:-1], rest[-1]
        if checksum([sid, ln, err] + list(params)) != chk:
            raise BusError('bad checksum')
        if err != 0:
            raise BusError(f'servo error flag 0x{err:02x}')
        return list(params)

    def ping(self, servo_id):
        return bool(self._exchange(packet(servo_id, INSTR_PING, []), servo_id, 0) is not None)

    def read(self, servo_id, addr, n):
        return self._exchange(packet(servo_id, INSTR_READ, [addr, n]), servo_id, n)

    def write(self, servo_id, addr, data):
        return self._exchange(packet(servo_id, INSTR_WRITE, [addr] + list(data)), servo_id, 0)

    def write_noreply(self, servo_id, addr, data):
        self.ser.reset_input_buffer()
        self.ser.write(packet(servo_id, INSTR_WRITE, [addr] + list(data)))
        self.ser.flush()

    def enable_torque(self, specs, on=True):
        for s in specs:
            self.write(s.servo_id, REG['TORQUE_ENABLE'], [1 if on else 0])

    def torque_off_all(self, specs):
        for s in specs:
            try:
                self.write_noreply(s.servo_id, REG['TORQUE_ENABLE'], [0])
            except Exception:
                pass

    def sync_write_goal(self, moves):
        data_len = 6
        params = [REG['GOAL_POSITION'], data_len]
        for servo_id, ticks, speed, acc in moves:
            ticks = int(ticks) % 4096
            params += [servo_id, ticks & 0xFF, (ticks >> 8) & 0xFF,
                       int(speed) & 0xFF, (int(speed) >> 8) & 0xFF, int(acc) & 0xFF]
        self.ser.reset_input_buffer()
        self.ser.write(packet(BROADCAST, INSTR_SYNC_WRITE, params))
        self.ser.flush()

    def read_states(self, specs):
        out = {}
        for s in specs:
            vals = self.read(s.servo_id, REG['PRESENT_POSITION'], 8)
            pos = vals[0] | (vals[1] << 8)
            speed = vals[2] | (vals[3] << 8)
            load = vals[4] | (vals[5] << 8)
            volt = vals[6]
            temp = vals[7]
            out[s.servo_id] = {'pos': pos, 'speed': speed, 'load': load,
                               'voltage': volt / 10.0, 'temp': temp}
        return out


class DryBus:
    def __init__(self, specs, rate_ticks_s=3000.0, tau_lag=0.03, seed=0):
        self.specs = {s.servo_id: s for s in specs}
        self.rate = rate_ticks_s
        self.tau = tau_lag
        self.pos = {}
        self.goal = {}
        self.speed_cmd = {}
        for s in specs:
            self.pos[s.servo_id] = 2048.0
            self.goal[s.servo_id] = 2048.0
            self.speed_cmd[s.servo_id] = 0
        self.pretend_load = {s.servo_id: 0 for s in specs}
        self.rng = np.random.default_rng(seed)

    def close(self):
        pass

    def ping(self, servo_id):
        return servo_id in self.specs

    def enable_torque(self, specs, on=True):
        pass

    def torque_off_all(self, specs):
        pass

    def sync_write_goal(self, moves):
        for servo_id, ticks, speed, acc in moves:
            self.goal[servo_id] = float(ticks) % 4096
            self.speed_cmd[servo_id] = int(speed)

    def tick(self, dt):
        for sid, target in self.goal.items():
            pos = self.pos[sid]
            err = (target - pos + 2048) % 4096 - 2048
            step = err * (1.0 - np.exp(-dt / self.tau))
            lim = self.rate * dt
            step = float(np.clip(step, -lim, lim))
            self.pos[sid] = (pos + step) % 4096

    def read_states(self, specs):
        out = {}
        for s in specs:
            sid = s.servo_id
            noisy = self.pos[sid] + self.rng.normal(0.0, 0.5)
            out[sid] = {'pos': int(round(noisy)),
                        'speed': 0,
                        'load': int(self.pretend_load.get(sid, 0)),
                        'voltage': 12.1,
                        'temp': 36}
        return out


def set_moving_speed(bus, specs, ticks_per_s):
    for s in specs:
        bus.write_noreply(s.servo_id, REG['MOVING_SPEED'], [int(ticks_per_s) & 0xFF,
                                                            (int(ticks_per_s) >> 8) & 0xFF])


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        print('usage: python feetech.py <PORT>')
        return
    specs = [ServoSpec(f'id{sid}', sid) for sid in range(1, 9)]
    bus = SerialBus(sys.argv[1])
    print(f'scanning {sys.argv[1]} ...')
    found = []
    for s in specs:
        try:
            bus.read(s.servo_id, REG['PRESENT_POSITION'], 8)
            st = bus.read_states([s])[s.servo_id]
            q0 = s.to_rad(st['pos'])
            print(f'  id {s.servo_id}: pos {st["pos"]:4d} ({q0:+.3f} rad)  '
                  f'load {decode_load(st["load"]):+4d}  {st["voltage"]:.1f} V  {st["temp"]} C')
            found.append(s.servo_id)
        except BusError:
            pass
    print('found ids:', found)


if __name__ == '__main__':
    main()
