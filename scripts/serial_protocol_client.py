"""AA55 USB CDC integration client. Live I/O requires: python -m pip install pyserial."""
import argparse
import json
import math
import struct
import time


def pack_frame(cmd, data=b''):
    body = bytes((cmd, len(data))) + data
    return b'\xaa\x55' + body + bytes((sum(body) & 255,))


class Decoder:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 2:
            if self.buffer[:2] != b'\xaa\x55':
                del self.buffer[0]
                continue
            if len(self.buffer) < 4:
                break
            cmd, length = self.buffer[2:4]
            if {1: 0, 2: 14, 3: 5}.get(cmd) != length:
                del self.buffer[0]
                continue
            total = length + 5
            if len(self.buffer) < total:
                break
            if sum(self.buffer[2:total - 1]) & 255 != self.buffer[total - 1]:
                del self.buffer[0]
                continue
            payload = bytes(self.buffer[4:total - 1])
            del self.buffer[:total]
            if cmd == 1:
                message = {'cmd': 1, 'ack': True}
            elif cmd == 2:
                hp, ammo, heat, zone, stop = struct.unpack('<iifBB', payload)
                message = dict(cmd=2, health=hp, fire_number=ammo, heat=heat,
                               ZoneIsOurs=bool(zone), Stop=bool(stop))
            else:
                target, fire = struct.unpack('<fB', payload)
                message = dict(cmd=3, target=target, fire_status=bool(fire))
            result.append(message)
        return result


def self_test():
    assert pack_frame(1, struct.pack('<ffB', 12.5, -3.75, 1)).hex(' ') == (
        'aa 55 01 09 00 00 48 41 00 00 70 c0 01 c4')
    assert pack_frame(2).hex() == 'aa55020002'
    assert pack_frame(3).hex() == 'aa55030003'
    assert pack_frame(4, b'\x01').hex() == 'aa5504010106'
    assert pack_frame(4, b'\x00').hex() == 'aa5504010005'
    status = pack_frame(2, struct.pack('<iifBB', 257, 513, 37.5, 0, 1))
    attitude = pack_frame(3, struct.pack('<fB', -20, 0))
    for split in range(len(status) + 1):
        decoder = Decoder()
        messages = decoder.feed(status[:split]) + decoder.feed(status[split:])
        assert len(messages) == 1 and messages[0]['health'] == 257
        assert messages[0]['fire_number'] == 513 and messages[0]['heat'] == 37.5
        assert messages[0]['Stop'] and not messages[0]['ZoneIsOurs']
    decoder = Decoder()
    bad = bytearray(status)
    bad[-1] ^= 1
    messages = decoder.feed(b'noise\xaa' + bad + status + attitude + pack_frame(1))
    assert [m['cmd'] for m in messages] == [2, 3, 1]
    assert messages[1]['target'] == -20 and not messages[1]['fire_status']
    print('Host codec: golden frames, all splits, bad checksum and concatenation passed')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--port', help='Upper-board USB CDC COM port, e.g. COM5')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--interval', type=float, default=0.05, help='Seconds between commands')
    sub = parser.add_subparsers(dest='command')
    sub.add_parser('status')
    sub.add_parser('attitude')
    aim = sub.add_parser('aim')
    aim.add_argument('--yaw', type=float, default=0)
    aim.add_argument('--pitch', type=float, default=0)
    aim.add_argument('--fire', action='store_true', help='Request automatic fire (default false)')
    sub.add_parser('stop')
    sub.add_parser('release')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return
    if not args.port or not args.command:
        parser.error('--port and a command are required for live I/O')
    if args.count < 1 or args.interval < 0:
        parser.error('count must be positive and interval nonnegative')
    if args.command == 'aim':
        if any(not math.isfinite(v) or abs(v) > 180 for v in (args.yaw, args.pitch)):
            parser.error('yaw/pitch must be finite degrees in [-180,180]')
        cmd, data = 1, struct.pack('<ffB', args.yaw, args.pitch, args.fire)
    else:
        cmd, data = {'status': (2, b''), 'attitude': (3, b''),
                     'stop': (4, b'\x01'), 'release': (4, b'\x00')}[args.command]
    try:
        import serial
    except ImportError:
        parser.error('Install live serial support with: python -m pip install pyserial')
    decoder = Decoder()
    with serial.Serial(args.port, args.baud, timeout=0.02, write_timeout=1) as port:
        # Wait for enumeration/reset; responses always use the same CMD as the request.
        time.sleep(0.3)
        port.reset_input_buffer()
        for _ in range(args.count):
            frame = pack_frame(cmd, data)
            print('TX', frame.hex(' '), flush=True)
            port.write(frame)
            if cmd == 4:
                print('CMD04 has no acknowledgement; query attitude/status separately.')
                time.sleep(args.interval)
                continue
            deadline = time.monotonic() + 1
            while time.monotonic() < deadline:
                messages = decoder.feed(port.read(port.in_waiting or 1))
                for message in messages:
                    print('RX', json.dumps(message, ensure_ascii=False), flush=True)
                if any(message['cmd'] == cmd for message in messages):
                    break
            else:
                raise TimeoutError(f'No valid CMD{cmd:02X} response within 1 second')
            time.sleep(args.interval)


if __name__ == '__main__':
    main()
