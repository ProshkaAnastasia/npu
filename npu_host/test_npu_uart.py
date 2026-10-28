"""Проверки клиента без COM-порта. Это эмулятор протокола, не FPGA."""
import json
from pathlib import Path
import struct
import unittest

from npu_uart import (Command, DeviceError, ExchangeTimeout, Model, NpuClient,
                      ProtocolError, crc32, encode_input, load_input, pack_packet)

ROOT = Path(__file__).resolve().parent


class FakeSerial:
    def __init__(self, *, drop_run=False, corrupt=False, reject_crc=False,
                 silent=False, status=0):
        self.rx = bytearray()
        self.sent = []
        self.cache = None
        self.run_count = 0
        self.drop_run, self.corrupt = drop_run, corrupt
        self.reject_crc, self.silent, self.status = reject_crc, silent, status
        self.model = bytearray()
        self.vector = bytearray()
        golden = json.loads((ROOT / 'examples/expected.json').read_text())
        self.memory = {0x3000 + 4*i: int(w, 16) for i, w in enumerate(golden['output_hex'])}
        self.memory.update({0x3200: 1, 0x3204: 3, 0x3208: 0x3000, 0x320c: 0})

    @property
    def in_waiting(self):
        return min(len(self.rx), 3)  # Ответ приходит фрагментами.

    def read(self, n):
        value = bytes(self.rx[:min(n, 3)])
        del self.rx[:len(value)]
        return value

    def flush(self):
        pass

    def write(self, frame):
        self.sent.append(frame)
        if self.silent:
            return len(frame)
        assert frame[:2] == b'\xa5\x5a'
        assert crc32(frame[2:-4]) == struct.unpack('<I', frame[-4:])[0]
        cmd, seq, size = struct.unpack_from('<BHH', frame, 2)
        payload = frame[7:-4]
        assert size == len(payload)
        if self.reject_crc:
            self.reject_crc = False
            self.rx.extend(pack_packet(0x80, 0xffff, struct.pack('<BBI', 0, 1, 0)))
            return len(frame)
        if self.cache is not None and self.cache[0] == frame:
            response = self.cache[1]
        else:
            value = 0
            if cmd == Command.CONFIG_BEGIN:
                self.model.clear()
            elif cmd == Command.INPUT_BEGIN:
                self.vector.clear()
            elif cmd in (Command.CONFIG_DATA, Command.INPUT_DATA):
                dest = self.model if cmd == Command.CONFIG_DATA else self.vector
                offset = struct.unpack_from('<I', payload)[0]
                assert offset == len(dest)
                dest.extend(payload[4:])
                value = len(dest)
            elif cmd == Command.CONFIG_END:
                Model.from_bytes(bytes(self.model))
                value = len(self.model)
            elif cmd == Command.INPUT_END:
                value = len(self.vector)
            elif cmd == Command.RUN:
                self.run_count += 1
                value = 0x3000
            elif cmd == Command.READ_WORD:
                value = self.memory[struct.unpack('<I', payload)[0]]
            response = pack_packet(0x80, seq, struct.pack('<BBI', cmd, self.status, value))
            self.cache = frame, response
            if cmd == Command.RUN and self.drop_run:
                self.drop_run = False
                return len(frame)
        if self.corrupt:
            self.corrupt = False
            bad = bytearray(response)
            bad[-1] ^= 1
            self.rx.extend(b'noise\xa5' + bad)
        else:
            self.rx.extend(response)
        return len(frame)


class ClientTests(unittest.TestCase):
    def client(self, fake):
        return NpuClient(fake, timeout=0.005, run_timeout=0.005, initial_seq=42)

    def test_crc_known_vector(self):
        self.assertEqual(crc32(b'123456789'), 0xcbf43926)

    def test_model_and_input_formats(self):
        model = Model.load(ROOT / 'examples/model.bin')
        self.assertEqual(model, Model.load(ROOT / 'examples/test_model.mem'))
        self.assertEqual((model.input_count, model.output_count, model.layers), (49, 3, 2))
        self.assertEqual(load_input(ROOT / 'examples/input.json'),
                         load_input(ROOT / 'examples/test_input.mem'))
        broken = bytearray(model.image)
        broken[100] ^= 1
        with self.assertRaisesRegex(ValueError, 'CRC'):
            Model.from_bytes(bytes(broken))

    def test_full_flow_and_lost_run_ack(self):
        fake = FakeSerial(drop_run=True)
        client = self.client(fake)
        model = Model.load(ROOT / 'examples/model.bin')
        client.upload_model(model)
        data = load_input(ROOT / 'examples/input.json')
        client.upload_input(data)
        result = client.run_and_read()
        expected = json.loads((ROOT / 'examples/expected.json').read_text())
        self.assertEqual(result, expected)
        self.assertEqual(fake.run_count, 1)
        runs = [p for p in fake.sent if p[2] == Command.RUN]
        self.assertEqual(len(runs), 2)
        self.assertEqual(runs[0], runs[1])
        self.assertEqual(bytes(fake.model), model.image)
        self.assertEqual(bytes(fake.vector), data)
        # Следующий вектор без повторной загрузки весов.
        result2 = client.infer(struct.unpack('<49f', data))
        self.assertEqual(result2, expected)
        self.assertEqual(fake.run_count, 2)

    def test_corrupt_response_then_replay(self):
        fake = FakeSerial(corrupt=True)
        client = self.client(fake)
        self.assertEqual(client.request(Command.CONFIG_BEGIN, struct.pack('<I', 2652)), 0)
        self.assertEqual(fake.sent[0], fake.sent[1])

    def test_anonymous_transport_error_retry(self):
        fake = FakeSerial(reject_crc=True)
        client = self.client(fake)
        client.request(Command.CONFIG_BEGIN, struct.pack('<I', 2652))
        self.assertEqual(fake.sent[0], fake.sent[1])

    def test_stale_reply_and_sequence_wrap(self):
        fake = FakeSerial()
        fake.rx.extend(pack_packet(0x80, 41, struct.pack('<BBI', 7, 0, 0x3000)))
        client = self.client(fake)
        client.seq = 0xffff
        client.request(Command.CONFIG_BEGIN, struct.pack('<I', 2652))
        client.request(Command.CONFIG_BEGIN, struct.pack('<I', 2652))
        self.assertEqual([struct.unpack_from('<H', p, 3)[0] for p in fake.sent], [65535, 0])

    def test_device_error_not_retried(self):
        fake = FakeSerial(status=0x10)
        with self.assertRaises(DeviceError) as cm:
            self.client(fake).request(Command.RUN)
        self.assertEqual(cm.exception.status, 0x10)
        self.assertEqual(len(fake.sent), 1)

    def test_timeout_aborts_session(self):
        fake = FakeSerial(silent=True)
        client = self.client(fake)
        with self.assertRaises(ExchangeTimeout):
            client.request(Command.RUN)
        self.assertEqual(len(fake.sent), 3)
        with self.assertRaises(ProtocolError):
            client.request(Command.RUN)
        self.assertEqual(len(fake.sent), 3)

    def test_partial_write_aborts_session(self):
        fake = FakeSerial()
        fake.write = lambda data: len(data) - 1
        client = self.client(fake)
        with self.assertRaises(ProtocolError):
            client.request(Command.RUN)
        self.assertFalse(client.usable)

    def test_input_validation(self):
        for data in ([float('nan')], [float('inf')], [1e100], [True], []):
            with self.assertRaises(ValueError):
                encode_input(data)
        client = self.client(FakeSerial())
        with self.assertRaises(ValueError):
            client.upload_input(encode_input([1]))
        client.model = Model.load(ROOT / 'examples/model.bin')
        with self.assertRaises(ValueError):
            client.upload_input(encode_input([1]))


if __name__ == '__main__':
    unittest.main()
