"""Клиент UART для текущего nn_processor_top. Python 3.10+, pyserial."""
from __future__ import annotations

import argparse
import json
import logging
import math
from pathlib import Path
import secrets
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from enum import IntEnum


class Command(IntEnum):
    CONFIG_BEGIN = 1
    CONFIG_DATA = 2
    CONFIG_END = 3
    INPUT_BEGIN = 4
    INPUT_DATA = 5
    INPUT_END = 6
    RUN = 7
    READ_WORD = 8


STATUS = {
    0x00: "OK", 0x01: "CRC_ERROR", 0x02: "FORMAT_ERROR",
    0x03: "TIMEOUT", 0x04: "SEQ_ERROR", 0x10: "BAD_STATE",
    0x11: "BAD_SIZE", 0x12: "BAD_OFFSET", 0x13: "BAD_STREAM",
    0x14: "MODEL_CRC", 0x15: "BAD_MODEL", 0x16: "MEMORY_ERROR",
    0x17: "PE_ERROR", 0x18: "COMPUTE_ERROR",
}
LOG = logging.getLogger("npu")


class ProtocolError(RuntimeError):
    pass


class DeviceError(ProtocolError):
    def __init__(self, command: int, status: int, value: int):
        self.command, self.status, self.value = command, status, value
        super().__init__(
            f"CMD=0x{command:02X}: {STATUS.get(status, 'UNKNOWN')} "
            f"(status=0x{status:02X}, value=0x{value:08X})"
        )


class ExchangeTimeout(ProtocolError):
    pass


def crc32(data: bytes) -> int:
    # zlib уже применяет начальное/финальное преобразование ISO-HDLC.
    return zlib.crc32(data) & 0xFFFFFFFF


def pack_packet(command: int, seq: int, payload: bytes = b"") -> bytes:
    body = struct.pack("<BHH", command, seq, len(payload)) + payload
    return b"\xA5\x5A" + body + struct.pack("<I", crc32(body))


def load_words(path: str | Path) -> bytes:
    """.mem: последовательность hex-слов FP32/uint32; остальное — binary."""
    path = Path(path)
    if path.suffix.lower() != ".mem":
        return path.read_bytes()
    words = []
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        for token in line.split("//", 1)[0].split():
            if len(token) > 8:
                raise ValueError(f"Некорректное 32-битное слово: {token}")
            words.append(struct.pack("<I", int(token, 16)))
    return b"".join(words)


@dataclass(frozen=True)
class Model:
    image: bytes
    input_count: int
    output_count: int
    output_addr: int
    layers: int

    @classmethod
    def from_bytes(cls, image: bytes) -> Model:
        """Проверка образа перед отправкой; RTL повторяет свою проверку."""
        if len(image) < 32 or len(image) % 4 or len(image) > 0x3000:
            raise ValueError("Образ должен быть выровнен по 4 байтам, размер 32..0x3000")
        h = struct.unpack_from("<8I", image)
        layers = h[1] >> 16
        ni, no = h[2] & 0xFFFF, h[2] >> 16
        if (h[0] != 0x32504E4E or h[1] & 0xFFFF != 1 or
                not 1 <= layers <= 8 or not 1 <= ni <= 49 or
                not 1 <= no <= 49 or h[3] != 0x101 or h[7] != 0):
            raise ValueError("Неподдерживаемый заголовок NNP2")
        if h[5] != len(image):
            raise ValueError("Размер в заголовке не совпадает с размером файла")
        if h[4] < 32 or h[4] % 4 or h[4] + 32 * layers > len(image):
            raise ValueError("Некорректный диапазон дескрипторов")
        if crc32(image[:24] + b"\0" * 4 + image[28:]) != h[6]:
            raise ValueError("CRC всего образа не совпадает")
        previous_n, previous_addr = ni, None
        for index in range(layers):
            d = struct.unpack_from("<8I", image, h[4] + 32 * index)
            n, m = d[1] & 0xFFFF, d[1] >> 16
            if (d[0] != 0x10101 or d[7] or n != previous_n or
                    not 1 <= n <= 49 or not 1 <= m <= 49 or d[6] != 4 * m):
                raise ValueError(f"Некорректный дескриптор слоя {index}")
            if previous_addr is not None and d[4] != previous_addr:
                raise ValueError("Нарушена цепочка буферов слоёв")
            spans = [(d[2], d[2] + 4*n*m), (d[3], d[3] + 4*m)]
            for start, end in spans:
                if start % 4 or start < h[4] + 32*layers or end > len(image):
                    raise ValueError("Веса/bias за пределами образа")
                if any(not math.isfinite(x[0]) for x in struct.iter_unpack("<f", image[start:end])):
                    raise ValueError("NaN/Infinity в весах или bias")
            if max(spans[0][0], spans[1][0]) < min(spans[0][1], spans[1][1]):
                raise ValueError("Веса и bias слоя пересекаются")
            buffers = [(d[4], d[4] + 4*n), (d[5], d[5] + 4*m)]
            for start, end in buffers:
                if start % 4 or start < 0x3000 or end > 0x4000 or (start < 0x3210 and end > 0x3200):
                    raise ValueError("Некорректный буфер активаций")
            if max(buffers[0][0], buffers[1][0]) < min(buffers[0][1], buffers[1][1]):
                raise ValueError("Вход и выход слоя пересекаются")
            previous_n, previous_addr = m, d[5]
        if previous_n != no:
            raise ValueError("Число выходов не совпадает с последним слоем")
        return cls(image, ni, no, previous_addr, layers)

    @classmethod
    def load(cls, path: str | Path) -> Model:
        return cls.from_bytes(load_words(path))


def encode_input(values) -> bytes:
    values = list(values)
    if not values or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in values):
        raise ValueError("Входной вектор должен содержать числа")
    if any(not math.isfinite(v) for v in values):
        raise ValueError("NaN/Infinity во входе запрещены")
    try:
        return struct.pack(f"<{len(values)}f", *values)
    except (OverflowError, struct.error) as e:
        raise ValueError("Значение входа не представимо в FP32") from e


def load_input(path: str | Path) -> bytes:
    path = Path(path)
    if path.suffix.lower() == ".json":
        values = json.loads(path.read_text(encoding="utf-8-sig"))
        if not isinstance(values, list):
            raise ValueError("JSON входа должен быть массивом чисел")
        return encode_input(values)
    if path.suffix.lower() in (".bin", ".mem"):
        return load_words(path)
    # Обычный текст: числа через пробелы, запятые или переводы строк.
    return encode_input(float(x) for x in path.read_text(encoding="utf-8-sig").replace(",", " ").split())


class NpuClient:
    """Один клиент и один outstanding запрос. Не использовать из разных потоков."""

    def __init__(self, port, *, timeout=5.0, run_timeout=30.0, retries=2,
                 chunk_size=256, initial_seq=None):
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(run_timeout) or run_timeout <= 0:
            raise ValueError("Тайм-ауты должны быть положительными")
        if not 4 <= chunk_size <= 256 or chunk_size % 4:
            raise ValueError("chunk_size: кратное 4 от 4 до 256")
        if not isinstance(retries, int) or retries < 0:
            raise ValueError("retries должен быть целым >= 0")
        self.port = port
        self.timeout, self.run_timeout = timeout, run_timeout
        self.retries, self.chunk_size = retries, chunk_size
        self.seq = secrets.randbelow(65536) if initial_seq is None else initial_seq & 0xFFFF
        self.buffer = bytearray()
        self.usable = True
        self.model = None

    def _response(self, deadline):
        # Принимаем только ответ CMD=80, LEN=6; CRC защищает и заголовок.
        # При повреждении сдвигаемся на байт, чтобы восстановить синхронизацию.
        while time.monotonic() < deadline:
            at = self.buffer.find(b"\xA5\x5A")
            if at < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b"\xA5" else b""
            else:
                del self.buffer[:at]
                if len(self.buffer) >= 7:
                    cmd, seq, length = struct.unpack_from("<BHH", self.buffer, 2)
                    if cmd != 0x80 or length != 6:
                        del self.buffer[0]
                        continue
                    if len(self.buffer) >= 17:
                        frame = bytes(self.buffer[:17])
                        if crc32(frame[2:13]) != struct.unpack_from("<I", frame, 13)[0]:
                            LOG.warning("Повреждён CRC ответа; ищем следующий кадр")
                            del self.buffer[0]
                            continue
                        del self.buffer[:17]
                        request_cmd, status, value = struct.unpack_from("<BBI", frame, 7)
                        return seq, request_cmd, status, value
            # У pySerial read(timeout) может вернуть меньше запрошенного.
            # Порт должен быть открыт с коротким read timeout (CLI: 0.05 s).
            data = self.port.read(min(max(getattr(self.port, "in_waiting", 0), 1), 1024))
            if data:
                self.buffer.extend(data)
        raise ExchangeTimeout("Истекло время ожидания ответа")

    def request(self, command: Command, payload=b"", *, timeout=None) -> int:
        if not self.usable:
            raise ProtocolError("Состояние соединения неопределённо после ошибки обмена; восстановите устройство/сеанс")
        seq = self.seq
        self.seq = (self.seq + 1) & 0xFFFF
        frame = pack_packet(command, seq, payload)
        wait_time = self.timeout if timeout is None else timeout
        try:
            for attempt in range(self.retries + 1):
                if attempt:
                    LOG.warning("Повтор CMD=%02X SEQ=%d (%d/%d)", command, seq, attempt, self.retries)
                # Весь кадр одним write: без искусственных межбайтовых пауз.
                if self.port.write(frame) != len(frame):
                    raise ProtocolError("Кадр отправлен не полностью; состояние UART неопределённо")
                self.port.flush()
                deadline = time.monotonic() + wait_time
                try:
                    while True:
                        rseq, rcmd, status, value = self._response(deadline)
                        if rseq == 0xFFFF and rcmd == 0 and status in (1, 2, 3):
                            # Ошибка приёма пакета: он не был передан секвенсору.
                            LOG.warning("IO отклонил пакет: %s", STATUS[status])
                            break
                        if rseq != seq or rcmd != command:
                            LOG.debug("Пропущен ответ на другой запрос SEQ=%d CMD=%02X", rseq, rcmd)
                            continue
                        if status:
                            raise DeviceError(command, status, value)
                        return value
                except ExchangeTimeout:
                    pass
            raise ExchangeTimeout(
                f"Нет подтверждения CMD=0x{command:02X}, SEQ={seq}. "
                "Команда могла выполниться. Автоматически начинать новый RUN нельзя."
            )
        except DeviceError:
            raise
        except Exception:
            self.usable = False
            raise

    @staticmethod
    def _expect(value, expected):
        if value != expected:
            raise ProtocolError(f"Ответ 0x{value:08X}, ожидалось 0x{expected:08X}")

    def _upload(self, begin, data_cmd, end, data):
        self._expect(self.request(begin, struct.pack("<I", len(data))), 0)
        for offset in range(0, len(data), self.chunk_size):
            part = data[offset:offset+self.chunk_size]
            value = self.request(data_cmd, struct.pack("<I", offset) + part)
            self._expect(value, offset + len(part))
            LOG.info("%s: %d/%d байт", data_cmd.name, value, len(data))
        self._expect(self.request(end), len(data))

    def upload_model(self, model: Model):
        self.model = None
        self._upload(Command.CONFIG_BEGIN, Command.CONFIG_DATA, Command.CONFIG_END, model.image)
        self.model = model

    def upload_input(self, data: bytes):
        if self.model is None:
            raise ValueError("Сначала загрузите модель этим клиентом")
        if len(data) != self.model.input_count * 4:
            raise ValueError(f"Ожидалось {self.model.input_count} значений FP32")
        if any(not math.isfinite(x[0]) for x in struct.iter_unpack("<f", data)):
            raise ValueError("Вход содержит NaN/Infinity")
        self._upload(Command.INPUT_BEGIN, Command.INPUT_DATA, Command.INPUT_END, data)

    def read_word(self, addr: int) -> int:
        if not 0 <= addr < 0x4000 or addr % 4:
            raise ValueError("Адрес shared memory должен быть выровнен по 4 байтам")
        return self.request(Command.READ_WORD, struct.pack("<I", addr))

    def run_and_read(self) -> dict:
        if self.model is None:
            raise ValueError("Сначала загрузите модель")
        address = self.request(Command.RUN, timeout=self.run_timeout)
        self._expect(address, self.model.output_addr)
        klass, count, pointer, status = [self.read_word(0x3200 + i*4) for i in range(4)]
        if status != 0 or pointer != address or count != self.model.output_count or klass >= count:
            raise ProtocolError("Некорректная служебная запись результата")
        words = [self.read_word(address + i*4) for i in range(count)]
        values = [struct.unpack("<f", struct.pack("<I", w))[0] for w in words]
        if any(not math.isfinite(v) or not 0 <= v <= 1 for v in values):
            raise ProtocolError("Выход sigmoid вне диапазона [0,1]")
        if max(range(count), key=values.__getitem__) != klass:
            raise ProtocolError("Класс не совпадает с argmax результатов")
        return {"class_index": klass, "output_address": address,
                "outputs": values, "output_hex": [f"{w:08x}" for w in words]}

    def infer(self, values) -> dict:
        """Повторные вызовы не перезагружают веса, но каждый загружает новый вход."""
        self.upload_input(encode_input(values))
        return self.run_and_read()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", help="COM3 или /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--model", help="Образ NNP2: .bin или .mem")
    parser.add_argument("--input", help="Вектор: JSON, текст, FP32 .bin или hex .mem")
    parser.add_argument("--output", help="Сохранить результат в JSON")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--run-timeout", type=float, default=30.0)
    parser.add_argument("--retries", type=int, default=2)
    parser.add_argument("--chunk-size", type=int, default=256)
    parser.add_argument("--list-ports", action="store_true")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO if args.verbose else logging.WARNING,
                        format="%(levelname)s: %(message)s")
    try:
        import serial
    except ImportError:
        parser.exit(2, "Установите зависимость: python -m pip install pyserial\n")
    if args.list_ports:
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"{p.device}: {p.description}")
        return 0
    if not args.port or not args.model or not args.input:
        parser.error("Нужны --port, --model и --input")
    try:
        model = Model.load(args.model)
        data = load_input(args.input)
        if len(data) != 4 * model.input_count:
            raise ValueError(f"Ожидалось {model.input_count} входных значений")
        if any(not math.isfinite(x[0]) for x in struct.iter_unpack("<f", data)):
            raise ValueError("NaN/Infinity во входном векторе")
        with serial.Serial(args.port, baudrate=args.baud, bytesize=8, parity="N",
                           stopbits=1, timeout=0.05, write_timeout=5,
                           xonxoff=False, rtscts=False, dsrdtr=False) as port:
            client = NpuClient(port, timeout=args.timeout, run_timeout=args.run_timeout,
                               retries=args.retries, chunk_size=args.chunk_size)
            print(f"Загрузка модели: {model.layers} слоёв, {len(model.image)} байт", file=sys.stderr)
            client.upload_model(model)
            print("Загрузка входов и запуск", file=sys.stderr)
            client.upload_input(data)
            result = client.run_and_read()
        text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)
        print(text)
        if args.output:
            Path(args.output).write_text(text + "\n", encoding="utf-8")
        return 0
    except (OSError, serial.SerialException, ValueError, ProtocolError) as e:
        print(f"Ошибка: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
