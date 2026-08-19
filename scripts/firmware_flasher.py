"""固件升级模块 —— 通过 GCANFD USB 适配器向 CAN 总线设备烧录固件。

使用方法
========

单台设备（最简）::

    from firmware_flasher import update_firmware

    update_firmware("app.bin", can_id=0x1001)

多台设备循环烧录（适配器只打开一次，推荐）::

    from firmware_flasher import FirmwareFlasher, FlashError

    with FirmwareFlasher() as flasher:
        for can_id in (0x1001, 0x1003, 0x1005):   # 设备 i: 0x1000 + 2*i - 1
            try:
                flasher.flash("app.bin", can_id=can_id)
            except FlashError as e:
                print(f"CAN 0x{can_id:X} 升级失败: {e}")

进度回调::

    def on_progress(written, total):
        print(f"{written}/{total} ({written * 100 // total}%)")

    update_firmware("app.bin", can_id=0x1001, on_progress=on_progress)

超时调整（默认值即推荐值，仅特殊场合需要）::

    from firmware_flasher import FirmwareFlasher, FlashTimeouts

    flasher = FirmwareFlasher(timeouts=FlashTimeouts(erase_ms=15000))

命令行直接运行::

    python firmware_flasher.py app.bin --can-id 0x1001

运行环境要求
============

- Windows，Python 3.8 及以上
- 适配器运行库（与本文件放同一目录，或加入 PATH）：
  ECANFDVCI64.dll、GCANUSB_x64.dll、MSVCR120.dll
- USBCANFD 适配器内核驱动（每台电脑安装一次）
- CAN 总线参数固定为：CANFD 帧、仲裁段 1 Mbps、数据段 5 Mbps、BRS、
  扩展帧 29 位 —— 与设备出厂配置一致，无需（也不支持）修改

注意事项
========

- 设备在一次连接后进入升级态，**断电重新上电**才能再次连接；
  连接失败时请先断电重上电再重试
- 固件文件为原始 BIN，原样写入，不做任何转换
- 本模块为同步阻塞模型；需要异步请自行放入后台线程
"""

import argparse
import logging
import os
import sys
import time
from dataclasses import dataclass

__all__ = [
    'update_firmware', 'FirmwareFlasher', 'FlashTimeouts',
    'FlashError', 'AdapterError', 'ConnectError', 'CommandError',
    'TransportError', 'FlashTimeoutError', 'VerifyError', 'FirmwareFileError',
    'FIRMWARE_BASE_ADDRESS',
]

# ---- 固定参数（与设备端约定一致，不对外暴露修改） ----
FIRMWARE_BASE_ADDRESS = 0x60000000  # 设备固件区起始地址
BLOCK_SIZE = 256                    # 单次写入块大小（字节）
RESPONSE_ID_OFFSET = 1              # 设备应答 CAN ID = 请求 ID + 1
DEVICE_TYPE_USBCANFD = 6            # 适配器型号常量（SDK 约定）
FRAME_FLAGS = 0x0B                  # CANFD | 扩展帧 | 数据帧 | BRS


# ---- 异常体系 ----
class FlashError(Exception):
    """固件升级错误基类。"""


class AdapterError(FlashError):
    """适配器打开或初始化失败（请检查 USB 连接与 DLL 部署）。"""


class ConnectError(FlashError):
    """设备连接失败（请将设备断电后重新上电再试）。"""


class CommandError(FlashError):
    """设备返回错误（附错误码中文说明）。"""


class TransportError(FlashError):
    """总线传输异常（多帧序列或流控错误）。"""


class FlashTimeoutError(FlashError):
    """等待设备响应超时。"""


class VerifyError(FlashError):
    """设备自校验未通过。"""


class FirmwareFileError(FlashError):
    """固件文件读取失败。"""


# ---- 超时配置 ----
@dataclass
class FlashTimeouts:
    """各阶段超时（毫秒）。默认值即实机验证的推荐值。"""
    connect_ms: int = 500          # 连接响应
    connect_retries: int = 3       # 连接重试次数
    command_ms: int = 1000         # 常规命令
    program_start_ms: int = 2000   # 进入升级模式
    erase_ms: int = 10000          # 擦除（较慢）
    verify_ms: int = 10000         # 自校验
    cf_gap_ms: int = 1000          # 多帧/流控帧间隔


# ---- 设备协议命令码（协议约定值） ----
CMD_CONNECT = 0xFF        # 连接
CMD_DISCONNECT = 0xFE
CMD_GET_STATUS = 0xFD
CMD_SET_MTA = 0xF6        # 设置访问地址
CMD_PROGRAM_START = 0xD2  # 进入写入模式
CMD_PROGRAM_CLEAR = 0xD1  # 擦除
CMD_PROGRAM = 0xD0        # 写入（短包）
CMD_PROGRAM_BULK = 0xC8   # 写入（数据块）
CMD_PROGRAM_RESET = 0xCF  # 写入完成复位
CMD_USER = 0xF1           # 自定义命令（自校验）
PID_POSITIVE = 0xFF       # 正响应
PID_NEGATIVE = 0xFE       # 负响应


LOG_TRACE = 5   # 逐帧字节日志级别（低于 DEBUG）


def make_logger():
    """模块 logger：默认静默，客户自行挂 handler 接管。"""
    logging.addLevelName(LOG_TRACE, 'TRACE')
    logger = logging.getLogger('firmware_flasher')
    logger.addHandler(logging.NullHandler())
    return logger


logger = make_logger()


def load_firmware(path):
    """读取固件 BIN 文件（原样字节，不做转换）。"""
    try:
        with open(path, 'rb') as f:
            data = f.read()
    except OSError as e:
        raise FirmwareFileError(f'固件文件读取失败: {path} ({e})') from e
    if not data:
        raise FirmwareFileError(f'固件文件为空: {path}')
    return data


def split_blocks(data):
    """按 BLOCK_SIZE 切块，末块取余。"""
    return [data[i:i + BLOCK_SIZE] for i in range(0, len(data), BLOCK_SIZE)]


# ---- 总线传输层：多帧拆包编码（ISO 15765-2 经典 8 字节帧） ----
_SF_MAX = 7      # 单帧最大载荷
_FF_PAYLOAD = 6  # 首帧载荷
_CF_PAYLOAD = 7  # 连续帧载荷


def _pad8(frame_head, payload):
    """帧头 + 载荷补 0 到 8 字节。"""
    return bytes(frame_head + list(payload)).ljust(8, b'\x00')


def encode_sf(payload):
    """单帧：0x0L + 载荷（≤7 字节）。"""
    if len(payload) > _SF_MAX:
        raise ValueError('单帧载荷超限')
    return _pad8([0x00 | len(payload)], payload)


def encode_ff(payload):
    """首帧：1L LL（12 位总长）+ 前 6 字节。"""
    if len(payload) < 8:
        raise ValueError('短载荷不应拆多帧')
    head = [0x10 | (len(payload) >> 8), len(payload) & 0xFF]
    return _pad8(head, payload[:_FF_PAYLOAD])


def encode_cfs(payload):
    """首帧之后的全部连续帧：2N（SN=1..15 循环跳 0）+ 7 字节；末帧只载剩余字节，不补 0。"""
    frames = []
    body = payload[_FF_PAYLOAD:]
    sn = 0
    for i in range(0, len(body), _CF_PAYLOAD):
        sn = (sn + 1) & 0x0F
        if sn == 0:
            sn = 1
        frames.append(bytes([0x20 | sn]) + body[i:i + _CF_PAYLOAD])
    return frames


def encode_message(payload):
    """拆帧总入口：返回 (首帧列表, 连续帧列表)。SF 时连续帧为空。"""
    if len(payload) <= _SF_MAX:
        return [encode_sf(payload)], []
    return [encode_ff(payload)], encode_cfs(payload)


# ---- 总线传输层：多帧接收重组（与 encode 层互逆） ----
_FC_CTS = bytes([0x30, 0x00, 0x00, 0, 0, 0, 0, 0])  # 流控：继续发送（BS=0, STmin=0）


class IsoTpReceiver:
    """总线多帧接收重组状态机（纯逻辑，不碰硬件）。

    feed() 每收到一帧调用一次，返回 (完成的消息, 需回发的帧)。
    收到首帧自动产出流控帧；序号错乱则丢弃当前消息并重置。
    流控帧与非本层帧静默忽略。
    """

    def __init__(self):
        self._buffer = bytearray()
        self._total = 0
        self._expected_sn = 0

    def feed(self, frame):
        if len(frame) < 1:
            return None, None
        nibble = frame[0] >> 4
        if nibble == 0 and 0 < frame[0] & 0x0F <= _SF_MAX:
            length = frame[0] & 0x0F
            return bytes(frame[1:1 + length]), None
        if nibble == 1:
            self._total = ((frame[0] & 0x0F) << 8) | frame[1]
            self._buffer = bytearray(frame[2:8])
            self._expected_sn = 0
            return None, _FC_CTS
        if nibble == 2:
            sn = frame[0] & 0x0F
            expected = (self._expected_sn + 1) & 0x0F or 1
            if sn != expected or self._total == 0:
                self.__init__()   # 序号错乱：丢弃重置
                return None, None
            self._expected_sn = sn
            need = self._total - len(self._buffer)
            self._buffer += frame[1:1 + min(7, need)]
            if len(self._buffer) >= self._total:
                msg = bytes(self._buffer[:self._total])
                self.__init__()
                return msg, None
        return None, None   # 流控帧(3x)/其他帧：忽略


class IsoTpLink:
    """总线帧与会话级消息之间的传输层。

    send_and_receive(): 发一条完整命令、等一条完整应答。
    多帧命令在首帧后等待设备流控（CTS 才发后续帧，WAIT 继续等，
    OVFLW 报错）；等应答期间收到设备首帧会自动回流控帧。
    """

    def __init__(self, send_raw, recv_raw, cf_gap_ms=None):
        self._send_raw = send_raw
        self._recv_raw = recv_raw
        self._cf_gap_ms = cf_gap_ms or 1000

    def _poll_one_frame(self, deadline):
        """取一帧；超时抛 FlashTimeoutError。"""
        while True:
            frame = self._recv_raw()
            if frame is not None:
                return frame
            if time.monotonic() >= deadline:
                raise FlashTimeoutError('等待设备应答超时')
            time.sleep(0.001)

    def send_and_receive(self, payload, timeout_ms):
        first, cfs = encode_message(payload)
        for frame in first:
            self._send_raw(frame)
        if cfs:
            self._await_flow_control_then_send(cfs)
        return self._receive_message(timeout_ms)

    def _await_flow_control_then_send(self, cfs):
        deadline = time.monotonic() + (self._cf_gap_ms / 1000.0)
        while True:
            frame = self._poll_one_frame(deadline)
            nibble = frame[0] >> 4
            if nibble != 3:
                continue                       # 忽略无关帧，继续等流控
            status = frame[0] & 0x0F
            if status == 0:                    # CTS：一次性连发（BS=0）
                for cf in cfs:
                    self._send_raw(cf)
                return
            if status == 1:                    # WAIT：重置时限继续等
                deadline = time.monotonic() + (self._cf_gap_ms / 1000.0)
                continue
            raise TransportError(f'设备流控溢出 (0x{frame[0]:02X})')

    def _receive_message(self, timeout_ms):
        receiver = IsoTpReceiver()
        deadline = time.monotonic() + (timeout_ms / 1000.0)
        while True:
            frame = self._poll_one_frame(deadline)
            msg, reply = receiver.feed(frame)
            if reply is not None:
                self._send_raw(reply)
            if msg is not None:
                return msg

    def send_and_receive_no_wait(self, payload):
        first, cfs = encode_message(payload)
        for frame in first:
            self._send_raw(frame)
        if cfs:
            # 复位命令只有 1 字节，实际不会走这里；保留通用性
            for cf in cfs:
                self._send_raw(cf)


# ---- 设备命令层（组包/判答，纯函数） ----
ERROR_MESSAGES = {
    0x00: '命令同步错误',
    0x10: '设备忙',
    0x20: '未知命令',
    0x22: '参数超范围',
    0x25: '访问被锁定',
    0x26: '页无效',
    0x29: '命令序列错误',
    0x31: '设备写入失败',
}


def _le32(value):
    return value.to_bytes(4, 'little')


def build_connect():
    return bytes([CMD_CONNECT, 0x00])


def build_get_status():
    return bytes([CMD_GET_STATUS])


def build_program_start():
    return bytes([CMD_PROGRAM_START])


def build_set_mta(address):
    return bytes([CMD_SET_MTA, 0, 0, 0]) + _le32(address)


def build_program_clear(total_bytes):
    return bytes([CMD_PROGRAM_CLEAR, 0, 0, 0]) + _le32(total_bytes)


def build_program_bulk(data):
    return bytes([CMD_PROGRAM_BULK]) + len(data).to_bytes(2, 'little') + data


def build_program(data):
    if len(data) > 255:
        raise ValueError('短包载荷超限')
    return bytes([CMD_PROGRAM, len(data)]) + data


def build_selfcheck(total_bytes):
    return bytes([CMD_USER, 0x02]) + _le32(total_bytes) + b'\x00\x00'


def build_program_reset():
    return bytes([CMD_PROGRAM_RESET])


def check_positive(resp):
    """判定正响应；负响应抛 CommandError（含错误码中文说明）。"""
    if not resp or resp[0] not in (PID_POSITIVE, PID_NEGATIVE):
        raise CommandError(f'设备应答异常: {resp.hex() if resp else "空"}')
    if resp[0] == PID_NEGATIVE:
        code = resp[1] if len(resp) > 1 else 0xFF
        reason = ERROR_MESSAGES.get(code, f'未知错误 0x{code:02X}')
        raise CommandError(f'设备返回错误: {reason} (0x{code:02X})')


def parse_connect(resp):
    """连接应答：返回单包上限（max_cto）。"""
    check_positive(resp)
    if len(resp) < 4:
        raise CommandError(f'连接应答长度不足: {resp.hex()}')
    return resp[3]


def parse_selfcheck(resp):
    """自校验应答：0x01 通过；0x02 设备端暂定放行（与上位机既有行为一致）。"""
    check_positive(resp)
    if len(resp) < 3 or resp[1] != 0x02:
        raise CommandError(f'自校验应答异常: {resp.hex() if resp else "空"}')
    if resp[2] not in (0x01, 0x02):
        raise VerifyError(f'设备自校验未通过 (result=0x{resp[2]:02X})')


# ---- 设备升级会话编排 ----
class DeviceSession:
    """单台设备的升级会话：连接、写入、自校验、复位。"""

    def __init__(self, link, timeouts):
        self._link = link
        self._t = timeouts

    def _command(self, payload, timeout_ms):
        resp = self._link.send_and_receive(payload, timeout_ms)
        check_positive(resp)
        return resp

    def connect(self):
        """连接设备（带重试；失败提示断电重上电）。"""
        last_error = None
        for _ in range(self._t.connect_retries):
            try:
                resp = self._link.send_and_receive(
                    build_connect(), self._t.connect_ms)
                parse_connect(resp)
                logger.debug('设备连接成功 (单包上限=%d)', resp[3])
                return
            except (FlashTimeoutError, CommandError) as e:
                last_error = e
        raise ConnectError(
            f'设备连接失败（已重试 {self._t.connect_retries} 次）：{last_error}。'
            '请将设备断电后重新上电再试（设备在一次连接后进入升级态，'
            '断电重上电才能再次连接）') from last_error

    def get_status(self):
        self._command(build_get_status(), self._t.command_ms)

    def program_start(self):
        self._command(build_program_start(), self._t.program_start_ms)

    def set_mta(self, address):
        self._command(build_set_mta(address), self._t.command_ms)

    def program_clear(self, total_bytes):
        logger.debug('擦除 %d 字节...', total_bytes)
        self._command(build_program_clear(total_bytes), self._t.erase_ms)

    def _write_block(self, block):
        if len(block) == BLOCK_SIZE:
            msg = build_program_bulk(block)
        else:
            msg = build_program(block)
        self._command(msg, self._t.command_ms)

    def selfcheck(self, total_bytes):
        logger.debug('设备自校验 %d 字节...', total_bytes)
        resp = self._link.send_and_receive(
            build_selfcheck(total_bytes), self._t.verify_ms)
        parse_selfcheck(resp)

    def program_reset(self):
        """复位命令不期待应答（设备收到即重启）。"""
        self._link.send_and_receive_no_wait(build_program_reset())

    def flash(self, firmware, on_progress=None):
        """完整升级流程：握手→擦除→逐块写入→自校验→复位。"""
        blocks = split_blocks(firmware)
        total = len(firmware)
        self.get_status()
        self.program_start()
        self.set_mta(FIRMWARE_BASE_ADDRESS)
        self.program_clear(total)
        written = 0
        for block in blocks:
            self._write_block(block)
            written += len(block)
            logger.debug('写入进度 %d/%d', written, total)
            if on_progress is not None:
                on_progress(written, total)
        self.set_mta(FIRMWARE_BASE_ADDRESS)
        self.selfcheck(total)
        self.program_reset()
        logger.info('升级完成: %d 字节', total)


# ---- 适配器驱动层（广成 USBCANFD，ctypes 直调运行库） ----
import ctypes


class INIT_CONFIG(ctypes.Structure):
    """适配器初始化参数（布局对齐 SDK 头文件，sizeof=80）。"""
    _fields_ = [
        ('CanReceMode', ctypes.c_ubyte),        # 3=标准+扩展全收
        ('CanSendMode', ctypes.c_ubyte),        # 0=正常发送（1 实测打不开设备）
        ('_pad0', ctypes.c_ubyte * 2),
        ('NominalBitRate', ctypes.c_ulong),     # 不填，用下面的枚举
        ('DataBitRate', ctypes.c_ulong),
        ('FilterUsedBits', ctypes.c_ubyte),
        ('StdOrExdBits', ctypes.c_ubyte),
        ('NominalBitRateSelect', ctypes.c_ubyte),   # 枚举 0=1 Mbps
        ('DataBitRateSelect', ctypes.c_ubyte),      # 枚举 0=5 Mbps
        ('Filters', (ctypes.c_ulong * 2) * 8),      # 全 0 = 不过滤
    ]


class CANFD_OBJ(ctypes.Structure):
    """总线帧对象（布局对齐 SDK 头文件，sizeof=80）。"""
    _fields_ = [
        ('Flags', ctypes.c_ubyte),      # bit0 CANFD / bit1 扩展 / bit2 远程 / bit3 BRS
        ('DataLen', ctypes.c_ubyte),
        ('_reserved', ctypes.c_ubyte * 2),
        ('ID', ctypes.c_ulong),
        ('_timestamp', ctypes.c_ubyte * 8),
        ('Data', ctypes.c_ubyte * 64),
    ]


def _dll_candidates(dll_path=None):
    """加载候选路径：显式路径优先；否则脚本同目录绝对路径，再回退 PATH 名字搜索。"""
    if dll_path:
        return [dll_path]
    script_dir = os.path.dirname(os.path.abspath(__file__))
    return [os.path.join(script_dir, 'ECANFDVCI64.dll'), 'ECANFDVCI64.dll']


def _load_adapter_dll(dll_path=None):
    script_dir = os.path.dirname(os.path.abspath(__file__))
    try:
        # 让依赖库解析（GCANUSB_x64 等）也搜脚本所在目录，不依赖当前目录/PATH
        os.add_dll_directory(script_dir)
    except (OSError, AttributeError, ValueError):
        pass
    last_error = None
    for candidate in _dll_candidates(dll_path):
        try:
            return ctypes.WinDLL(candidate)
        except OSError as e:
            last_error = e
    raise AdapterError(
        '适配器运行库加载失败：请确认 ECANFDVCI64.dll / GCANUSB_x64.dll / '
        f'MSVCR120.dll 与本文件同目录或已加入 PATH ({last_error})') from last_error


def frame_id_matches(frame_id, expected):
    """应答 ID 匹配：掩掉运行库可能带回的高位标志位后比较（29 位有效）。

    与上位机 C++ 实机路径一致：SDK 收到的扩展帧 ID 可能带高位标志位，
    裸相等比较会静默丢弃全部设备应答。
    """
    return (frame_id & 0x1FFFFFFF) == (expected & 0x1FFFFFFF)


def format_frame_log(direction, can_id, data):
    """逐帧 TRACE 日志行（纯函数，便于单测）。direction 为「发送/接收」。"""
    return f'{direction}帧 id=0x{can_id:03X} data={bytes(data).hex()}'


class GcanAdapter:
    """USBCANFD 适配器：打开/关闭/收发（单帧、非阻塞接收）。"""

    def __init__(self, dll_path=None, device_ind=0, can_ind=0, dll=None):
        # dll：测试注入假运行库对象；None 时才真实加载
        self._dll = dll if dll is not None else _load_adapter_dll(dll_path)
        self._dev = device_ind
        self._can = can_ind
        self._opened = False

    def _check(self, ret, action):
        if ret != 0:
            raise AdapterError(f'适配器{action}失败 (0x{ret & 0xFFFFFFFF:X})')

    def open(self):
        dt = DEVICE_TYPE_USBCANFD
        self._check(self._dll.OpenDeviceFD(dt, self._dev), '打开')
        try:
            cfg = INIT_CONFIG()
            cfg.CanReceMode = 3
            cfg.CanSendMode = 0
            cfg.NominalBitRateSelect = 0    # 仲裁段 1 Mbps
            cfg.DataBitRateSelect = 0       # 数据段 5 Mbps
            self._check(self._dll.InitCANFD(dt, self._dev, self._can,
                                            ctypes.byref(cfg)), '初始化')
            self._check(self._dll.StartCANFD(dt, self._dev, self._can), '启动')
        except Exception:
            # 半途失败必须回收句柄：异常退出时 __exit__ 不会触发，
            # 不关则长驻进程重试恒报「设备已打开」
            self._dll.CloseDeviceFD(dt, self._dev)
            raise
        self._opened = True

    def close(self):
        if self._opened:
            dt = DEVICE_TYPE_USBCANFD
            self._dll.StopCANFD(dt, self._dev, self._can)
            self._dll.CloseDeviceFD(dt, self._dev)
            self._opened = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def send_frame(self, can_id, data):
        """发一帧（CANFD 壳、扩展 ID、BRS；载荷补齐到 8 字节，超长截断）。"""
        obj = CANFD_OBJ()
        obj.Flags = FRAME_FLAGS
        obj.DataLen = 8
        obj.ID = can_id
        padded = bytes(data)[:8].ljust(8, b'\x00')
        obj.Data[:8] = list(padded)
        ret = self._dll.TransmitFD(DEVICE_TYPE_USBCANFD, self._dev, self._can,
                                   ctypes.byref(obj), 1)
        self._check(ret, '发送')
        logger.log(LOG_TRACE, '%s', format_frame_log('发送', can_id, padded))

    def recv_frame(self):
        """非阻塞收一帧；无数据返回 None。"""
        obj = CANFD_OBJ()
        length = ctypes.c_ulong(1)
        ret = self._dll.ReceiveFD(DEVICE_TYPE_USBCANFD, self._dev, self._can,
                                  ctypes.byref(obj), ctypes.byref(length))
        if ret != 0 or length.value == 0:
            return None
        frame_id = obj.ID
        data = bytes(obj.Data[:obj.DataLen if obj.DataLen else 8])
        logger.log(LOG_TRACE, '%s', format_frame_log('接收', frame_id, data))
        return frame_id, data


class FirmwareFlasher:
    """升级会话：管理适配器生命周期，可对多台设备循环烧录。

    with FirmwareFlasher() as flasher:
        flasher.flash("app.bin", can_id=0x1001)
    """

    def __init__(self, timeouts=None, dll_path=None, io_factory=None):
        self._timeouts = timeouts or FlashTimeouts()
        self._dll_path = dll_path
        self._io_factory = io_factory     # 测试注入点；生产走真实适配器
        self._adapter = None

    def _make_io(self, can_id):
        """返回 (send_raw, recv_raw)：测试注入或真实适配器。"""
        if self._io_factory is not None:
            result = self._io_factory(can_id)
            return result[0], result[1]
        adapter = self._adapter

        def send_raw(frame, _a=adapter, _id=can_id):
            _a.send_frame(_id, frame)

        response_id = can_id + RESPONSE_ID_OFFSET

        def recv_raw(_a=adapter, _rid=response_id):
            got = _a.recv_frame()
            if got is None:
                return None
            frame_id, data = got
            # 只认本机应答 ID（掩掉高位标志位再比较）
            return data if frame_id_matches(frame_id, _rid) else None

        return send_raw, recv_raw

    def __enter__(self):
        if self._io_factory is None:
            self._adapter = GcanAdapter(self._dll_path)
            self._adapter.open()
        return self

    def __exit__(self, *exc):
        if self._adapter is not None:
            self._adapter.close()
            self._adapter = None

    def flash(self, firmware_path, can_id, on_progress=None):
        """烧录一台设备：读文件 → 连接 → 写入 → 自校验 → 复位。"""
        firmware = load_firmware(firmware_path)
        send_raw, recv_raw = self._make_io(can_id)
        link = IsoTpLink(send_raw, recv_raw, cf_gap_ms=self._timeouts.cf_gap_ms)
        session = DeviceSession(link, self._timeouts)
        session.connect()
        session.flash(firmware, on_progress)


def update_firmware(firmware_path, can_id, on_progress=None,
                    timeouts=None, io_factory=None):
    """单台设备一键升级（便捷入口；内部开关一次适配器）。"""
    with FirmwareFlasher(timeouts=timeouts, io_factory=io_factory) as flasher:
        flasher.flash(firmware_path, can_id, on_progress)


# ---- 命令行入口 ----
def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='firmware_flasher.py',
        description='固件升级工具：通过 USBCANFD 适配器向设备烧录固件 BIN')
    parser.add_argument('firmware', help='固件 BIN 文件路径')
    parser.add_argument('--can-id', required=True, type=lambda s: int(s, 0),
                        help='目标设备 CAN ID（支持 0x 十六进制，如 0x1001）')
    return parser.parse_args(argv)


def _main(argv=None, io_factory=None):
    args = _parse_args(argv)
    last_percent = -1

    def on_progress(written, total):
        nonlocal last_percent
        percent = written * 100 // total
        if percent != last_percent:
            last_percent = percent
            print(f'\r写入进度: {percent}%', end='', flush=True)

    try:
        update_firmware(args.firmware, args.can_id,
                        on_progress=on_progress, io_factory=io_factory)
    except FlashError as e:
        print()
        print(f'升级失败: {e}', file=sys.stderr)
        return 1
    print()
    print('升级完成')
    return 0


if __name__ == '__main__':
    sys.exit(_main())
