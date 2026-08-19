"""Linux 板载 CANFD 固件烧录模块 —— 协议层复用 firmware_flasher，传输层换 socketcan。
使用方法
已经在地瓜X5上完成验证
========

单台设备(最简)::

    from firmware.firmware_flasher_linux import update_firmware

    update_firmware("app.bin", can_id=0x1001, interface="can0")

多台设备循环烧录(套接字只打开一次，推荐)::

    from firmware.firmware_flasher_linux import LinuxFlasher, FlashError

    with LinuxFlasher(interface="can0") as flasher:
        for can_id in (0x1001, 0x1003, 0x1005):
            try:
                flasher.flash("app.bin", can_id=can_id)
            except FlashError as e:
                print(f"CAN 0x{can_id:X} 升级失败: {e}")

命令行直接运行::

    python firmware_flasher_linux.py app.bin --can-id 0x1001 --interface can0

板端环境要求
============

- Linux，内核支持 CANFD，且接口已配置并启用(每次上电配置一次)::

    sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
    sudo ip link set can0 up

- 仲裁段 1 Mbps / 数据段 5 Mbps 为设备出厂约定，不可改
- 仅依赖 Python 标准库，无 pip 依赖
- 设备一次连接后进入升级态，**断电重新上电**才能再次连接(同 Windows 版)
"""

import argparse
import socket
import struct
import sys

try:
    from .firmware_flasher import (
        IsoTpLink, DeviceSession, FlashTimeouts, FlashError, AdapterError,
        load_firmware, RESPONSE_ID_OFFSET, frame_id_matches,
    )
except ImportError:   
    from firmware_flasher import (
        IsoTpLink, DeviceSession, FlashTimeouts, FlashError, AdapterError,
        load_firmware, RESPONSE_ID_OFFSET, frame_id_matches,
    )

__all__ = ['update_firmware', 'LinuxFlasher', 'SocketcanAdapter']


# 帧格式常量
CAN_EFF_FLAG = 0x80000000   # 扩展帧(29 位 ID)
CANFD_BRS = 0x01            # 数据段波特率切换(5 Mbps)
DLC = 8                     # 协议帧固定 8 字节载荷

# 内核帧布局:
#   struct can_frame     can_id(4) len(1) 保留(3) data(8)  = 16 字节(经典帧)
#   struct canfd_frame   can_id(4) len(1) flags(1) 保留(2) data(64) = 72 字节(CANFD 帧)
_CAN_FRAME = struct.Struct('<IB3x8s')
_CANFD_FRAME = struct.Struct('<IBB2x64s')

_SOL_CAN_RAW = getattr(socket, 'SOL_CAN_RAW', 101)
_CAN_RAW_FD_FRAMES = getattr(socket, 'CAN_RAW_FD_FRAMES', 5)


class SocketcanAdapter:
    """板载 CANFD 适配器：打开/关闭/收发(单帧、非阻塞接收)。

    行为与 GcanAdapter 对齐：发送载荷补 0 到 8 字节，接收取 8 字节；
    CAN ID 带 EFF 标志位，由 frame_id_matches 掩掉高位再比较(同 Windows 版)。
    """

    def __init__(self, interface='can0'):
        self._ifname = interface
        self._sock = None

    def open(self):
        try:
            self._sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                       socket.CAN_RAW)
            # 不设 CAN_RAW_FD_FRAMES 时，CANFD 帧收发都会被内核静默丢弃/拒绝
            self._sock.setsockopt(_SOL_CAN_RAW, _CAN_RAW_FD_FRAMES, 1)
            self._sock.bind((self._ifname,))
            self._sock.setblocking(False)
        except OSError as e:
            raise AdapterError(
                f'socketcan 打开失败: {e}。请确认接口已配置并启用: '
                'sudo ip link set can0 type can bitrate 1000000 '
                'dbitrate 5000000 fd on && sudo ip link set can0 up') from e

    def close(self):
        if self._sock is not None:
            self._sock.close()
            self._sock = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *exc):
        self.close()

    def send_frame(self, can_id, data):
        """发一帧(CANFD、扩展 ID、BRS；载荷补齐到 8 字节，超长截断)。"""
        padded = bytes(data)[:DLC].ljust(DLC, b'\x00')
        frame = _CANFD_FRAME.pack(can_id | CAN_EFF_FLAG, DLC, CANFD_BRS, padded)
        try:
            self._sock.send(frame)
        except OSError as e:
            raise AdapterError(f'socketcan 发送失败: {e}') from e

    def recv_frame(self):
        try:
            frame = self._sock.recv(_CANFD_FRAME.size)
        except (BlockingIOError, OSError):
            return None
        if len(frame) == _CAN_FRAME.size:
            can_id, dlc, data = _CAN_FRAME.unpack(frame)
            return can_id, bytes(data[:dlc if dlc else 8])
        if len(frame) == _CANFD_FRAME.size:
            can_id, dlc, _flags, data = _CANFD_FRAME.unpack(frame)
            return can_id, bytes(data[:DLC])
        return None   # 未知帧长，忽略


class LinuxFlasher:

    def __init__(self, interface='can0', timeouts=None):
        self._interface = interface
        self._timeouts = timeouts or FlashTimeouts()
        self._adapter = None

    def _make_io(self, can_id):
        """返回 (send_raw, recv_raw)，与 Windows 版同构。"""
        adapter = self._adapter

        def send_raw(frame, _a=adapter, _id=can_id):
            _a.send_frame(_id, frame)

        response_id = can_id + RESPONSE_ID_OFFSET

        def recv_raw(_a=adapter, _rid=response_id):
            got = _a.recv_frame()
            if got is None:
                return None
            frame_id, data = got
            return data if frame_id_matches(frame_id, _rid) else None

        return send_raw, recv_raw

    def __enter__(self):
        self._adapter = SocketcanAdapter(self._interface)
        self._adapter.open()
        return self

    def __exit__(self, *exc):
        if self._adapter is not None:
            self._adapter.close()
            self._adapter = None

    def flash(self, firmware_path, can_id, on_progress=None):
        firmware = load_firmware(firmware_path)
        send_raw, recv_raw = self._make_io(can_id)
        link = IsoTpLink(send_raw, recv_raw, cf_gap_ms=self._timeouts.cf_gap_ms)
        session = DeviceSession(link, self._timeouts)
        session.connect()
        session.flash(firmware, on_progress)


def update_firmware(firmware_path, can_id, interface='can0',
                    on_progress=None, timeouts=None):
    with LinuxFlasher(interface, timeouts) as flasher:
        flasher.flash(firmware_path, can_id, on_progress)


# 命令行入口
def _parse_args(argv):
    parser = argparse.ArgumentParser(
        prog='firmware_flasher_linux.py',
        description='固件升级工具(Linux 板载 CANFD)：向设备烧录固件 BIN')
    parser.add_argument('firmware', help='固件 BIN 文件路径')
    parser.add_argument('--can-id', required=True, type=lambda s: int(s, 0),
                        help='目标设备 CAN ID(支持 0x 十六进制，如 0x1001)')
    parser.add_argument('--interface', default='can0',
                        help='CAN 接口名(默认 can0)')
    return parser.parse_args(argv)


def _main(argv=None):
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
                        interface=args.interface, on_progress=on_progress)
    except FlashError as e:
        print()
        print(f'升级失败: {e}', file=sys.stderr)
        return 1
    print()
    print('升级完成')
    return 0


if __name__ == '__main__':
    sys.exit(_main())
