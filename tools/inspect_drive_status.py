#!/usr/bin/env python3
"""Read-only CANopen status inspection for the ZLAC8015D motor driver.

Only expedited SDO UPLOAD requests (0x40) are transmitted; this utility does
not write any SDO, send NMT frames, enable the drive or command motor current.
The ZLAC8015D uses node ID 1 by default.
"""
import argparse
import select
import socket
import struct
import time

CAN_FRAME = struct.Struct("=IB3x8s")


def decode_statusword(sw):
    # Standard CiA 402 PDS finite state machine, excluding unrelated bits.
    masked = sw & 0x006F
    states = {
        0x0000: "Not ready to switch on",
        0x0040: "Switch on disabled",
        0x0021: "Ready to switch on",
        0x0023: "Switched on (NOT operation enabled)",
        0x0027: "Operation enabled",
        0x0007: "Quick stop active",
        0x000F: "Fault reaction active",
        0x0008: "Fault",
    }
    return states.get(masked, f"Unknown state (masked=0x{masked:04X})")


def query(sock, node, index, sub, timeout):
    cob_tx = 0x600 + node
    cob_rx = 0x580 + node
    payload = bytes([0x40, index & 0xFF, (index >> 8) & 0xFF, sub, 0, 0, 0, 0])
    frame = CAN_FRAME.pack(cob_tx, 8, payload)
    if sock.send(frame) != len(frame):
        raise OSError("CAN frame send was incomplete")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        waiting = max(0, deadline - time.monotonic())
        ready, _, _ = select.select([sock], [], [], waiting)
        if not ready:
            break
        raw = sock.recv(CAN_FRAME.size)
        if len(raw) != CAN_FRAME.size:
            continue
        can_id, dlc, data = CAN_FRAME.unpack(raw)
        if can_id != cob_rx or dlc < 8 or data[1:4] != payload[1:4]:
            continue
        if data[0] == 0x80:
            abort_code = struct.unpack_from("<I", data, 4)[0]
            return ("abort", abort_code)
        command = data[0]
        if (command & 0xE3) != 0x43:  # SDO expedited upload and size indicated
            return ("unsupported-response", command)
        count = 4 - ((command >> 2) & 0x03)
        return ("value", int.from_bytes(data[4:4+count], byteorder="little"))
    return ("timeout", None)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--interface", default="can0")
    p.add_argument("--node-id", type=int, default=1)
    p.add_argument("--timeout", type=float, default=0.30)
    args = p.parse_args()
    if not 1 <= args.node_id <= 127 or args.timeout <= 0:
        p.error("node-id must be 1..127 and timeout > 0")
    specs = (
        ("CiA402 controlword (readback if supported)", 0x6040, 0),
        ("CiA402 statusword", 0x6041, 0),
        ("CiA402 statusword left (if supported)", 0x6041, 1),
        ("CiA402 statusword right (if supported)", 0x6041, 2),
        ("Mode of operation display", 0x6061, 0),
        ("Error code", 0x603F, 0),
        ("Configured rated current left [0.1 A]", 0x2014, 1),
        ("Configured rated current right [0.1 A]", 0x2014, 2),
        ("Configured max current left [0.1 A]", 0x2015, 1),
        ("Configured max current right [0.1 A]", 0x2015, 2),
        ("Torque slope left (vendor units)", 0x6087, 1),
        ("Torque slope right (vendor units)", 0x6087, 2),
        ("Target current left (if readable)", 0x6071, 1),
        ("Target current right (if readable)", 0x6071, 2),
        ("Actual current left (if readable)", 0x6077, 1),
        ("Actual current right (if readable)", 0x6077, 2),
        ("Command update mode (if readable)", 0x200F, 0),
    )
    print("READ-ONLY diagnostics: sending SDO upload requests only; NO motor commands.")
    sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.interface,))
        for label, index, sub in specs:
            status, value = query(sock, args.node_id, index, sub, args.timeout)
            name = f"0x{index:04X}:{sub:02X}"
            if status == "value":
                extra = ""
                if index == 0x6041 and sub == 0:
                    # ZLAC8015D 6041:00 is U32: two independently reported
                    # 16-bit motor statuswords packed together.
                    low = value & 0xFFFF
                    high = (value >> 16) & 0xFFFF
                    extra = (f" => lower16=0x{low:04X} [{decode_statusword(low)}]; "
                             f"upper16=0x{high:04X} [{decode_statusword(high)}]")
                    if ((low & 0x006F) != 0x0027 or
                            (high & 0x006F) != 0x0027):
                        extra += " [NOT BOTH OPERATION ENABLED]"
                elif index == 0x6061:
                    extra = f" => {'Profile Torque' if value == 4 else 'NOT Profile Torque'}"
                elif index in (0x2014, 0x2015):
                    extra = f" => {value / 10.0:.1f} A (configured, not an independent rating check)"
                elif index == 0x603F and value:
                    extra = " => DRIVE ERROR REPORTED"
                print(f"{name}  {label}: {value} (0x{value:X}){extra}")
            elif status == "abort":
                print(f"{name}  {label}: SDO abort 0x{value:08X} (may be unsupported)")
            else:
                print(f"{name}  {label}: {status}")
    finally:
        sock.close()


if __name__ == "__main__":
    main()
