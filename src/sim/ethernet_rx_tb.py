import cocotb
import os
import random
import sys
import numpy
import logging
import numpy as np
import math
import random


from math import log
from pathlib import Path

from cocotb.clock import Clock
from cocotb.triggers import (
    Timer,
    ClockCycles,
    RisingEdge,
    FallingEdge,
    ReadOnly,
    ReadWrite,
    with_timeout,
)
from cocotb.utils import get_sim_time as gst
from cocotb.binary import BinaryValue

# from cocotb.runner          import get_runner
from cocotb.runner import get_runner
from cocotb_bus.bus import Bus
from cocotb_bus.drivers import BusDriver
from cocotb_bus.monitors import Monitor
from cocotb_bus.monitors import BusMonitor
from cocotb_bus.scoreboard import Scoreboard
from scapy.all import Ether, IP, UDP, Raw


test_file = os.path.basename(__file__).replace(".py", "")


########################################################################################################
########################################################################################################


class AXIS_Monitor(BusMonitor):
    """
    monitors axi streaming bus
    """

    transactions = 0  # use this variable to track good ready/valid handshakes

    def __init__(self, dut, name, clk, callback=None):
        self._signals = [
            "axis_tvalid",
            "axis_tready",
            "axis_tlast",
            "axis_tdata",
            "axis_tkeep",
        ]
        BusMonitor.__init__(self, dut, name, clk, callback=callback)
        self.clock = clk
        self.transactions = 0
        self.dut = dut

    async def _monitor_recv(self):
        """
        Monitor receiver
        """
        # make these coroutines once and reuse
        rising_edge = RisingEdge(self.clock)
        falling_edge = FallingEdge(self.clock)
        read_only = ReadOnly()

        while True:
            # await rising_edge #can either wait for just edge...
            # or you can also wait for falling edge/read_only (see note in lab)

            await falling_edge  # sometimes see in AXI shit
            await read_only  # readonly (the postline)
            valid = self.bus.axis_tvalid.value
            ready = self.bus.axis_tready.value
            last = self.bus.axis_tlast.value
            data = self.bus.axis_tdata.value  # .signed_integer
            strb = self.bus.axis_tkeep.value
            if valid and ready:
                self.transactions += 1
                thing = dict(
                    data=data,
                    strb=strb,
                    last=last,
                    name=self.name,
                    count=self.transactions,
                )
                # self.dut._log.info(f"{self.name}: {self.transactions}")
                self._recv(thing)


########################################################################################################
########################################################################################################


class AXIS_Driver(BusDriver):
    def __init__(self, dut, name, clk, role="M"):
        self._signals = [
            "axis_tvalid",
            "axis_tready",
            "axis_tlast",
            "axis_tdata",
            "axis_tkeep",
        ]
        BusDriver.__init__(self, dut, name, clk)
        self.clock = clk
        self.dut = dut


class M_AXIS_Driver(AXIS_Driver):
    def __init__(self, dut, name, clk):
        super().__init__(dut, name, clk)
        self.bus.axis_tdata.value = 0
        self.bus.axis_tkeep.value = 0xF
        self.bus.axis_tlast.value = 0
        self.bus.axis_tvalid.value = 0

    async def _driver_send(self, value, sync=True):
        rising_edge = RisingEdge(self.clock)  # make these coroutines once and reuse
        falling_edge = FallingEdge(self.clock)
        read_write = ReadWrite()
        read_only = ReadOnly()  # This is
        _type = value.get("type")
        if _type == "pause":
            await falling_edge
            await read_write
            self.bus.axis_tvalid.value = 0  # set to 0 and be done.
            self.bus.axis_tlast.value = 0  # set to 0 and be done.
            for i in range(value.get("duration", 1)):
                await rising_edge
        elif _type == "write_single":
            contents = value["contents"]
            # keep feeding as long as we don't have a transaction
            await falling_edge
            data = contents["data"]
            self.bus.axis_tdata.value = data
            self.bus.axis_tkeep.value = contents["strb"]
            self.bus.axis_tlast.value = contents["last"]
            self.bus.axis_tvalid.value = 1
            while True:
                await read_only
                if self.bus.axis_tready.value == 1:
                    # await falling_edge
                    # await read_write
                    # self.bus.axis_tvalid.value = 0
                    # self.bus.axis_tlast.value = 0
                    await rising_edge
                    await read_only
                    break
                else:
                    await rising_edge

        elif _type == "write_burst":
            contents = value["contents"]
            # keep feeding as long as we don't have a transaction
            data = contents["data"]
            for i, entry in enumerate(data):
                await self._driver_send(
                    {
                        "type": "write_single",
                        "contents": {"data": int(entry), "last": i == len(data) - 1},
                    }
                )


class S_AXIS_Driver(BusDriver):
    def __init__(self, dut, name, clk):
        self._signals = [
            "axis_tvalid",
            "axis_tready",
            "axis_tlast",
            "axis_tdata",
            "axis_tkeep",
        ]
        AXIS_Driver.__init__(self, dut, name, clk)
        self.bus.axis_tready.value = 0

    async def _driver_send(self, value, sync=True):
        rising_edge = RisingEdge(self.clock)  # make these coroutines once and reuse
        falling_edge = FallingEdge(self.clock)
        read_only = ReadOnly()  # This is
        read_write = ReadWrite()  # This is
        if value.get("type") == "pause":
            await falling_edge
            self.bus.axis_tready.value = 0  # set to 0 and be done.
            for i in range(value.get("duration", 1)):
                await rising_edge
        elif value.get("type") == "read":
            duration = value.get("duration")
            count_beats = 0
            await falling_edge
            self.bus.axis_tready.value = 1
            while True:
                await read_only
                if count_beats < duration:
                    if self.bus.axis_tvalid.value == 1:
                        count_beats += 1
                    await rising_edge
                else:
                    await rising_edge
                    self.bus.axis_tready.value = 0
                    break
        elif value.get("type") == "temp_read":
            duration = value.get("duration")
            count_beats = 0
            await falling_edge
            self.bus.axis_tready.value = 1
            while True:
                await read_only
                if count_beats < duration:
                    count_beats += 1
                    await rising_edge
                else:
                    await rising_edge
                    self.bus.axis_tready.value = 0
                    break

        else:
            raise Exception("Cannot happen")


########################################################################################################
########################################################################################################


########################################################################################################
########################################################################################################


def generate_packet(
    src_mac, dst_mac, src_ip, dst_ip, src_port, dst_port, payload_byte_array
):
    pkt = (
        Ether(dst=dst_mac, src=src_mac)
        / IP(dst=dst_ip, src=src_ip)
        / UDP(dport=dst_port, sport=src_port)
        / Raw(load=bytes(payload_byte_array))
    )
    # binary
    raw_bytes = bytes(pkt)
    return raw_bytes


def construct_axis_frame(raw_bytes, randomized_strb=False):
    # generator that yields AXIS transaction given the raw frame bytes
    # can randomize the strb for all but the first 64-byte chunk if desired
    frame_chunks = 0
    while raw_bytes:
        raw_frame_bits = BinaryValue(n_bits=512)
        # set all bits to 0 initially
        raw_frame_bits.assign(0)
        frame_strb = BinaryValue(n_bits=512 // 8)
        frame_strb.assign(0)

        # fill the 512-bit frame with the first 64 bytes of the raw_bytes
        # note: bit 0 = LSB of tdata[7:0]
        # we need to map word bytes correctly (little-endian per word)

        # frame chunks after the first one get randomized strb
        i = 0
        temp_byte_frame = []
        while i < 64:
            strb_randomization = (
                random.choice([0, 1]) if randomized_strb and frame_chunks > 0 else 1
            )
            if strb_randomization == 1:
                byte = raw_bytes[i] if i < len(raw_bytes) else 0
                strb_randomization = 1 if i < len(raw_bytes) else 0
                frame_strb[i] = strb_randomization
                i += 1
            else:
                byte = random.randint(0, 255)
                frame_strb[i] = strb_randomization

            temp_byte_frame.append(byte)
        frame_chunks += 1
        raw_frame_bits.buff = bytes(temp_byte_frame)
        yield {
            "data": raw_frame_bits,
            "strb": frame_strb,
            "last": 1 if len(raw_bytes) <= 64 else 0,
        }
        if len(raw_bytes) <= 64:
            break
        raw_bytes = raw_bytes[64:]  # move to the next 64 bytes


########################################################################################################
########################################################################################################


async def reset(clk, rst, cycles_held=3, polarity=1):
    rst.value = polarity
    await ClockCycles(clk, cycles_held)
    rst.value = not polarity


########################################################################################################
########################################################################################################

expected_packets = []
current_eframe_bytes = bytearray()

actual_packets = []
current_aframe_bytes = bytearray()

expected_length_match = []
actual_length_match = []


def packet_model(packet_transaction):
    # reconstruct the byte version of a packet given an input transaction
    global expected_packets
    global current_eframe_bytes
    data = packet_transaction["data"]
    # split data into a byte array corresponding to strb
    data = data.buff
    strb = packet_transaction["strb"]
    tlast = packet_transaction["last"]

    # condense all the bytes from tdata and tkeep
    num_bytes = len(strb)
    for i in range(num_bytes):
        if strb[i]:
            current_eframe_bytes.append(data[i])

    if tlast:
        # construct a udp pck from the bytes and extract its payload
        expected_packets.append(Ether(current_eframe_bytes)[UDP].payload)
        current_eframe_bytes = bytearray()


def process_packet(packet, tlast_callback=None):
    # reconstruct the byte version of a packet given an output transaction
    global actual_packets
    global current_aframe_bytes
    data = packet["data"]
    # split data into a byte array corresponding to strb
    data = data.buff
    strb = packet["strb"]
    tlast = packet["last"]

    # condense all the bytes from tdata and tkeep
    num_bytes = len(strb)
    for i in range(num_bytes):
        if strb[i]:
            current_aframe_bytes.append(data[i])

    if tlast:
        # construct a udp pck from the bytes and extract its payload
        actual_packets.append(current_aframe_bytes)
        current_aframe_bytes = bytearray()
        if tlast_callback:
            tlast_callback()


########################################################################################################
########################################################################################################


# @cocotb.test()
async def ethernet_rx_basic_test(dut):
    """Basic test for ethernet_rx module"""

    # setup clock
    clock = Clock(dut.aclk, 4, units="ns")  # 250MHz
    cocotb.start_soon(clock.start())

    # reset
    await reset(dut.aclk, dut.aresetn, cycles_held=10, polarity=0)

    dut._log.info("Reset complete")
    # my config struct definition
    # typedef struct packed {
    #   logic [MAC_ADDR_SIZE-1:0] mac_addr;
    #   logic [31:0]              ip_addr;
    #   logic [15:0]              port;
    # } connection_config_t;

    config_raw_binary = BinaryValue(n_bits=48 + 32 + 16)
    config_raw_binary.buff = (
        b"\x11\x22\x33\x44\x55\x66" + b"\xc0\xa8\x01\x65" + b"\x00\x35"
    )
    dut.my_config.value = config_raw_binary

    def dut_tlast_callback():
        # checking that at the tlast cycle the length check is passing
        assert dut.m_axis_length_valid.value, "Length check failed at tlast"
        dut._log.info("Received packet!")

    # establish monitors on AXIS channels
    inm = AXIS_Monitor(dut, "s", dut.aclk, callback=packet_model)
    outm = AXIS_Monitor(
        dut,
        "m",
        dut.aclk,
        callback=lambda x: process_packet(
            x,
        ),
    )

    dut.m_axis_tready.value = 1  # always ready

    # establish drivers on AXIS channels
    s_axis_driver = M_AXIS_Driver(dut, "s", dut.aclk)
    # m_axis_driver = M_AXIS_Driver(dut, "m", dut.aclk)

    # feed in a dummy input in the following shape
    # // Ethernet Frame:
    # // tdata[ 7:0]   = 0x11   // Dest MAC byte 0
    # // tdata[15:8]   = 0x22
    # // tdata[23:16]  = 0x33
    # // tdata[31:24]  = 0x44
    # // tdata[39:32]  = 0x55
    # // tdata[47:40]  = 0x66

    # // tdata[55:48]  = 0xAA   // Src MAC byte 0
    # // tdata[63:56]  = 0xBB
    # // tdata[71:64]  = 0xCC
    # // tdata[79:72]  = 0xDD
    # // tdata[87:80]  = 0xEE
    # // tdata[95:88]  = 0xFF

    # // tdata[103:96] = 0x08   // Ethertype byte 0
    # // tdata[111:104]= 0x00
    # // tdata[119:112]= 0x45   // IP Version, IHL, TOS
    # // tdata[127:120]= 0x00
    # // tdata[135:128]= 0x00   // Total Length byte 0
    # // tdata[143:136]= 0x2C
    # // tdata[151:144]= 0x1C   // Identification byte 0
    # // tdata[159:152]= 0x46
    # // tdata[167:160]= 0x40   // Flags, Fragment Offset
    # // tdata[175:168]= 0x00
    # // tdata[183:176]= 0x40   // TTL
    # // tdata[191:184]= 0x11
    # // tdata[199:192]= 0xB1   // Header Checksum
    # // tdata[207:200]= 0xE6
    # // tdata[215:208]= 0xC0   // Src IP byte
    # // tdata[223:216]= 0xA8
    # // tdata[231:224]= 0x01
    # // tdata[239:232]= 0x64
    # // tdata[247:240]= 0xC0   // Dest IP byte
    # // tdata[255:248]= 0xA8
    # // tdata[263:256]= 0x01
    # // tdata[271:264]= 0x65
    # // tdata[279:272]= 0x04   // Src Port byte 0
    # // tdata[287:280]= 0xD2
    # // tdata[295:288]= 0x00   // Dest Port byte
    # // tdata[303:296]= 0x35
    # // tdata[311:304]= 0x00   // UDP Length byte
    # // tdata[319:312]= 0x18
    # // tdata[327:320]= 0x00   // UDP Checksum byte
    # // tdata[335:328]= 0x00
    # // tdata[343:336]= 0xDE   // UDP Payload byte
    # // tdata[351:344]= 0xAD
    # .....

    # construct a raw udp packet with a payload of size 10 bytes and make sure the strb accounts for that
    N = 100
    for i in range(N):
        random_payload = []
        payload_size = random.randint(1, 1000)
        for j in range(payload_size):
            random_payload.append(random.randint(0, 255))

        payload_bytes = bytes(random_payload)

        frame_transaction_gen = construct_axis_frame(
            generate_packet(
                src_mac="AA:BB:CC:DD:EE:FF",
                dst_mac="11:22:33:44:55:66",
                src_ip="192.168.1.100",
                dst_ip="192.168.1.101",
                src_port=1234,
                dst_port=53,
                payload_byte_array=payload_bytes,
            )
        )
        for transaction in frame_transaction_gen:
            # send the test frame
            s_axis_driver.append(
                {
                    "type": "write_single",
                    "contents": transaction,
                }
            )
            if random.choice([0, 1, 2]) == 0:
                s_axis_driver.append(
                    {"type": "pause", "duration": random.randint(1, 10)}
                )
    s_axis_driver.append({"type": "pause", "duration": 100})

    await ClockCycles(dut.aclk, 10000)
    print(f"Expected packets: {len(expected_packets)}")
    print(f"Actual packets: {len(actual_packets)}")
    for i in range(len(expected_packets)):
        exp_payload = bytes(expected_packets[i])
        act_payload = bytes(actual_packets[i])
        # dut._log.info(f"Expected packet {i}: {exp_payload.hex()}")
        # dut._log.info(f"Actual packet {i}:   {act_payload.hex()}")
        assert exp_payload == act_payload, f"Packet {i} payload mismatch"


########################################################################################################
########################################################################################################


@cocotb.test()
async def test_with_packets_length_mismatch(dut):
    """Basic test for ethernet_rx module"""

    # setup clock
    clock = Clock(dut.aclk, 4, units="ns")  # 250MHz
    cocotb.start_soon(clock.start())

    global expected_packets
    global actual_packets
    expected_packets = []
    actual_packets = []

    global expected_length_match
    global actual_length_match
    expected_length_match = []
    actual_length_match = []

    # reset
    await reset(dut.aclk, dut.aresetn, cycles_held=10, polarity=0)

    dut._log.info("Reset complete")
    # my config struct definition
    # typedef struct packed {
    #   logic [MAC_ADDR_SIZE-1:0] mac_addr;
    #   logic [31:0]              ip_addr;
    #   logic [15:0]              port;
    # } connection_config_t;

    config_raw_binary = BinaryValue(n_bits=48 + 32 + 16)
    config_raw_binary.buff = (
        b"\x11\x22\x33\x44\x55\x66" + b"\xc0\xa8\x01\x65" + b"\x00\x35"
    )
    dut.my_config.value = config_raw_binary

    def dut_tlast_callback():
        # checking that at the tlast cycle the length check is passing
        # assert dut.m_axis_length_valid.value, "Length check failed at tlast"
        actual_length_match.append(int(dut.m_axis_length_valid.value))
        dut._log.info("Received packet!")

    # establish monitors on AXIS channels
    inm = AXIS_Monitor(dut, "s", dut.aclk, callback=packet_model)
    outm = AXIS_Monitor(
        dut,
        "m",
        dut.aclk,
        callback=lambda x: process_packet(x, dut_tlast_callback),
    )

    dut.m_axis_tready.value = 1  # always ready

    # establish drivers on AXIS channels
    s_axis_driver = M_AXIS_Driver(dut, "s", dut.aclk)

    # construct a raw udp packet with a payload of size 10 bytes and make sure the strb accounts for that
    payload_intended_sizes = []
    N = 11
    for k in range(2):
        for i in range(N):
            random_payload = []
            payload_size = random.randint(100, 1000)
            payload_intended_sizes.append(payload_size)
            for j in range(payload_size):
                random_payload.append(random.randint(0, 255))

            payload_bytes = bytes(random_payload)

            frame_transaction_gen = construct_axis_frame(
                generate_packet(
                    src_mac="AA:BB:CC:DD:EE:FF",
                    dst_mac="11:22:33:44:55:66",
                    src_ip="192.168.1.100",
                    dst_ip="192.168.1.101",
                    src_port=1234,
                    dst_port=53,
                    payload_byte_array=payload_bytes,
                )
            )
            actually_dropped = 0
            drop_transaction = random.choice([0, 1])
            transactions = list(frame_transaction_gen)
            for i, transaction in enumerate(transactions):
                if i != 0 and i != len(transactions) - 1 and drop_transaction:
                    drop_transaction = 0
                    actually_dropped = 1
                    continue
                # send the test frame
                # ensure we're not dropping tlast or first
                s_axis_driver.append(
                    {
                        "type": "write_single",
                        "contents": transaction,
                    }
                )
                if random.choice([0, 1, 2]) == 0:
                    # if 0:
                    s_axis_driver.append(
                        {"type": "pause", "duration": random.randint(1, 10)}
                    )

                expected_length_match.append(int(not actually_dropped))

    s_axis_driver.append({"type": "pause", "duration": 100})

    await ClockCycles(dut.aclk, 10000)
    print(f"Expected packets: {len(expected_packets)}")
    print(f"Actual packets: {len(actual_packets)}")
    for i in range(len(expected_packets)):
        exp_payload = bytes(expected_packets[i])
        act_payload = bytes(actual_packets[i])
        # dut._log.info(f"Expected packet {i}: {exp_payload.hex()}")
        # dut._log.info(f"Actual packet {i}:   {act_payload.hex()}")

        dut._log.info(
            f"Packet {i} expected length: {len(exp_payload)}, actual length: {len(act_payload)}"
        )
        if actual_length_match[i] != (payload_intended_sizes[i] == len(act_payload)):
            dut._log.error(
                f"Length match flag mismatch for packet {i}: expected {expected_length_match[i]}, actual {actual_length_match[i]}"
            )
            dut._log.error(f"Payload intended size: {payload_intended_sizes[i]}")
        assert exp_payload == act_payload, f"Packet {i} payload mismatch"


########################################################################################################
########################################################################################################


def ethernet_rx_tb_runner():

    hdl_toplevel_lang = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    sim = os.getenv("SIM", "icarus")
    proj_path = Path(__file__).resolve().parent.parent
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl"))
    sys.path.append(str(proj_path / "sim"))

    sources = [
        proj_path / "hdl" / "ethernet_rx.sv",
        proj_path / "hdl" / "zeus_rpc.svh",
        proj_path / "hdl" / "pipeline.sv",
        proj_path / "hdl" / "connection_manager.sv",
        proj_path / "hdl" / "bram_wrapper.sv",
    ]
    build_test_args = ["-Wall", f"-I{proj_path / 'hdl'}"]
    parameters = {}
    hdl_toplevel = "ethernet_rx"

    runner = get_runner(sim)
    runner.build(
        sources=sources,
        hdl_toplevel=hdl_toplevel,
        always=True,
        build_args=build_test_args,
        parameters=parameters,
        timescale=("1ns", "1ps"),
        waves=True,
    )
    run_test_args = []
    runner.test(
        hdl_toplevel=hdl_toplevel,
        test_module=test_file,
        test_args=run_test_args,
        waves=True,
    )


if __name__ == "__main__":
    ethernet_rx_tb_runner()
