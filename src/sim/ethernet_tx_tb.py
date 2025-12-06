import cocotb
import os
import random
import sys
import numpy
import logging
import numpy    as np
import math
import random


from math                   import log
from pathlib                import Path

from cocotb.clock           import Clock
from cocotb.triggers        import Timer, ClockCycles, RisingEdge, FallingEdge, ReadOnly, with_timeout
from cocotb.utils           import get_sim_time as gst
# from cocotb.runner          import get_runner
from vicoco.vivado_runner   import get_runner
from cocotb_bus.bus         import Bus
from cocotb_bus.drivers     import BusDriver
from cocotb_bus.monitors    import Monitor
from cocotb_bus.monitors    import BusMonitor
from cocotb_bus.scoreboard  import Scoreboard



# =====================================================================================================================================
# COCOTB INFRA
# =====================================================================================================================================

class AXIS_Monitor(BusMonitor):
    """
    monitors axi streaming bus
    """
    transactions = 0 #use this variable to track good ready/valid handshakes
    def __init__(self, dut, name, clk, callback=None):
        self._signals   = ['axis_tvalid','axis_tready','axis_tlast','axis_tdata','axis_tstrb']
        BusMonitor.__init__(self, dut, name, clk, callback=callback)
        self.clock          = clk
        self.transactions   = 0
        self.dut            = dut

    async def _monitor_recv(self):
        """
        Monitor receiver
        """
        # make these coroutines once and reuse
        rising_edge  = RisingEdge(self.clock)
        falling_edge = FallingEdge(self.clock)
        read_only    = ReadOnly()

        while True:
            #await rising_edge #can either wait for just edge...
            #or you can also wait for falling edge/read_only (see note in lab)

            await falling_edge #sometimes see in AXI shit
            await read_only    #readonly (the postline)
            valid = self.bus.axis_tvalid.value
            ready = self.bus.axis_tready.value
            last = self.bus.axis_tlast.value
            data = self.bus.axis_tdata.value #.signed_integer
            if valid and ready:
                self.transactions+=1
                thing = dict(data=hex(data.integer),last=last,
                             name=self.name,count=self.transactions)
                self.dut._log.info(f"{self.name}: {thing}")
                tdata = int(self.bus.axis_tdata.value)
                tkeep = int(self.bus.axis_tstrb.value)
                tlast = int(self.bus.axis_tlast.value)
                self._recv((tdata, tkeep, tlast))



class AXIS_Driver(BusDriver):
    def __init__(self, dut, name, clk, role="M"):
        self._signals = ['axis_tvalid', 'axis_tready', 'axis_tlast', 'axis_tdata','axis_tstrb']
        BusDriver.__init__(self, dut, name, clk)
        self.clock = clk
        self.dut = dut



class M_AXIS_Driver(AXIS_Driver):
    def __init__(self, dut, name, clk):
        super().__init__(dut,name,clk)
        self.bus.axis_tdata.value = 0
        self.bus.axis_tstrb.value = 0
        self.bus.axis_tlast.value = 0
        self.bus.axis_tvalid.value = 0

    async def _driver_send(self, value, sync=True):

        # make these coroutines once and reuse
        rising_edge     = RisingEdge(self.clock)
        falling_edge    = FallingEdge(self.clock)
        read_only       = ReadOnly()

        if value.get("type") == "pause":
            await falling_edge
            self.bus.axis_tvalid.value  = 0
            self.bus.axis_tlast.value   = 0
            for i in range(value.get("duration",1)):
                await rising_edge

        elif value.get("type") == "write_single":
            await falling_edge
            self.bus.axis_tvalid.value  = 1
            self.bus.axis_tlast.value   = value.get("contents").get("last")
            self.bus.axis_tdata.value   = int(value.get("contents").get("data"))
            self.bus.axis_tstrb.value   = 0xF 
            await read_only
            while (self.bus.axis_tready.value != 1):
                await rising_edge

        elif value.get("type") == "write_burst":

            data_array = value.get("contents").get("data")
            keep_array = value.get("contents").get("keep")

            for i, d in enumerate(data_array):

                # drive at falling edge of the clock
                await falling_edge
                self.bus.axis_tvalid.value  = 1
                self.bus.axis_tlast.value   = 1 if (i == len(data_array)-1) else 0
                self.bus.axis_tdata.value   = int(d)
                self.bus.axis_tstrb.value   = int(keep_array[i])

                # wait until the driven data has taken effect and 
                # check if the ready signal (entering the next raising edge) is on
                await read_only

                while (self.bus.axis_tready.value != 1):
                    await rising_edge
                    # await read_only, adding this is wrong because we want to see if ready- was valid, 
                    #                  not after the module reacts to our valid

        else:
            raise KeyError("invalid command type to master driver")





class S_AXIS_Driver(BusDriver):
    def __init__(self, dut, name, clk):
        self._signals = ['axis_tvalid', 'axis_tready', 'axis_tlast', 'axis_tdata','axis_tstrb']
        AXIS_Driver.__init__(self, dut, name, clk)
        self.bus.axis_tready.value = 0

    async def _driver_send(self, value, sync=True):
        rising_edge     = RisingEdge(self.clock) # make these coroutines once and reuse
        falling_edge    = FallingEdge(self.clock)
        read_only       = ReadOnly() #This is

        if value.get("type") == "pause":
            await falling_edge
            self.bus.axis_tready.value = 0
            for i in range(value.get("duration",1)):
                await rising_edge

        elif value.get("type") == "read":

            await falling_edge
            self.bus.axis_tready.value = 1
            # await read_only # this is useless here

            for i in range(value.get("duration",1)):
                await rising_edge

        else:
            raise KeyError("invalid command type to slave driver")







# =====================================================================================================================================
# PYTHON MODEL (CONNECTION MANAGER)
# =====================================================================================================================================


def hash_fun_ip_port(ip, port):
    """
    Bit-accurate Python version of the SystemVerilog hash function:
    
        key = {ip, port};     // 48 bits
        mix = key[31:0] ^ {key[47:32], key[15:0]};
        mix = mix * 0x9E3779B1;
        return mix[31:16];
    """

    # ------------------------------------------------------------
    # 1. Construct 48-bit key = concat(ip[31:0], port[15:0])
    # ------------------------------------------------------------
    key = ((ip & 0xFFFFFFFF) << 16) | (port & 0xFFFF)
    key &= (1 << 48) - 1   # keep 48 bits

    # ------------------------------------------------------------
    # 2. Extract pieces like in SystemVerilog slicing
    # ------------------------------------------------------------
    low_32     = key & 0xFFFFFFFF                    # key[31:0]
    high_16    = (key >> 32) & 0xFFFF                # key[47:32]
    low_16     = key & 0xFFFF                        # key[15:0]

    # Equivalent to `{key[47:32], key[15:0]}` → 32-bit concat
    folded     = ((high_16 << 16) | low_16) & 0xFFFFFFFF

    # ------------------------------------------------------------
    # 3. XOR fold into 32 bits
    # ------------------------------------------------------------
    mix = (low_32 ^ folded) & 0xFFFFFFFF

    # ------------------------------------------------------------
    # 4. Multiplicative hash (Knuth constant)
    # ------------------------------------------------------------
    mix = (mix * 0x9E3779B1) & 0xFFFFFFFF

    # ------------------------------------------------------------
    # 5. Return upper 16 bits
    # ------------------------------------------------------------
    return (mix >> 16) & 0xFFFF


def generate_collision_entries(num_chains=5, chain_len=5):
    """
    Returns a list of dictionaries.
    Each dictionary looks like:
        { 'ip': <32-bit>, 'port': <16-bit> }

    All IPs are unique. Each group of chain_len entries
    shares the same xor32->16 hash.
    """

    result = []
    used_ips = set()

    for _ in range(num_chains):
        target_hash = random.getrandbits(16)
        chain = []

        while len(chain) < chain_len:
            ip   = random.getrandbits(32)
            port = random.getrandbits(16)
            hash_v = hash_fun_ip_port(ip, port)
            if hash_v == target_hash and (ip, port) not in used_ips:
                entry = {
                    "ip": ip,
                    "port": port,
                }

                chain.append(entry)
                used_ips.add((ip, port))

        result.extend(chain)

    return result


WAYS = 4
TABLE_SIZE = 2**16

my_hash_table_vlds      = [[0] * TABLE_SIZE for _ in range(WAYS)]
my_hash_table_udpPort   = [[0] * TABLE_SIZE for _ in range(WAYS)]
my_hash_table_ipAddr    = [[0] * TABLE_SIZE for _ in range(WAYS)]


wr_sig_in           = []
wr_sig_out_exp      = []
wr_sig_out_act      = []

existing_connection_ids = []


def connection_manager_model_wr(val):
    val = val[0]
    ipAddr  = val & 0xFFFFFFFF
    udpPort = (val>>32) & 0xFFFF
    bind    = (val>>48) & 0x1

    hash_key = hash_fun_ip_port(ipAddr, udpPort)

    wr_sig_in.append({"ipAddr": ipAddr, "udpPort": udpPort, "bind": bind, "hash_key": hash_key})


    if bind:
        inserted = False

        # check if already exists
        for w in range(WAYS):
            if (my_hash_table_vlds[w][hash_key] == 1) and (my_hash_table_ipAddr[w][hash_key] == ipAddr) and (my_hash_table_udpPort[w][hash_key] == udpPort):
                connectionId = (hash_key & 0xFFFF) | (w << 16)
                wr_sig_out_exp.append({"ack": 1, "full": 0, "connectionId":connectionId})
                return

        for w in range(WAYS):
            if my_hash_table_vlds[w][hash_key] == 0:
                connectionId = (hash_key & 0xFFFF) | (w << 16)
                my_hash_table_vlds[w][hash_key] = 1
                my_hash_table_ipAddr[w][hash_key] = ipAddr
                my_hash_table_udpPort[w][hash_key] = udpPort
                inserted = True
                break

        if (inserted):
            existing_connection_ids.append(connectionId)

        expected = {"ack": 1, "full": 0 if inserted else 1, "connectionId": connectionId if inserted else 0}

    else:
        for w in range(WAYS):
            if my_hash_table_vlds[w][hash_key] and (my_hash_table_ipAddr[w][hash_key] == ipAddr) and (my_hash_table_udpPort[w][hash_key] == udpPort):
                my_hash_table_vlds[w][hash_key]     = 0
                my_hash_table_ipAddr[w][hash_key]   = 0
                my_hash_table_udpPort[w][hash_key]  = 0
        expected = {"ack": 1, "full": 0, "connectionId": 0}

    wr_sig_out_exp.append(expected)



def appending_values_wr(val):
    val = val[0]
    ack  = (val >> 18) & 1
    full = (val >> 19) & 1
    connectionId = val & (0x3FFFF)

    wr_sig_out_act.append({"ack": ack, "full": full, "connectionId": connectionId})




# =====================================================================================================================================
# PYTHON MODEL (UDP)
# =====================================================================================================================================

MY_CONFIG_DST_MAC   =   random.getrandbits(48)
MY_CONFIG_SRC_MAC   =   random.getrandbits(48)
MY_CONFIG_SRC_IP    =   random.getrandbits(32)
MY_CONFIG_SRC_PORT  =   random.getrandbits(16)

IP_UDP_DSCP                   = 0
IP_UDP_ENC                    = 0
IP_UDP_IDEN                   = 0
IP_UDP_FLAGS                  = 0
IP_UDP_FRAG_OFFSET            = 0
IP_UDP_TTL                    = 64


udp_sig_out_exp      = []
udp_sig_out_act      = []

def generate_random_packet(existing_connection_ids):
    """
    Returns AXIS beats [(tdata, tkeep, tlast), ...]
    tdata width = 544 bits  (512 payload + 32 connId in MSB of first beat only)
    """

    # ------------------------------------------------------------
    # 1. Generate packet length
    # ------------------------------------------------------------
    n = random.randint(0, 3)
    mode = random.randint(0, 2)

    if mode == 0:
        base_len = random.randrange(8, 176, 8)
    elif mode == 1:
        base_len = 176
    else:
        base_len = random.randrange(184, 513, 8)

    payload_length_bits = base_len + n * 512
    assert(payload_length_bits % 8 == 0)
    payload_length_bytes = payload_length_bits // 8

    # ------------------------------------------------------------
    # 2. Payload bytes (list of ints)
    # ------------------------------------------------------------
    payload = [random.getrandbits(8) for _ in range(payload_length_bytes)]

    # ------------------------------------------------------------
    # 3. Pick connectionId
    # ------------------------------------------------------------
    if existing_connection_ids and random.random() < 0.9:
        connection_id = random.choice(existing_connection_ids)
    else:
        connection_id = random.getrandbits(18)

    # Convert connectionId to 4 big-endian bytes
    # conn_bytes = [
    #     (connection_id >> 24) & 0xFF,
    #     (connection_id >> 16) & 0xFF,
    #     (connection_id >>  8) & 0xFF,
    #     (connection_id >>  0) & 0xFF,
    # ]

    full_packet_bytes = []

    # append connectionId as MSB → LSB (like your IP header builder)
    for i in reversed(range(4)):    # MSB → LSB
        full_packet_bytes.append((connection_id >> (8*i)) & 0xFF)

    # append payload bytes (already LSB-first from RAM)
    full_packet_bytes.extend(payload)

    # ------------------------------------------------------------
    # 5. Slice into AXI beats (512-bit = 64 bytes)
    # ------------------------------------------------------------
    BEAT_BYTES = 64
    beats = []

    total_bytes = len(full_packet_bytes)
    total_beats = (total_bytes + BEAT_BYTES - 1) // BEAT_BYTES

    for beat_idx in range(total_beats):
        start = beat_idx * BEAT_BYTES
        end   = min(start + BEAT_BYTES, total_bytes)

        chunk = full_packet_bytes[start:end]
        chunk_len = len(chunk)

        # ---- assemble tdata ----
        # LSB-first inside the beat (AXI convention)
        tdata = 0
        for i, b in enumerate(chunk):
            tdata |= (b << (8 * i))

        # ---- tkeep ----
        tkeep = (1 << chunk_len) - 1

        # ---- tlast ----
        tlast = (beat_idx == total_beats - 1)

        beats.append((tdata, tkeep, tlast))

    return beats, connection_id, payload_length_bytes+4, full_packet_bytes


def swap_bytes(val, num_bytes):
    out = 0
    for i in range(num_bytes):
        out = (out << 8) | ((val >> (8*i)) & 0xFF)
    return out

def build_ipv4_header(payload_length_bytes, 
                      src_ip, dst_ip,
                      dscp, ecn,
                      identification,
                      flags, frag_offset,
                      ttl, protocol,
                      IP_HEADER_BYTES=20,
                      UDP_HEADER_BYTES=8):


    version_ihl = (4 << 4) | (IP_HEADER_BYTES // 4)
    dscp_ecn    = (dscp << 2) | ecn

    # Total length = header + UDP + payload
    total_len   = payload_length_bytes + IP_HEADER_BYTES + UDP_HEADER_BYTES

    # Flags (3 bits) + Fragment offset (13 bits)
    flags_frag = ((flags & 0x7) << 13) | (frag_offset & 0x1FFF)

    # Build header exactly in the same field order as the Verilog
    ip_header = 0
    ip_header |= version_ihl
    ip_header |= (dscp_ecn << 8)
    ip_header |= (swap_bytes(total_len, 2) << 16)
    ip_header |= (swap_bytes(identification, 2) << 32)
    ip_header |= (swap_bytes(flags_frag, 2) << 48)
    ip_header |= (ttl << 64)
    ip_header |= (protocol << 72)
    ip_header |= (0 << 80)
    ip_header |= (swap_bytes(src_ip, 4) << 96)
    ip_header |= (swap_bytes(dst_ip, 4) << 128)

    return ip_header


def model_udp(connection_id, payload_length_bytes, payload_bytes, MY_CONFIG_SRC_MAC, MY_CONFIG_SRC_IP, MY_CONFIG_SRC_PORT, MY_CONFIG_DST_MAC):

    hash_key     = connection_id & 0xFFFF
    hash_way     = connection_id >> 16

    # If not valid — drop packet
    if my_hash_table_vlds[hash_way][hash_key] == 0:
        print("packet dropped by sw")
        return

    dst_ipAddr  = my_hash_table_ipAddr[hash_way][hash_key]
    dst_udpPort = my_hash_table_udpPort[hash_way][hash_key]

    ETHTYPE_IP       = 0x0800
    IP_HEADER_BYTES  = 20
    UDP_HEADER_BYTES = 8
    IPPROTO_UDP      = 17

    # ======================================================
    # Build Ethernet Header (big-endian)
    # ======================================================
    ethernet_header = (
        (swap_bytes(ETHTYPE_IP, 2) << (2*48))   |
        (swap_bytes(MY_CONFIG_SRC_MAC, 6) << 48)    |
        swap_bytes(MY_CONFIG_DST_MAC, 6)
    )

    # ======================================================
    # Build IPv4 Header (exact RTL layout)
    # ======================================================
    ip_header = build_ipv4_header(
        payload_length_bytes          = payload_length_bytes,
        src_ip                        = MY_CONFIG_SRC_IP,
        dst_ip                        = dst_ipAddr,
        dscp                          = IP_UDP_DSCP,
        ecn                           = IP_UDP_ENC,
        identification                = IP_UDP_IDEN,
        flags                         = IP_UDP_FLAGS,
        frag_offset                   = IP_UDP_FRAG_OFFSET,
        ttl                           = IP_UDP_TTL,
        protocol                      = IPPROTO_UDP,
        IP_HEADER_BYTES               = IP_HEADER_BYTES,
        UDP_HEADER_BYTES              = UDP_HEADER_BYTES
    )

    # ======================================================
    # Build UDP Header
    # ======================================================
    udp_header = (
        (swap_bytes(0, 2) << 48) |
        (swap_bytes(payload_length_bytes + UDP_HEADER_BYTES, 2)  << 32)   |
        (swap_bytes(dst_udpPort, 2) << 16) |
        swap_bytes(MY_CONFIG_SRC_PORT, 2)   # checksum (unused in IPv4)
    )

    # ======================================================
    # Combine headers: 14 + 20 + 8 = 42 bytes
    # ======================================================
    ALL_HDR_BYTES = 42
    ALL_HDR_BITS  = ALL_HDR_BYTES * 8

    all_headers = (
        (udp_header << (20 + 14) * 8) |
        (ip_header  << (14 * 8))      |
        ethernet_header
    )

    # ======================================================
    # Build FINAL PACKET = headers || payload
    # ======================================================
    full_packet_bytes = []

    # append header bytes MSB → LSB
    for i in range(ALL_HDR_BYTES):
        full_packet_bytes.append((all_headers >> (8*i)) & 0xFF)

    # append payload (already LSB-first chunks)
    full_packet_bytes.extend(payload_bytes)

    # ======================================================
    # Now resplit into AXIS beats (512-bit = 64 bytes)
    # ======================================================
    BEAT_BYTES = 64
    num_beats = (len(full_packet_bytes) + BEAT_BYTES - 1) // BEAT_BYTES

    for b in range(num_beats):
        start = b * BEAT_BYTES
        end   = min(start + BEAT_BYTES, len(full_packet_bytes))
        chunk = full_packet_bytes[start:end]

        # Build tdata (LSB-first)
        tdata = 0
        for i, byte in enumerate(chunk):
            tdata |= (byte << (8*i))

        # tkeep
        tkeep = (1 << len(chunk)) - 1

        # last beat?
        tlast = 1 if (b == num_beats - 1) else 0

        udp_sig_out_exp.append((hex(tdata), hex(tkeep), hex(tlast)))

def appending_values_udp(val):
    d = val[0]
    k = val[1]
    l = val[2]
    udp_sig_out_act.append((hex(d), hex(k), hex(l)))






# =====================================================================================================================================
# TESTING LOGIC
# =====================================================================================================================================

async def reset(clk,rst, cycles_held = 3,polarity=1):
    rst.value = polarity
    await ClockCycles(clk, cycles_held)
    rst.value = not polarity


async def test_structure(dut, NUM_TEST_PACKETS = 100, WR_NUM_OPERATIONS = 300, NUM_CHAINS=128, CHAIN_LEN=8):
 
    wr_in_monitor       = AXIS_Monitor(dut,'s02',dut.s00_axis_aclk,callback = connection_manager_model_wr)
    wr_out_monitor      = AXIS_Monitor(dut,'m02',dut.s00_axis_aclk,callback = lambda x: appending_values_wr(x))

    wr_in_driver        = M_AXIS_Driver(dut,'s02',dut.s00_axis_aclk)
    wr_out_driver       = S_AXIS_Driver(dut,'m02',dut.s00_axis_aclk)

    udp_in_monitor       = AXIS_Monitor(dut,'s00',dut.s00_axis_aclk, callback = lambda x: None)
    udp_out_monitor      = AXIS_Monitor(dut,'m00',dut.s00_axis_aclk, callback = lambda x: appending_values_udp(x))

    udp_in_driver        = M_AXIS_Driver(dut,'s00',dut.s00_axis_aclk)
    udp_out_driver       = S_AXIS_Driver(dut,'m00',dut.s00_axis_aclk)

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    await ClockCycles(dut.s00_axis_aclk, 3)
    dut.tx_engine_enable.value = 1
    await ClockCycles(dut.s00_axis_aclk, 3)

    dut.my_config_dst_macAddr.value = MY_CONFIG_DST_MAC
    dut.my_config_src_macAddr.value = MY_CONFIG_SRC_MAC
    dut.my_config_src_ipAddr.value  = MY_CONFIG_SRC_IP
    dut.my_config_src_udpPort.value = MY_CONFIG_SRC_PORT

    await ClockCycles(dut.s00_axis_aclk, 3)

    #
    # ----------------------------- MAIN TEST ----------------------------------
    #

    COLLISION_POOL = list(generate_collision_entries(NUM_CHAINS, CHAIN_LEN))

    async def writer_thread(num_operations):

        for _ in range(num_operations):

            pick = random.choice(COLLISION_POOL)
            bind = (random.random() < 0.9)

            val  = 0
            val  |= (pick['ip'])
            val  |= (pick['port'] << 32)
            val  |= (bind         << 48)

            wr_in_driver.append({'type':'write_single', "contents":{"data": val, "last":1}})
            wr_in_driver.append({"type":"pause", "duration": random.randint(0,3)})


    #
    # -------------------- Pre-fill connection manager -------------------------
    #
    cocotb.start_soon(writer_thread(WR_NUM_OPERATIONS))
    wr_out_driver.append({'type':'read', "duration": WR_NUM_OPERATIONS + 5000})
    await ClockCycles(dut.s00_axis_aclk, WR_NUM_OPERATIONS + 5000)


    assert wr_in_monitor.transactions == wr_out_monitor.transactions,       f"WR transaction count mismatch!"
    assert len(wr_sig_in) == len(wr_sig_out_exp) == len(wr_sig_out_act), "WR bookkeeping mismatch!"
    for idx, (sig_in, expected, actual) in enumerate(zip(wr_sig_in, wr_sig_out_exp, wr_sig_out_act)):
        if expected != actual:
            raise RuntimeError(
                f"ERROR:    WR mismatch {idx}: ipAddr=0x{hex(sig_in['ipAddr'])} "
                f"hash=0x{hex(sig_in['hash_key'])} "
                f"bind={sig_in['bind']}"
                f"expected {expected}, got {actual}"
            )

    #
    # ------------------------------- UDP Packets  ---------------------------------
    #

    for _ in range(NUM_TEST_PACKETS):
        beats, connection_id, payload_length_bytes, payload = generate_random_packet(existing_connection_ids)
        print(f"packet with payload bytes length = {payload_length_bytes}, connectionId = {connection_id}")
        model_udp(connection_id, payload_length_bytes, payload, MY_CONFIG_SRC_MAC, MY_CONFIG_SRC_IP, MY_CONFIG_SRC_PORT, MY_CONFIG_DST_MAC)

        udp_in_driver.append({'type':'write_burst', "contents": {"data": [tdata for (tdata, tkeep, tlast) in beats],
                                                                 "keep": [tkeep for (tdata, tkeep, tlast) in beats]}})
        
        if (random.random() < 0.9):
            continue
        udp_in_driver.append({"type":"pause", "duration": random.randint(0,10)})

    udp_in_driver.append({"type":"pause", "duration": 5})
    udp_out_driver.append({'type':'read', "duration": NUM_TEST_PACKETS * 100})
    await ClockCycles(dut.s00_axis_aclk, NUM_TEST_PACKETS * 100)

    #
    # ------------------------------- Validation  ---------------------------------
    #

    assert len(udp_sig_out_exp) == len(udp_sig_out_act), f"UDP count mismatch! len(udp_sig_out_exp) = {len(udp_sig_out_exp)}, len(udp_sig_out_act) = {len(udp_sig_out_act)}" 

    for i, (exp, act) in enumerate(zip(udp_sig_out_exp, udp_sig_out_act)):
        if exp != act:
            raise RuntimeError(f"UDP mismatch at beat {i}: expected {exp}, got {act}")

    udp_sig_out_exp.clear()
    udp_sig_out_act.clear()


# =====================================================================================================================================
# TEST CASES
# =====================================================================================================================================


@cocotb.test()
async def test_1(dut):
    await test_structure(dut, NUM_TEST_PACKETS = 500, WR_NUM_OPERATIONS = 300, NUM_CHAINS=128, CHAIN_LEN=8)



# =====================================================================================================================================
# TEST RUNNER
# =====================================================================================================================================


def ethernet_tx_tb_runner():

    hdl_toplevel_lang   = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    #sim                 = os.getenv("SIM", "icarus")
    sim                 = os.getenv("SIM", "vivado")
    test_file           = os.path.basename(__file__).replace(".py","")
    proj_path           = Path(__file__).resolve().parent

    sys.path.append(str(proj_path / "model"))

    sources             = [proj_path / ".."/ "hdl" / "connection_manager.sv",
                           proj_path / ".."/ "hdl" /  "bram_wrapper.sv",
                           proj_path / ".."/ "hdl" /  "fifo_axis_wrapper.sv",
                           proj_path / ".."/ "hdl" /  "ethernet_tx.sv",
                           proj_path / ".."/ "hdl" /  "tx_headers_prepend.sv",
                           proj_path / ".."/ "hdl" /  "ethernet_tx_wrapper.sv",
                           "/tools/Xilinx/2025.1/Vivado/data/verilog/src/glbl.v"
                        ]

    build_test_args     = ["-L", "xpm"]
    parameters          = {}
    hdl_toplevel        = "ethernet_tx_wrapper"

    runner              = get_runner(sim,  extra_global_modules=["work.glbl"])
    runner.build(
        sources         =   sources,
        hdl_toplevel    =   hdl_toplevel,
        always          =   True,
        build_args      =   build_test_args,
        parameters      =   parameters,
        timescale       =   ('1ns','1ps'),
        waves           =   True,
    )
    run_test_args       = []
    runner.test(
        hdl_toplevel    =   hdl_toplevel,
        test_module     =   test_file,
        test_args       =   run_test_args,
        waves           =   True
    )


if __name__ == "__main__":
    ethernet_tx_tb_runner()
