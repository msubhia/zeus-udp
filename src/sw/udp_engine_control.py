import numpy as np
import random
import time
import pprint

from pynq import PL
from pynq import Overlay
from pynq import Clocks
from pynq import allocate



# ======================================================================================================
# ======================================================================================================


class connection_manager_sw:

    def __init__(self, WAYS=4, HASH_WIDTH=16):
        self.WAYS = WAYS
        self.HASH_WIDTH = HASH_WIDTH
        self.TABLE_SIZE = 1 << HASH_WIDTH

        self.my_hash_table_vlds    = [[0] * self.TABLE_SIZE for _ in range(WAYS)]
        self.my_hash_table_macAddr = [[0] * self.TABLE_SIZE for _ in range(WAYS)]
        self.my_hash_table_udpPort = [[0] * self.TABLE_SIZE for _ in range(WAYS)]
        self.my_hash_table_ipAddr  = [[0] * self.TABLE_SIZE for _ in range(WAYS)]
        self.existing_connection_ids = []

    @staticmethod
    def xor32to16(x):
        x &= 0xFFFFFFFF
        return ((x >> 16) & 0xFFFF) ^ (x & 0xFFFF)

    def write(self, macAddr, ipAddr, udpPort, bind):

        tag = ipAddr
        hash_key = self.xor32to16(ipAddr) & ((1 << self.HASH_WIDTH) - 1)

        if bind:
            # already exists?
            for w in range(self.WAYS):
                if (self.my_hash_table_vlds[w][hash_key] and
                    self.my_hash_table_ipAddr[w][hash_key] == tag):

                    connectionId = (w << self.HASH_WIDTH) | hash_key
                    return {"ack": 1, "full": 0, "connectionId": connectionId}

            # find free way
            for w in range(self.WAYS):
                if not self.my_hash_table_vlds[w][hash_key]:

                    connectionId = (w << self.HASH_WIDTH) | hash_key

                    self.my_hash_table_vlds[w][hash_key]    = 1
                    self.my_hash_table_ipAddr[w][hash_key]  = tag
                    self.my_hash_table_macAddr[w][hash_key] = macAddr
                    self.my_hash_table_udpPort[w][hash_key] = udpPort

                    self.existing_connection_ids.append(connectionId)
                    return {"ack": 1, "full": 0, "connectionId": connectionId}

            return {"ack": 1, "full": 1, "connectionId": 0}

        else:
            # unbind
            for w in range(self.WAYS):
                if (self.my_hash_table_vlds[w][hash_key] and
                    self.my_hash_table_ipAddr[w][hash_key] == tag):

                    self.my_hash_table_vlds[w][hash_key] = 0
                    self.my_hash_table_ipAddr[w][hash_key] = 0
                    self.my_hash_table_macAddr[w][hash_key] = 0
                    self.my_hash_table_udpPort[w][hash_key] = 0

                    connectionId = (w << self.HASH_WIDTH) | hash_key
                    if connectionId in self.existing_connection_ids:
                        self.existing_connection_ids.remove(connectionId)

            return {"ack": 1, "full": 0, "connectionId": 0}


    def read_fw(self, ipAddr):
        hash_key = self.xor32to16(ipAddr) & ((1 << self.HASH_WIDTH) - 1)

        for w in range(self.WAYS):
            if (self.my_hash_table_vlds[w][hash_key] and
                self.my_hash_table_ipAddr[w][hash_key] == ipAddr):

                connectionId = (w << self.HASH_WIDTH) | hash_key
                return {"hit": 1, "connectionId": connectionId}

        return {"hit": 0, "connectionId": 0}


    def read_rv(self, connectionId):

        mask = (1 << self.HASH_WIDTH) - 1
        hash_key = connectionId & mask
        hash_way = connectionId >> self.HASH_WIDTH

        if self.my_hash_table_vlds[hash_way][hash_key]:
            return {
                "hit": 1,
                "ipAddr": self.my_hash_table_ipAddr[hash_way][hash_key],
                "macAddr": self.my_hash_table_macAddr[hash_way][hash_key],
                "udpPort": self.my_hash_table_udpPort[hash_way][hash_key],
            }

        return {"hit": 0, "ipAddr": 0, "macAddr": 0, "udpPort": 0}



# ======================================================================================================
# ======================================================================================================


class udp_engine_controller:
    addr_csr_udp_engine_100g__ctrl                         = 0x00

    addr_csr_udp_engine_100g__myConfig_macAddr_upper       = 0x04
    addr_csr_udp_engine_100g__myConfig_macAddr_lower       = 0x08
    addr_csr_udp_engine_100g__myConfig_ipAddr              = 0x0C
    addr_csr_udp_engine_100g__myConfig_udpPort             = 0x10

    addr_csr_udp_engine_100g__connManager_wr_macAddr_upper = 0x14
    addr_csr_udp_engine_100g__connManager_wr_macAddr_lower = 0x18
    addr_csr_udp_engine_100g__connManager_wr_ip_addr       = 0x1C
    addr_csr_udp_engine_100g__connManager_wr_port          = 0x20
    addr_csr_udp_engine_100g__connManager_wr_bind          = 0x24
    addr_csr_udp_engine_100g__connManager_wr_trigger       = 0x28
    addr_csr_udp_engine_100g__connManager_wr_status        = 0x2C
    addr_csr_udp_engine_100g__connManager_wr_connectedId   = 0x30


    def __init__(self, udp_mmio, connection_manager):
        self.udp_mmio           = udp_mmio
        self.connection_manager = connection_manager
        self.HASH_WIDTH         = connection_manager.HASH_WIDTH
        self.TABLE_SIZE         = connection_manager.TABLE_SIZE


    def write_confirmed(self, addr, value):
        self.udp_mmio.write(addr, value)
        readback = self.udp_mmio.read(addr)
        if readback != value:
            print(f"ERROR: Write confirmation failed at 0x{addr:X}: wrote 0x{value:X}, read back 0x{readback:X}")
        return readback


    def tx_enable(self):
        old = self.udp_mmio.read(self.addr_csr_udp_engine_100g__ctrl)
        old |= 1
        self.write_confirmed(self.addr_csr_udp_engine_100g__ctrl, old)


    def tx_disable(self):
        old = self.udp_mmio.read(self.addr_csr_udp_engine_100g__ctrl)
        old &= ~1
        self.write_confirmed(self.addr_csr_udp_engine_100g__ctrl, old)

    def configure(self, macAddr, ipAddr, udpPort):
        self.tx_disable()
        self.write_confirmed(self.addr_csr_udp_engine_100g__myConfig_macAddr_upper, (macAddr >> 32) & 0xFFFFFFFF)
        self.write_confirmed(self.addr_csr_udp_engine_100g__myConfig_macAddr_lower, macAddr & 0xFFFFFFFF)
        self.write_confirmed(self.addr_csr_udp_engine_100g__myConfig_ipAddr, ipAddr)
        self.write_confirmed(self.addr_csr_udp_engine_100g__myConfig_udpPort, udpPort)
        self.tx_enable()

    def bind_connection(self, macAddr, ipAddr, udpPort):
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_macAddr_upper, (macAddr >> 32) & 0xFFFFFFFF)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_macAddr_lower, macAddr & 0xFFFFFFFF)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_ip_addr, ipAddr)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_port, udpPort)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_bind, 1)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_trigger, 1)

        time.sleep(0.5)
        tmp = self.udp_mmio.read(self.addr_csr_udp_engine_100g__connManager_wr_status)
        ack = tmp & 0x1
        full = (tmp >> 1) & 0x1
        connectionId = self.udp_mmio.read(self.addr_csr_udp_engine_100g__connManager_wr_connectedId)

        actual = {"ack": ack, "full": full, "connectionId": connectionId}
        expected = self.connection_manager.write(macAddr, ipAddr, udpPort, bind=1)

        if expected != actual:
            print(f"ERROR: expected = {expected}, actual = {actual}")

        return actual

    def unbind_connection(self, macAddr, ipAddr, udpPort):
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_macAddr_upper, (macAddr >> 32) & 0xFFFFFFFF)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_macAddr_lower, macAddr & 0xFFFFFFFF)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_ip_addr, ipAddr)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_port, udpPort)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_bind, 0)
        self.write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_trigger, 1)
        
        time.sleep(0.5)

        tmp = self.udp_mmio.read(self.addr_csr_udp_engine_100g__connManager_wr_status)
        ack = tmp & 0x1
        full = (tmp >> 1) & 0x1
        connectionId = self.udp_mmio.read(self.addr_csr_udp_engine_100g__connManager_wr_connectedId)

        actual = {"ack": ack, "full": full, "connectionId": connectionId}
        expected = self.connection_manager.write(macAddr, ipAddr, udpPort, bind=0)

        if expected != actual:
            print(f"ERROR: expected = {expected}, actual = {actual}")

        return actual



# ======================================================================================================
# ======================================================================================================


def generate_collision_entries(num_chains=5, chain_len=5):
    """
    Returns a list of dictionaries.
    Each dictionary looks like:
        { 'ip': <32-bit>, 'mac': <48-bit>, 'port': <16-bit> }

    All IPs are unique. Each group of chain_len entries
    shares the same xor32->16 hash.
    """

    result = []
    used_ips = set()

    for _ in range(num_chains):
        target_hash = random.getrandbits(16)
        chain = []

        while len(chain) < chain_len:
            ip = random.getrandbits(32)

            if xor32to16(ip) == target_hash and ip not in used_ips:
                entry = {
                    "ip": ip,
                    "mac": random.getrandbits(48),   # 48-bit MAC
                    "port": random.getrandbits(16),  # 16-bit UDP port
                }

                chain.append(entry)
                used_ips.add(ip)

        result.extend(chain)

    return result





# # ======================================================================================================
# # ======================================================================================================
# class udp_model:

#     IP_UDP_DSCP         = 0
#     IP_UDP_ENC          = 0
#     IP_UDP_IDEN         = 0
#     IP_UDP_FLAGS        = 0
#     IP_UDP_FRAG_OFFSET  = 0
#     IP_UDP_TTL          = 64


#     def __init__(self):
#         pass


#     def swap_bytes(val, num_bytes):
#         out = 0
#         for i in range(num_bytes):
#             out = (out << 8) | ((val >> (8*i)) & 0xFF)
#         return out


#     def generate_random_packet(existing_connection_ids):
#         """
#         Returns AXIS beats [(tdata, tkeep, tlast), ...]
#         tdata width = 544 bits  (512 payload + 32 connId in MSB of first beat only)
#         """

#         # ------------------------------------------------------------
#         # 1. Generate packet length
#         # ------------------------------------------------------------
#         n = random.randint(0, 3)
#         mode = random.randint(0, 2)

#         if mode == 0:
#             base_len = random.randrange(8, 176, 8)
#         elif mode == 1:
#             base_len = 176
#         else:
#             base_len = random.randrange(184, 513, 8)

#         payload_length_bits  = base_len + n * 512
#         payload_length_bytes = payload_length_bits // 8

#         # ------------------------------------------------------------
#         # 2. Payload bytes (list of ints)
#         # ------------------------------------------------------------
#         payload = [random.getrandbits(8) for _ in range(payload_length_bytes)]

#         # ------------------------------------------------------------
#         # 3. Pick connectionId
#         # ------------------------------------------------------------
#         if existing_connection_ids and random.random() < 0.9:
#             connection_id = random.choice(existing_connection_ids)
#         else:
#             connection_id = random.getrandbits(18)

#         # Convert connectionId to 4 big-endian bytes
#         conn_bytes = [
#             (connection_id >> 24) & 0xFF,
#             (connection_id >> 16) & 0xFF,
#             (connection_id >>  8) & 0xFF,
#             (connection_id >>  0) & 0xFF,
#         ]

#         # ------------------------------------------------------------
#         # 4. Slice payload into 64-byte beats
#         # ------------------------------------------------------------
#         BEAT_BYTES = 64
#         beats = []

#         total_beats = (payload_length_bytes + BEAT_BYTES - 1) // BEAT_BYTES

#         for beat_idx in range(total_beats):
#             start = beat_idx * BEAT_BYTES
#             end   = min(start + BEAT_BYTES, payload_length_bytes)

#             chunk = payload[start:end]
#             chunk_len = len(chunk)

#             # ---- tdata assembly (little-endian inside beat) ----
#             tdata = 0

#             # LSBs = payload bytes (little-endian)
#             for i, b in enumerate(chunk):
#                 tdata |= (b << (8 * i))

#             # First beat: add connId in MSB 32 bits
#             if beat_idx == 0:
#                 # shift payload up by 32 bytes? No — connId is above the payload.
#                 tdata |= (
#                     conn_bytes[3] << 512 |
#                     conn_bytes[2] << 520 |
#                     conn_bytes[1] << 528 |
#                     conn_bytes[0] << 536
#                 )

#             # ---- tkeep assembly ----
#             # Total bytes = 64 payload + 4 connId = 68
#             # But connId valid only on beat 0
#             byte_count = chunk_len
#             tkeep = (1 << byte_count) - 1

#             # ---- tlast ----
#             tlast = (beat_idx == total_beats - 1)

#             beats.append((tdata, tkeep, tlast))

#         return beats, connection_id, payload_length_bytes, payload



#     def build_ipv4_header(payload_length_bytes, 
#                       src_ip, dst_ip,
#                       dscp, ecn,
#                       identification,
#                       flags, frag_offset,
#                       ttl, protocol,
#                       IP_HEADER_BYTES=20,
#                       UDP_HEADER_BYTES=8):

#         version_ihl = (4 << 4) | (IP_HEADER_BYTES // 4)
#         dscp_ecn    = (dscp << 2) | ecn

#         # Total length = header + UDP + payload
#         total_len   = payload_length_bytes + IP_HEADER_BYTES + UDP_HEADER_BYTES

#         # Flags (3 bits) + Fragment offset (13 bits)
#         flags_frag = ((flags & 0x7) << 13) | (frag_offset & 0x1FFF)

#         # Build header exactly in the same field order as the Verilog
#         ip_header = 0
#         ip_header |= version_ihl
#         ip_header |= (dscp_ecn << 8)
#         ip_header |= (swap_bytes(total_len, 2) << 16)
#         ip_header |= (swap_bytes(identification, 2) << 32)
#         ip_header |= (swap_bytes(flags_frag, 2) << 48)
#         ip_header |= (ttl << 64)
#         ip_header |= (protocol << 72)
#         ip_header |= (0 << 80)
#         ip_header |= (swap_bytes(src_ip, 4) << 96)
#         ip_header |= (swap_bytes(dst_ip, 4) << 128)

#         return ip_header

#     def model_udp_tx(connection_id, payload_length_bytes, payload_bytes, MY_CONFIG_MAC, MY_CONFIG_IP, MY_CONFIG_PORT):














# def model_udp(connection_id, payload_length_bytes, payload_bytes, MY_CONFIG_MAC, MY_CONFIG_IP, MY_CONFIG_PORT):

#     hash_key     = connection_id & 0xFFFF
#     hash_way     = connection_id >> 16

#     # If not valid — drop packet
#     if my_hash_table_vlds[hash_way][hash_key] == 0:
#         print("packet dropped by sw")
#         return

#     dst_ipAddr  = my_hash_table_ipAddr[hash_way][hash_key]
#     dst_macAddr = my_hash_table_macAddr[hash_way][hash_key]
#     dst_udpPort = my_hash_table_udpPort[hash_way][hash_key]

#     ETHTYPE_IP       = 0x0800
#     IP_HEADER_BYTES  = 20
#     UDP_HEADER_BYTES = 8
#     IPPROTO_UDP      = 17

#     # ======================================================
#     # Build Ethernet Header (big-endian)
#     # ======================================================
#     ethernet_header = (
#         (swap_bytes(ETHTYPE_IP, 2) << (2*48))   |
#         (swap_bytes(MY_CONFIG_MAC, 6) << 48)    |
#         swap_bytes(dst_macAddr, 6)
#     )

#     # ======================================================
#     # Build IPv4 Header (exact RTL layout)
#     # ======================================================
#     ip_header = build_ipv4_header(
#         payload_length_bytes          = payload_length_bytes,
#         src_ip                        = MY_CONFIG_IP,
#         dst_ip                        = dst_ipAddr,
#         dscp                          = IP_UDP_DSCP,
#         ecn                           = IP_UDP_ENC,
#         identification                = IP_UDP_IDEN,
#         flags                         = IP_UDP_FLAGS,
#         frag_offset                   = IP_UDP_FRAG_OFFSET,
#         ttl                           = IP_UDP_TTL,
#         protocol                      = IPPROTO_UDP,
#         IP_HEADER_BYTES               = IP_HEADER_BYTES,
#         UDP_HEADER_BYTES              = UDP_HEADER_BYTES
#     )

#     # ======================================================
#     # Build UDP Header
#     # ======================================================
#     udp_header = (
#         (swap_bytes(0, 2) << 48) |
#         (swap_bytes(payload_length_bytes + UDP_HEADER_BYTES, 2)  << 32)   |
#         (swap_bytes(dst_udpPort, 2) << 16) |
#         swap_bytes(MY_CONFIG_PORT, 2)   # checksum (unused in IPv4)
#     )

#     # ======================================================
#     # Combine headers: 14 + 20 + 8 = 42 bytes
#     # ======================================================
#     ALL_HDR_BYTES = 42
#     ALL_HDR_BITS  = ALL_HDR_BYTES * 8

#     all_headers = (
#         (udp_header << (20 + 14) * 8) |
#         (ip_header  << (14 * 8))      |
#         ethernet_header
#     )

#     # ======================================================
#     # Build FINAL PACKET = headers || payload
#     # ======================================================
#     full_packet_bytes = []

#     # append header bytes MSB → LSB
#     for i in range(ALL_HDR_BYTES):
#         full_packet_bytes.append((all_headers >> (8*i)) & 0xFF)

#     # append payload (already LSB-first chunks)
#     full_packet_bytes.extend(payload_bytes)

#     # ======================================================
#     # Now resplit into AXIS beats (512-bit = 64 bytes)
#     # ======================================================
#     BEAT_BYTES = 64
#     num_beats = (len(full_packet_bytes) + BEAT_BYTES - 1) // BEAT_BYTES

#     for b in range(num_beats):
#         start = b * BEAT_BYTES
#         end   = min(start + BEAT_BYTES, len(full_packet_bytes))
#         chunk = full_packet_bytes[start:end]

#         # Build tdata (LSB-first)
#         tdata = 0
#         for i, byte in enumerate(chunk):
#             tdata |= (byte << (8*i))

#         # tkeep
#         tkeep = (1 << len(chunk)) - 1

#         # last beat?
#         tlast = 1 if (b == num_beats - 1) else 0

#         udp_sig_out_exp.append((hex(tdata), hex(tkeep), hex(tlast)))



