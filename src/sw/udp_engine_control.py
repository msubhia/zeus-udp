"""
FPGA UDP Engine 100g Controller and Model

This module provides software models and hardware controllers for managing UDP connections
and packet processing on PYNQ FPGA platforms. It includes:
- Connection management with hash-based lookup tables
- Hardware register control via MMIO
- UDP packet generation and modeling

Authors:          M.Subhi Abordan (msubhi_a@mit.edu)
                  Mena Filfil     (menaf@mit.edu)
Last Modified:    Dec 5, 2025
"""

import random
import time

from pynq import PL
from pynq import Overlay
from pynq import Clocks
from pynq import allocate


# ======================================================================================================
# CONNECTION MANAGER - Software Model
# ======================================================================================================


class connection_manager_sw:
    """
    Software model of a hardware connection manager using a set-associative hash table.

    This class maintains UDP connection mappings (IP address, port) and assigns unique
    connection IDs. It supports bind/unbind operations and collision handling through
    a multi-way associative structure.

    Attributes:
        WAYS: Number of ways in the set-associative hash table (default: 4)
        HASH_WIDTH             :  Bit width of the hash key (default: 16)
        TABLE_SIZE             :  Total entries per way (2^HASH_WIDTH)
        my_hash_table_vlds      (list): Valid bits for each entry
        my_hash_table_udpPort   (list): UDP port numbers stored in table
        my_hash_table_ipAddr    (list): IP addresses stored in table
        existing_connection_ids (list): List of currently active connection IDs
    """

    def __init__(self, WAYS=4, HASH_WIDTH=16):
        self.WAYS = WAYS
        self.HASH_WIDTH = HASH_WIDTH
        self.TABLE_SIZE = 1 << HASH_WIDTH

        self.my_hash_table_vlds = [[0] * self.TABLE_SIZE for _ in range(WAYS)]
        self.my_hash_table_udpPort = [[0] * self.TABLE_SIZE for _ in range(WAYS)]
        self.my_hash_table_ipAddr = [[0] * self.TABLE_SIZE for _ in range(WAYS)]
        self.existing_connection_ids = []

    @staticmethod
    def _hash_fun_ip_port(ip, port):
        """
        Bit-accurate Python version of the simplified XOR hash:

            key  = {ip, port};  // 48 bits
            hash = key[15:0] ^ key[31:16] ^ {8'b0, key[47:40]};
        """

        # ------------------------------------------------------------
        # 1. Construct 48-bit key = concat(ip[31:0], port[15:0])
        # ------------------------------------------------------------
        key = ((ip & 0xFFFFFFFF) << 16) | (port & 0xFFFF)
        key &= (1 << 48) - 1  # keep 48 bits

        # ------------------------------------------------------------
        # 2. Extract pieces like SystemVerilog slices
        # ------------------------------------------------------------
        low_16 = (key >> 0) & 0xFFFF  # key[15:0]
        mid_16 = (key >> 16) & 0xFFFF  # key[31:16]
        top_8 = (key >> 40) & 0xFF  # key[47:40]

        # Equivalent to {8'b0, key[47:40]}
        top_16 = top_8  # zero-extended in top 8 bits

        # ------------------------------------------------------------
        # 3. XOR-fold into 16 bits
        # ------------------------------------------------------------
        hash16 = (low_16 ^ mid_16 ^ top_16) & 0xFFFF

        return hash16

    def write(self, ipAddr, udpPort, bind):
        """
        Bind or unbind a UDP connection in the hash table.

        When binding, this method:
        1. Checks if the connection already exists (returns existing ID)
        2. Finds an empty slot in one of the ways
        3. Inserts the connection and returns a new connection ID

        When unbinding, it removes the matching connection from the table.

        Args:
            ipAddr:     32-bit destination IP address
            udpPort:    16-bit destination UDP port
            bind:       True to bind (add), False to unbind (remove)

        Returns:
            dict: Response with keys:
                - 'ack': Acknowledgment (always 1)
                - 'full': 1 if table is full (bind only), 0 otherwise
                - 'connectionId': 18-bit connection ID (0 if failed)
        """

        hash_key = connection_manager_sw._hash_fun_ip_port(ipAddr, udpPort)
        connectionId = 0

        if bind:
            inserted = False

            # check if already exists
            for w in range(self.WAYS):
                if (
                    (self.my_hash_table_vlds[w][hash_key] == 1)
                    and (self.my_hash_table_ipAddr[w][hash_key] == ipAddr)
                    and (self.my_hash_table_udpPort[w][hash_key] == udpPort)
                ):
                    connectionId = (hash_key & 0xFFFF) | (w << 16)
                    return {"ack": 1, "full": 0, "connectionId": connectionId}

            for w in range(self.WAYS):
                if self.my_hash_table_vlds[w][hash_key] == 0:
                    connectionId = (hash_key & 0xFFFF) | (w << 16)
                    self.my_hash_table_vlds[w][hash_key] = 1
                    self.my_hash_table_ipAddr[w][hash_key] = ipAddr
                    self.my_hash_table_udpPort[w][hash_key] = udpPort
                    inserted = True
                    break

            if inserted:
                self.existing_connection_ids.append(connectionId)

            expected = {
                "ack": 1,
                "full": 0 if inserted else 1,
                "connectionId": connectionId if inserted else 0,
            }

        else:
            for w in range(self.WAYS):
                if (
                    self.my_hash_table_vlds[w][hash_key]
                    and (self.my_hash_table_ipAddr[w][hash_key] == ipAddr)
                    and (self.my_hash_table_udpPort[w][hash_key] == udpPort)
                ):
                    self.my_hash_table_vlds[w][hash_key] = 0
                    self.my_hash_table_ipAddr[w][hash_key] = 0
                    self.my_hash_table_udpPort[w][hash_key] = 0
            expected = {"ack": 1, "full": 0, "connectionId": 0}

        return expected

    def read_fw(self, ipAddr, udpPort):
        """
        Forward lookup: Find connection ID from IP address and port.

        Args:
            ipAddr :    32-bit IP address to lookup
            udpPort :   16-bit UDP port to lookup

        Returns:
            dict: Response with keys:
                - 'hit':            1 if found, 0 otherwise
                - 'connectionId':   Connection ID if found, 0 otherwise
        """

        hash_key = connection_manager_sw._hash_fun_ip_port(ipAddr, udpPort)

        for w in range(self.WAYS):
            if (
                self.my_hash_table_vlds[w][hash_key]
                and self.my_hash_table_ipAddr[w][hash_key] == ipAddr
                and self.my_hash_table_udpPort[w][hash_key] == udpPort
            ):
                return {"hit": 1, "connectionId": (w << 16) | hash_key}

        return {"hit": 0, "connectionId": 0}

    def read_rv(self, connectionId):
        """
        Reverse lookup: Find IP address and port from connection ID.

        Args:
            connectionId: 18-bit connection ID (bits [17:16] = way, bits [15:0] = hash key)

        Returns:
            dict: Response with keys:
                - 'hit': 1 if valid entry exists, 0 otherwise
                - 'ipAddr': 32-bit IP address (0 if miss)
                - 'udpPort': 16-bit UDP port (0 if miss)
        """

        hash_key = connectionId & 0xFFFF  # first 16 bits
        hash_way = connectionId >> 16

        expected = {"hit": 0, "ipAddr": 0, "udpPort": 0}
        if self.my_hash_table_vlds[hash_way][hash_key]:
            expected = {
                "hit": 1,
                "ipAddr": self.my_hash_table_ipAddr[hash_way][hash_key],
                "udpPort": self.my_hash_table_udpPort[hash_way][hash_key],
            }

        return expected

    @staticmethod
    def generate_collision_entries(num_chains=5, chain_len=5):
        """
        Generate test entries that intentionally collide in the hash table.

        This is useful for testing collision handling in the set-associative structure.
        Creates multiple chains of entries where all entries in a chain map to the
        same hash value but have unique IP/port combinations.

        Args:
            num_chains: Number of collision chains to generate (default: 5)
            chain_len: Number of entries per chain (default: 5)

        Returns:
            list: List of dictionaries, each containing:
                - 'ip': 32-bit IP address
                - 'port': 16-bit UDP port

        Note:
            All generated IP/port combinations are unique across all chains.
        """

        result = []
        used_ips = set()

        for _ in range(num_chains):
            target_hash = random.getrandbits(16)
            chain = []

            while len(chain) < chain_len:
                ip = random.getrandbits(32)
                port = random.getrandbits(16)
                hash_v = connection_manager_sw._hash_fun_ip_port(ip, port)
                if hash_v == target_hash and (ip, port) not in used_ips:
                    entry = {
                        "ip": ip,
                        "port": port,
                    }

                    chain.append(entry)
                    used_ips.add((ip, port))

            result.extend(chain)

        return result


# ======================================================================================================
# UDP ENGINE CONTROLLER - Hardware Interface
# ======================================================================================================


class udp_engine_controller:
    """
    Hardware controller for UDP engine via memory-mapped I/O (MMIO).

    This class provides methods to configure the UDP engine hardware, manage TX enable/disable,
    and bind/unbind connections through hardware registers. It validates all register writes
    and compares hardware behavior against the software model.

    Attributes:
        udp_mmio: PYNQ MMIO object for register access
        connection_manager: Software connection manager for validation

    Register Map:
        0x00:       Control register (bit 0: TX enable)
        0x04-0x08:  Source MAC address
        0x0C:       Source IP address
        0x10:       Source UDP port
        0x14-0x18:  Destination MAC address
        0x1C-0x30:  Connection manager control/status registers
    """

    addr_csr_udp_engine_100g__ctrl = 0x00

    addr_csr_udp_engine_100g__myConfig_macAddr_upper = 0x04
    addr_csr_udp_engine_100g__myConfig_macAddr_lower = 0x08
    addr_csr_udp_engine_100g__myConfig_ipAddr = 0x0C
    addr_csr_udp_engine_100g__myConfig_udpPort = 0x10
    addr_csr_udp_engine_100g__myConfig_macAddr_upper_dst = 0x14
    addr_csr_udp_engine_100g__myConfig_macAddr_lower_dst = 0x18

    addr_csr_udp_engine_100g__connManager_wr_ip_addr = 0x1C
    addr_csr_udp_engine_100g__connManager_wr_port = 0x20
    addr_csr_udp_engine_100g__connManager_wr_bind = 0x24
    addr_csr_udp_engine_100g__connManager_wr_trigger = 0x28
    addr_csr_udp_engine_100g__connManager_wr_status = 0x2C
    addr_csr_udp_engine_100g__connManager_wr_connectedId = 0x30

    def __init__(self, udp_mmio, connection_manager):
        self.udp_mmio = udp_mmio
        self.connection_manager = connection_manager

    def _write_confirmed(self, addr, value):
        """
        Write to a register and verify the write succeeded.
        """
        self.udp_mmio.write(addr, value)
        readback = self.udp_mmio.read(addr)
        if readback != value:
            print(
                f"ERROR: Write confirmation failed at 0x{addr:X}: wrote 0x{value:X}, read back 0x{readback:X}"
            )
        return readback

    def tx_enable(self):
        """Enable UDP transmit engine by setting control register bit 0."""
        old = self.udp_mmio.read(self.addr_csr_udp_engine_100g__ctrl)
        old |= 1
        self._write_confirmed(self.addr_csr_udp_engine_100g__ctrl, old)

    def tx_disable(self):
        """Disable UDP transmit engine by clearing control register bit 0."""
        old = self.udp_mmio.read(self.addr_csr_udp_engine_100g__ctrl)
        old &= ~1
        self._write_confirmed(self.addr_csr_udp_engine_100g__ctrl, old)

    def configure(self, src_macAddr, src_ipAddr, src_udpPort, dst_macAddr):
        """
        Configure the UDP engine with source and destination network parameters.

        This method temporarily disables TX, writes all configuration registers,
        then re-enables TX.
        """
        self.tx_disable()
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__myConfig_macAddr_upper,
            (src_macAddr >> 32) & 0xFFFFFFFF,
        )
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__myConfig_macAddr_lower,
            src_macAddr & 0xFFFFFFFF,
        )
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__myConfig_ipAddr, src_ipAddr
        )
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__myConfig_udpPort, src_udpPort
        )
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__myConfig_macAddr_upper_dst,
            (dst_macAddr >> 32) & 0xFFFFFFFF,
        )
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__myConfig_macAddr_lower_dst,
            dst_macAddr & 0xFFFFFFFF,
        )
        self.tx_enable()

    def bind_connection(self, dst_ipAddr, dst_udpPort):
        """
        Bind a new UDP connection in hardware and validate against software model.

        Sends a bind request to the hardware connection manager and waits for response.
        Compares hardware result with software model to detect discrepancies.

        Args:
            dst_ipAddr: 32-bit destination IP address
            dst_udpPort: 16-bit destination UDP port

        Returns:
            dict: Hardware response with keys:
                - 'ack': Acknowledgment bit
                - 'full': Table full indicator
                - 'connectionId': Assigned connection ID (0 if failed)
        """
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__connManager_wr_ip_addr, dst_ipAddr
        )
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__connManager_wr_port, dst_udpPort
        )
        self._write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_bind, 1)
        self._write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_trigger, 1)

        time.sleep(0.5)
        tmp = self.udp_mmio.read(self.addr_csr_udp_engine_100g__connManager_wr_status)
        ack = tmp & 0x1
        full = (tmp >> 1) & 0x1
        connectionId = self.udp_mmio.read(
            self.addr_csr_udp_engine_100g__connManager_wr_connectedId
        )

        actual = {"ack": ack, "full": full, "connectionId": connectionId}
        expected = self.connection_manager.write(dst_ipAddr, dst_udpPort, bind=1)

        if expected != actual:
            print(f"ERROR: expected = {expected}, actual = {actual}")

        return actual

    def unbind_connection(self, dst_ipAddr, dst_udpPort):
        """
        Unbind (remove) a UDP connection from hardware and validate against software model.

        Sends an unbind request to the hardware connection manager and waits for response.
        Compares hardware result with software model to detect discrepancies.

        Args:
            dst_ipAddr: 32-bit destination IP address
            dst_udpPort: 16-bit destination UDP port

        Returns:
            dict: Hardware response with keys:
                - 'ack': Acknowledgment bit
                - 'full': Always 0 for unbind
                - 'connectionId': Always 0 for unbind
        """
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__connManager_wr_ip_addr, dst_ipAddr
        )
        self._write_confirmed(
            self.addr_csr_udp_engine_100g__connManager_wr_port, dst_udpPort
        )
        self._write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_bind, 0)
        self._write_confirmed(self.addr_csr_udp_engine_100g__connManager_wr_trigger, 1)

        time.sleep(0.5)

        tmp = self.udp_mmio.read(self.addr_csr_udp_engine_100g__connManager_wr_status)
        ack = tmp & 0x1
        full = (tmp >> 1) & 0x1
        connectionId = self.udp_mmio.read(
            self.addr_csr_udp_engine_100g__connManager_wr_connectedId
        )

        actual = {"ack": ack, "full": full, "connectionId": connectionId}
        expected = self.connection_manager.write(dst_ipAddr, dst_udpPort, bind=0)

        if expected != actual:
            print(f"ERROR: expected = {expected}, actual = {actual}")

        return actual

    def rx_internal_loopback_enable(self):
        old = self.udp_mmio.read(self.addr_csr_udp_engine_100g__ctrl)
        old |= 1 << 3
        self._write_confirmed(self.addr_csr_udp_engine_100g__ctrl, old)

    def rx_internal_loopback_disable(self):
        old = self.udp_mmio.read(self.addr_csr_udp_engine_100g__ctrl)
        old &= ~(1 << 3)
        self._write_confirmed(self.addr_csr_udp_engine_100g__ctrl, old)


# ======================================================================================================
# UDP MODEL - Packet Generation and Processing
# ======================================================================================================


class udp_model:
    """
    Software model for UDP packet generation and encapsulation.

    This class generates random UDP packets with proper Ethernet/IP/UDP headers and
    validates hardware behavior by comparing expected vs actual packet outputs.

    Protocol Constants:
        ETHTYPE_IP:         Ethernet type for IPv4 (0x0800)
        IP_HEADER_BYTES:    IPv4 header length (20 bytes)
        UDP_HEADER_BYTES:   UDP header length (8 bytes)
        IPPROTO_UDP:        IP protocol number for UDP (17)

    IPv4 Header Defaults:
        DSCP: 0 (Differentiated Services Code Point)
        ECN: 0 (Explicit Congestion Notification)
        TTL: 64 (Time To Live)
        Flags: 0 (no fragmentation)

    Attributes:
        connection_manager: Software connection manager for lookups
        MY_CONFIG_DST_MAC: Destination MAC address
        MY_CONFIG_SRC_MAC: Source MAC address
        MY_CONFIG_SRC_IP: Source IP address
        MY_CONFIG_SRC_PORT: Source UDP port
    """

    ETHTYPE_IP = 0x0800
    IP_HEADER_BYTES = 20
    UDP_HEADER_BYTES = 8
    IPPROTO_UDP = 17

    IP_UDP_DSCP = 0
    IP_UDP_ENC = 0
    IP_UDP_IDEN = 0
    IP_UDP_FLAGS = 0
    IP_UDP_FRAG_OFFSET = 0
    IP_UDP_TTL = 64

    def __init__(
        self,
        connection_manager,
        MY_CONFIG_DST_MAC,
        MY_CONFIG_SRC_MAC,
        MY_CONFIG_SRC_IP,
        MY_CONFIG_SRC_PORT,
    ):
        self.connection_manager = connection_manager
        self.MY_CONFIG_DST_MAC = MY_CONFIG_DST_MAC
        self.MY_CONFIG_SRC_MAC = MY_CONFIG_SRC_MAC
        self.MY_CONFIG_SRC_IP = MY_CONFIG_SRC_IP
        self.MY_CONFIG_SRC_PORT = MY_CONFIG_SRC_PORT

    def _generate_random_packet(self):
        """
        Generate a random UDP packet payload with connection ID header.

        Packet structure:
            - 4 bytes: Connection ID (MSB first)
            - Variable: Random payload data

        Payload length is determined by:
            - Mode 0: 8-168 bytes (8-byte aligned, small packets)
            - Mode 1: 176 bytes (medium packet)
            - Mode 2: 184-504 bytes (8-byte aligned, large packets)
            - Plus 0-3 additional 512-byte blocks

        Connection ID selection:
            - 90% chance: Pick from existing connections (realistic traffic)
            - 10% chance: Random ID (tests error handling)

        Returns:
            tuple: (connection_id, total_length_bytes, packet_bytes_list)
                - connection_id: 18-bit connection ID
                - total_length_bytes: Total packet length including ID header
                - packet_bytes_list (list): Byte array of complete packet
        """

        # ------------------------------------------------------------
        # 1. Generate packet length
        # ------------------------------------------------------------
        n = random.randint(0, 3)
        mode = random.randint(0, 2)

        if mode == 0:
            base_len = random.randrange(0, 176, 8)
        elif mode == 1:
            base_len = 176
        else:
            base_len = random.randrange(184, 513, 8)

        payload_length_bits = base_len + n * 512
        assert payload_length_bits % 8 == 0
        payload_length_bytes = payload_length_bits // 8
        payload_length_bytes = max(payload_length_bytes, 60)

        # ------------------------------------------------------------
        # 2. Payload bytes (list of ints)
        # ------------------------------------------------------------
        payload = [random.getrandbits(8) for _ in range(payload_length_bytes)]

        # ------------------------------------------------------------
        # 3. Pick connectionId
        # ------------------------------------------------------------
        if self.connection_manager.existing_connection_ids and random.random() < 0.9:
            connection_id = random.choice(
                self.connection_manager.existing_connection_ids
            )
        else:
            connection_id = random.getrandbits(18)

        full_packet_bytes = []

        # append connectionId as MSB → LSB (like your IP header builder)
        for i in reversed(range(4)):  # MSB → LSB
            full_packet_bytes.append((connection_id >> (8 * i)) & 0xFF)

        # append payload bytes (already LSB-first from RAM)
        full_packet_bytes.extend(payload)

        return connection_id, payload_length_bytes + 4, full_packet_bytes

    @staticmethod
    def _swap_bytes(val, num_bytes):
        """Convert between little-endian and big-endian byte order."""
        out = 0
        for i in range(num_bytes):
            out = (out << 8) | ((val >> (8 * i)) & 0xFF)
        return out

    @staticmethod
    def _build_ipv4_header(
        payload_length_bytes,
        src_ip,
        dst_ip,
        dscp,
        ecn,
        identification,
        flags,
        frag_offset,
        ttl,
        protocol,
        IP_HEADER_BYTES=20,
        UDP_HEADER_BYTES=8,
    ):
        """
        Build a 160-bit (20-byte) IPv4 header matching hardware layout.

        This constructs the IPv4 header exactly as the hardware does, with proper
        byte ordering and field positioning. The header layout matches the RTL
        implementation for bit-accurate verification.

        Returns:
            int: 160-bit IPv4 header as a single integer

        Header Format (RFC 791):
            Bytes 0-1:   Version(4), IHL(4), DSCP(6), ECN(2)
            Bytes 2-3:   Total Length
            Bytes 4-5:   Identification
            Bytes 6-7:   Flags(3), Fragment Offset(13)
            Byte 8:      TTL
            Byte 9:      Protocol
            Bytes 10-11: Header Checksum (set to 0)
            Bytes 12-15: Source IP
            Bytes 16-19: Destination IP
        """

        version_ihl = (4 << 4) | (IP_HEADER_BYTES // 4)
        dscp_ecn = (dscp << 2) | ecn

        # Total length = header + UDP + payload
        total_len = payload_length_bytes + IP_HEADER_BYTES + UDP_HEADER_BYTES

        # Flags (3 bits) + Fragment offset (13 bits)
        flags_frag = ((flags & 0x7) << 13) | (frag_offset & 0x1FFF)

        # Build header exactly in the same field order as the Verilog
        ip_header = 0
        ip_header |= version_ihl
        ip_header |= dscp_ecn << 8
        ip_header |= udp_model._swap_bytes(total_len, 2) << 16
        ip_header |= udp_model._swap_bytes(identification, 2) << 32
        ip_header |= udp_model._swap_bytes(flags_frag, 2) << 48
        ip_header |= ttl << 64
        ip_header |= protocol << 72
        ip_header |= 0 << 80
        ip_header |= udp_model._swap_bytes(src_ip, 4) << 96
        ip_header |= udp_model._swap_bytes(dst_ip, 4) << 128

        return ip_header

    def _model_udp(self, connection_id, payload_length_bytes, payload_bytes):
        """
        Model complete UDP packet encapsulation (Ethernet + IP + UDP + Payload).

        This method performs the complete packet construction process:
        1. Look up connection metadata (destination IP/port)
        2. Build Ethernet header (14 bytes)
        3. Build IPv4 header (20 bytes)
        4. Build UDP header (8 bytes)
        5. Append payload data

        The packet structure mirrors the hardware implementation exactly for
        verification purposes.

        Packet Structure:
            [0-13]:    Ethernet header (Dst MAC, Src MAC, EtherType)
            [14-33]:   IPv4 header (20 bytes)
            [34-41]:   UDP header (8 bytes)
            [42-end]:  Payload data
        """

        connection_meta_data = self.connection_manager.read_rv(connection_id)
        if not connection_meta_data["hit"]:
            return {"dropped": 1, "packet": []}

        dst_ipAddr = connection_meta_data["ipAddr"]
        dst_udpPort = connection_meta_data["udpPort"]

        # ======================================================
        # Build Ethernet Header (big-endian)
        # ======================================================
        ethernet_header = (
            (udp_model._swap_bytes(udp_model.ETHTYPE_IP, 2) << (2 * 48))
            | (udp_model._swap_bytes(self.MY_CONFIG_SRC_MAC, 6) << 48)
            | udp_model._swap_bytes(self.MY_CONFIG_DST_MAC, 6)
        )

        # ======================================================
        # Build IPv4 Header (exact RTL layout)
        # ======================================================
        ip_header = udp_model._build_ipv4_header(
            payload_length_bytes=payload_length_bytes,
            src_ip=self.MY_CONFIG_SRC_IP,
            dst_ip=dst_ipAddr,
            dscp=udp_model.IP_UDP_DSCP,
            ecn=udp_model.IP_UDP_ENC,
            identification=udp_model.IP_UDP_IDEN,
            flags=udp_model.IP_UDP_FLAGS,
            frag_offset=udp_model.IP_UDP_FRAG_OFFSET,
            ttl=udp_model.IP_UDP_TTL,
            protocol=udp_model.IPPROTO_UDP,
            IP_HEADER_BYTES=udp_model.IP_HEADER_BYTES,
            UDP_HEADER_BYTES=udp_model.UDP_HEADER_BYTES,
        )

        # ======================================================
        # Build UDP Header
        # ======================================================
        udp_header = (
            (udp_model._swap_bytes(0, 2) << 48)
            | (
                udp_model._swap_bytes(
                    payload_length_bytes + udp_model.UDP_HEADER_BYTES, 2
                )
                << 32
            )
            | (udp_model._swap_bytes(dst_udpPort, 2) << 16)
            | udp_model._swap_bytes(self.MY_CONFIG_SRC_PORT, 2)
        )

        # ======================================================
        # Build FINAL PACKET = headers || payload
        # ======================================================
        ALL_HDR_BYTES = 42

        all_headers = (
            (udp_header << (20 + 14) * 8) | (ip_header << (14 * 8)) | ethernet_header
        )

        full_packet_bytes = []

        # append header bytes MSB → LSB
        for i in range(ALL_HDR_BYTES):
            full_packet_bytes.append((all_headers >> (8 * i)) & 0xFF)

        # append payload (already LSB-first chunks)
        full_packet_bytes.extend(payload_bytes)

        return {"dropped": 0, "packet": full_packet_bytes}

    def _model_loopback_expanded_payload(self, is_valid, connection_id, payload_bytes):
        """
        Model the loopback module output:
            [0–3]   = connection_id   (little-endian)
            [4–7]   = 0x00 00 00 00
            [8–...] = payload_bytes

        Output is returned as 512-bit (64-byte) AXIS beats.
        """
        payload_length = len(payload_bytes)

        # --- Build the expanded returned payload ---
        combined = []

        # 4-byte little-endian connection_id
        for i in range(4):
            combined.append((connection_id >> (8 * i)) & 0xFF)

        # Next 4 bytes = zeros
        combined.extend([0] * 4)

        # Append original payload
        combined.extend(payload_bytes)

        if not is_valid:
            return []  # no output beats

        return combined

    def generate_end_to_end_test(self):
        """
        Generate a realistic loopback end-to-end test:
            Input:  raw payload + connection_id
            Output: expanded loopback payload in AXIS form
        """

        # Generate random input
        connection_id, payload_length, payload_bytes = self._generate_random_packet()

        # Old UDP model still used only to determine dropped/valid
        expected_output = self._model_udp(connection_id, payload_length, payload_bytes)

        expected_beats = self._model_loopback_expanded_payload(
            is_valid=not expected_output["dropped"],
            connection_id=connection_id,
            payload_bytes=payload_bytes,
        )

        return {
            "connection_id": connection_id,
            "input_payload_bytes": payload_bytes,
            "input_payload_bytes_length": payload_length,
            "dropped": expected_output["dropped"],
            # raw UDP-modeled output
            "raw_output_packet_bytes": expected_output["packet"],
            "raw_output_packet_bytes_length": len(expected_output["packet"]),
            # loopback-expanded AXIS packet (this is what hardware returns)
            "rx_output_packet_bytes": expected_beats,
            "rx_output_packet_bytes_length": len(expected_beats),
        }

    def generate_test(self):
        """
        Generate a complete test case with input packet and expected output.

        This is the main entry point for generating test vectors. It creates a
        random input packet, processes it through the UDP model, and returns
        both the input and expected output for hardware verification.
        """
        connection_id, payload_length_bytes, payload_bytes = (
            self._generate_random_packet()
        )
        expected_output = self._model_udp(
            connection_id, payload_length_bytes, payload_bytes
        )

        return {
            "connection_id": connection_id,
            "input_payload_bytes": payload_bytes,
            "input_payload_bytes_length": payload_length_bytes,
            "dropped": expected_output["dropped"],
            "output_packet_bytes": expected_output["packet"],
            "output_packet_bytes_length": len(expected_output["packet"]),
        }
