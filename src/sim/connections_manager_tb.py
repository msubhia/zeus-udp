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
            await read_only  #readonly (the postline)
            valid = self.bus.axis_tvalid.value
            ready = self.bus.axis_tready.value
            last = self.bus.axis_tlast.value
            data = self.bus.axis_tdata.value #.signed_integer
            if valid and ready:
                self.transactions+=1
                thing = dict(data=data.integer,last=last,
                             name=self.name,count=self.transactions)
                self.dut._log.info(f"{self.name}: {thing}")
                self._recv(data.integer)



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
        self.bus.axis_tstrb.value = 0xF
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

            for i, d in enumerate(data_array):

                # drive at falling edge of the clock
                await falling_edge
                self.bus.axis_tvalid.value  = 1
                self.bus.axis_tlast.value   = 1 if (i == len(data_array)-1) else 0
                self.bus.axis_tdata.value   = int(d)
                self.bus.axis_tstrb.value   = 0xF 

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
# PYTHON MODEL
# =====================================================================================================================================


def xor32to16(x):
    x &= 0xFFFFFFFF
    return ((x >> 16) & 0xFFFF) ^ (x & 0xFFFF)


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





WAYS = 4
TABLE_SIZE = 2**16

my_hash_table_vlds      = [[0] * TABLE_SIZE for _ in range(WAYS)]
my_hash_table_macAddr   = [[0] * TABLE_SIZE for _ in range(WAYS)]
my_hash_table_udpPort   = [[0] * TABLE_SIZE for _ in range(WAYS)]
my_hash_table_ipAddr    = [[0] * TABLE_SIZE for _ in range(WAYS)]


fw_rd_sig_in        = []
fw_rd_sig_out_exp   = []
fw_rd_sig_out_act   = []

rv_rd_sig_in        = []
rv_rd_sig_out_exp   = []
rv_rd_sig_out_act   = []

wr_sig_in           = []
wr_sig_out_exp      = []
wr_sig_out_act      = []

existing_connection_ids = []



def connection_manager_model_fw_rd(val):
    ipAddr = val
    hash_key = xor32to16(ipAddr)

    fw_rd_sig_in.append({"ipAddr": ipAddr, "hash_key": hash_key})

    expected = {"hit": 0, "connectionId": 0}
    for w in range(WAYS):
        if my_hash_table_vlds[w][hash_key] and my_hash_table_ipAddr[w][hash_key] == ipAddr:
            expected = {"hit": 1, "connectionId": (w << 16) | hash_key}
            break

    fw_rd_sig_out_exp.append(expected)


def appending_values_fw_rd(val):
    hit = (val >> 32) & 1
    connectionId = val & 0x3FFFF
    fw_rd_sig_out_act.append({"hit": hit, "connectionId": connectionId})

def connection_manager_model_rv_rd(val):
    connectionId = val
    hash_key     = connectionId & 0xFFFF    # first 16 bits
    hash_way     = connectionId >> 16

    rv_rd_sig_in.append({"connectionId": connectionId, "hash_key": hash_key, "hash_way": hash_way})

    expected = {"hit": 0, "ipAddr": 0, "macAddr":0, "udpPort":0}
    if my_hash_table_vlds[hash_way][hash_key]:
        expected = {"hit"       : 1, 
                    "ipAddr"    : my_hash_table_ipAddr[hash_way][hash_key], 
                    "macAddr"   : my_hash_table_macAddr[hash_way][hash_key],
                    "udpPort"   : my_hash_table_udpPort[hash_way][hash_key]}

    rv_rd_sig_out_exp.append(expected)


def appending_values_rv_rd(val):
    # .m01_axis_rv_lookup_hit(m01_axis_tdata[96]),
    # .m01_axis_rv_lookup_macAddr(m01_axis_tdata[47:0]),
    # .m01_axis_rv_lookup_udpPort(m01_axis_tdata[63:48]),
    # .m01_axis_rv_lookup_ipAddr(m01_axis_tdata[95:64]),

    ipAddr  = (val>>64) & 0xFFFFFFFF
    macAddr = val & 0xFFFFFFFFFFFF
    udpPort = (val>>48) & 0xFFFF
    hit     = (val>>96) & 0x1

    rv_rd_sig_out_act.append({"hit": hit, "ipAddr": ipAddr, "macAddr": macAddr, "udpPort": udpPort})





def connection_manager_model_wr(val):
    ipAddr  = (val>>64) & 0xFFFFFFFF
    macAddr = val & 0xFFFFFFFFFFFF
    udpPort = (val>>48) & 0xFFFF
    bind    = (val>>96) & 0x1

    tag      = ipAddr
    hash_key = xor32to16(ipAddr)

    wr_sig_in.append({"ipAddr": ipAddr, "macAddr": macAddr, "udpPort": udpPort, "bind": bind, "hash_key": hash_key})


    if bind:
        inserted = False

        # check if already exists
        for w in range(WAYS):
            if (my_hash_table_vlds[w][hash_key] == 1) and (my_hash_table_ipAddr[w][hash_key] == tag):
                connectionId = (xor32to16(ipAddr) & 0xFFFF) | (w << 16)
                wr_sig_out_exp.append({"ack": 1, "full": 0, "connectionId":connectionId})
                return

        for w in range(WAYS):
            if my_hash_table_vlds[w][hash_key] == 0:
                connectionId = (xor32to16(ipAddr) & 0xFFFF) | (w << 16)
                my_hash_table_vlds[w][hash_key] = 1
                my_hash_table_ipAddr[w][hash_key] = tag
                my_hash_table_macAddr[w][hash_key] = macAddr
                my_hash_table_udpPort[w][hash_key] = udpPort
                inserted = True
                break

        if (inserted):
            existing_connection_ids.append(connectionId)

        expected = {"ack": 1, "full": 0 if inserted else 1, "connectionId": connectionId if inserted else 0}

    else:
        for w in range(WAYS):
            if my_hash_table_vlds[w][hash_key] and my_hash_table_ipAddr[w][hash_key] == tag:
                my_hash_table_vlds[w][hash_key] = 0
                my_hash_table_ipAddr[w][hash_key] = 0
                my_hash_table_macAddr[w][hash_key] = 0
                my_hash_table_udpPort[w][hash_key] = 0
        expected = {"ack": 1, "full": 0, "connectionId": 0}

    wr_sig_out_exp.append(expected)



def appending_values_wr(val):

    # .m02_axis_ctrl_ack(m02_axis_tdata[18]),
    # .m02_axis_ctrl_full(m02_axis_tdata[19]),
    # .m02_axis_ctrl_connectionId(m02_axis_tdata[17:0])

    ack  = (val >> 18) & 1
    full = (val >> 19) & 1
    connectionId = val & (0x3FFFF)

    wr_sig_out_act.append({"ack": ack, "full": full, "connectionId": connectionId})



# =====================================================================================================================================
# TESTING LOGIC
# =====================================================================================================================================

async def reset(clk,rst, cycles_held = 3,polarity=1):
    rst.value = polarity
    await ClockCycles(clk, cycles_held)
    rst.value = not polarity


async def test_structure(dut, WR_NUM_OPERATIONS = 300, RD_NUM_OPERATIONS = 1000, NUM_CHAINS=128, CHAIN_LEN=8, INTERLAVE = False):
    fw_rd_in_monitor    = AXIS_Monitor(dut,'s00',dut.s00_axis_aclk,callback = connection_manager_model_fw_rd)
    fw_rd_out_monitor   = AXIS_Monitor(dut,'m00',dut.s00_axis_aclk,callback = lambda x: appending_values_fw_rd(x))

    rv_rd_in_monitor    = AXIS_Monitor(dut,'s01',dut.s00_axis_aclk,callback = connection_manager_model_rv_rd)
    rv_rd_out_monitor   = AXIS_Monitor(dut,'m01',dut.s00_axis_aclk,callback = lambda x: appending_values_rv_rd(x))

    wr_in_monitor       = AXIS_Monitor(dut,'s02',dut.s00_axis_aclk,callback = connection_manager_model_wr)
    wr_out_monitor      = AXIS_Monitor(dut,'m02',dut.s00_axis_aclk,callback = lambda x: appending_values_wr(x))

    fw_rd_in_driver     = M_AXIS_Driver(dut,'s00',dut.s00_axis_aclk)
    fw_rd_out_driver    = S_AXIS_Driver(dut,'m00',dut.s00_axis_aclk)

    rv_rd_in_driver     = M_AXIS_Driver(dut,'s01',dut.s00_axis_aclk)
    rv_rd_out_driver    = S_AXIS_Driver(dut,'m01',dut.s00_axis_aclk)

    wr_in_driver        = M_AXIS_Driver(dut,'s02',dut.s00_axis_aclk)
    wr_out_driver       = S_AXIS_Driver(dut,'m02',dut.s00_axis_aclk)

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())
    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)

    #
    # ----------------------------- MAIN TEST ----------------------------------
    #

    COLLISION_POOL = list(generate_collision_entries(NUM_CHAINS, CHAIN_LEN))

    async def writer_thread(num_operations):

        for _ in range(num_operations):

            pick = random.choice(COLLISION_POOL)
            bind = (random.random() < 0.9)

            # .s02_axis_ctrl_macAddr(s02_axis_tdata[47:0]),
            # .s02_axis_ctrl_ipAddr(s02_axis_tdata[95:64]),
            # .s02_axis_ctrl_udpPort(s02_axis_tdata[63:48]),
            # .s02_axis_ctrl_bind(s02_axis_tdata[96]),

            val  = 0
            val  |= (pick['mac'])
            val  |= (pick['port'] << 48)
            val  |= (pick['ip']   << 64)
            val  |= (bind         << 96)

            wr_in_driver.append({'type':'write_single', "contents":{"data": val, "last":1}})
            wr_in_driver.append({"type":"pause", "duration": random.randint(0,3)})

    async def fw_reader_thread(num_operations):
        for _ in range(num_operations):

            pick = random.choice(COLLISION_POOL)
            val  = pick['ip']

            fw_rd_in_driver.append({'type':'write_single', "contents":{"data": val, "last":1}})
            fw_rd_in_driver.append({"type":"pause", "duration": random.randint(0,3)})

    async def rv_reader_thread(num_operations):
        for _ in range(num_operations):

            if existing_connection_ids and (random.random() < 0.9):
                val = random.choice(existing_connection_ids)
            else:
                val = random.getrandbits(18)

            rv_rd_in_driver.append({'type':'write_single', "contents":{"data": val, "last":1}})
            rv_rd_in_driver.append({"type":"pause", "duration": random.randint(0,3)})



    if INTERLAVE:
        cocotb.start_soon(writer_thread(WR_NUM_OPERATIONS))
        wr_out_driver.append({'type':'read', "duration": WR_NUM_OPERATIONS + 5000})

        cocotb.start_soon(fw_reader_thread(RD_NUM_OPERATIONS))
        fw_rd_out_driver.append({'type':'read', "duration": RD_NUM_OPERATIONS * 30 + 5000})

        cocotb.start_soon(rv_reader_thread(RD_NUM_OPERATIONS))
        rv_rd_out_driver.append({'type':'read', "duration": RD_NUM_OPERATIONS * 30 + 5000})

        await ClockCycles(dut.s00_axis_aclk, max(RD_NUM_OPERATIONS,WR_NUM_OPERATIONS) + 5000)

    else:
        cocotb.start_soon(writer_thread(WR_NUM_OPERATIONS))
        wr_out_driver.append({'type':'read', "duration": WR_NUM_OPERATIONS + 5000})
        await ClockCycles(dut.s00_axis_aclk, WR_NUM_OPERATIONS + 1000)

        # buffer time between writes and reads
        await ClockCycles(dut.s00_axis_aclk, 1000)

        cocotb.start_soon(fw_reader_thread(RD_NUM_OPERATIONS))
        fw_rd_out_driver.append({'type':'read', "duration": RD_NUM_OPERATIONS * 30 + 5000})

        cocotb.start_soon(rv_reader_thread(RD_NUM_OPERATIONS))
        rv_rd_out_driver.append({'type':'read', "duration": RD_NUM_OPERATIONS * 30 + 5000})

        await ClockCycles(dut.s00_axis_aclk, RD_NUM_OPERATIONS + 5000)

    #
    # ------------------------------- CHECKING ---------------------------------
    #
    print("\nVALIDATION:")
    assert fw_rd_in_monitor.transactions == fw_rd_out_monitor.transactions, f"FW RD transaction count mismatch!"
    assert rv_rd_in_monitor.transactions == rv_rd_out_monitor.transactions, f"RV RD transaction count mismatch!"
    assert wr_in_monitor.transactions == wr_out_monitor.transactions,       f"WR transaction count mismatch!"

    assert len(fw_rd_sig_in) == len(fw_rd_sig_out_exp) == len(fw_rd_sig_out_act), "FW RD bookkeeping mismatch!"
    for idx, (sig_in, expected, actual) in enumerate(zip(fw_rd_sig_in, fw_rd_sig_out_exp, fw_rd_sig_out_act)):
        if expected != actual:
            raise RuntimeError(
                f"ERROR:    FW RD mismatch {idx}: ipAddr=0x{hex(sig_in['ipAddr'])} "
                f"hash=0x{hex(sig_in['hash_key'])} "
                f"expected {expected}, got {actual}"
            )

    assert len(rv_rd_sig_in) == len(rv_rd_sig_out_exp) == len(rv_rd_sig_out_act), "RV RD bookkeeping mismatch!"
    for idx, (sig_in, expected, actual) in enumerate(zip(rv_rd_sig_in, rv_rd_sig_out_exp, rv_rd_sig_out_act)):
        if expected != actual:
            raise RuntimeError(
                f"ERROR:    RV RD mismatch {idx}: connectionId=0x{hex(sig_in['connectionId'])} "
                f"expected {expected}, got {actual}"
            )

    assert len(wr_sig_in) == len(wr_sig_out_exp) == len(wr_sig_out_act), "WR bookkeeping mismatch!"
    for idx, (sig_in, expected, actual) in enumerate(zip(wr_sig_in, wr_sig_out_exp, wr_sig_out_act)):
        if expected != actual:
            raise RuntimeError(
                f"ERROR:    WR mismatch {idx}: ipAddr=0x{hex(sig_in['ipAddr'])} "
                f"hash=0x{hex(sig_in['hash_key'])} "
                f"bind={sig_in['bind']}"
                f"expected {expected}, got {actual}"
            )


# =====================================================================================================================================
# TEST CASES
# =====================================================================================================================================


@cocotb.test()
async def test_1(dut):
    """
    This test assumes all writes happens much before reads:
        Simulating Software setup all connections before the traffic flow starts
        This is to relax the consistensy between reads and writes
    """
    await test_structure(dut, WR_NUM_OPERATIONS = 300, RD_NUM_OPERATIONS = 2000, NUM_CHAINS=128, CHAIN_LEN=8, INTERLAVE = False)

@cocotb.test()
async def test_2(dut):
    """
    same as test_1 but more write collisions
    """
    await test_structure(dut, WR_NUM_OPERATIONS = 500, RD_NUM_OPERATIONS = 2000, NUM_CHAINS=8, CHAIN_LEN=128, INTERLAVE = False)

# @cocotb.test()
async def test_3(dut):
    """
    same as test_1 but interleaving
    """
    await test_structure(dut, WR_NUM_OPERATIONS = 300, RD_NUM_OPERATIONS = 2000, NUM_CHAINS=128, CHAIN_LEN=8, INTERLAVE = True)

# @cocotb.test()
async def test_4(dut):
    """
    same as test_1 but larger testing set
    """
    await test_structure(dut, WR_NUM_OPERATIONS = 1000, RD_NUM_OPERATIONS = 20000, NUM_CHAINS=128, CHAIN_LEN=10, INTERLAVE = False)




# =====================================================================================================================================
# TEST RUNNER
# =====================================================================================================================================


def connection_manager_tb_runner():

    hdl_toplevel_lang   = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    #sim                 = os.getenv("SIM", "icarus")
    sim                 = os.getenv("SIM", "vivado")
    test_file           = os.path.basename(__file__).replace(".py","")
    proj_path           = Path(__file__).resolve().parent

    sys.path.append(str(proj_path / "model"))

    sources             = [proj_path /  "connection_manager.sv",
                           proj_path /  "connection_manager_wrapper.v",
                           proj_path /  "bram_wrapper.sv"]

    build_test_args     = []
    parameters          = {}
    hdl_toplevel        = "connection_manager_wrapper"

    runner              = get_runner(sim)
    runner.build(
        sources         =   sources,
        hdl_toplevel    =   hdl_toplevel,
        always          =   True,
        build_args      =   build_test_args,
        parameters      =   parameters,
        timescale       =   ('1ns','1ps'),
        waves           =   True
    )
    run_test_args       = []
    runner.test(
        hdl_toplevel    =   hdl_toplevel,
        test_module     =   test_file,
        test_args       =   run_test_args,
        waves           =   True
    )


if __name__ == "__main__":
    connection_manager_tb_runner()
