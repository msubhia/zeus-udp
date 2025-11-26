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
#from cocotb.runner          import get_runner
from vicoco.vivado_runner import get_runner
from cocotb_bus.bus         import Bus
from cocotb_bus.drivers     import BusDriver
from cocotb_bus.monitors    import Monitor
from cocotb_bus.monitors    import BusMonitor
from cocotb_bus.scoreboard  import Scoreboard


test_file = os.path.basename(__file__).replace(".py","")



########################################################################################################
########################################################################################################

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
                thing = dict(data=data.signed_integer,last=last,
                             name=self.name,count=self.transactions)
                self.dut._log.info(f"{self.name}: {thing}")
                self._recv(data.signed_integer)





########################################################################################################
########################################################################################################


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

        # elif value.get("type") == "write_burst":

        #     data_array = value.get("contents").get("data")

        #     for i, d in enumerate(data_array):

        #         # drive at falling edge of the clock
        #         await falling_edge
        #         self.bus.axis_tvalid.value  = 1
        #         self.bus.axis_tlast.value   = 1 if (i == len(data_array)-1) else 0
        #         self.bus.axis_tdata.value   = int(d)
        #         self.bus.axis_tstrb.value   = 0xF 

        #         # wait until the driven data has taken effect and 
        #         # check if the ready signal (entering the next raising edge) is on
        #         await read_only

        #         while (self.bus.axis_tready.value != 1):
        #             await rising_edge
        #             # await read_only, adding this is wrong because we want to see if ready- was valid, 
        #             #                  not after the module reacts to our valid


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







########################################################################################################
########################################################################################################

WAYS = 4
TABLE_SIZE = 2**16

my_hash_table_tags = [[0] * TABLE_SIZE for _ in range(WAYS)]
my_hash_table_vlds = [[0] * TABLE_SIZE for _ in range(WAYS)]

rd_sig_in = []
rd_sig_out_exp = []
rd_sig_out_act = []

wr_sig_in = []
wr_sig_out_exp = []
wr_sig_out_act = []


def xor32to16(x):
    x &= 0xFFFFFFFF
    return ((x >> 16) & 0xFFFF) ^ (x & 0xFFFF)


def connection_manager_model_rd(val):
    key = val
    hash_key = xor32to16(key)

    print({"key": key, "hash_key": hex(hash_key)})
    rd_sig_in.append({"key": key, "hash_key": hash_key})

    expected = {"hit": 0, "resp": 0}
    for w in range(WAYS):
        if my_hash_table_vlds[w][hash_key] and my_hash_table_tags[w][hash_key] == key:
            expected = {"hit": 1, "resp": (w << 16) | hash_key}
            break

    rd_sig_out_exp.append(expected)


def appending_values_rd(val):
    hit = (val >> 32) & 1
    resp = val & 0xFFFFFFFF
    rd_sig_out_act.append({"hit": hit, "resp": resp})


def connection_manager_model_wr(val):
    activate = (val >> 32) & 1
    key      = val & 0xFFFFFFFF
    hash_key = xor32to16(key)

    wr_sig_in.append({"key": key, "hash_key": hash_key, "activate": activate})
    print({"key": key, "hash_key": hex(hash_key), "activate": activate})

    if activate:  # insert
        inserted = False
        for w in range(WAYS):
            if my_hash_table_vlds[w][hash_key] == 0:
                my_hash_table_vlds[w][hash_key] = 1
                my_hash_table_tags[w][hash_key] = key
                inserted = True
                break

        expected = {"ack": 1, "full": 0 if inserted else 1}

    else:  # delete
        found = False
        for w in range(WAYS):
            if my_hash_table_vlds[w][hash_key] and my_hash_table_tags[w][hash_key] == key:
                my_hash_table_vlds[w][hash_key] = 0
                my_hash_table_tags[w][hash_key] = 0
                found = True
        expected = {"ack": 1, "full": 0}

    wr_sig_out_exp.append(expected)


def appending_values_wr(val):
    ack = (val >> 0) & 1
    full = (val >> 1) & 1
    wr_sig_out_act.append({"ack": ack, "full": full})



########################################################################################################
########################################################################################################

async def reset(clk,rst, cycles_held = 3,polarity=1):
    rst.value = polarity
    await ClockCycles(clk, cycles_held)
    rst.value = not polarity

@cocotb.test()
async def test_a(dut):

    #
    # ---------------------- BUILD HIGH-COLLISION IP POOL ----------------------
    #

    from collections import defaultdict

    def build_hash_buckets(num_samples=500_000):
        buckets = defaultdict(list)
        for _ in range(num_samples):
            key = random.getrandbits(32)
            h = xor32to16(key)
            buckets[h].append(key)
        return buckets

    def extract_multi_collision_buckets(buckets, min_chain=5):
        return {h: keys for h, keys in buckets.items() if len(keys) >= min_chain}

    def build_collision_pool(target_size=1000, min_chain=5):
        buckets = build_hash_buckets()
        multi   = extract_multi_collision_buckets(buckets, min_chain=min_chain)

        pool = []
        for key_list in multi.values():
            pool.extend(key_list)
            if len(pool) >= target_size:
                return pool[:target_size]

        return pool  # return whatever we found

    # Build pool of 1000 high-collision IPs
    COLLISION_POOL = build_collision_pool(target_size=1000, min_chain=5)
    print(f"[TB] Built collision pool: {len(COLLISION_POOL)} keys")


    #
    # ----------------------- SETUP MONITORS / DRIVERS -------------------------
    #

    rd_in_monitor   = AXIS_Monitor(dut,'s00',dut.s00_axis_aclk,callback = connection_manager_model_rd)
    rd_out_monitor  = AXIS_Monitor(dut,'m00',dut.s00_axis_aclk,callback = lambda x: appending_values_rd(x))

    wr_in_monitor   = AXIS_Monitor(dut,'s02',dut.s00_axis_aclk,callback = connection_manager_model_wr)
    wr_out_monitor  = AXIS_Monitor(dut,'m02',dut.s00_axis_aclk,callback = lambda x: appending_values_wr(x))

    rd_in_driver    = M_AXIS_Driver(dut,'s00',dut.s00_axis_aclk)
    rd_out_driver   = S_AXIS_Driver(dut,'m00',dut.s00_axis_aclk)

    wr_in_driver    = M_AXIS_Driver(dut,'s02',dut.s00_axis_aclk)
    wr_out_driver   = S_AXIS_Driver(dut,'m02',dut.s00_axis_aclk)


    #
    # --------------------------- CLOCK + RESET --------------------------------
    #

    cocotb.start_soon(Clock(dut.s00_axis_aclk, 10, units="ns").start())

    await reset(dut.s00_axis_aclk, dut.s00_axis_aresetn, cycles_held=5, polarity=0)
    # If your wrapper ties s02/m00/m02 resets together then it’s fine.
    # If not, assert/deassert them here as well.


    #
    # ----------------------------- MAIN TEST ----------------------------------
    #

    NUM_OPERATIONS = 1000

    for _ in range(NUM_OPERATIONS):

        is_rd    = (random.random() < 0.9)   # 90% reads
        activate = (random.random() < 0.5)   # 50% delete/insert

        # 70% from high-collision pool, 30% pure random
        if random.random() < 0.7:
            key = random.choice(COLLISION_POOL)
        else:
            key = random.getrandbits(32)

        val = key

        if is_rd:
            rd_in_driver.append(
                {'type':'write_single', "contents":{"data": val, "last":1}}
            )
            rd_in_driver.append(
                {"type":"pause", "duration": random.randint(1,6)}
            )

        else:
            # Insert/delete command
            val |= (activate << 32)
            wr_in_driver.append(
                {'type':'write_single', "contents":{"data": val, "last":1}}
            )
            wr_in_driver.append(
                {"type":"pause", "duration": random.randint(1,6)}
            )

    #
    # ---------------------------- READ OUT RESPONSES --------------------------
    #

    rd_out_driver.append({'type':'read', "duration": NUM_OPERATIONS * 30})
    wr_out_driver.append({'type':'read', "duration": NUM_OPERATIONS * 30})

    await ClockCycles(dut.s00_axis_aclk, NUM_OPERATIONS + 20000)


    #
    # ------------------------------- CHECKING ---------------------------------
    #

    assert rd_in_monitor.transactions == rd_out_monitor.transactions, \
           "RD transaction count mismatch!"

    assert wr_in_monitor.transactions == wr_out_monitor.transactions, \
           "WR transaction count mismatch!"

    assert len(rd_sig_in) == len(rd_sig_out_exp) == len(rd_sig_out_act), \
           "RD bookkeeping mismatch!"

    for idx, (sig_in, expected, actual) in enumerate(
        zip(rd_sig_in, rd_sig_out_exp, rd_sig_out_act)
    ):
        if expected != actual:
            raise AssertionError(
                f"RD mismatch {idx}: key=0x{sig_in['key']:08X} "
                f"hash=0x{sig_in['hash_key']:04X} "
                f"expected {expected}, got {actual}"
            )

    assert len(wr_sig_in) == len(wr_sig_out_exp) == len(wr_sig_out_act), \
           "WR bookkeeping mismatch!"

    for idx, (sig_in, expected, actual) in enumerate(
        zip(wr_sig_in, wr_sig_out_exp, wr_sig_out_act)
    ):
        if expected != actual:
            raise AssertionError(
                f"WR mismatch {idx}: key=0x{sig_in['key']:08X} "
                f"hash=0x{sig_in['hash_key']:04X} "
                f"activate={sig_in['activate']} "
                f"expected {expected}, got {actual}"
            )


########################################################################################################
########################################################################################################

def connection_manager_tb_runner():

    hdl_toplevel_lang   = os.getenv("HDL_TOPLEVEL_LANG", "verilog")
    #sim                 = os.getenv("SIM", "icarus")
    sim                 = os.getenv("SIM", "vivado")
    proj_path           = Path(__file__).resolve().parent.parent
    sys.path.append(str(proj_path / "sim" / "model"))
    sys.path.append(str(proj_path / "hdl" ))
    sys.path.append(str(proj_path / "sim"))

    sources             = [proj_path / "hdl" / "connection_manager.sv",
                           proj_path / "hdl" / "connection_manager_wrapper.sv",
                           proj_path / "hdl" / "bram_wrapper.sv"]
    build_test_args     = ["-Wall"]
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
