#!/usr/bin/env python3
#
# Copyright (c) 2021 Facebook, Inc. and its affiliates
#
# SPDX-License-Identifier: Apache-2.0

import binascii
import logging
import struct

from gdbstubs.gdbstub import GdbStub


logger = logging.getLogger("gdbstub")

# RISC-V target.xml for GDB - matches actual DebugServer format
# Note: The regnum= values for CSRs use DECIMAL notation (e.g., 833 not 0x341)
# This matches the real cklink debugserver XML from 'maint print xml-tdesc'
RISCV_TARGET_XML = b'''<?xml version="1.0"?>
<target>
  <architecture>riscv</architecture>
  <feature name="org.gnu.gdb.riscv.cpu">
    <reg name="zero" bitsize="32" type="int" regnum="0"/>
    <reg name="ra" bitsize="32" type="code_ptr" regnum="1"/>
    <reg name="sp" bitsize="32" type="data_ptr" regnum="2"/>
    <reg name="gp" bitsize="32" type="data_ptr" regnum="3"/>
    <reg name="tp" bitsize="32" type="data_ptr" regnum="4"/>
    <reg name="t0" bitsize="32" type="int" regnum="5"/>
    <reg name="t1" bitsize="32" type="int" regnum="6"/>
    <reg name="t2" bitsize="32" type="int" regnum="7"/>
    <reg name="fp" bitsize="32" type="data_ptr" regnum="8"/>
    <reg name="s1" bitsize="32" type="int" regnum="9"/>
    <reg name="a0" bitsize="32" type="int" regnum="10"/>
    <reg name="a1" bitsize="32" type="int" regnum="11"/>
    <reg name="a2" bitsize="32" type="int" regnum="12"/>
    <reg name="a3" bitsize="32" type="int" regnum="13"/>
    <reg name="a4" bitsize="32" type="int" regnum="14"/>
    <reg name="a5" bitsize="32" type="int" regnum="15"/>
    <reg name="a6" bitsize="32" type="int" regnum="16"/>
    <reg name="a7" bitsize="32" type="int" regnum="17"/>
    <reg name="s2" bitsize="32" type="int" regnum="18"/>
    <reg name="s3" bitsize="32" type="int" regnum="19"/>
    <reg name="s4" bitsize="32" type="int" regnum="20"/>
    <reg name="s5" bitsize="32" type="int" regnum="21"/>
    <reg name="s6" bitsize="32" type="int" regnum="22"/>
    <reg name="s7" bitsize="32" type="int" regnum="23"/>
    <reg name="s8" bitsize="32" type="int" regnum="24"/>
    <reg name="s9" bitsize="32" type="int" regnum="25"/>
    <reg name="s10" bitsize="32" type="int" regnum="26"/>
    <reg name="s11" bitsize="32" type="int" regnum="27"/>
    <reg name="t3" bitsize="32" type="int" regnum="28"/>
    <reg name="t4" bitsize="32" type="int" regnum="29"/>
    <reg name="t5" bitsize="32" type="int" regnum="30"/>
    <reg name="t6" bitsize="32" type="int" regnum="31"/>
    <reg name="pc" bitsize="32" type="code_ptr" regnum="32"/>
  </feature>
  <feature name="org.gnu.gdb.riscv.fpu">
    <union id="float64_union">
      <field name="double" type="ieee_double"/>
      <field name="float" type="ieee_single"/>
      <field name="uint64" type="uint64"/>
    </union>
    <reg name="ft0"  bitsize="64" type="float64_union" regnum="33"/>
    <reg name="ft1"  bitsize="64" type="float64_union" regnum="34"/>
    <reg name="ft2"  bitsize="64" type="float64_union" regnum="35"/>
    <reg name="ft3"  bitsize="64" type="float64_union" regnum="36"/>
    <reg name="ft4"  bitsize="64" type="float64_union" regnum="37"/>
    <reg name="ft5"  bitsize="64" type="float64_union" regnum="38"/>
    <reg name="ft6"  bitsize="64" type="float64_union" regnum="39"/>
    <reg name="ft7"  bitsize="64" type="float64_union" regnum="40"/>
    <reg name="fs0"  bitsize="64" type="float64_union" regnum="41"/>
    <reg name="fs1"  bitsize="64" type="float64_union" regnum="42"/>
    <reg name="fa0"  bitsize="64" type="float64_union" regnum="43"/>
    <reg name="fa1"  bitsize="64" type="float64_union" regnum="44"/>
    <reg name="fa2"  bitsize="64" type="float64_union" regnum="45"/>
    <reg name="fa3"  bitsize="64" type="float64_union" regnum="46"/>
    <reg name="fa4"  bitsize="64" type="float64_union" regnum="47"/>
    <reg name="fa5"  bitsize="64" type="float64_union" regnum="48"/>
    <reg name="fa6"  bitsize="64" type="float64_union" regnum="49"/>
    <reg name="fa7"  bitsize="64" type="float64_union" regnum="50"/>
    <reg name="fs2"  bitsize="64" type="float64_union" regnum="51"/>
    <reg name="fs3"  bitsize="64" type="float64_union" regnum="52"/>
    <reg name="fs4"  bitsize="64" type="float64_union" regnum="53"/>
    <reg name="fs5"  bitsize="64" type="float64_union" regnum="54"/>
    <reg name="fs6"  bitsize="64" type="float64_union" regnum="55"/>
    <reg name="fs7"  bitsize="64" type="float64_union" regnum="56"/>
    <reg name="fs8"  bitsize="64" type="float64_union" regnum="57"/>
    <reg name="fs9"  bitsize="64" type="float64_union" regnum="58"/>
    <reg name="fs10" bitsize="64" type="float64_union" regnum="59"/>
    <reg name="fs11" bitsize="64" type="float64_union" regnum="60"/>
    <reg name="ft8"  bitsize="64" type="float64_union" regnum="61"/>
    <reg name="ft9"  bitsize="64" type="float64_union" regnum="62"/>
    <reg name="ft10" bitsize="64" type="float64_union" regnum="63"/>
    <reg name="ft11" bitsize="64" type="float64_union" regnum="64"/>
    <reg name="fflags" bitsize="32" type="int" regnum="65"/>
    <reg name="frm"    bitsize="32" type="int" regnum="66"/>
    <reg name="fcsr"   bitsize="32" type="int" regnum="67"/>
  </feature>
  <feature name="org.gnu.gdb.riscv.csr">
    <!-- Regnum for CSR is (csr_regno + 65), used by remote protocol -->
    <!-- M-mode Information -->
    <reg name="mvendorid" bitsize="32" type="int" regnum="0xf52"/>
    <reg name="marchid"   bitsize="32" type="int" regnum="0xf53"/>
    <reg name="mimpid"    bitsize="32" type="int" regnum="0xf54"/>
    <reg name="mhartid"   bitsize="32" type="int" regnum="0xf55"/>
    <!-- M-mode Exception Setting -->
    <reg name="mstatus"   bitsize="32" type="int" regnum="0x341"/>
    <reg name="misa"      bitsize="32" type="int" regnum="0x342"/>
    <reg name="mie"       bitsize="32" type="int" regnum="0x345"/>
    <reg name="mtvec"     bitsize="32" type="int" regnum="0x346"/>
    <reg name="mtvt"      bitsize="32" type="int" regnum="0x348"/>
    <!-- M-mode Exception Handling -->
    <reg name="mscratch"  bitsize="32" type="int" regnum="0x381"/>
    <reg name="mepc"      bitsize="32" type="int" regnum="0x382"/>
    <reg name="mcause"    bitsize="32" type="int" regnum="0x383"/>
    <reg name="mtval"     bitsize="32" type="int" regnum="0x384"/>
    <reg name="mip"       bitsize="32" type="int" regnum="0x385"/>
    <reg name="mnxti"     bitsize="32" type="int" regnum="0x386"/>
    <reg name="mintstatus"     bitsize="32" type="int" regnum="0x387"/>
    <reg name="mscratchcsw"    bitsize="32" type="int" regnum="0x389"/>
    <reg name="mscratchcswl"   bitsize="32" type="int" regnum="0x38a"/>
    <reg name="mclicbase" bitsize="32" type="int" regnum="0x391"/>
    <!-- M-mode Memory Protect -->
    <reg name="pmpcfg0"   bitsize="32" type="int" regnum="0x3e1"/>
    <reg name="pmpcfg1"   bitsize="32" type="int" regnum="0x3e2"/>
    <reg name="pmpcfg2"   bitsize="32" type="int" regnum="0x3e3"/>
    <reg name="pmpcfg3"   bitsize="32" type="int" regnum="0x3e4"/>
    <reg name="pmpaddr0"  bitsize="32" type="int" regnum="0x3f1"/>
    <reg name="pmpaddr1"  bitsize="32" type="int" regnum="0x3f2"/>
    <reg name="pmpaddr2"  bitsize="32" type="int" regnum="0x3f3"/>
    <reg name="pmpaddr3"  bitsize="32" type="int" regnum="0x3f4"/>
    <reg name="pmpaddr4"  bitsize="32" type="int" regnum="0x3f5"/>
    <reg name="pmpaddr5"  bitsize="32" type="int" regnum="0x3f6"/>
    <reg name="pmpaddr6"  bitsize="32" type="int" regnum="0x3f7"/>
    <reg name="pmpaddr7"  bitsize="32" type="int" regnum="0x3f8"/>
    <reg name="pmpaddr8"  bitsize="32" type="int" regnum="0x3f9"/>
    <reg name="pmpaddr9"  bitsize="32" type="int" regnum="0x3fa"/>
    <reg name="pmpaddr10" bitsize="32" type="int" regnum="0x3fb"/>
    <reg name="pmpaddr11" bitsize="32" type="int" regnum="0x3fc"/>
    <reg name="pmpaddr12" bitsize="32" type="int" regnum="0x3fd"/>
    <reg name="pmpaddr13" bitsize="32" type="int" regnum="0x3fe"/>
    <reg name="pmpaddr14" bitsize="32" type="int" regnum="0x3ff"/>
    <reg name="pmpaddr15" bitsize="32" type="int" regnum="0x400"/>
    <!-- M-mode Perf and Counter -->
    <reg name="mcounteren"     bitsize="32" type="int" regnum="0x347"/>
    <reg name="mcountinhibit"  bitsize="32" type="int" regnum="0x361"/>
    <reg name="mhpmevent3"  bitsize="32" type="int" regnum="0x364"/>
    <reg name="mhpmevent4"  bitsize="32" type="int" regnum="0x365"/>
    <reg name="mhpmevent5"  bitsize="32" type="int" regnum="0x366"/>
    <reg name="mhpmevent6"  bitsize="32" type="int" regnum="0x367"/>
    <reg name="mhpmevent7"  bitsize="32" type="int" regnum="0x368"/>
    <reg name="mhpmevent8"  bitsize="32" type="int" regnum="0x369"/>
    <reg name="mhpmevent9"  bitsize="32" type="int" regnum="0x36a"/>
    <reg name="mhpmevent10" bitsize="32" type="int" regnum="0x36b"/>
    <reg name="mhpmevent11" bitsize="32" type="int" regnum="0x36c"/>
    <reg name="mhpmevent12" bitsize="32" type="int" regnum="0x36d"/>
    <reg name="mhpmevent13" bitsize="32" type="int" regnum="0x36e"/>
    <reg name="mhpmevent14" bitsize="32" type="int" regnum="0x36f"/>
    <reg name="mhpmevent15" bitsize="32" type="int" regnum="0x370"/>
    <reg name="mhpmevent16" bitsize="32" type="int" regnum="0x371"/>
    <reg name="mhpmevent17" bitsize="32" type="int" regnum="0x372"/>
    <reg name="mcycle"    bitsize="32" type="int" regnum="0xb41"/>
    <reg name="minstret"  bitsize="32" type="int" regnum="0xb43"/>
    <reg name="mcycleh"   bitsize="32" type="int" regnum="0xbc1"/>
    <reg name="minstreth" bitsize="32" type="int" regnum="0xbc3"/>
    <reg name="mhpmcounter3" bitsize="32" type="int" regnum="0xb44"/>
    <reg name="mhpmcounter4" bitsize="32" type="int" regnum="0xb45"/>
    <reg name="mhpmcounter5" bitsize="32" type="int" regnum="0xb46"/>
    <reg name="mhpmcounter6" bitsize="32" type="int" regnum="0xb47"/>
    <reg name="mhpmcounter7" bitsize="32" type="int" regnum="0xb48"/>
    <reg name="mhpmcounter8" bitsize="32" type="int" regnum="0xb49"/>
    <reg name="mhpmcounter9" bitsize="32" type="int" regnum="0xb4a"/>
    <reg name="mhpmcounter10" bitsize="32" type="int" regnum="0xb4b"/>
    <reg name="mhpmcounter11" bitsize="32" type="int" regnum="0xb4c"/>
    <reg name="mhpmcounter12" bitsize="32" type="int" regnum="0xb4d"/>
    <reg name="mhpmcounter13" bitsize="32" type="int" regnum="0xb4e"/>
    <reg name="mhpmcounter14" bitsize="32" type="int" regnum="0xb4f"/>
    <reg name="mhpmcounter15" bitsize="32" type="int" regnum="0xb50"/>
    <reg name="mhpmcounter16" bitsize="32" type="int" regnum="0xb51"/>
    <reg name="mhpmcounter17" bitsize="32" type="int" regnum="0xb52"/>
    <reg name="mhpmcounter3h" bitsize="32" type="int" regnum="0xbc4"/>
    <reg name="mhpmcounter4h" bitsize="32" type="int" regnum="0xbc5"/>
    <reg name="mhpmcounter5h" bitsize="32" type="int" regnum="0xbc6"/>
    <reg name="mhpmcounter6h" bitsize="32" type="int" regnum="0xbc7"/>
    <reg name="mhpmcounter7h" bitsize="32" type="int" regnum="0xbc8"/>
    <reg name="mhpmcounter8h" bitsize="32" type="int" regnum="0xbc9"/>
    <reg name="mhpmcounter9h" bitsize="32" type="int" regnum="0xbca"/>
    <reg name="mhpmcounter10h" bitsize="32" type="int" regnum="0xbcb"/>
    <reg name="mhpmcounter11h" bitsize="32" type="int" regnum="0xbcc"/>
    <reg name="mhpmcounter12h" bitsize="32" type="int" regnum="0xbcd"/>
    <reg name="mhpmcounter13h" bitsize="32" type="int" regnum="0xbce"/>
    <reg name="mhpmcounter14h" bitsize="32" type="int" regnum="0xbcf"/>
    <reg name="mhpmcounter15h" bitsize="32" type="int" regnum="0xbd0"/>
    <reg name="mhpmcounter16h" bitsize="32" type="int" regnum="0xbd1"/>
    <reg name="mhpmcounter17h" bitsize="32" type="int" regnum="0xbd2"/>
    <!-- M-mode Control vs Status -->
    <reg name="mxstatus"  bitsize="32" type="int" regnum="0x801"/>
    <reg name="mhcr"      bitsize="32" type="int" regnum="0x802"/>
    <reg name="mhint"     bitsize="32" type="int" regnum="0x806"/>
    <reg name="mraddr"    bitsize="32" type="int" regnum="0x821"/>
    <reg name="mexstatus" bitsize="32" type="int" regnum="0x822"/>
    <reg name="mnmicause" bitsize="32" type="int" regnum="0x823"/>
    <reg name="mnmipc"    bitsize="32" type="int" regnum="0x824"/>
    <reg name="mcpuid"    bitsize="32" type="int" regnum="0x1001"/>
    <!-- For Debug -->
    <reg name="dcsr" bitsize="32" type="int" regnum="0x7f1"/>
    <reg name="dpc"  bitsize="32" type="int" regnum="0x7f2"/>
    <!-- For Float Extension -->
    <reg name="fxcr"   bitsize="32" type="int" regnum="0x841"/>
    <!-- For P Extension -->
    <reg name="vxsat" bitsize="32" type="int" regnum="0x4a"/>
  </feature>
  <feature name="org.gnu.gdb.riscv.virtual">
    <reg name="priv"  bitsize="32" type="int" regnum="0x1041"/>
  </feature>
</target>'''

class RegNum():
    ZERO = 0
    RA = 1
    SP = 2
    GP = 3
    TP = 4
    T0 = 5
    T1 = 6
    T2 = 7
    FP = 8
    S1 = 9
    A0 = 10
    A1 = 11
    A2 = 12
    A3 = 13
    A4 = 14
    A5 = 15
    A6 = 16
    A7 = 17
    S2 = 18
    S3 = 19
    S4 = 20
    S5 = 21
    S6 = 22
    S7 = 23
    S8 = 24
    S9 = 25
    S10 = 26
    S11 = 27
    T3 = 28
    T4 = 29
    T5 = 30
    T6 = 31
    PC = 32


class GdbStub_RISC_V(GdbStub):
    # Version 1 (32-bit, no sp/fp): 18 registers
    ARCH_DATA_BLK_STRUCT    = "<IIIIIIIIIIIIIIIIII"
    # Version 2 (64-bit, no sp/fp): 18 registers
    ARCH_DATA_BLK_STRUCT_2  = "<QQQQQQQQQQQQQQQQQQ"
    # Version 3 (64-bit, with sp + fp for backtrace): 20 registers
    ARCH_DATA_BLK_STRUCT_3_64  = "<QQQQQQQQQQQQQQQQQQQQ"
    # Version 4 (32-bit, with sp + fp + 12 CSRs): 32 registers (128 bytes)
    ARCH_DATA_BLK_STRUCT_4_32  = "<IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII"

    GDB_SIGNAL_DEFAULT = 7

    GDB_G_PKT_NUM_REGS = 33  # Standard 32 GPRs + pc

    # Mapping from GDB 'p' packet regnum= values (decimal after parsing hex) to internal CSR indices
    # Based on real debugserver XML regnum= values
    # Note: GDB parses hex regnum= values and sends them as decimal in 'p' packets
    # e.g., regnum="0x341" -> GDB sends "p833" (0x341 = 833 decimal)
    GDB_REG_TO_CSR_IDX = {
        # Standard RISC-V CSRs - key is DECIMAL genus value GDB sends in 'p' packet
        # e.g., genus="0x341" in XML -> GDB sends "p833" (833 decimal = 0x341 hex)
        833: 36,   # mstatus (genus=0x341 -> 833 decimal)
        834: None, # misa (genus=0x342 -> 834 decimal)
        837: 38,   # mie (genus=0x345 -> 837 decimal)
        838: 40,   # mtvec (genus=0x346 -> 838 decimal)
        840: None, # mtvt (genus=0x348 -> 840 decimal)
        897: 41,   # mscratch (genus=0x381 -> 897 decimal)
        898: 34,   # mepc (genus=0x382 -> 898 decimal)
        899: 33,   # mcause (genus=0x383 -> 899 decimal)
        900: 35,   # mtval (genus=0x384 -> 900 decimal)
        901: 39,   # mip (genus=0x385 -> 901 decimal)
        902: None, # mnxti (genus=0x386 -> 902 decimal)
        903: 42,   # mintstatus (genus=0x387 -> 903 decimal)
        905: None, # mscratchcsw (genus=0x389 -> 905 decimal)
        906: None, # mscratchcswl (genus=0x38a -> 906 decimal)
        913: None, # mclicbase (genus=0x391 -> 913 decimal)
        993: None, # pmpcfg0 (genus=0x3e1 -> 993 decimal)
        1009: None, # pmpaddr0 (genus=0x3b1 -> 1009 decimal)
        0x347: None, # mcounteren (decimal 839)
        0x361: None, # mcountinhibit (decimal 865)
        0xb00: 44,   # mcycle (decimal 2816)
        0xb02: 43,   # minstret (decimal 2818)
        # LinkedSemi Custom CSRs
        0x801: None, # mxstatus (decimal 2049)
        0x802: None, # mhcr (decimal 2050)
        0x806: None, # mhint (decimal 2054)
        0x811: None, # mraddr (decimal 2065)
        0x822: 37,   # mexstatus (decimal 2082)
        0x823: None, # mnmicause (decimal 2083)
        0x824: None, # mnmipc (decimal 2084)
        0xc00: None, # mcpuid (decimal 3072)
        # Debug
        0x7b0: None, # dcsr (decimal 1968)
        0x7b1: None, # dpc (decimal 1969)
    }

    def __init__(self, logfile, elffile):
        super().__init__(logfile=logfile, elffile=elffile)
        self.registers = None
        self.csr_registers = {}  # Separate dict for CSR registers
        self.gdb_signal = self.GDB_SIGNAL_DEFAULT

        self.parse_arch_data_block()

    def parse_arch_data_block(self):
        arch_data_blk = self.logfile.get_arch_data()['data']
        self.arch_data_ver = self.logfile.get_arch_data()['hdr_ver']

        # Version 1: 32-bit original (18 regs, no sp)
        # Version 2: 64-bit original (18 regs, no sp)
        # Version 3: 64-bit with sp + fp (20 regs)
        # Version 4: 32-bit with sp + fp + 8 CSRs (28 regs)
        if self.arch_data_ver == 1:
            tu = struct.unpack(self.ARCH_DATA_BLK_STRUCT, arch_data_blk)
        elif self.arch_data_ver == 2:
            tu = struct.unpack(self.ARCH_DATA_BLK_STRUCT_2, arch_data_blk)
        elif self.arch_data_ver == 3:
            tu = struct.unpack(self.ARCH_DATA_BLK_STRUCT_3_64, arch_data_blk)
        elif self.arch_data_ver == 4:
            tu = struct.unpack(self.ARCH_DATA_BLK_STRUCT_4_32, arch_data_blk)

        self.registers = dict()

        # GPR registers (indices 0-32)
        self.registers[RegNum.RA] = tu[0]
        self.registers[RegNum.TP] = tu[1]
        self.registers[RegNum.T0] = tu[2]
        self.registers[RegNum.T1] = tu[3]
        self.registers[RegNum.T2] = tu[4]
        self.registers[RegNum.A0] = tu[5]
        self.registers[RegNum.A1] = tu[6]
        self.registers[RegNum.A2] = tu[7]
        self.registers[RegNum.A3] = tu[8]
        self.registers[RegNum.A4] = tu[9]
        self.registers[RegNum.A5] = tu[10]
        self.registers[RegNum.A6] = tu[11]
        self.registers[RegNum.A7] = tu[12]
        self.registers[RegNum.T3] = tu[13]
        self.registers[RegNum.T4] = tu[14]
        self.registers[RegNum.T5] = tu[15]
        self.registers[RegNum.T6] = tu[16]
        self.registers[RegNum.SP] = tu[17]
        self.registers[RegNum.FP] = tu[18]
        self.registers[RegNum.PC] = tu[19]

        # CSR registers (version 4 only): stored separately, not in self.registers
        # mcause=33, mepc=34, mtval=35, mstatus=36, mexstatus=37, mie=38, mip=39, mtvec=40
        # mscratch=41, mintstatus=42, minstret=43, mcycle=44
        if self.arch_data_ver == 4 and len(tu) >= 32:
            self.csr_registers[33] = tu[20]  # mcause
            self.csr_registers[34] = tu[21]  # mepc
            self.csr_registers[35] = tu[22]  # mtval
            self.csr_registers[36] = tu[23]  # mstatus
            self.csr_registers[37] = tu[24]  # mexstatus
            self.csr_registers[38] = tu[25]  # mie
            self.csr_registers[39] = tu[26]  # mip
            self.csr_registers[40] = tu[27]  # mtvec
            self.csr_registers[41] = tu[28]  # mscratch
            self.csr_registers[42] = tu[29]  # mintstatus
            self.csr_registers[43] = tu[30]  # minstret
            self.csr_registers[44] = tu[31]  # mcycle

    def handle_general_query_packet(self, pkt):
        if pkt.startswith(b'qSupported'):
            # Minimal support - just qXfer for target.xml
            self.put_gdb_packet(b'qXfer:features:read+;PacketSize=ffb')
        elif pkt.startswith(b'qXfer:features:read:target.xml'):
            # Handle qXfer:features:read:target.xml:offset,length
            try:
                pkt_str = pkt.decode('ascii')
                # Format: qXfer:features:read:target.xml:read:offset,length
                suffix = pkt_str.split('target.xml:', 1)[-1]
                if suffix.startswith('read:'):
                    suffix = suffix[5:]
                parts = suffix.split(',')
                offset = int(parts[0], 16)
                length = int(parts[1], 16) if len(parts) > 1 else 4096

                xml_data = RISCV_TARGET_XML
                data_len = len(xml_data)

                logger.info(f"qXfer request: offset=0x{offset:x}, length=0x{length:x}, total={data_len}")

                if offset >= data_len:
                    logger.info("qXfer: offset >= data_len, sending 'l'")
                    self.put_gdb_packet(b'l')
                    return

                # Limit chunk size to avoid GDB buffer overflow
                # GDB's 'length' parameter is the max characters it will accept
                max_len = min(length, 4000)
                end = min(offset + max_len, data_len)

                # Ensure we break at XML tag boundary
                if end < data_len:
                    search_start = max(offset, end - 50)
                    for i in range(end - 1, search_start - 1, -1):
                        if xml_data[i:i+1] == b'>':
                            end = i + 1
                            break

                chunk = xml_data[offset:end]
                is_last = (end >= data_len)

                logger.info(f"qXfer: sending chunk offset=0x{offset:x}..0x{end:x}, is_last={is_last}, chunk_len={len(chunk)}")

                # Send raw XML data (not hexlified) - matching cklink behavior
                # Some GDB servers send raw XML and GDB tolerates this
                response = (b'l' if is_last else b'm') + chunk
                self.put_gdb_packet(response)
            except Exception as e:
                logger.error(f"qXfer error: {e}")
                import traceback
                traceback.print_exc()
                self.put_gdb_packet(b'l')
        else:
            self.put_gdb_packet(b'')

    def handle_register_group_read_packet(self):
        # Version 1 and 4 are 32-bit, version 2 and 3 are 64-bit
        is_32bit = self.arch_data_ver in (1, 4)
        reg_fmt = "<I" if is_32bit else "<Q"

        idx = 0
        pkt = b''

        while idx < self.GDB_G_PKT_NUM_REGS:
            if idx in self.registers:
                bval = struct.pack(reg_fmt, self.registers[idx])
                pkt += binascii.hexlify(bval)
            else:
                # Register not in coredump -> unknown value
                # Send in "xxxxxxxx"
                length = 8 if is_32bit else 16
                pkt += b'x' * length

            idx += 1

        self.put_gdb_packet(pkt)

    def handle_register_single_read_packet(self, pkt):
        """
        Handle 'p' packet for reading a single register.
        Format: p<hex-regnum>
        Returns hex-encoded register value.
        """
        if len(pkt) < 2:
            self.put_gdb_packet(b'xxxxxxxx')
            return

        try:
            # Parse register number from packet (hex format)
            reg_str = pkt[1:].decode('ascii')
            regnum = int(reg_str, 16)
        except (ValueError, UnicodeDecodeError):
            logger.error(f"Failed to parse register number from: {pkt}")
            self.put_gdb_packet(b'xxxxxxxx')
            return

        logger.info(f"'p' packet: regnum=0x{regnum:x} ({regnum})")
        logger.info(f"  GPR registers available: {list(self.registers.keys())}")
        logger.info(f"  CSR registers available: {list(self.csr_registers.keys())}")

        is_32bit = self.arch_data_ver in (1, 4)
        val = None

        # Check if it's a GPR register (0-32)
        if regnum <= 32:
            if regnum in self.registers:
                val = self.registers[regnum]
                logger.info(f"  Found GPR reg {regnum} = 0x{val:x}")
            else:
                logger.info(f"  GPR reg {regnum} not in coredump")
        # Check if it's a CSR register
        elif regnum in self.GDB_REG_TO_CSR_IDX:
            csr_idx = self.GDB_REG_TO_CSR_IDX[regnum]
            if csr_idx is not None and csr_idx in self.csr_registers:
                val = self.csr_registers[csr_idx]
                logger.info(f"  Found CSR at index {csr_idx} (from GDB reg 0x{regnum:x}) = 0x{val:x}")
            else:
                logger.info(f"  CSR reg 0x{regnum:x} -> idx {csr_idx}, not available in coredump")
        else:
            logger.info(f"  Register 0x{regnum:x} not found in any mapping")

        if val is not None:
            if is_32bit:
                bval = struct.pack('<I', val & 0xFFFFFFFF)
            else:
                bval = struct.pack('<Q', val & 0xFFFFFFFFFFFFFFFF)
            self.put_gdb_packet(binascii.hexlify(bval))
        else:
            # Register not in coredump -> return 0
            logger.info(f"  Returning 0 (not in coredump)")
            zero_bytes = b'00000000' if is_32bit else b'0000000000000000'
            self.put_gdb_packet(zero_bytes)

    def handle_register_write_packet(self, pkt):
        # We don't support register write in coredump mode
        self.put_gdb_packet(b'E00')

    def run(self, conn):
        # Store socket for get_gdb_packet() to use
        self.socket = conn
        try:
            while True:
                pkt = self.get_gdb_packet()
                if not pkt:
                    logger.warning("get_gdb_packet returned None (connection closed)")
                    break

                # Log all packets received
                pkt_repr = pkt[:80] if len(pkt) > 80 else pkt
                logger.info(f"Got packet: {pkt_repr}")

                if pkt == b'?':
                    logger.info(">>> Stop reply packet - sending S%02x" % self.gdb_signal)
                    self.put_gdb_packet(b'S%02x' % self.gdb_signal)
                elif pkt == b'k':
                    logger.info("GDB requested kill")
                    break
                elif pkt.startswith(b'qSupported'):
                    self.handle_general_query_packet(pkt)
                elif pkt.startswith(b'qXfer:features:read:target.xml'):
                    self.handle_general_query_packet(pkt)
                elif pkt == b'qAttached':
                    self.put_gdb_packet(b'1')
                elif pkt == b'qOffsets':
                    self.put_gdb_packet(b'')
                elif pkt.startswith(b'g'):
                    self.handle_register_group_read_packet()
                elif pkt.startswith(b'p'):
                    self.handle_register_single_read_packet(pkt)
                elif pkt.startswith(b'P'):
                    self.handle_register_write_packet(pkt)
                elif pkt.startswith(b'm'):
                    self.handle_memory_read_packet(pkt)
                elif pkt.startswith(b'M'):
                    self.handle_memory_write_packet(pkt)
                elif pkt == b'c':
                    logger.info("Continue not supported in coredump mode")
                    self.put_gdb_packet(b'S05')
                elif pkt == b's':
                    logger.info("Step not supported in coredump mode")
                    self.put_gdb_packet(b'S05')
                elif pkt.startswith(b'Z'):
                    self.put_gdb_packet(b'')
                elif pkt.startswith(b'z'):
                    self.put_gdb_packet(b'')
                elif pkt == b'Hg':
                    self.put_gdb_packet(b'OK')
                elif pkt.startswith(b'QNonStop'):
                    # Handle QNonStop:0 (disable non-stop mode)
                    logger.info(f"QNonStop packet: {pkt}")
                    self.put_gdb_packet(b'OK')
                elif pkt.startswith(b'qTStatus'):
                    # Trace status query - return empty (no trace) like real debugserver
                    logger.info("qTStatus query - responding with empty")
                    self.put_gdb_packet(b'')
                elif pkt.startswith(b'vMustReplyEmpty'):
                    # Must reply with empty packet
                    logger.info("vMustReplyEmpty query - responding with empty")
                    self.put_gdb_packet(b'')
                else:
                    logger.info(f"Unhandled packet, returning OK: {pkt[:50]}")
                    self.put_gdb_packet(b'OK')
        except Exception as e:
            logger.error(f"Error in run: {e}")
            import traceback
            traceback.print_exc()