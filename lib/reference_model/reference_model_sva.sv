//
// Copyright 2022 OpenHW Group
//
// Licensed under the Solderpad Hardware Licence, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://solderpad.org/licenses/
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
//


`ifndef __REFERENCE_MODEL_SVA_SV__
`define __REFERENCE_MODEL_SVA_SV__

import uvma_rvfi_pkg::*;
import uvm_pkg::*;

module rvfi_compare_sva(
  input logic reset_n,
  rvfi_if_t rvfi_core,
  rvfi_if_t rvfi_rm
);

  rvfi_pc_a: assert property(@ (posedge rvfi_core.clk) disable iff (!reset_n)
    rvfi_core.valid |=> rvfi_rm.valid ##0 (rvfi_rm.pc_rdata == $past(rvfi_core.pc_rdata)))
    else `uvm_error("RVFI_PC", $sformatf("rvfi_rm.pc_rdata=%0h rvfi_core.pc_rdata=%0h",rvfi_rm.pc_rdata, rvfi_core.pc_rdata));

  rvfi_insn_a: assert property(@ (posedge rvfi_core.clk) disable iff (!reset_n)
    rvfi_core.valid |=> rvfi_rm.valid ##0 (rvfi_rm.insn == $past(rvfi_core.insn)))
    else `uvm_error("RVFI_INSN", $sformatf("rvfi_rm.insn=%0h rvfi_core.insn=%0h", rvfi_rm.insn, rvfi_core.insn));

  compare_pc_a: assert property(@ (posedge rvfi_core.clk) disable iff (!reset_n)
    rvfi_rm.valid |-> (rvfi_rm.pc_rdata == $past(rvfi_core.pc_rdata))) 
    else `uvm_error("RVFI_INSN", $sformatf("rvfi_rm.pc_rdata=%0h rvfi_core.pc_rdata=%0h", rvfi_rm.pc_rdata, rvfi_core.pc_rdata));

  compare_insn_a: assert property(@ (posedge rvfi_core.clk) disable iff (!reset_n)
    rvfi_rm.valid |-> (rvfi_rm.insn == $past(rvfi_core.insn))) 
    else `uvm_error("RVFI_INSN", $sformatf("rvfi_rm.insn=%0h rvfi_core.insn=%0h", rvfi_rm.insn, rvfi_core.insn));

//TODO: implement the rest of the checks from rvfi_compare as SVAs here


endmodule
`endif // __REFERENCE_MODEL_SVA_SV__