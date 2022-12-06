//
// Copyright 2022 OpenHW Group
// Copyright 2022 Silicon Labs
//
// Licensed under the Solderpad Hardware Licence, Version 2.1 (the "License");
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

module uvmt_cv32e40s_support_logic
  import uvm_pkg::*;
  import uvma_rvfi_pkg::*;
  import cv32e40s_pkg::*;
  (
    uvma_rvfi_instr_if rvfi,
    uvmt_cv32e40s_input_to_support_logic_module_if.driver_mp in_support_if,
    uvmt_cv32e40s_support_logic_for_assert_coverage_modules_if.master_mp out_support_if
  );

  // ---------------------------------------------------------------------------
  // Local parameters
  // ---------------------------------------------------------------------------



  // ---------------------------------------------------------------------------
  // Local variables
  // ---------------------------------------------------------------------------

  // Signal indicates an exception is active for a multiop instruction,
  // in other words a subop has triggered an exception. WB stage timing.
  logic exception_active;

  // Signal indicates data bus address phase completed last cycle
  logic data_bus_gnt_q;


  // ---------------------------------------------------------------------------
  // Support logic blocks
  // ---------------------------------------------------------------------------


  // Check if a new obi data req arrives after an exception is triggered.
  // Used to verify exception timing with multiop instruction
  always @(posedge in_support_if.clk or negedge in_support_if.rst_n) begin
    if (!in_support_if.rst_n) begin
      out_support_if.req_after_exception <= 0;
      exception_active <= 0;
      data_bus_gnt_q <= 0;
    end else  begin
      // set prev bus gnt
      data_bus_gnt_q <= in_support_if.data_bus_gnt;

      // is a trap taken in WB?
      if (in_support_if.ctrl_fsm_o.pc_set && (in_support_if.ctrl_fsm_o.pc_mux == PC_TRAP_DBE || in_support_if.ctrl_fsm_o.pc_mux == PC_TRAP_EXC)) begin
        if (in_support_if.data_bus_req && data_bus_gnt_q) begin
          out_support_if.req_after_exception <= 1;
        end
        exception_active <= 1;
      end else if (rvfi.rvfi_valid) begin
        exception_active <= 0;
        out_support_if.req_after_exception <= 0;

      end else if (exception_active && data_bus_gnt_q && in_support_if.data_bus_req) begin
        out_support_if.req_after_exception <= 1;
      end
    end

  end //always

  // Support logic for obi interfaces:

  //obi data bus:
  uvmt_cv32e40s_obi_phases_monitor data_bus_obi_phases_monitor (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .obi_req (in_support_if.data_bus_req),
    .obi_gnt (in_support_if.data_bus_gnt),
    .obi_rvalid (in_support_if.data_bus_rvalid),

    .addr_ph_cont (out_support_if.data_bus_addr_ph_cont),
    .resp_ph_cont (out_support_if.data_bus_resp_ph_cont),
    .v_addr_ph_cnt (out_support_if.data_bus_v_addr_ph_cnt)
  );

  //obi instr bus:
  uvmt_cv32e40s_obi_phases_monitor instr_bus_obi_phases_monitor (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .obi_req (in_support_if.instr_bus_req),
    .obi_gnt (in_support_if.instr_bus_gnt),
    .obi_rvalid (in_support_if.instr_bus_rvalid),

    .addr_ph_cont (out_support_if.instr_bus_addr_ph_cont),
    .resp_ph_cont (out_support_if.instr_bus_resp_ph_cont),
    .v_addr_ph_cnt (out_support_if.instr_bus_v_addr_ph_cnt)
  );

  //obi protocol between alignmentbuffer (ab) and instructoin (i) interface (i) mpu (m) (=> abiim)
  uvmt_cv32e40s_obi_phases_monitor abiim_bus_obi_phases_monitor (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .obi_req (in_support_if.abiim_bus_req),
    .obi_gnt (in_support_if.abiim_bus_gnt),
    .obi_rvalid (in_support_if.abiim_bus_rvalid),

    .addr_ph_cont (out_support_if.abiim_bus_addr_ph_cont),
    .resp_ph_cont (out_support_if.abiim_bus_resp_ph_cont),
    .v_addr_ph_cnt (out_support_if.abiim_bus_v_addr_ph_cnt)
  );

  //obi protocol between LSU (l) MPU (m) and LSU (l) (=> lml)
  uvmt_cv32e40s_obi_phases_monitor lml_bus_obi_phases_monitor (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .obi_req (in_support_if.lml_bus_req),
    .obi_gnt (in_support_if.lml_bus_gnt),
    .obi_rvalid (in_support_if.lml_bus_rvalid),

    .addr_ph_cont (out_support_if.lml_bus_addr_ph_cont),
    .resp_ph_cont (out_support_if.lml_bus_resp_ph_cont),
    .v_addr_ph_cnt (out_support_if.lml_bus_v_addr_ph_cnt)
  );

  //obi protocol between LSU (l) respons (r) filter (f) and the OBI (o) data (d) interface (i) (=> lrfodi)
  uvmt_cv32e40s_obi_phases_monitor lrfodi_bus_obi_phases_monitor (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .obi_req (in_support_if.lrfodi_bus_req),
    .obi_gnt (in_support_if.lrfodi_bus_gnt),
    .obi_rvalid (in_support_if.lrfodi_bus_rvalid),

    .addr_ph_cont (out_support_if.lrfodi_bus_addr_ph_cont),
    .resp_ph_cont (out_support_if.lrfodi_bus_resp_ph_cont),
    .v_addr_ph_cnt (out_support_if.lrfodi_bus_v_addr_ph_cnt)
  );


  uvmt_cv32e40s_sl_req_setting_in_respons is_store_in_respons_data_i
  (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .gnt (in_support_if.data_bus_gnt),
    .req (in_support_if.data_bus_req),
    .rvalid (in_support_if.data_bus_rvalid),
    .setting_i (in_support_if.req_is_store), //xsecure_if.core_i_load_store_unit_i_bus_trans_we),

    .setting_in_respons (out_support_if.is_store_in_respons_data)
  );


  uvmt_cv32e40s_sl_req_setting_in_respons integrity_in_respons_instr_i
  (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .gnt (in_support_if.instr_bus_gnt),
    .req (in_support_if.instr_bus_req),
    .rvalid (in_support_if.instr_bus_rvalid),
    .setting_i (in_support_if.req_instr_integrity),

    .setting_in_respons (out_support_if.integrity_in_respons_instr)
  );


  uvmt_cv32e40s_sl_req_setting_in_respons integrity_in_respons_data_i
  (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .gnt (in_support_if.data_bus_gnt),
    .req (in_support_if.data_bus_req),
    .rvalid (in_support_if.data_bus_rvalid),
    .setting_i (in_support_if.req_data_integrity),

    .setting_in_respons (out_support_if.integrity_in_respons_data)
  );


  uvmt_cv32e40s_sl_req_gntpar_error_in_resp sl_req_gntpar_error_in_resp_instr_i
  (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .gnt (in_support_if.instr_bus_gnt),
    .gntpar (in_support_if.instr_bus_gntpar),
    .req (in_support_if.instr_bus_req),
    .rvalid (in_support_if.instr_bus_rvalid),

    .gnt_error_in_respons (out_support_if.gnt_error_in_respons_instr)
  );


  uvmt_cv32e40s_sl_req_gntpar_error_in_resp sl_req_gntpar_error_in_resp_data_i
  (
    .clk_i (in_support_if.clk),
    .rst_ni (in_support_if.rst_n),

    .gnt (in_support_if.data_bus_gnt),
    .gntpar (in_support_if.data_bus_gntpar),
    .req (in_support_if.data_bus_req),
    .rvalid (in_support_if.data_bus_rvalid),

    .gnt_error_in_respons (out_support_if.gnt_error_in_respons_data)
  );

endmodule : uvmt_cv32e40s_support_logic

module uvmt_cv32e40s_obi_phases_monitor
  import uvm_pkg::*;
  (
    input logic clk_i,
    input logic rst_ni,

    input logic obi_req,
    input logic obi_gnt,
    input logic obi_rvalid,


    // continued address and respons phase indicators, indicates address and respons phases
    // of more than one cycle
    output logic addr_ph_cont,
    output logic resp_ph_cont,

    // address phase counter, used to verify no response phase preceedes an address phase
    output integer v_addr_ph_cnt
  );

  logic addr_ph_valid;
  logic rsp_ph_valid;
  logic obi_rready;

  assign obi_rready = 1'b1; //This is an assumption

  assign addr_ph_valid = obi_req == 1'b1 && obi_gnt == 1'b1;
  assign rsp_ph_valid = obi_rready == 1'b1 && obi_rvalid == 1'b1;


  always @(posedge clk_i, negedge rst_ni) begin
    if (!rst_ni) begin
      addr_ph_cont <= 1'b0;
    end
    else begin
      if (obi_req == 1'b1 && obi_gnt == 1'b0) begin
        addr_ph_cont <= 1'b1;
      end
      else begin
        addr_ph_cont <= 1'b0;
      end
    end
  end

  always @(posedge clk_i, negedge rst_ni) begin
    if (!rst_ni) begin
      resp_ph_cont <= 1'b0;
    end
    else begin
      if (obi_rvalid == 1'b1 && obi_rready == 1'b0) begin
        resp_ph_cont <= 1'b1;
      end
      else begin
        resp_ph_cont <= 1'b0;
      end
    end
  end

  always @(posedge clk_i, negedge rst_ni) begin
    if (!rst_ni) begin
      v_addr_ph_cnt <= '0;
    end
    else begin
      if (addr_ph_valid && !rsp_ph_valid) begin
        v_addr_ph_cnt <= v_addr_ph_cnt + 1'b1;
      end
      else if (!addr_ph_valid && rsp_ph_valid && v_addr_ph_cnt > 0) begin
        v_addr_ph_cnt <= v_addr_ph_cnt - 1'b1;
      end
    end
  end
endmodule : uvmt_cv32e40s_obi_phases_monitor

module uvmt_cv32e40s_sl_req_setting_in_respons
  import uvm_pkg::*;
  import cv32e40s_pkg::*;
  (
    input logic rst_ni,
    input logic clk_i,

    input logic gnt,
    input logic req,
    input logic rvalid,
    input logic setting_i,

    output logic setting_in_respons
  );


  logic [2:0] fifo_req_settings;
  logic req_setting;
  logic [1:0] req_setting_pointer;

  assign setting_in_respons = rvalid && fifo_req_settings[2];
  assign req_setting = setting_i && rst_ni;

  always @(posedge clk_i, negedge rst_ni) begin
    if(!rst_ni) begin
      fifo_req_settings <= 3'b000;
      req_setting_pointer = 2;
    end else begin

      if ((gnt && req) && !rvalid) begin
        fifo_req_settings[req_setting_pointer] <= req_setting;
        req_setting_pointer <= req_setting_pointer - 1;

      end else if (!(gnt && req) && rvalid) begin
        req_setting_pointer <= req_setting_pointer + 1;
        fifo_req_settings <= {fifo_req_settings[1:0], 1'b0};

      end else if ((gnt && req) && rvalid) begin
        fifo_req_settings[req_setting_pointer] <= req_setting;
        fifo_req_settings <= {fifo_req_settings[1:0], 1'b0};

      end
    end
  end

endmodule : uvmt_cv32e40s_sl_req_setting_in_respons

module uvmt_cv32e40s_sl_req_gntpar_error_in_resp
  import uvm_pkg::*;
  import cv32e40s_pkg::*;
  (
    input logic rst_ni,
    input logic clk_i,

    input logic gnt,
    input logic gntpar,
    input logic req,
    input logic rvalid,

    output logic gnt_error_in_respons
  );


  logic [2:0] fifo_req_errors;
  logic req_error;
  logic [1:0] req_error_pointer;
  logic req_error_prev;

  assign gnt_error_in_respons = rvalid && fifo_req_errors[2];
  assign req_error = ((gnt == gntpar || req_error_prev) && req) && rst_ni;

  always @(posedge clk_i, negedge rst_ni) begin
    if(!rst_ni) begin
      fifo_req_errors <= 3'b000;
      req_error_pointer = 2;
      req_error_prev <= 1'b0;
    end else begin

      if ((req && gnt) && !rvalid) begin
        fifo_req_errors[req_error_pointer] <= req_error;
        req_error_pointer <= req_error_pointer - 1;

      end else if (!(req && gnt) && rvalid) begin
        req_error_pointer <= req_error_pointer + 1;
        fifo_req_errors <= {fifo_req_errors[1:0], 1'b0};

      end else if ((req && gnt) && rvalid) begin
        fifo_req_errors[req_error_pointer] <= req_error;
        fifo_req_errors <= {fifo_req_errors[1:0], 1'b0};

      end

      if ((req && !gnt)) begin
        req_error_prev <= req_error;
      end else begin
        req_error_prev <= 1'b0;
      end
    end
  end

endmodule : uvmt_cv32e40s_sl_req_gntpar_error_in_resp

