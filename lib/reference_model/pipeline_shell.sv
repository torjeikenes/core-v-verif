// Copyright 2024 Torje Nygaard Eikenes
//
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Licensed under the Solderpad Hardware License v 2.1 (the "License"); you may
// not use this file except in compliance with the License, or, at your option,
// the Apache License version 2.0.
//
// You may obtain a copy of the License at
// https://solderpad.org/licenses/SHL-2.1/
//
// Unless required by applicable law or agreed to in writing, any work
// distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
// WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//
// See the License for the specific language governing permissions and
// limitations under the License.

`ifndef __PIPELINE_SHELL_SV__
`define __PIPELINE_SHELL_SV__


`define DUT_PATH dut_wrap.cv32e40s_wrapper_i
`define CONTROLLER_FSM `DUT_PATH.core_i.controller_i.controller_fsm_i

`define CSR_MSTATUS_ADDR    32'h300
`define CSR_MIE_ADDR        32'h304

`define INSN_DRET           32'h7b200073

typedef struct packed {
    st_rvfi rvfi;
    logic valid;
} pipe_stage_t;

module if_stage
    import iss_wrap_pkg::*;
    (
        input logic clk,
        input logic rst_n,
        input logic step, 
        input logic flush_i,

        output pipe_stage_t if_id_pipe_o
    );

    always_ff @(posedge clk) begin
        if(flush_i) begin
            if_id_pipe_o.rvfi <= '0;
            if_id_pipe_o.valid <= 1'b0;
        end else if(step) begin
            if_id_pipe_o.rvfi <= iss_step();
            if_id_pipe_o.valid <= 1'b1;
        end
        else begin
            if_id_pipe_o.rvfi <= if_id_pipe_o.rvfi;
            if_id_pipe_o.valid <= if_id_pipe_o.valid;
        end
    end
endmodule

module id_stage
    import iss_wrap_pkg::*;
    (
        input logic clk,
        input logic rst_n,
        input logic step, 
        input logic flush_i,
        input pipe_stage_t pipe_i,

        output pipe_stage_t pipe_o
    );

    always_ff @(posedge clk) begin
        if (flush_i) begin
            pipe_o.rvfi <= '0;
            pipe_o.valid <= 1'b0;
        end else if(step) begin
            pipe_o.rvfi <= pipe_i.rvfi;
            pipe_o.valid <= pipe_i.valid;
        end
        else begin
            pipe_o.rvfi <= pipe_o.rvfi;
            pipe_o.valid <= pipe_o.valid;
        end
    end
endmodule

module ex_stage
    import iss_wrap_pkg::*;
    (
        input logic clk,
        input logic rst_n,
        input logic step, 
        input logic flush_i,
        input pipe_stage_t pipe_i,

        output pipe_stage_t pipe_o
    );

    always_ff @(posedge clk) begin
        if (flush_i) begin
            pipe_o.rvfi <= '0;
            pipe_o.valid <= 1'b0;
        end else if(step) begin
            pipe_o.rvfi <= pipe_i.rvfi;
            pipe_o.valid <= pipe_i.valid;
        end
        else begin
            pipe_o.rvfi <= pipe_o.rvfi;
            pipe_o.valid <= pipe_i.valid; 
        end
    end
endmodule

module wb_stage
    import iss_wrap_pkg::*;
    (
        input logic clk,
        input logic rst_n,
        input logic step, 
        input logic flush_i,
        input pipe_stage_t pipe_i,

        output pipe_stage_t pipe_o
    );

    always_ff @(posedge clk) begin
        if(step) begin
            pipe_o.rvfi <= pipe_i.rvfi;
            pipe_o.valid <= pipe_i.valid;
        end
        else begin
            pipe_o.rvfi <= pipe_o.rvfi;
            pipe_o.valid <= 1'b0; //Only output valid at first valid clock cycle
        end
    end

endmodule

module controller
    import iss_wrap_pkg::*;
    (
        input logic clk, 
        input logic rst_n,
        input logic valid,
        input logic [31:0] irq_i,
        input logic debug_req_i,

        input pipe_stage_t if_id_pipe_i,
        input pipe_stage_t id_ex_pipe_i,
        input pipe_stage_t ex_wb_pipe_i, 
        input pipe_stage_t wb_pipe_i, 

        output logic if_step_o,
        output logic id_step_o,
        output logic ex_step_o,
        output logic wb_step_o,

        output logic if_flush_o,
        output logic id_flush_o,
        output logic ex_flush_o,
        output logic wb_flush_o
    );

    localparam LSU_DEPTH = 2;
    localparam LSU_CNT_WIDTH = $clog2(LSU_DEPTH+1);
    localparam PIPELINE_DEPTH = 4;
    localparam ROLLBACK_STEPS = 2;


    logic lsu_interruptible;
    logic [LSU_CNT_WIDTH-1:0] lsu_cnt;
    logic [LSU_CNT_WIDTH-1:0] lsu_cnt_q;
    logic lsu_cnt_up;
    logic lsu_cnt_down;
    logic mem_in_ex;
    logic mem_in_wb;
    logic mem_in_lsu;

    int     pipe_count; // Count the number of filled pipeline stages
    logic   pipeline_full;
    logic   flush_pipeline;
    logic   flush_pipeline_q;
    logic   step;
    logic   step_q;

    logic   interrupt_enabled;
    logic   interrupt_allowed;
    logic   interrupt_taken, interrupt_taken_q;
    logic   [31:0] irq_q, irq_qq;

    logic   [MAX_XLEN-1:0] wb_mstatus;
    logic   wb_mstatus_mie;
    logic   [MAX_XLEN-1:0] wb_mie, mie;

    logic   debug_taken, debug_taken_q;
    logic   debug_mode;
    ////////////////////////////////////////////////////////////////////////////
    // STEP CONTROL
    ////////////////////////////////////////////////////////////////////////////

    assign pipeline_full = pipe_count >= (PIPELINE_DEPTH - 1);

    // Count the amount of filled pipeline stages at the start or after a flush 
    always_ff @(posedge clk) begin
        if (rst_n == 1'b0 || flush_pipeline)begin 
            pipe_count <= 0;
            flush_pipeline_q <= 1'b0;
        end else if (pipe_count < (PIPELINE_DEPTH - 1)) begin
            pipe_count <= pipe_count + 1;
            flush_pipeline_q <= flush_pipeline;
        end else begin
            pipe_count <= pipe_count;
            flush_pipeline_q <= flush_pipeline;
        end
    end

    // step the pipeline until the first stages are filled up to be in sync with the core
    always_comb begin
        if (pipe_count < (PIPELINE_DEPTH - 1)) begin
            step <= 1'b1;
        end
        else if (rvfi_i.rvfi_valid) begin
            step <= 1'b1;
        end
        else begin
            step <= 1'b0;
        end

    end

    // Halt the the pipeline when a debug is taken
    always_comb begin
        if_step_o <= step && !debug_taken;
        id_step_o <= step && !debug_taken;
        ex_step_o <= step && !debug_taken;
        wb_step_o <= step;
    end

    ////////////////////////////////////////////////////////////////////////////
    // FLUSH CONTROL
    ////////////////////////////////////////////////////////////////////////////

    assign flush_pipeline = interrupt_taken || debug_taken_q;

    always_comb begin
        if_flush_o <= flush_pipeline;
        id_flush_o <= flush_pipeline;
        ex_flush_o <= flush_pipeline;
        wb_flush_o <= flush_pipeline;
    end

    ////////////////////////////////////////////////////////////////////////////
    // Delayed CSRs
    ////////////////////////////////////////////////////////////////////////////

    // The effects of the MSTATUS and MIE CSRs have to be applied in the WB stage
    // to properly match the core. When these CSRs are changed in the ISS in IF,
    // they travel through the pipeline, and are applied in the WB stage through 
    // the iss_intr() function.


    logic mstatus_found, mie_found;
    always_comb begin
        mstatus_found   = 1'b0;
        mie_found       = 1'b0;
        wb_mie          = '0;

        foreach(ex_wb_pipe_i.rvfi.csr_valid[i]) begin
            if(ex_wb_pipe_i.rvfi.csr_valid[i]) begin 
                if(ex_wb_pipe_i.rvfi.csr_addr[i] == `CSR_MSTATUS_ADDR) begin // IF MSTATUS
                    wb_mstatus = ex_wb_pipe_i.rvfi.csr_wdata[i];
                    wb_mstatus_mie = (wb_mstatus & (32'h00000008 )) != 0;
                    mstatus_found = 1'b1;
                end            
                if(ex_wb_pipe_i.rvfi.csr_addr[i] == `CSR_MIE_ADDR) begin 
                    wb_mie = ex_wb_pipe_i.rvfi.csr_wdata[i];
                    mie_found = 1'b1;
                end
            end
        end

        if(!mstatus_found) begin
            wb_mstatus = wb_mstatus;
            wb_mstatus_mie = 0;
        end

        if(!mie_found) begin
            wb_mie = wb_mie;
        end
    end

    // Check if the MIE bit is set in the MSTATUS register

    always_ff @(posedge clk, negedge rst_n) begin
        if (rst_n == 1'b0) begin
            mie <= 1'b0;
        end else if (mie_found && step) begin
            mie <= wb_mie;
        end else begin
            mie <= mie;
        end
    end

    ////////////////////////////////////////////////////////////////////////////
    // INTERRUPTS
    ////////////////////////////////////////////////////////////////////////////

    // interrupt_enabled only enabled interrupts when the mstatus.mie bit is set 
    // in WB and it is disabled the cycle after an interrupt is taken.
    always_ff @(posedge clk, negedge rst_n) begin
        if (rst_n == 1'b0) begin
            interrupt_enabled <= 1'b0;
        end else if (wb_mstatus_mie && step) begin
            interrupt_enabled <= 1'b1;
        end else if (interrupt_taken_q) begin
            interrupt_enabled <= 1'b0;
        end else begin
            interrupt_enabled <= interrupt_enabled;
        end
    end


    // Temporarily inject the interrupt allowed signal directly from the core
    assign interrupt_allowed = `CONTROLLER_FSM.interrupt_allowed && interrupt_enabled; 
    // TODO: Use the content of the pipeline stages to recreate the interrupt_allowed signal
    // signal independently of the core
    //assign interrupt_allowed = lsu_interruptible && debug_interruptible && !fencei_ongoing && !clic_ptr_in_pipeline && 
    //                           sequence_interruptible && !interrupt_blanking_q && !csr_flush_ack_q && !(ctrl_fsm_cs == SLEEP);

    // Delay the irq 2 cycles take the correct irq after the flush
    always_ff @(posedge clk, negedge rst_n) begin
        if (rst_n == 1'b0) begin
            irq_q <= 1'b0;
            irq_qq <= 1'b0;
        end else begin
            irq_q <= irq_i;
            irq_qq <= irq_q;
        end
    end

    // Call iss_intr every cycle to inform the ISS changes in irq and mie, and 
    // determine if an interrupt can be taken
    always_ff @(posedge clk) begin
        interrupt_taken <= iss_intr(irq_qq, mie, interrupt_allowed, ROLLBACK_STEPS);
    end

    // Delay interrupt_taken to properly time the interrupt_enabled signal
    always_ff @(posedge clk, negedge rst_n) begin
        if (rst_n == 1'b0) begin
            interrupt_taken_q <= 1'b0;
        end else begin
            interrupt_taken_q <= interrupt_taken;
        end
    end

    ////////////////////////////////////////////////////////////////////////////
    // LSU INTERRUPTIBLE
    ////////////////////////////////////////////////////////////////////////////
    // NOTE: Not currently used, but required when not directly injecting interrupt_allowed from the core

    // lsu_cnt holds the amount of memory requests without a response in the LSU

    // Increase lsu_cnt when a memory operation is in EX
    assign mem_in_ex = id_ex_pipe_i.rvfi.mem_rmask || id_ex_pipe_i.rvfi.mem_wmask;
    assign lsu_cnt_up = mem_in_ex && step_q;

    //assign mem_in_wb = ex_wb_pipe_i.rvfi.mem_rmask || ex_wb_pipe_i.rvfi.mem_wmask;

    // Decrease lsu_cnt from the first clock when a memory operation is moved to WB
    assign lsu_cnt_down = mem_in_ex && step;

    always_comb begin
        case ({lsu_cnt_up, lsu_cnt_down})
            2'b00: begin
                lsu_cnt <= lsu_cnt_q;
            end
            2'b01: begin 
                lsu_cnt <= lsu_cnt_q - 1'b1;
            end
            2'b10: begin
                lsu_cnt <= lsu_cnt_q + 1'b1;
            end
            2'b11: begin
                lsu_cnt <= lsu_cnt_q;
            end
            default:;
        endcase 
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if (rst_n == 1'b0)begin 
            lsu_cnt_q <= '0;
        end else begin
            lsu_cnt_q <= lsu_cnt;
        end
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if (rst_n == 1'b0) begin 
            step_q <= 1'b0;
        end else begin
            step_q <= step;
        end
    end

    assign lsu_interruptible = (lsu_cnt_q == '0);


    ////////////////////////////////////////////////////////////////////////////
    // DEBUG
    ////////////////////////////////////////////////////////////////////////////

    // RVFI has dbg_mode high during dret, but we want to disable debug_mode at 
    // dret and not the next instruction, so we check if insn is dret
    always_comb begin
        if (wb_pipe_i.rvfi.insn == `INSN_DRET) begin
            debug_mode <= 1'b0;
        end else begin
            debug_mode <= wb_pipe_i.rvfi.dbg_mode;
        end
    end

    logic debug_req_q; 
    always_ff @(posedge clk, negedge rst_n) begin
        if(rst_n == 1'b0) begin
            debug_req_q <= 1'b0; 
        end else begin 
            debug_req_q <= debug_req_i;
        end
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if(rst_n == 1'b0) begin
            debug_taken <= 1'b0;
        end else begin
            // Only allow debug if we are not in debug mode, and the pipeline is not being filled up(e.g. at very beginning of the simulation)
            debug_taken <= iss_set_debug(debug_req_q, ROLLBACK_STEPS, !debug_mode && pipeline_full); 
        end
    end

    always_ff @(posedge clk, negedge rst_n) begin
        if(rst_n == 1'b0) begin
            debug_taken_q <= 1'b0;
        end else begin
            debug_taken_q <= debug_taken;
        end
    end

endmodule

module pipeline_shell 
    import uvma_rvfi_pkg::*;
    import iss_wrap_pkg::*;
    (
        uvma_clknrst_if_t clknrst_if,
        uvma_rvfi_instr_if_t rvfi_i,
        uvma_interrupt_if_t interrupt_if_i,
        logic debug_req_i,
        rvfi_if_t rvfi_o
    );

    pipe_stage_t if_id_pipe;
    pipe_stage_t id_ex_pipe;
    pipe_stage_t ex_wb_pipe;
    pipe_stage_t wb_pipe;
    logic if_step;
    logic id_step;
    logic ex_step;
    logic wb_step;
    logic if_flush;
    logic id_flush;
    logic ex_flush;
    logic wb_flush;
    logic interrupt_allowed;
    logic interrupt_taken;

    controller controller_i(
        .clk                    (clknrst_if.clk     ),
        .rst_n                  (clknrst_if.reset_n ),
        .valid                  (                   ),
        .irq_i                  (interrupt_if_i.irq ),
        .debug_req_i            (debug_req_i        ),
        .if_id_pipe_i           (if_id_pipe         ),
        .id_ex_pipe_i           (id_ex_pipe         ),
        .ex_wb_pipe_i           (ex_wb_pipe         ),
        .wb_pipe_i              (wb_pipe            ),
        .if_step_o              (if_step            ),
        .id_step_o              (id_step            ),
        .ex_step_o              (ex_step            ),
        .wb_step_o              (wb_step            ),
        .if_flush_o              (if_flush            ),
        .id_flush_o              (id_flush            ),
        .ex_flush_o              (ex_flush            ),
        .wb_flush_o              (wb_flush            )
    );

    if_stage if_stage_i(
        .clk            (clknrst_if.clk     ),
        .rst_n          (clknrst_if.reset_n ),
        .step           (if_step            ),
        .flush_i         (if_flush            ),
        .if_id_pipe_o   (if_id_pipe         )
        );
    
    id_stage id_stage_i(
        .clk            (clknrst_if.clk     ),
        .rst_n          (clknrst_if.reset_n ),
        .step           (id_step            ),
        .flush_i         (id_flush            ),
        .pipe_i         (if_id_pipe         ),
        .pipe_o         (id_ex_pipe         )
    );

    ex_stage ex_stage_i(
        .clk            (clknrst_if.clk     ),
        .rst_n          (clknrst_if.reset_n ),
        .step           (ex_step            ),
        .flush_i         (ex_flush            ),
        .pipe_i         (id_ex_pipe         ),
        .pipe_o         (ex_wb_pipe         )
    );

    wb_stage wb_stage_i(
        .clk            (clknrst_if.clk     ),
        .rst_n          (clknrst_if.reset_n ),
        .step           (wb_step            ),
        .flush_i         (wb_flush            ),
        .pipe_i         (ex_wb_pipe         ),
        .pipe_o         (wb_pipe            )
    );



    initial begin
        $display("Pipeline Shell: Starting");
    end



    always_comb begin
        rvfi_o.clk          <= clknrst_if.clk;
        rvfi_o.valid        <= wb_pipe.valid;
        rvfi_o.order        <= wb_pipe.rvfi.order;
        rvfi_o.insn         <= wb_pipe.rvfi.insn;
        rvfi_o.trap         <= wb_pipe.rvfi.trap;
        rvfi_o.halt         <= wb_pipe.rvfi.halt;
        rvfi_o.dbg          <= wb_pipe.rvfi.dbg;
        rvfi_o.dbg_mode     <= wb_pipe.rvfi.dbg_mode;
        rvfi_o.nmip         <= wb_pipe.rvfi.nmip;
        rvfi_o.intr         <= wb_pipe.rvfi.intr;
        rvfi_o.mode         <= wb_pipe.rvfi.mode;
        rvfi_o.ixl          <= wb_pipe.rvfi.ixl;
        rvfi_o.pc_rdata     <= wb_pipe.rvfi.pc_rdata;
        rvfi_o.pc_wdata     <= wb_pipe.rvfi.pc_wdata;
        rvfi_o.rs1_addr     <= wb_pipe.rvfi.rs1_addr;
        rvfi_o.rs1_rdata    <= wb_pipe.rvfi.rs1_rdata;
        rvfi_o.rs2_addr     <= wb_pipe.rvfi.rs2_addr;
        rvfi_o.rs2_rdata    <= wb_pipe.rvfi.rs2_rdata;
        rvfi_o.rs3_addr     <= wb_pipe.rvfi.rs3_addr;
        rvfi_o.rs3_rdata    <= wb_pipe.rvfi.rs3_rdata;
        rvfi_o.rd1_addr     <= wb_pipe.rvfi.rd1_addr;
        rvfi_o.rd1_wdata    <= wb_pipe.rvfi.rd1_wdata;
        rvfi_o.rd2_addr     <= wb_pipe.rvfi.rd2_addr;
        rvfi_o.rd2_wdata    <= wb_pipe.rvfi.rd2_wdata;
        rvfi_o.mem_addr     <= wb_pipe.rvfi.mem_addr;
        rvfi_o.mem_rdata    <= wb_pipe.rvfi.mem_rdata;
        rvfi_o.mem_rmask    <= wb_pipe.rvfi.mem_rmask;
        rvfi_o.mem_wdata    <= wb_pipe.rvfi.mem_wdata;
        rvfi_o.mem_wmask    <= wb_pipe.rvfi.mem_wmask;
    end

endmodule //pipeline_shell

`endif //__PIPELINE_SHELL_SV__