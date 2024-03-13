`ifndef __PIPELINE_SHELL_SV__
`define __PIPELINE_SHELL_SV__

module pipeline_shell 
    import uvma_rvfi_pkg::*;
    import iss_wrap_pkg::*;
    (
        uvma_clknrst_if_t clknrst_if,
        uvma_rvfi_instr_if_t rvfi_i,
        uvma_interrupt_if_t interrupt_if_i,
        rvfi_if_t rvfi_o
    );

    st_rvfi rvfi_iss;
    st_rvfi rvfi_core;

    initial begin
        $display("Pipeline Shell: Starting");
    end

    logic [31:0] irq_drv_ff;

    assign rvfi_o.clk = clknrst_if.clk;

    always_ff @(posedge clknrst_if.clk) begin
        irq_drv_ff <= interrupt_if_i.irq_drv;
        if (irq_drv_ff != interrupt_if_i.irq_drv) begin
            iss_intr(interrupt_if_i.irq_drv);
        end

        if(rvfi_i.rvfi_valid) begin
            //TEMP: insert rvfi from core into dummy iss, to return the same rvfi back
            rvfi_core.order = rvfi_i.rvfi_order;
            rvfi_core.insn = rvfi_i.rvfi_insn;
            rvfi_core.trap = rvfi_i.rvfi_trap;
            rvfi_core.halt = rvfi_i.rvfi_halt;
            rvfi_core.dbg = rvfi_i.rvfi_dbg;
            rvfi_core.dbg_mode = rvfi_i.rvfi_dbg_mode;
            rvfi_core.nmip = rvfi_i.rvfi_nmip;
            rvfi_core.intr = rvfi_i.rvfi_intr;
            rvfi_core.mode = rvfi_i.rvfi_mode;
            rvfi_core.ixl = rvfi_i.rvfi_ixl;
            rvfi_core.pc_rdata = rvfi_i.rvfi_pc_rdata;
            rvfi_core.pc_wdata = rvfi_i.rvfi_pc_wdata;
            rvfi_core.rs1_addr = rvfi_i.rvfi_rs1_addr;
            rvfi_core.rs1_rdata = rvfi_i.rvfi_rs1_rdata;
            rvfi_core.rs2_addr = rvfi_i.rvfi_rs2_addr;
            rvfi_core.rs2_rdata = rvfi_i.rvfi_rs2_rdata;
            rvfi_core.rs3_addr = rvfi_i.rvfi_rs3_addr;
            rvfi_core.rs3_rdata = rvfi_i.rvfi_rs3_rdata;
            rvfi_core.rd1_addr = rvfi_i.rvfi_rd1_addr;
            rvfi_core.rd1_wdata = rvfi_i.rvfi_rd1_wdata;
            rvfi_core.rd2_addr = rvfi_i.rvfi_rd2_addr;
            rvfi_core.rd2_wdata = rvfi_i.rvfi_rd2_wdata;
            rvfi_core.mem_addr = rvfi_i.rvfi_mem_addr;
            rvfi_core.mem_rdata = rvfi_i.rvfi_mem_rdata;
            rvfi_core.mem_rmask = rvfi_i.rvfi_mem_rmask;
            rvfi_core.mem_wdata = rvfi_i.rvfi_mem_wdata;
            rvfi_core.mem_wmask = rvfi_i.rvfi_mem_wmask;

            iss_step(rvfi_core, rvfi_iss);
            rvfi_o.valid = 1'b1;     
        end
        else begin
            rvfi_o.valid = 1'b0;     
            rvfi_core = rvfi_core;
        end
        
        rvfi_o.order = rvfi_iss.order;
        rvfi_o.insn = rvfi_iss.insn;
        rvfi_o.trap = rvfi_iss.trap;
        rvfi_o.halt = rvfi_iss.halt;
        rvfi_o.dbg = rvfi_iss.dbg;
        rvfi_o.dbg_mode = rvfi_iss.dbg_mode;
        rvfi_o.nmip = rvfi_iss.nmip;
        rvfi_o.intr = rvfi_iss.intr;
        rvfi_o.mode = rvfi_iss.mode;
        rvfi_o.ixl = rvfi_iss.ixl;
        rvfi_o.pc_rdata = rvfi_iss.pc_rdata;
        rvfi_o.pc_wdata = rvfi_iss.pc_wdata;
        rvfi_o.rs1_addr = rvfi_iss.rs1_addr;
        rvfi_o.rs1_rdata = rvfi_iss.rs1_rdata;
        rvfi_o.rs2_addr = rvfi_iss.rs2_addr;
        rvfi_o.rs2_rdata = rvfi_iss.rs2_rdata;
        rvfi_o.rs3_addr = rvfi_iss.rs3_addr;
        rvfi_o.rs3_rdata = rvfi_iss.rs3_rdata;
        rvfi_o.rd1_addr = rvfi_iss.rd1_addr;
        rvfi_o.rd1_wdata = rvfi_iss.rd1_wdata;
        rvfi_o.rd2_addr = rvfi_iss.rd2_addr;
        rvfi_o.rd2_wdata = rvfi_iss.rd2_wdata;
        rvfi_o.mem_addr = rvfi_iss.mem_addr;
        rvfi_o.mem_rdata = rvfi_iss.mem_rdata;
        rvfi_o.mem_rmask = rvfi_iss.mem_rmask;
        rvfi_o.mem_wdata = rvfi_iss.mem_wdata;
        rvfi_o.mem_wmask = rvfi_iss.mem_wmask;
    end

endmodule //pipeline_shell

`endif //__PIPELINE_SHELL_SV__
