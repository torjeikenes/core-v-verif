
proc cvfv_rerun {} {
    delete_design -golden
    start_message_log -force onespin.log
    puts  "cvfv: compiling verilog"

    set CV_CORE     cv32e40s
    set CORE_V_VERIF /home/torjene/core-v-verif

    set RM_HOME               $CORE_V_VERIF/lib/reference_model

    set CV_CORE_PKG  $CORE_V_VERIF/core-v-cores/$CV_CORE

    set DV_ISA_DECODER_PATH   $CORE_V_VERIF/lib/isa_decoder
    set DV_SUPPORT_PATH       $CORE_V_VERIF/lib/support
    set DV_UVM_TESTCASE_PATH  $CORE_V_VERIF/$CV_CORE/tests/uvmt
    set DV_UVMA_PATH          $CORE_V_VERIF/lib/uvm_agents
    set DV_UVME_PATH          $CORE_V_VERIF/$CV_CORE/env/uvme
    set DV_UVMT_PATH          $CORE_V_VERIF/$CV_CORE/tb/uvmt

    puts $DV_UVMT_PATH
    
    set DESIGN_RTL_DIR $CV_CORE_PKG/rtl

    set_read_hdl_option -verilog_compilation_unit ext

    add_read_hdl_option -verilog_include_path $DV_UVMT_PATH

    add_read_hdl_option -verilog_include_path $DV_UVME_PATH
    add_read_hdl_option -verilog_include_path $DV_UVMA_PATH/uvma_rvfi
    add_read_hdl_option -verilog_include_path $DV_UVMA_PATH/uvma_fencei
    add_read_hdl_option -verilog_include_path $DV_UVMA_PATH/uvma_clic
    add_read_hdl_option -verilog_include_path $DV_UVMA_PATH/uvma_obi_memory/src


    add_read_hdl_option -verilog_include_path $DESIGN_RTL_DIR/include
    add_read_hdl_option -verilog_include_path $DESIGN_RTL_DIR/../bhv
    add_read_hdl_option -verilog_include_path $DESIGN_RTL_DIR/../bhv/include
    add_read_hdl_option -verilog_include_path $DESIGN_RTL_DIR/../sva

    add_read_hdl_option -verilog_include_path $DV_ISA_DECODER_PATH

    add_read_hdl_option -verilog_include_path $DV_SUPPORT_PATH


    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        uvm_pkg.sv 
        defines.sv
    }

    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        $DESIGN_RTL_DIR/include/cv32e40s_pkg.sv
        $DESIGN_RTL_DIR/cv32e40s_if_c_obi.sv
        $DESIGN_RTL_DIR/../bhv/include/cv32e40s_rvfi_pkg.sv
        $DESIGN_RTL_DIR/../bhv/cv32e40s_wrapper.sv
        $DESIGN_RTL_DIR/cv32e40s_dummy_instr.sv
        $DESIGN_RTL_DIR/cv32e40s_if_stage.sv
        $DESIGN_RTL_DIR/cv32e40s_csr.sv
        $DESIGN_RTL_DIR/cv32e40s_debug_triggers.sv
        $DESIGN_RTL_DIR/cv32e40s_cs_registers.sv
        $DESIGN_RTL_DIR/cv32e40s_register_file.sv
        $DESIGN_RTL_DIR/cv32e40s_register_file_ecc.sv
        $DESIGN_RTL_DIR/cv32e40s_register_file_wrapper.sv
        $DESIGN_RTL_DIR/cv32e40s_write_buffer.sv
        $DESIGN_RTL_DIR/cv32e40s_lsu_response_filter.sv
        $DESIGN_RTL_DIR/cv32e40s_load_store_unit.sv
        $DESIGN_RTL_DIR/cv32e40s_id_stage.sv
        $DESIGN_RTL_DIR/cv32e40s_i_decoder.sv
        $DESIGN_RTL_DIR/cv32e40s_m_decoder.sv
        $DESIGN_RTL_DIR/cv32e40s_b_decoder.sv
        $DESIGN_RTL_DIR/cv32e40s_decoder.sv
        $DESIGN_RTL_DIR/cv32e40s_compressed_decoder.sv
        $DESIGN_RTL_DIR/cv32e40s_sequencer.sv
        $DESIGN_RTL_DIR/cv32e40s_alignment_buffer.sv
        $DESIGN_RTL_DIR/cv32e40s_prefetch_unit.sv
        $DESIGN_RTL_DIR/cv32e40s_mult.sv
        $DESIGN_RTL_DIR/cv32e40s_int_controller.sv
        $DESIGN_RTL_DIR/cv32e40s_clic_int_controller.sv
        $DESIGN_RTL_DIR/cv32e40s_ex_stage.sv
        $DESIGN_RTL_DIR/cv32e40s_wb_stage.sv
        $DESIGN_RTL_DIR/cv32e40s_div.sv
        $DESIGN_RTL_DIR/cv32e40s_alu.sv
        $DESIGN_RTL_DIR/cv32e40s_ff_one.sv
        $DESIGN_RTL_DIR/cv32e40s_popcnt.sv
        $DESIGN_RTL_DIR/cv32e40s_alu_b_cpop.sv
        $DESIGN_RTL_DIR/cv32e40s_controller_fsm.sv
        $DESIGN_RTL_DIR/cv32e40s_controller_bypass.sv
        $DESIGN_RTL_DIR/cv32e40s_controller.sv
        $DESIGN_RTL_DIR/cv32e40s_obi_integrity_fifo.sv
        $DESIGN_RTL_DIR/cv32e40s_instr_obi_interface.sv
        $DESIGN_RTL_DIR/cv32e40s_data_obi_interface.sv
        $DESIGN_RTL_DIR/cv32e40s_prefetcher.sv
        $DESIGN_RTL_DIR/cv32e40s_sleep_unit.sv
        $DESIGN_RTL_DIR/cv32e40s_alert.sv
        $DESIGN_RTL_DIR/cv32e40s_core.sv
        $DESIGN_RTL_DIR/cv32e40s_mpu.sv
        $DESIGN_RTL_DIR/cv32e40s_pma.sv
        $DESIGN_RTL_DIR/cv32e40s_pmp.sv
        $DESIGN_RTL_DIR/cv32e40s_pc_target.sv
        $DESIGN_RTL_DIR/cv32e40s_wpt.sv
        $DESIGN_RTL_DIR/cv32e40s_pc_check.sv
        $DESIGN_RTL_DIR/cv32e40s_rchk_check.sv
        $DESIGN_RTL_DIR/cv32e40s_lfsr.sv

        $DESIGN_RTL_DIR/../bhv/cv32e40s_sim_sffr.sv
        $DESIGN_RTL_DIR/../bhv/cv32e40s_sim_sffs.sv
        $DESIGN_RTL_DIR/../bhv/cv32e40s_sim_clock_gate.sv
        $DESIGN_RTL_DIR/../bhv/cv32e40s_rvfi_instr_obi.sv
        $DESIGN_RTL_DIR/../bhv/cv32e40s_rvfi_data_obi.sv
        $DESIGN_RTL_DIR/../bhv/cv32e40s_rvfi.sv
        $DESIGN_RTL_DIR/../bhv/cv32e40s_rvfi_sim_trace.sv


    } 


    read_verilog -golden  -pragma_ignore {} -version sv2012 {
        $DV_ISA_DECODER_PATH/isa_decoder_pkg.sv}

    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        $DV_SUPPORT_PATH/support_pkg.sv}

    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        $DV_UVM_TESTCASE_PATH/base-tests/uvmt_cv32e40s_base_test_pkg.sv
        $DV_UVMA_PATH/uvma_obi_memory/src/uvma_obi_memory_assert.sv
        $DV_UVMA_PATH/uvma_obi_memory/src/uvma_obi_memory_1p2_assert.sv
    }
        
    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        ./dummy_pkg.sv
    }

    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        $DV_UVMA_PATH/uvma_clic/uvma_clic_if.sv
        $DV_UVMA_PATH/uvma_debug/uvma_debug_if.sv
        $DV_UVMA_PATH/uvma_fencei/uvma_fencei_if.sv
        $DV_UVMA_PATH/uvma_interrupt/uvma_interrupt_if.sv
        $DV_UVMA_PATH/uvma_obi_memory/src/uvma_obi_memory_assert_if_wrp.sv
        $DV_UVMA_PATH/uvma_obi_memory/src/uvma_obi_memory_if.sv
        $DV_UVMA_PATH/uvma_rvfi/uvma_rvfi_csr_if.sv
        $DV_UVMA_PATH/uvma_rvfi/uvma_rvfi_instr_if.sv
        $DV_UVMA_PATH/uvma_wfe_wu/uvma_wfe_wu_if.sv
        $DV_UVME_PATH/uvme_cv32e40s_core_cntrl_if.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_tb_ifs.sv
    }


    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        $DV_UVMT_PATH/uvmt_cv32e40s_dut_wrap.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_tb.sv

        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_fencei_assert.sv
        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_pmp_assert.sv
        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_pmprvfi_assert.sv
        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_rvfi_assert.sv
        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_umode_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_clic_interrupt_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_debug_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_integration_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_interrupt_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_pma_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_triggers_assert_cov.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_bus_protocol_hardening_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_data_independent_timing_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_dummy_and_hint_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_hardened_csrs_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_hardened_csrs_clic_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_hardened_csrs_interrupt_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_hardened_csrs_pmp_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_hardened_pc_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_interface_integrity_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_reduced_profiling_infrastructure_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_register_file_ecc_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_xsecure_assert/uvmt_cv32e40s_xsecure_security_alerts_assert.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_zc_assert.sv

        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_pma_model.sv
        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_pmp_model.sv
        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_rvfi_cov.sv
        $DV_UVMT_PATH/../assertions/uvmt_cv32e40s_umode_cov.sv
        $DV_UVMT_PATH/support_logic/uvmt_cv32e40s_rchk_shim.sv
        $DV_UVMT_PATH/support_logic/uvmt_cv32e40s_sl_fifo.sv
        $DV_UVMT_PATH/support_logic/uvmt_cv32e40s_sl_obi_phases_monitor.sv
        $DV_UVMT_PATH/support_logic/uvmt_cv32e40s_sl_trigger_match.sv
        $DV_UVMT_PATH/support_logic/uvmt_cv32e40s_sl_trigger_match_mem.sv
        $DV_UVMT_PATH/support_logic/uvmt_cv32e40s_support_logic.sv
        $DV_UVMT_PATH/uvmt_cv32e40s_pma_cov.sv
    }
    

    #Reference model
    add_read_hdl_option -verilog_include_path $RM_HOME
    
    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        $DV_UVMT_PATH/uvmt_cv32e40s_reference_model_wrap.sv
        $RM_HOME/iss_wrap_formal_pkg.sv
        $RM_HOME/reference_model.sv
        $RM_HOME/pipeline_shell.sv

    }
    read_verilog -golden  -pragma_ignore {}  -version sv2012 {
        $RM_HOME/reference_model_sva.sv
    }

    set_elaborate_option -top uvmt_cv32e40s_tb

    elaborate


    add_compile_option -cut_signal clknrst_if/reset_n
    add_compile_option -cut_signal clknrst_if/clk

    #add_compile_option -cut_signal debug_if/debug_req
    #add_compile_option -cut_signal interrupt_if/irq
    add_compile_option -cut_signal core_cntrl_if/boot_addr
    add_compile_option -cut_signal core_cntrl_if/mtvec_addr
    add_compile_option -cut_signal core_cntrl_if/dm_halt_addr
    add_compile_option -cut_signal core_cntrl_if/dm_exception_addr
    add_compile_option -cut_signal core_cntrl_if/mhartid
    add_compile_option -cut_signal core_cntrl_if/mimpid_patch

    add_compile_option -cut_signal obi_instr_if/err
    add_compile_option -cut_signal obi_instr_if/gntpar
    add_compile_option -cut_signal obi_instr_if/gnt
    add_compile_option -cut_signal obi_instr_if/rchk
    add_compile_option -cut_signal obi_instr_if/rdata
    add_compile_option -cut_signal obi_instr_if/rvalidpar
    add_compile_option -cut_signal obi_instr_if/rvalid
    add_compile_option -cut_signal obi_data_if/err
    add_compile_option -cut_signal obi_data_if/gntpar
    add_compile_option -cut_signal obi_data_if/gnt
    add_compile_option -cut_signal obi_data_if/rchk
    add_compile_option -cut_signal obi_data_if/rdata
    add_compile_option -cut_signal obi_data_if/rvalidpar
    add_compile_option -cut_signal obi_data_if/rvalid
    add_compile_option -cut_signal fencei_if/flush_ack

    #add_compile_option -cut_signal core_cntrl_if/scan_cg_en
    #add_compile_option -cut_signal core_cntrl_if/fetch_en
    #add_compile_option -signal_domain {core_cntrl_if.scan_cg_en 0}
    #add_compile_option -signal_domain {core_cntrl_if.fetch_en 1}


    compile

    set_mode mv
    check  [ list reference_model.rvfi_compare_i.compare_insn_a reference_model.rvfi_compare_i.compare_pc_a]; 
    check -pass [ list reference_model.rvfi_compare_i.compare_insn_a reference_model.rvfi_compare_i.compare_pc_a]


}

cvfv_rerun