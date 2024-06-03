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

`ifndef __ISS_WRAP_PKG_SV
`define __ISS_WRAP_PKG_SV

package iss_wrap_pkg;

import uvma_rvfi_pkg::*;
import uvma_core_cntrl_pkg::*;
import uvmc_rvfi_reference_model_pkg::*;

import "DPI-C" function bit  spike_interrupt(int unsigned mip, int unsigned mie, int unsigned revert_steps, bit interrupt_allowed);
import "DPI-C" function void spike_revert_state(int num_steps);

import "DPI-C" function bit spike_set_debug(bit debug_req, int unsigned revert_steps, bit debug_allowed);


    function automatic void iss_init(string core_name, st_core_cntrl_cfg core_cfg);
        rvfi_initialize_spike(core_name, core_cfg);
    endfunction

    function automatic st_rvfi iss_step();
        st_rvfi core,rm;
        rvfi_spike_step(core, rm);
        return rm;

    endfunction

    //Sets mip in Spike and steps one time to apply the state changes
    //This step does not step through an instruction, so iss_step() must be 
    //called to step through the first instruction of the trap handler.
    //Returns true if the interrupt can be taken, and false if CSRs cause the interrupt to not be taken
    function automatic logic iss_intr(bit [31:0] irq, bit [31:0] mie, logic interrupt_allowed, int num_revert_steps);
        logic interrupt_taken;
        
        interrupt_taken = spike_interrupt(irq, mie, num_revert_steps, interrupt_allowed);

        return interrupt_taken;
    endfunction

    function automatic logic iss_set_debug(bit debug_req, int revert_steps, bit debug_allowed);
        logic debug_taken;
        debug_taken =  spike_set_debug(debug_req, revert_steps, debug_allowed);
        return debug_taken;
    endfunction

    function automatic void iss_revert_state(int num_steps);
        spike_revert_state(num_steps);
    endfunction

endpackage : iss_wrap_pkg

`endif // __ISS_WRAP_PKG_SV
