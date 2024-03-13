`ifndef __ISS_WRAP_FORMAL_PKG_SV
`define __ISS_WRAP_FORMAL_PKG_SV

package iss_wrap_pkg;

import uvma_rvfi_pkg::*;

    function automatic void iss_init(string binary);

    endfunction

    function automatic void iss_step(ref st_rvfi rvfi_core, ref st_rvfi s_reference_model);
        s_reference_model = rvfi_core;
    endfunction

    //Sets mip in Spike and steps one time to apply the state changes
    //This step does not step through an instruction, so iss_step() must be 
    //called to step through the first instruction of the trap handler
    function automatic void iss_intr(bit [31:0] irq);
    endfunction

endpackage : iss_wrap_pkg

`endif // __ISS_WRAP_FORMAL_PKG_SV
