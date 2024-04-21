#include "Proc.h"
#include "disasm.h"
#include "extension.h"
#include <algorithm>
#include <assert.h>
#include <cinttypes>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <stdexcept>
#include <string>

#include "mmu.h"

namespace openhw {
st_rvfi Processor::step(size_t n, st_rvfi reference) {
  st_rvfi rvfi;
  FILE *log_file = this->get_log_file();
  memset(&rvfi, 0, sizeof(st_rvfi));

  this->taken_trap = false;

  // Store the state before stepping
  state_t prev_state = *this->get_state();

  rvfi.pc_rdata = this->get_state()->pc; // Current PC
  processor_t::step(n);

  // Add overwritten values from memory writes during the step
  prev_changes_t prev_changes(prev_state, this->get_state()->log_mem_pre_write);
  previous_states.push_front(prev_changes);

  if(previous_states.size() > max_previous_states) {
    previous_states.pop_back();
  }
  //printf("num previous states: %d, pc0: %x, mem writes0: %d\n",previous_states.size(), std::get<0>(previous_states[0]).pc, std::get<1>(previous_states[0]).size());

  rvfi.pc_wdata = this->get_state()->pc; // Next predicted PC

  rvfi.mode = this->get_state()->last_inst_priv;
  rvfi.insn =
      (uint32_t)(this->get_state()->last_inst_fetched.bits() & 0xffffffffULL);

  // TODO FIXME Handle multiple/zero writes in a single insn.
  auto &reg_commits = this->get_state()->log_reg_write;
  auto &mem_write_commits = this->get_state()->log_mem_write;
  auto &mem_read_commits = this->get_state()->log_mem_read;
  int xlen = this->get_state()->last_inst_xlen;
  int flen = this->get_state()->last_inst_flen;

  rvfi.rs1_addr = this->get_state()->last_inst_fetched.rs1();
  // TODO add rs1_value
  rvfi.rs2_addr = this->get_state()->last_inst_fetched.rs2();
  // TODO add rs2_value

 if(this->next_rvfi_intr){
    rvfi.intr = next_rvfi_intr;
    this->next_rvfi_intr = 0;
  }

  if(this->taken_trap) {
    //interrrupts are marked with the msb high in which_trap
    if(this->which_trap & ((reg_t)1 << (isa->get_max_xlen() - 1))) { 
      //Since spike steps two times to take an interrupt, we store the intr value to the next step to return with rvfi
      this->next_rvfi_intr |= 1 << 0; //intr [0]
      this->next_rvfi_intr |= 1 << 2; //interrupt [2]
      this->next_rvfi_intr |= 0x3FF8 & ((this->which_trap & 0xFF) << 3); //cause[13:3]
    } else{
      rvfi.trap |= 1 << 0; //intr [0]
      rvfi.trap |= 1 << 1; //exception [1]
      rvfi.trap |= 0x1F8 & ((this->which_trap) << 3); //exception_cause [8:3]
      //TODO:
      //debug_cause     [11:9] debug cause
      //cause_type      [13:12]
      //clicptr         [14]  CLIC interrupt pending
      this->next_rvfi_intr = rvfi.trap; //store value to return with rvfi.intr on the next step
    }
  }

  rvfi.cause = this->which_trap;

  bool got_commit = false;
  for (auto &reg : reg_commits) {
    // popret(z) should return rd1_addr = 0 to match with the core
    if (((this->get_state()->last_inst_fetched.bits() & MASK_CM_POPRET) == MATCH_CM_POPRET) ||
        ((this->get_state()->last_inst_fetched.bits() & MASK_CM_POPRETZ) == MATCH_CM_POPRETZ)) {
      got_commit = true;
      continue;
    }
    if (!got_commit) {
      rvfi.rd1_addr = reg.first >> 4;
      if (rvfi.rd1_addr > 32)
        continue;
      // TODO FIXME Take into account the XLEN/FLEN for int/FP values.
      rvfi.rd1_wdata = reg.second.v[0];
      // TODO FIXME Handle multiple register commits per cycle.
      // TODO FIXME This must be handled on the RVFI side as well.
      got_commit = true; // FORNOW Latch only the first commit.
    }
  }

  //TODO FIXME handle multiple memory accesses in a single insn
  bool mem_access = false;

  int read_len;
  for (auto &mem : mem_read_commits) {
    //mem format: (addr, 0, size) (value is not stored for reads, but should be the same as rd)
    if(!mem_access) {
      rvfi.mem_addr = std::get<0>(mem);
      rvfi.mem_rdata = rvfi.rd1_wdata; 
      //mem_rmask should hold a bitmask of which bytes in mem_rdata contain valid data
      read_len = std::get<2>(mem);
      rvfi.mem_rmask = (1 << read_len) - 1;
      mem_access = true;
    }
  }

  int write_len;
  for (auto &mem : mem_write_commits) {
    //mem format: (addr, value, size)
    if(!mem_access) {
      rvfi.mem_addr = std::get<0>(mem);
      rvfi.mem_wdata = std::get<1>(mem); // value
      //mem_wmask should hold a bitmask of which bytes in mem_wdata contain valid data
      write_len = std::get<2>(mem);
      rvfi.mem_wmask = (1 << write_len) - 1;
      mem_access = true;
    }

  }


  // Inject values comming from the reference
  if ((rvfi.insn & MASK_CSRRS) == MATCH_CSRRS) {
    if (rvfi.rs1_addr == 0) {
      reg_t read_csr = this->get_state()->last_inst_fetched.csr();
      switch (read_csr) {
      case 0xC00: // cycle
      case 0xC80: // cycleh
      case 0xB00: // mcycle
      case 0xB80: // mcycleh
        this->set_XPR(reference.rd1_addr, reference.rd1_wdata);
        rvfi.rd1_wdata = reference.rd1_wdata;
        break;
      default:
        break;
      }
    }
  }

  // Remove sign extension applied by Spike in 32b mode.
  if (this->get_xlen() == 32) {
    rvfi.pc_rdata &= 0xffffffffULL;
    rvfi.rd1_wdata &= 0xffffffffULL;
  }

  return rvfi;
}

void Processor::revert_step(int num_steps) {
  FILE *log_file = this->get_log_file();


  if (previous_states.size() < num_steps) {
    throw std::runtime_error("Cannot revert more states than stored");
  }

  for(auto state: previous_states) {
    fprintf(log_file, "pc: %x | ", std::get<0>(state).pc);
  }
  fprintf(log_file, "\n");



  fprintf(log_file, "revert from PC: %x", this->state.pc);

  prev_changes_t prev_changes = previous_states[num_steps];
  this->state = std::get<0>(prev_changes);

  fprintf(log_file, " to PC: %x\n", this->state.pc);

  for (int i = 0; i <= num_steps; i++) {
    prev_changes_t prev_changes = previous_states.front();
    previous_states.pop_front();

    commit_log_mem_t log_mem_pre_write = std::get<1>(prev_changes);
    fprintf(log_file, "revert mem pc: %x num: %d\n", std::get<0>(prev_changes).pc, log_mem_pre_write.size());

    for (auto mem_write : log_mem_pre_write) {
      fprintf(log_file, "revert mem: addr: %x val: %x size: %x", std::get<0>(mem_write), std::get<1>(mem_write), std::get<2>(mem_write));
      printf("revert\n");
      switch (std::get<2>(mem_write))
      {
      case 1:
        this->get_mmu()->store<uint8_t>(std::get<0>(mem_write), (uint8_t)std::get<1>(mem_write),0);
        break;
      case 2:
        this->get_mmu()->store<uint16_t>(std::get<0>(mem_write), (uint16_t)std::get<1>(mem_write),0);
        break;
      case 4:
        this->get_mmu()->store<uint32_t>(std::get<0>(mem_write), (uint32_t)std::get<1>(mem_write),0);
        break;
      
      default:
        break;
      }
      fprintf(log_file, " OK\n");
    }
  }

  //Clear write commit log to discard the writes generated above
  this->get_state()->log_mem_write.clear();
}


Processor::Processor(
    const isa_parser_t *isa, const cfg_t *cfg, simif_t *sim, uint32_t id,
    bool halt_on_reset, FILE *log_file, std::ostream &sout_,
    Params &params) // because of command line option --log and -s we need both
    : processor_t::processor_t(isa, cfg, sim, id, halt_on_reset, log_file,
                               sout_) {

  this->params.set("/top/core/0/", "isa", any(std::string("RV32GC")));
  this->params.set("/top/core/0/", "priv", DEFAULT_PRIV);
  this->params.set("/top/core/0/", "boot_addr", any(0x80000000UL));
  this->params.set("/top/core/0/", "mmu_mode", any(std::string("sv39")));

  this->params.set("/top/core/0/", "pmpregions", any(0x0UL));
  this->params.set("/top/core/0/", "pmpaddr0", any(0x0UL));
  this->params.set("/top/core/0/", "pmpcfg0", any(0x0UL));
  this->params.set("/top/core/0/", "marchid", any(0x15UL));
  this->params.set("/top/core/0/", "mvendorid", any(0x00000602UL));
  this->params.set("/top/core/0/", "status_fs_field_we_enable", any(false));
  this->params.set("/top/core/0/", "status_fs_field_we", any(false));
  this->params.set("/top/core/0/", "status_vs_field_we_enable", any(false));
  this->params.set("/top/core/0/", "status_vs_field_we", any(false));
  this->params.set("/top/core/0/", "misa_we_enable", any(true));
  this->params.set("/top/core/0/", "misa_we", any(false));

  this->params.set("/top/core/0/", "extensions", any(std::string("")));

  std::map<string, bool> registered_extensions_v;
  registered_extensions_v["cv32a60x"] = false;

  // Process User Params
  ParseParams("/top/core/0/", this->params, params);

  string isa_str = std::any_cast<string>(this->params["/top/core/0/isa"]);
  string priv_str = std::any_cast<string>(this->params["/top/core/0/priv"]);
  std::cout << "[SPIKE] Proc 0 | ISA: " << isa_str << " PRIV: " << priv_str
            << std::endl;
  this->isa =
      (const isa_parser_t *)new isa_parser_t(isa_str.c_str(), priv_str.c_str());

  disassembler = new disassembler_t(isa);

  for (auto e : isa->get_extensions()) {
    register_extension(e.second);
  }

  string extensions_str =
      std::any_cast<string>(this->params["/top/core/0/extensions"]);
  string delimiter = ",";
  size_t found = extensions_str.rfind(delimiter);

  if (found == string::npos && extensions_str != "") {
    extensions_str = extensions_str + delimiter;
  }

  while (found != string::npos) {
    string token = extensions_str.substr(found + delimiter.length(),
                                         extensions_str.length() - 1);
    extensions_str = extensions_str.substr(0, found);
    auto it = registered_extensions_v.find(token);
    if (it != registered_extensions_v.end())
      it->second = true;
    else
      std::cout << "[SPIKE] Extension \"" << token << "\" can not be registered"
                << std::endl;

    found = extensions_str.rfind(delimiter);
  }

  for (auto ext : registered_extensions_v) {
    if (ext.second) {
      extension_t *extension = find_extension(ext.first.c_str())();
      this->register_extension(extension);
      extension->reset();
    }
  }

  this->reset();

  uint64_t new_pc =
      std::any_cast<uint64_t>(this->params["/top/core/0/boot_addr"]);
  this->state.pc = new_pc;

  this->put_csr(CSR_PMPADDR0,
                std::any_cast<uint64_t>(this->params["/top/core/0/pmpaddr0"]));
  this->put_csr(CSR_PMPCFG0,
                std::any_cast<uint64_t>(this->params["/top/core/0/pmpcfg0"]));

  this->put_csr(CSR_MVENDORID,
                std::any_cast<uint64_t>(this->params["/top/core/0/mvendorid"]));
  this->put_csr(CSR_MARCHID,
                std::any_cast<uint64_t>(this->params["/top/core/0/marchid"]));

  bool fs_field_we_enable = std::any_cast<bool>(
      this->params["/top/core/0/status_fs_field_we_enable"]);
  bool fs_field_we =
      std::any_cast<bool>(this->params["/top/core/0/status_fs_field_we"]);
  bool vs_field_we_enable = std::any_cast<bool>(
      this->params["/top/core/0/status_vs_field_we_enable"]);
  bool vs_field_we =
      std::any_cast<bool>(this->params["/top/core/0/status_vs_field_we"]);

  reg_t sstatus_mask = this->state.mstatus->get_param_write_mask();
  if (fs_field_we_enable)
    sstatus_mask = (fs_field_we ? (sstatus_mask | MSTATUS_FS)
                                : (sstatus_mask & ~MSTATUS_FS));
  if (vs_field_we_enable)
    sstatus_mask = (vs_field_we ? (sstatus_mask | MSTATUS_VS)
                                : (sstatus_mask & ~MSTATUS_VS));
  this->state.mstatus->set_param_write_mask(sstatus_mask);


  this->put_csr(CSR_MSTATUS, MSTATUS_MPP);

  bool misa_we_enable =
      std::any_cast<bool>(this->params["/top/core/0/misa_we_enable"]);
  bool misa_we = std::any_cast<bool>(this->params["/top/core/0/misa_we"]);
  if (misa_we_enable)
    this->state.misa->set_we(misa_we);

  this->next_rvfi_intr = 0;

  this->max_previous_states = 4; //TODO: make this a parameter
}

void Processor::take_trap(trap_t &t, reg_t epc) {
  this->taken_trap = true;
  this->which_trap = t.cause();
  processor_t::take_trap(t, epc);
}

Processor::~Processor() { delete this->isa; }

inline void Processor::set_XPR(reg_t num, reg_t value) {
  this->state.XPR.write(num, value);
}

inline void Processor::set_FPR(reg_t num, float128_t value) {
  this->state.FPR.write(num, value);
}

} // namespace openhw
