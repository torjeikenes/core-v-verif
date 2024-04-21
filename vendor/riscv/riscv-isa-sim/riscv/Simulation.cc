// See LICENSE for license details.

#include "Simulation.h"
#include "mmu.h"
#include <cassert>
#include <climits>
#include <cstdlib>
#include <inttypes.h>
#include <iostream>
#include <map>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#define ENABLED_IRQ_MASK 0xFFFF0888 //Enable IRQ 3, 7, 11, and 16-31 (From CV32E40S documentation)

using namespace openhw;

std::vector<std::pair<reg_t, abstract_device_t *>> plugin_devs;

void sim_thread_main(void *arg) {
  ((sim_t *)arg)->run();
  ((sim_t *)arg)->switch_to_host(); // To get the first point
}

// FIXME TODO Review settings of dm_config below.
debug_module_config_t dm_config = {.progbufsize = 2,
                                   .max_sba_data_width = 0,
                                   .require_authentication = false,
                                   .abstract_rti = 0,
                                   .support_hasel = true,
                                   .support_abstract_csr_access = true,
                                   .support_abstract_fpr_access = true,
                                   .support_haltgroups = true,
                                   .support_impebreak = true};

Simulation::Simulation(
    const cfg_t *cfg, bool halted, std::vector<std::pair<reg_t, mem_t *>> mems,
    std::vector<std::pair<reg_t, abstract_device_t *>> plugin_devices,
    const std::vector<std::string> &args,
    const debug_module_config_t &dm_config, const char *log_path,
    bool dtb_enabled, const char *dtb_file, bool socket_enabled,
    FILE *cmd_file, // needed for command line option --cmd
    openhw::Params &params)
    : sim_t(cfg, halted, mems, plugin_devices, args, dm_config, log_path,
            dtb_enabled, dtb_file, socket_enabled, cmd_file, params) {
  // It seems mandatory to set cache block size for MMU.
  // FIXME TODO: Use actual cache configuration (on/off, # of ways/sets).
  // FIXME TODO: Support multiple cores.
  get_core(0)->get_mmu()->set_cache_blocksz(reg_t(64));

  this->params.set("/top/", "isa", any(std::string("RV32IMC")));
  this->params.set("/top/", "priv", any(std::string(DEFAULT_PRIV)));

  this->params.set("/top/", "bootrom", std::any(true));
  this->params.set("/top/", "bootrom_base", std::any(0x10000UL));
  this->params.set("/top/", "bootrom_size", std::any(0x1000UL));

  this->params.set("/top/", "dram", std::any(true));
  this->params.set("/top/", "dram_base", std::any(0x00000000UL));
  this->params.set("/top/", "dram_size", std::any(0x400000UL));

  this->params.set("/top/", "log_commits", std::any(true));

  this->params.set("/top/", "max_steps_enabled", std::any(false));
  this->params.set("/top/", "max_steps", std::any(200000UL));

  ParseParams("/top/", this->params, params);

  const std::vector<mem_cfg_t> layout;

  this->make_mems(layout);

  for (auto &x : this->mems)
    bus.add_device(x.first, x.second);

  string isa_str = std::any_cast<string>(this->params["/top/isa"]);
  string priv_str = std::any_cast<string>(this->params["/top/priv"]);
  this->isa = isa_parser_t(isa_str.c_str(), priv_str.c_str());

  this->reset();

  bool commitlog = std::any_cast<bool>(this->params["/top/log_commits"]);
  this->configure_log(commitlog, commitlog);

  this->max_steps = std::any_cast<uint64_t>(this->params["/top/max_steps"]);
  this->max_steps_enabled =
      std::any_cast<bool>(this->params["/top/max_steps_enabled"]);

  printf("U extension enabled = %d\n", procs[0]->extension_enabled('U'));
  printf("S extension enabled = %d\n", procs[0]->extension_enabled('S'));

  target.init(sim_thread_main, this);
  host = context_t::current();
  target.switch_to(); // To get the first point
}

Simulation::Simulation(const cfg_t *cfg, string elf_path,
                       Params &params)
    : Simulation(cfg,                                      // cfg
                 false,                                    // halted
                 std::vector<std::pair<reg_t, mem_t *>>(), // mems
                 plugin_devs, std::vector<std::string>() = {elf_path},
                 dm_config,
                 "tandem.log", // log_path
                 false,        // dtb_enabled
                 nullptr,      // dtb_file
                 false,        // socket_enabled
                 NULL,         // cmd_file
                 params) {}

Simulation::~Simulation() {}

int Simulation::run() {
  try {
    while (!sim_t::done()) {
      st_rvfi reference;
      std::vector<st_rvfi> vreference, vspike;
      vreference.push_back(reference);
      vspike = this->step(1, vreference);
    }
  } catch (std::ios_base::failure e) {
    std::cout << "[SPIKE] Max steps exceed" << std::endl;
  }
  return sim_t::exit_code();
}

void Simulation::make_mems(const std::vector<mem_cfg_t> &layout) {
  for (const auto &cfg : layout)
    mems.push_back(std::make_pair(cfg.get_base(), new mem_t(cfg.get_size())));

  

/*
  bool bootrom = std::any_cast<bool>(this->params["/top/bootrom"]);
  uint64_t bootrom_base =
      std::any_cast<uint64_t>(this->params["/top/bootrom_base"]);
  uint64_t bootrom_size =
      std::any_cast<uint64_t>(this->params["/top/bootrom_size"]);
  if (bootrom) {
    auto bootrom_device = std::make_pair(bootrom_base, new mem_t(bootrom_size));

    std::cerr << "[SPIKE] Initializing memories...\n";
    uint8_t rom_check_buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    // Populate the ROM.  Reset vector size is in 32-bit words and must be
    // scaled.
#include "bootrom.h"
    if (!bootrom_device.second->store(reg_t(0), reset_vec_size << 2,
                                      (const uint8_t *)reset_vec)) {
      std::cerr << "[SPIKE] *** ERROR: Failed to initialize ROM!\n";
      bootrom_device.second->load(reg_t(0), 8, rom_check_buffer);
      fprintf(stderr,
              "[SPIKE] ROM content head(8) = %02x %02x %02x %02x %02x %02x "
              "%02x %02x\n",
              rom_check_buffer[0], rom_check_buffer[1], rom_check_buffer[2],
              rom_check_buffer[3], rom_check_buffer[4], rom_check_buffer[5],
              rom_check_buffer[6], rom_check_buffer[7]);
    }

    this->mems.push_back(bootrom_device);
  }
*/
  bool dram = std::any_cast<bool>(this->params["/top/dram"]);
  uint64_t dram_base = std::any_cast<uint64_t>(this->params["/top/dram_base"]);
  uint64_t dram_size = std::any_cast<uint64_t>(this->params["/top/dram_size"]);
  if (dram)
    this->mems.push_back(std::make_pair(dram_base, new mem_t(dram_size)));

  //dbg
  this->mems.push_back(std::make_pair(0x1a110800, new mem_t(0x1000)));

  //stdout?
  this->mems.push_back(std::make_pair(0x10000000, new mem_t(0x1000)));
  //mmram?
  this->mems.push_back(std::make_pair(0x20000000, new mem_t(0x1000)));

  //CV_VP_REGISTER used in _write()
  this->mems.push_back(std::make_pair(0x00800000, new mem_t(0x10000)));
}

std::vector<st_rvfi> Simulation::step(size_t n,
                                      std::vector<st_rvfi> &vreference) {

  // The state PC is the *next* insn fetch address.
  // Catch it before exec which yields a new value.
  std::vector<st_rvfi> vspike(n);
  for (size_t i = 0; i < n; i++) {
    if (i >= procs.size())
      continue;

    vspike[i] = ((Processor *)procs[i])->step(1, vreference[i]);

    host = context_t::current();
    if (!sim_t::done()) {
      if (this->max_steps_enabled && (this->max_steps < this->total_steps)) {
        throw std::ios_base::failure("Max steps exceeded");
      }

      ++total_steps;
      target.switch_to();
    }
  }
  return vspike;
}

bool Simulation::will_trigger_interrupt(reg_t mip) {
  state_t *state = procs[0]->get_state();

  uint32_t old_mip = state->mip->read();
  uint32_t mie = state->mie->read();
  uint32_t mstatus = state->mstatus->read();

  uint32_t old_en_irq = old_mip & mie;
  uint32_t new_en_irq = mip & mie;

  // Only take interrupt if interrupt is enabled, not in debug mode, not halted, 
  // and the interrupt is new and not zero
  if( get_field(mstatus, MSTATUS_MIE) &&
      !state->debug_mode  &&
      (procs[0]->halt_request != processor_t::HR_REGULAR) &&
      //(old_en_irq == 0 ) && 
      (new_en_irq != 0)) 
  {
    return true;
  } else {
    return false;
  }
}


bool Simulation::interrupt(reg_t mip, uint32_t revert_steps) {
  state_t *state = procs[0]->get_state();

  st_rvfi vref; //Passed to step, but not used

  if(will_trigger_interrupt(mip)) {
    fprintf(procs[0]->get_log_file(), "revert state mip %lx\n", mip);
    revert_state(revert_steps);
    state->mip->write_with_mask(ENABLED_IRQ_MASK, mip);

    // This step only sets the correct state for the interrupt, but does not actually execute an instruction
    // Another step needs to be taken to actually step through the instruction
    // Therefore we discard the rvfi values returned from this step
    ((Processor *)procs[0])->step(1, vref);
    fprintf(procs[0]->get_log_file(), "mip %lx set to %lx\n", mip, mip & ENABLED_IRQ_MASK);

    return true;
  } else {
    // Set mip to 0 when the the interrupt will not be taken, to control when spike should take the interrrupt
    state->mip->write_with_mask(ENABLED_IRQ_MASK, 0);
    //fprintf(procs[0]->get_log_file(), "mip %lx set to %lx\n", mip, mip & ENABLED_IRQ_MASK);

    return false;
  }
  
}

void Simulation::revert_state(int num_steps) {
  ((Processor *)procs[0])->revert_step(num_steps);
}

#if 0 // FORNOW Unused code, disable until needed.
void Simulation::set_debug(bool value)
{
  debug = value;
}

void Simulation::set_log(bool value)
{
  log = value;
}

void Simulation::set_histogram(bool value)
{
  histogram_enabled = value;
  for (size_t i = 0; i < procs.size(); i++) {
    procs[i]->set_histogram(histogram_enabled);
  }
}

void Simulation::set_procs_debug(bool value)
{
  for (size_t i=0; i< procs.size(); i++)
    procs[i]->set_debug(value);
}

bool Simulation::mmio_load(reg_t addr, size_t len, uint8_t* bytes)
{
  if (addr + len < addr)
    return false;
  return bus.load(addr, len, bytes);
}

bool Simulation::mmio_store(reg_t addr, size_t len, const uint8_t* bytes)
{
  if (addr + len < addr)
    return false;
  return bus.store(addr, len, bytes);
}

void Simulation::make_bootrom()
{
  start_pc = 0x80000000;

#include "bootrom.h"

  std::vector<char> rom((char*)reset_vec, (char*)reset_vec + sizeof(reset_vec));

  boot_rom.reset(new rom_device_t(rom));
  bus.add_device(DEFAULT_RSTVEC, boot_rom.get());
}

char* Simulation::addr_to_mem(reg_t addr) {
  auto desc = bus.find_device(addr);
  if (auto mem = dynamic_cast<mem_t*>(desc.second))
    if (addr - desc.first < mem->size())
      return mem->contents() + (addr - desc.first);
  return NULL;
}
#endif
