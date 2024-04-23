
#include "Types.h"
#include "processor.h"

namespace openhw {
typedef std::tuple<state_t, commit_log_mem_t> prev_changes_t;
class Processor : public processor_t {
public:
  Processor(const isa_parser_t *isa, const cfg_t *cfg, simif_t *sim,
            uint32_t id, bool halt_on_reset, FILE *log_file,
            std::ostream &sout_,
            Params &params); // because of command line option --log and -s we
                             // need both
  ~Processor();
  st_rvfi step(size_t n, st_rvfi reference);
  void revert_step(int num_steps);
  bool will_trigger_interrupt(reg_t mip);
  bool interrupt(reg_t mip, uint32_t revert_steps, bool interrupt_allowed);

  inline void set_XPR(reg_t num, reg_t value);
  inline void set_FPR(reg_t num, float128_t value);

protected:
  std::deque<prev_changes_t> previous_states;
  int max_previous_states;
  bool taken_trap;
  uint32_t which_trap;
  reg_t next_rvfi_intr;
  virtual void take_trap(trap_t &t, reg_t epc); // take an exception
};

} // namespace openhw
