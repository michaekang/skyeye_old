Function *cpu_create_function(cpu_t *cpu, const char *name, BasicBlock **p_bb_ret, BasicBlock **p_bb_trap, BasicBlock **p_bb_timeout, BasicBlock **p_label_entry);
void
spill_reg_state_helper(uint32_t count, Value **in_ptr_r, Value **ptr_r,
	BasicBlock *bb);

