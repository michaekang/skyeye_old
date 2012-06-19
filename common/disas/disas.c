/* General "disassemble this chunk" code.  Used for debugging. */
//#include "config.h"
#include "dis-asm.h"
#include "elf.h"
#include <errno.h>

//#include "cpu.h"
//#include "exec-all.h"
#include "disas.h"

/* Filled in by elfload.c.  Simplistic, but will do for now. */
struct syminfo *syminfos = NULL;
#if 0
/* Get LENGTH bytes from info's buffer, at target address memaddr.
   Transfer them to myaddr.  */
int
buffer_read_memory(bfd_vma memaddr, bfd_byte *myaddr, int length,
                   struct disassemble_info *info)
{
    if (memaddr < info->buffer_vma
        || memaddr + length > info->buffer_vma + info->buffer_length)
        /* Out of bounds.  Use EIO because GDB uses it.  */
        return EIO;
    memcpy (myaddr, info->buffer + (memaddr - info->buffer_vma), length);
    return 0;
}

/* Get LENGTH bytes from info's buffer, at target address memaddr.
   Transfer them to myaddr.  */
static int
target_read_memory (bfd_vma memaddr,
                    bfd_byte *myaddr,
                    int length,
                    struct disassemble_info *info)
{
    cpu_memory_rw_debug(cpu_single_env, memaddr, myaddr, length, 0);
    return 0;
}

/* Print an error message.  We can assume that this is in response to
   an error return from buffer_read_memory.  */
void
perror_memory (int status, bfd_vma memaddr, struct disassemble_info *info)
{
  if (status != EIO)
    /* Can't happen.  */
    (*info->fprintf_func) (info->stream, "Unknown error %d\n", status);
  else
    /* Actually, address between memaddr and memaddr + len was
       out of bounds.  */
    (*info->fprintf_func) (info->stream,
			   "Address 0x%" PRIx64 " is out of bounds.\n", memaddr);
}

/* This could be in a separate file, to save miniscule amounts of space
   in statically linked executables.  */

/* Just print the address is hex.  This is included for completeness even
   though both GDB and objdump provide their own (to print symbolic
   addresses).  */


/* Just return the given address.  */

int
generic_symbol_at_address (bfd_vma addr, struct disassemble_info *info)
{
  return 1;
}

bfd_vma bfd_getl64 (const bfd_byte *addr)
{
  unsigned long long v;

  v = (unsigned long long) addr[0];
  v |= (unsigned long long) addr[1] << 8;
  v |= (unsigned long long) addr[2] << 16;
  v |= (unsigned long long) addr[3] << 24;
  v |= (unsigned long long) addr[4] << 32;
  v |= (unsigned long long) addr[5] << 40;
  v |= (unsigned long long) addr[6] << 48;
  v |= (unsigned long long) addr[7] << 56;
  return (bfd_vma) v;
}

bfd_vma bfd_getl32 (const bfd_byte *addr)
{
  unsigned long v;

  v = (unsigned long) addr[0];
  v |= (unsigned long) addr[1] << 8;
  v |= (unsigned long) addr[2] << 16;
  v |= (unsigned long) addr[3] << 24;
  return (bfd_vma) v;
}

bfd_vma bfd_getb32 (const bfd_byte *addr)
{
  unsigned long v;

  v = (unsigned long) addr[0] << 24;
  v |= (unsigned long) addr[1] << 16;
  v |= (unsigned long) addr[2] << 8;
  v |= (unsigned long) addr[3];
  return (bfd_vma) v;
}

bfd_vma bfd_getl16 (const bfd_byte *addr)
{
  unsigned long v;

  v = (unsigned long) addr[0];
  v |= (unsigned long) addr[1] << 8;
  return (bfd_vma) v;
}

bfd_vma bfd_getb16 (const bfd_byte *addr)
{
  unsigned long v;

  v = (unsigned long) addr[0] << 24;
  v |= (unsigned long) addr[1] << 16;
  return (bfd_vma) v;
}
#endif
void
generic_print_address (bfd_vma addr, struct disassemble_info *info)
{
    (*info->fprintf_func) (info->stream, "0x%" PRIx64, addr);
}

static int
print_insn_thumb1(bfd_vma pc, disassemble_info *info)
{
  return print_insn_arm(pc | 1, info);
}

/* Disassemble this for me please... (debugging). 'flags' has the following
   values:
    i386 - nonzero means 16 bit code
    arm  - nonzero means thumb code
    ppc  - nonzero means little endian
    other targets - unused
 */
#define TARGET_FMT_lx "%08x"
typedef uint32_t target_ulong;
void target_disas(FILE *out, target_ulong code, target_ulong size, int flags)
{
    target_ulong pc;
    int count;
    struct disassemble_info disasm_info;
    int (*print_insn)(bfd_vma pc, disassemble_info *info);

//    INIT_DISASSEMBLE_INFO(disasm_info, out, fprintf);

//    disasm_info.read_memory_func = target_read_memory;
    disasm_info.buffer_vma = code;
    disasm_info.buffer_length = size;

#ifdef TARGET_WORDS_BIGENDIAN
    disasm_info.endian = BFD_ENDIAN_BIG;
#else
    disasm_info.endian = BFD_ENDIAN_LITTLE;
#endif
    if (flags)
	print_insn = print_insn_thumb1;
    else
	print_insn = print_insn_arm;

	count = print_insn(pc, &disasm_info);
	return;
    for (pc = code; size > 0; pc += count, size -= count) {
	fprintf(out, "0x" TARGET_FMT_lx ":  ", pc);
	count = print_insn(pc, &disasm_info);
#if 0
        {
            int i;
            uint8_t b;
            fprintf(out, " {");
            for(i = 0; i < count; i++) {
                target_read_memory(pc + i, &b, 1, &disasm_info);
                fprintf(out, " %02x", b);
            }
            fprintf(out, " }");
        }
#endif
	fprintf(out, "\n");
	if (count < 0)
	    break;
        if (size < count) {
            fprintf(out,
                    "Disassembler disagrees with translator over instruction "
                    "decoding\n"
                    "Please report this to qemu-devel@nongnu.org\n");
            break;
        }
    }
}

/* Disassemble this for me please... (debugging). */
void disas(FILE *out, void *code, unsigned long size, unsigned long pc)
{
//    unsigned long pc;
    int count;
    struct disassemble_info disasm_info;
    int (*print_insn)(bfd_vma pc, disassemble_info *info);

    INIT_DISASSEMBLE_INFO(disasm_info, out, fprintf);

    disasm_info.buffer = code;
    disasm_info.buffer_vma = (unsigned long)code;
    disasm_info.buffer_length = size;

#ifdef HOST_WORDS_BIGENDIAN
    disasm_info.endian = BFD_ENDIAN_BIG;
#else
    disasm_info.endian = BFD_ENDIAN_LITTLE;
#endif
    print_insn = print_insn_arm;

//    for (pc = (unsigned long)code; size > 0; pc += count, size -= count) {
    for (; size > 0; pc += count, size -= count) {
	fprintf(out, "0x%08lx:  ", pc);
	count = print_insn(pc, &disasm_info);
	fprintf(out, "\n");
	if (count < 0)
	    break;
    }
}

/* Look up symbol for debugging purpose.  Returns "" if unknown. */
const char *lookup_symbol(target_ulong orig_addr)
{
    const char *symbol = "";
    struct syminfo *s;

    for (s = syminfos; s; s = s->next) {
        symbol = s->lookup_symbol(s, orig_addr);
        if (symbol[0] != '\0') {
            break;
        }
    }

    return symbol;
}

#if 0
#if !defined(CONFIG_USER_ONLY)

#include "monitor.h"

static int monitor_disas_is_physical;
static CPUState *monitor_disas_env;

static int
monitor_read_memory (bfd_vma memaddr, bfd_byte *myaddr, int length,
                     struct disassemble_info *info)
{
    if (monitor_disas_is_physical) {
        cpu_physical_memory_rw(memaddr, myaddr, length, 0);
    } else {
 //       cpu_memory_rw_debug(monitor_disas_env, memaddr,myaddr, length, 0);
    }
    return 0;
}

static int GCC_FMT_ATTR(2, 3)
monitor_fprintf(FILE *stream, const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    monitor_vprintf((Monitor *)stream, fmt, ap);
    va_end(ap);
    return 0;
}

void monitor_disas(Monitor *mon, CPUState *env,
                   target_ulong pc, int nb_insn, int is_physical, int flags)
{
    int count, i;
    struct disassemble_info disasm_info;
    int (*print_insn)(bfd_vma pc, disassemble_info *info);

    INIT_DISASSEMBLE_INFO(disasm_info, (FILE *)mon, monitor_fprintf);

    monitor_disas_env = env;
    monitor_disas_is_physical = is_physical;
//    disasm_info.read_memory_func = monitor_read_memory;

    disasm_info.buffer_vma = pc;

#ifdef TARGET_WORDS_BIGENDIAN
    disasm_info.endian = BFD_ENDIAN_BIG;
#else
    disasm_info.endian = BFD_ENDIAN_LITTLE;
#endif
    print_insn = print_insn_arm;

    for(i = 0; i < nb_insn; i++) {
	monitor_printf(mon, "0x" TARGET_FMT_lx ":  ", pc);
	count = print_insn(pc, &disasm_info);
	monitor_printf(mon, "\n");
	if (count < 0)
	    break;
        pc += count;
    }
}
#endif
#endif
