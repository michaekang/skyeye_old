#define SUPER_IRQ 0x10

.text
	.align 4
	.global begin
	.type begin, #function

begin:
	nop
	nop
	/*set hello, %l4
	jmp  %l4
	set init_stack, %sp 
	*/
	call hello
	nop
	nop
	/* bl hello */
	b begin

.data
	.align 4
init_stack:
	.space 8092


