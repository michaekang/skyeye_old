/*
 * skyeye_mach_omap4460x.c - define machine omap4460 for skyeye
 *
 * Copyright (C) 2012 Oubang Shen <shenoubang@gmail.com>
 * Skyeye Develop Group, for help please send mail to
 * <skyeye-developer@lists.gro.clinux.org>
 *
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */
#include <stdlib.h>

#include <skyeye.h>
#include <skyeye_config.h>
#include <skyeye_mach.h>
#include <skyeye_arch.h>
#include <skyeye_types.h>
#include <skyeye_obj.h>
#include <skyeye_internal.h>
#include <skyeye_addr_space.h>

#include "omap4460x.h"

omap4460x_io_t omap4460x_io;
#define io omap4460x_io

static void omap4460x_io_do_cycle(generic_arch_t* state) {
	// TODO: function body
	//SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, omap4460x_io_do_cycle not realize\n", __func__, __LINE__);
}

static void omap4460x_io_reset(generic_arch_t* arch_instance) {
	int i = 0;
	arch_instance->set_regval_by_id(1, 2160);
	io.ckgen_prm.cm_abe_pll_ref_clksel = 0x01;
	io.ckgen_prm.cm_sys_clksel =  0x01;
	io.ckgen_cm1.cm_clkmode_dpll_core = 0x4;
	io.ckgen_cm1.cm_clksel_dpll_core = 0x0;
	io.ckgen_cm1.cm_clkmode_dpll_mpu = 0x4;
	io.ckgen_cm1.cm_clkmode_dpll_iva = 0x4;
	io.ckgen_cm1.cm_clkmode_dpll_abe = 0x4;
	io.ckgen_cm1.cm_div_m2_dpll_abe = 0x1;
	io.ckgen_cm1.cm_div_m3_dpll_abe = 0x1;
	io.ckgen_cm2.cm_clksel_mpu_m3_iss_root = 0x0;
	io.ckgen_cm2.cm_clkmode_dpll_per = 0x4;
	io.ckgen_cm2.cm_clksel_dpll_per = 0x0;
	io.ckgen_cm2.cm_clkmode_dpll_usb = 0x4;
	io.ckgen_cm2.cm_scale_fclk = 0x0;
	io.ckgen_cm2.cm_clksel_usb_60mhz = 0x1;
	io.ckgen_cm2.cm_div_m2_dpll_usb = 0x1;
	io.ckgen_cm2.cm_div_m7_dpll_per = 0x4;
	io.ckgen_cm2.cm_div_m6_dpll_per = 0x4;
	io.ckgen_cm2.cm_div_m5_dpll_per = 0x4;
	io.ckgen_cm2.cm_div_m4_dpll_per = 0x4;
	io.ckgen_cm2.cm_div_m3_dpll_per = 0x1;
	io.ckgen_cm2.cm_div_m2_dpll_per = 0x1;
	io.abe_cm1.cm1_abe_dmic_clkctrl = 0x30000;
	io.abe_cm1.cm1_abe_gptimer5_clkctrl = 0x30000;
	io.abe_cm1.cm1_abe_gptimer6_clkctrl = 0x30000;
	io.abe_cm1.cm1_abe_gptimer7_clkctrl = 0x30000;
	io.abe_cm1.cm1_abe_gptimer8_clkctrl = 0x30000;
	io.abe_cm1.cm1_abe_mcasp_clkctrl = 0x30000;
	io.abe_cm1.cm1_abe_mcbsp1_clkctrl = 0x3000;
	io.abe_cm1.cm1_abe_mcbsp1_clkctrl = 0x3000;
	io.abe_cm1.cm1_abe_mcbsp1_clkctrl = 0x3000;
	io.l3int_cm2.cm_l3init_hsmmc1_clkctrl = 0x70000;
	io.l3int_cm2.cm_l3init_hsmmc2_clkctrl = 0x70000;
	io.l3int_cm2.cm_l3init_hsusbhost_clkctrl = 0x70000;
	io.l3int_cm2.cm_l3init_hsusbotg_clkctrl = 0x70000;
	io.l3int_cm2.cm_l3init_hsi_clkctrl = 0x70000;
	io.gpmc.gpmc_sysconfig = 0x0;
	for (i = 0; i < 8; i++) {
		io.gpmc.gpmc_config7[i] = 0xF00;
	}
	// TODO: function body
	SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, omap4460x_io_reset not realize\n",  __func__, __LINE__);
}

static void gpio_read(omap4460x_io_t* io, uint32 addr, uint32* data) {
	int i = 0;

	if (((addr - GPIO1_BASE) >= 0) &&
		((addr - GPIO1_BASE) <= 0x194)) {
		i = 0;
	} // GPIO_1
	else {
		i = ((addr - GPIO2_BASE) / 0x2000) + 1;
	}
	//printf("IN %s, line = %d, i = %d, addr = 0x%x\n", __func__, __LINE__, i, addr);

	switch ((addr & 0xfff)) {
		case 0x00:						/* IP revision identifier FIXME: modify it*/
			*data = 0;
			break;
		default:
			SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, addr = 0x%x\n",
					__func__, __LINE__, addr);
			break;
	}

	return;
}

static void gp_timer_read(omap4460x_io_t* io, uint32 addr, uint32* data) {
	int i = 0;

	if (((addr -GPTIMER1_L4_BASE) >= 0) && (addr -GPTIMER1_L4_BASE) <=	0x58) {
		i = 0;
	}

	switch (addr & 0xff) {
		case 0x10:
			*data = io->gp_timer[i].gpt1ms_tiocp_cfg;
			break;
		case 0x24:
			*data = io->gp_timer[i].gpt_tclr;
			break;
		case 0x34:
			*data = 0x0;
			break;
		default:
			SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, addr = 0x%x, data = 0x%x\n",
					__func__, __LINE__, addr, *data);
			break;
	}

	return;
}

static uint32 omap4460x_io_read_word(void* arch_instance, uint32 addr) {
	uint32 data = 0;

	// TODO: read word, how to do?
	conf_object_t* conf_obj = get_conf_obj("omap4460_mach_space");
    addr_space_t* phys_mem = (addr_space_t*)conf_obj->obj;
    exception_t ret = phys_mem->memory_space->read(conf_obj, addr, &data, 4);
    /* Read the data successfully */
    if(ret == No_exp){
        return data;
    }

	if ((addr >= 0x4A0081C0) && (addr <= 0x4A0081EC)) {
		data = 0;
		return data;
	}
	else if ((((addr - GPIO1_BASE) >= 0) && ((addr - GPIO1_BASE) <= 0x194)) ||		/* GPIO_1 */
			(((addr - GPIO2_BASE) >= 0) && ((addr - GPIO2_BASE) <= 0x194)) ||		/* GPIO_2 */
			(((addr - GPIO3_BASE) >= 0) && ((addr - GPIO3_BASE) <= 0x194)) ||		/* GPIO_3 */
			(((addr - GPIO4_BASE) >= 0) && ((addr - GPIO4_BASE) <= 0x194)) ||		/* GPIO_4 */
			(((addr - GPIO5_BASE) >= 0) && ((addr - GPIO5_BASE) <= 0x194)) ||		/* GPIO_5 */
			(((addr - GPIO6_BASE) >= 0) && ((addr - GPIO6_BASE) <= 0x194))){		/* GPIO_6 */
				gpio_read(&io, addr, &data);
				return data;
		} // GPIO
	else if (((addr -GPTIMER1_L4_BASE) >= 0) && (addr -GPTIMER1_L4_BASE) <=	0x58) {
		gp_timer_read(&io, addr, &data);
		return data;
	} // GP Timer

	switch (addr) {
		case CONTROL_ID_CODE:
			data = 0x0B94E02F;
			break;
		case SCU_CONFIG:
			/* 2 cpus, SMP adn 32KB cache*/
			data = 0x0531;
			break;
		case GPMC_REVISION: /* GPMC_REVISION */
			data = 0; /* FIXME: please modify it */
			break;
		case INTER_CONTR_TYPE:
			/* 128 interrupts 2 cpus */
			data = 0xec23;
			break;
		case GPMC_SYSCONFIG:
			data = io.gpmc.gpmc_sysconfig;
			break;
		case CM_ABE_PLL_REF_CLKSEL:
			data = io.ckgen_prm.cm_abe_pll_ref_clksel;
			break;
		case CM_SYS_CLKSEL:
			data = io.ckgen_prm.cm_sys_clksel;
			break;
		case CM_CLKMODE_DPLL_CORE:
			data = io.ckgen_cm1.cm_clkmode_dpll_core;
			break;
		case CM_CLKSEL_DPLL_CORE:
			data = io.ckgen_cm1.cm_clksel_dpll_core;
			break;
		case CM_CLKMODE_DPLL_MPU:
			data = io.ckgen_cm1.cm_clkmode_dpll_mpu;
			break;
		case CM_CLKMODE_DPLL_IVA:
			data = io.ckgen_cm1.cm_clkmode_dpll_iva;
			break;
		case CM_CLKMODE_DPLL_ABE:
			data = io.ckgen_cm1.cm_clkmode_dpll_abe;
			break;
		case CM_DIV_M2_DPLL_ABE:
			data = io.ckgen_cm1.cm_div_m2_dpll_abe;
			break;
		case CM_DIV_M3_DPLL_ABE:
			data = io.ckgen_cm1.cm_div_m3_dpll_abe;
			break;
		case CM_CLKSEL_MPU_M3_ISS_ROOT:
			data = io.ckgen_cm2.cm_clksel_mpu_m3_iss_root;
			break;
		case CM_CLKMODE_DPLL_PER:
			data = io.ckgen_cm2.cm_clkmode_dpll_per;
			break;
		case CM_CLKSEL_DPLL_PER:
			data = io.ckgen_cm2.cm_clksel_dpll_per;
			break;
		case CM_CLKMODE_DPLL_USB:
			data = io.ckgen_cm2.cm_clkmode_dpll_usb;
			break;
		case CM_SCALE_FCLK:
			data = io.ckgen_cm2.cm_scale_fclk;
			break;
		case CM_CLKSEL_USB_60MHZ:
			data = io.ckgen_cm2.cm_clksel_usb_60mhz;
			break;
		case CM_DIV_M2_DPLL_USB:
			data = io.ckgen_cm2.cm_div_m2_dpll_usb;
			break;
		case CM_DIV_M7_DPLL_PER:
			data = io.ckgen_cm2.cm_div_m7_dpll_per;
			break;
		case CM_DIV_M6_DPLL_PER:
			data = io.ckgen_cm2.cm_div_m6_dpll_per;
			break;
		case CM_DIV_M5_DPLL_PER:
			data = io.ckgen_cm2.cm_div_m5_dpll_per;
			break;
		case CM_DIV_M4_DPLL_PER:
			data = io.ckgen_cm2.cm_div_m4_dpll_per;
			break;
		case CM_DIV_M3_DPLL_PER:
			data = io.ckgen_cm2.cm_div_m3_dpll_per;
			break;
		case CM_DIV_M2_DPLL_PER:
			data = io.ckgen_cm2.cm_div_m2_dpll_per;
			break;
		case CM1_ABE_DMIC_CLKCTRL:
			data = io.abe_cm1.cm1_abe_dmic_clkctrl;
			break;
		case  CM1_ABE_GPTIMER5_CLKCTRL:
			data = io.abe_cm1.cm1_abe_gptimer5_clkctrl;
			break;
		case  CM1_ABE_GPTIMER6_CLKCTRL:
			data = io.abe_cm1.cm1_abe_gptimer6_clkctrl;
			break;
		case  CM1_ABE_GPTIMER7_CLKCTRL:
			data = io.abe_cm1.cm1_abe_gptimer7_clkctrl;
			break;
		case  CM1_ABE_GPTIMER8_CLKCTRL:
			data = io.abe_cm1.cm1_abe_gptimer8_clkctrl;
			break;
		case CM1_ABE_MCASP_CLKCTRL:
			data = io.abe_cm1.cm1_abe_mcasp_clkctrl;
			break;
		case CM1_ABE_MCBSP1_CLKCTRL:
			data = io.abe_cm1.cm1_abe_mcbsp1_clkctrl;
			break;
		case CM1_ABE_MCBSP2_CLKCTRL:
			data = io.abe_cm1.cm1_abe_mcbsp2_clkctrl;
			break;
		case CM1_ABE_MCBSP3_CLKCTRL:
			data = io.abe_cm1.cm1_abe_mcbsp3_clkctrl;
			break;
		case CM_L3INIT_HSMMC1_CLKCTRL:
			data = io.l3int_cm2.cm_l3init_hsmmc1_clkctrl;
			break;
		case CM_L3INIT_HSMMC2_CLKCTRL:
			data = io.l3int_cm2.cm_l3init_hsmmc2_clkctrl;
			break;
		case CM_L3INIT_HSUSBHOST_CLKCTRL:
			data = io.l3int_cm2.cm_l3init_hsusbhost_clkctrl;
			break;
		case CM_L3INIT_HSUSBOTG_CLKCTRL:
			data = io.l3int_cm2.cm_l3init_hsusbotg_clkctrl;
			break;
		case CM_L3INIT_HSI_CLKCTRL:
			data = io.l3int_cm2.cm_l3init_hsi_clkctrl;
			break;
		case 0x4a009428:
		case 0x4a009430:
		case 0x4a009438:
		case 0x4a009440:
		case 0x4a009448:
		case 0x4a009450:
		case 0x4a0094e0:
		case 0x4a307840:
		case 0x4a009470:
		case 0x4a009478:
		case 0x4a009480:
			data = 0x30000;
			break;
		case 0x4a008b38:
		case 0x4a008b30:
			data = 0x30001;
			break;
		case 0x4a306100:
		case 0x4a306104:
		case 0x4a306108:
		case 0x4a004108:
		case 0x4a0041b8:
		case 0x4a00419c:
		case 0x4a0041dc:
		case 0x4a004100:
		case 0x4a307858:
			data = 0;
			break;
		case 0x4a009220:
		case 0x4a004528:
		case 0x4a009028:
			data = 0x70000;
			break;
		case 0x4a307a20:
			data = 0x9000000;
			break;
		case 0x4a004144:
		case 0x4a004138:
		case 0x4a00413c:
		case 0x4a0041bc:
			data = 0x4;
			break;
		case 0x4a004170:
		case 0x4a004130:
		case 0x4a004134:
			data = 0x1;
			break;
		case 0x4a004140:
			data = 0x8;
			break;
		default:
			if (((addr - GPMC_CONFIG7_BASE) >= 0) &&
				((addr - GPMC_CONFIG7_BASE) <= 0x150)) {
				data = io.gpmc.gpmc_config7[((addr - GPMC_CONFIG7_BASE) / 0x30)];
			} // CS address
			else if (((addr - SPI_TARGET_BASE) >= 0) &&
					(addr - SPI_TARGET_BASE) <= 0xFC) {
				data = io.interrupt.SPI_target[((addr - SPI_TARGET_BASE) >> 2)];
			} // SPI Target
			else {
				SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, omap4460x_io_read_word not realize, addr = 0x%x\n",
					__func__, __LINE__, addr);
			}
			break;
	}

	return data;
}

static uint32 omap4460x_io_read_byte(void* arch_instance, uint32 addr) {
	return omap4460x_io_read_word(arch_instance, addr);
}

static uint32 omap4460x_io_read_halfword(void* arch_instance, uint32 addr) {
	return omap4460x_io_read_word(arch_instance, addr);
}

static void gpio_write(omap4460x_io_t* io, uint32 addr, uint32 data) {
	int i = 0;

	if (((addr - GPIO1_BASE) >= 0) &&
		((addr - GPIO1_BASE) <= 0x194)) {
		i = 0;
	} // GPIO_1
	else {
		i = ((addr - GPIO2_BASE) / 0x2000) + 1;
	}

	switch ((addr & 0xfff)) {
		case 0x10:										/* System configuration register */
			io->gpio[i].gpio_sysconfig = data;
			break;
		case 0x3c:										/* Per-event interrupt enable clear vector */
			io->gpio[i].gpio_irqstatus_clr_0 = data;
			break;
		case 0x130:										 /* GPIO control register */
			io->gpio[i].gpio_ctrl = data;
			break;
		case 0x150:										 /* Debouncing enable register */
			io->gpio[i].gpio_debouncenable = data;
			break;
		default:
			SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, GPIO function not implemented!!!\n", __func__, __LINE__);
			break;
	}

	return;
}

static void gp_timer_write(omap4460x_io_t* io, uint32 addr, uint32 data) {
	int i = 0;

	if (((addr - GPTIMER1_L4_BASE) >= 0) && (addr - GPTIMER1_L4_BASE) <=	0x58) {
		i = 0;
	}

	switch (addr & 0xff) {
		case 0x10:
			io->gp_timer[i].gpt1ms_tiocp_cfg = data;
			break;
		case 0x1c:
			io->gp_timer[i].gpt_tier = data;
			break;
		case 0x18:
			io->gp_timer[i].gpt_tisr = data;
			break;
		case 0x20:
			io->gp_timer[i].gpt_twer = data;
			break;
		case 0x24:
			io->gp_timer[i].gpt_tclr = data;
			break;
		case 0x28:
			io->gp_timer[i].gpt_irqstatus = data;
			break;
		case 0x2c:
			io->gp_timer[i].gpt_tldr = data;
			break;
		case 0x40:
			io->gp_timer[i].gpt_tsicr = data;
			break;
		default:
			SKYEYE_LOG_IN_CLR(RED, "IN %s, line = %d, addr = 0x%x, data = 0x%x\n",
					__func__, __LINE__, addr, data);
			break;
	}

	return;
}

static void omap4460x_io_write_word(generic_arch_t* state, uint32 addr, uint32 data) {
	// TODO: write word how to do?
	conf_object_t* conf_obj = get_conf_obj("omap4460_mach_space");
    addr_space_t* phys_mem = (addr_space_t*)conf_obj->obj;
    exception_t ret = phys_mem->memory_space->write(conf_obj, addr, &data, 4);
    /* Read the data successfully */
    if(ret == No_exp){
        return;
    }

	if ((((addr - GPIO1_BASE) >= 0) && ((addr - GPIO1_BASE) <= 0x194)) ||		/* GPIO_1 */
			(((addr - GPIO2_BASE) >= 0) && ((addr - GPIO2_BASE) <= 0x194)) ||	/* GPIO_2 */
			(((addr - GPIO3_BASE) >= 0) && ((addr - GPIO3_BASE) <= 0x194)) ||	/* GPIO_3 */
			(((addr - GPIO4_BASE) >= 0) && ((addr - GPIO4_BASE) <= 0x194)) ||	/* GPIO_4 */
			(((addr - GPIO5_BASE) >= 0) && ((addr - GPIO5_BASE) <= 0x194)) ||	/* GPIO_5 */
			(((addr - GPIO6_BASE) >= 0) && ((addr - GPIO6_BASE) <= 0x194))){	/* GPIO_6 */
				gpio_write(&io, addr, data);
				return;
	} // GPIO
	else if (((addr -GPTIMER1_L4_BASE) >= 0) && (addr -GPTIMER1_L4_BASE) <=	0x58) {
		gp_timer_write(&io, addr, data);
		return;
	} // GP Timer

	switch (addr) {
		case GPMC_SYSCONFIG:
			io.gpmc.gpmc_sysconfig = data;
			break;
		case DISTRIBUTOR_CONTR:
			io.interrupt.distributor_contr = data;
			break;
		case CPU_INTERF_CONTR:
			io.interrupt.cpu_interf_contr = data;
			break;
		case INTER_PRIORIRY_MASK:
			io.interrupt.inter_prioriry_mask = data;
			break;
		case ICDSGIR:
			io.interrupt.icdsgir = data;
			break;
		case 0x4a008b30:		 /* manages the EMIF_1 clocks */
		case 0x4a008b38:		 /* manages the EMIF_2 clocks. */
		case 0x4a009450:		 /* manages the DMTIMER9 clock */
		case 0x4a009470:		 /* manages the GPIO4 clocks */
		case 0x4a009478:		 /* manages the GPIO5 clocks */
		case 0x4a009480:		 /* manages the GPIO6 clocks */
			break;
		default:
			if (((addr - INTER_ENABLE_SET_BASE) >= 0) &&
					(addr - INTER_ENABLE_SET_BASE) <= 0x1C) {
				io.interrupt.enable_set[((addr - INTER_ENABLE_SET_BASE) >> 2)] = data;
			} // interrupt Set-Enable
			else if (((addr - ENABLE_CLEAR_BASE) >= 0) &&
					(addr - ENABLE_CLEAR_BASE) <= 0x1C) {
				io.interrupt.priority_level[((addr - ENABLE_CLEAR_BASE) >> 2)] = data;
			} // Priority level
			else if (((addr - PRIORITY_LEVEL_BASE) >= 0) &&
					(addr - PRIORITY_LEVEL_BASE) <= 0xFC) {
				io.interrupt. priority_level[((addr - PRIORITY_LEVEL_BASE) >> 2)] = data;
			} // Priority level
			else if (((addr - SPI_TARGET_BASE) >= 0) &&
					(addr - SPI_TARGET_BASE) <= 0xFC) {
				io.interrupt.SPI_target[((addr - SPI_TARGET_BASE) >> 2)] = data;
			} // SPI Target
			else if (((addr - INTER_CONFIG_BASE) >= 0) &&
					(addr - INTER_CONFIG_BASE) <= 0x3c) {
				io.interrupt.inter_config[((addr - INTER_CONFIG_BASE) >> 2)] = data;
				//printf("i = %d, data = 0x%x, addr = 0x%x, offset = 0x%x\n",
				//		(addr - INTER_CONFIG_BASE) >> 2, data, addr, addr - INTER_CONFIG_BASE);
			} // Interrupt configuration
			else {
				SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, addr = 0x%x, write_word function not realize\n", __func__, __LINE__, addr);
			}
			break;
	}

	return;
}

static void omap4460x_io_write_byte(generic_arch_t* state, uint32 addr, uint32 data) {
	omap4460x_io_write_word(state, addr, data);
}

static void omap4460x_io_write_halfword(generic_arch_t* state, uint32 addr, uint32 data) {
	omap4460x_io_write_word(state, addr, data);
}

static void omap4460x_update_int(generic_arch_t* state) {
	// TODO: function body
	SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, omap4460x_update_int not realize\n", __func__, __LINE__);
}

static void omap4460x_set_ext_intr(uint32 interrupt) {
	// TODO: function body
	SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, omap4460x_set_intr not realize\n", __func__, __LINE__);
}

static void omap4460x_pending_ext_intr(uint32 interrupt) {
	// TODO: function body
	SKYEYE_LOG_IN_CLR(RED, "In %s, line = %d, omap4460x_pending_intr not realize\n", __func__, __LINE__);
}

static void omap4460x_update_intr(void* mach) {
	generic_arch_t* arch_instance = get_arch_instance("");
	omap4460x_update_int(arch_instance);
}

void omap4460x_mach_init(void *arch_instance, machine_config_t *this_mach) {
	this_mach->mach_io_do_cycle = omap4460x_io_do_cycle;
	this_mach->mach_io_reset = omap4460x_io_reset;
	this_mach->mach_io_read_byte = omap4460x_io_read_byte;
	this_mach->mach_io_read_halfword = omap4460x_io_read_halfword;
	this_mach->mach_io_read_word = omap4460x_io_read_word;
	this_mach->mach_io_write_byte = omap4460x_io_write_byte;
	this_mach->mach_io_write_halfword = omap4460x_io_write_halfword;
	this_mach->mach_io_write_word = omap4460x_io_write_word;
	this_mach->mach_update_int = omap4460x_update_int;
	this_mach->mach_set_intr = omap4460x_set_ext_intr;
	this_mach->mach_pending_intr = omap4460x_pending_ext_intr;
	this_mach->mach_update_intr = omap4460x_update_intr;
	this_mach->state = (void*)arch_instance;

	add_chp_data(&omap4460x_io, sizeof(omap4460x_io_t), "omap4460io");
	/* The whole address space */
	addr_space_t* phys_mem = new_addr_space("omap4460_mach_space");
	SKYEYE_DBG("In %s, line = %d, omap4460x_mach_init ok!!!!\n", __func__, __LINE__);
}
