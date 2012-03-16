/*
 * omap4460x.h - definitions of "s3c6410" machine  for skyeye
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

#ifndef __OMAP4460X_H__
#define __OMAP4460X_H__  1

#ifndef u32
#define u32 unsigned int
#endif

/******************************************/
/***** SYSCTRL_GENERAL_CORE Registers *****/
/******************************************/
#define SYSCTRL_CORE_BASE (0x4A002000)
#define CONTROL_ID_CODE  (SYSCTRL_CORE_BASE + 0x0204)	/* ID_CODE Key Register */


/******************************************/
/********* SCU Registers     **************/
/******************************************/
#define SCU_BASE (0x48240000)
#define SCU_CTRL (0x48240000 + 0x00)			/* SCU Control Register */
#define SCU_CONFIG (SCU_BASE + 0x04)			/* SCU Configuration Register */

/******************************************/
/*********  CKGEN_PRM Register     ********/
/******************************************/
#define CKGEN_PRM_BASE  (0x4A306100)
#define CM_ABE_DSS_SYS_CLKSEL (CKGEN_PRM_BASE + 0x00)	/* Select the SYS CLK for ABE and DSS subsystems */
#define CM_L4_WKUP_CLKSEL (CKGEN_PRM_BASE + 0x08)		/* control the functional clock source of L4_WKUP, PRM and Smart Reflex functional clock. */
#define CM_ABE_PLL_REF_CLKSEL (CKGEN_PRM_BASE + 0x0C)	/* Control the source of the reference clock for DPLL_ABE */
#define CM_SYS_CLKSEL (CKGEN_PRM_BASE + 0x10)			/* Software sets the SYS_CLK configuration */

/******************************************/
/*********  CKGEN_CM1 Register     ********/
/******************************************/
#define CKGEN_CM1_BASE (0x4A004100)
#define CM_CLKSEL_CORE (CKGEN_CM1_BASE + 0x00)			/* CORE module clock selection. */
#define CM_CLKMODE_DPLL_CORE  (CKGEN_CM1_BASE + 0x20)	/* allows controlling the DPLL modes */
#define CM_CLKSEL_DPLL_CORE  (CKGEN_CM1_BASE + 0x2C)	/* provides controls over the DPLL */
#define CM_CLKMODE_DPLL_MPU (CKGEN_CM1_BASE + 0x60)		/* allows controlling the DPLL modes */
#define CM_CLKMODE_DPLL_IVA  (CKGEN_CM1_BASE  + 0xA0)	/* allows controlling the DPLL modes */
#define CM_CLKMODE_DPLL_ABE  (CKGEN_CM1_BASE + 0xE0)	/* allows controlling the DPLL modes */
#define CM_DIV_M2_DPLL_ABE (CKGEN_CM1_BASE + 0xF0)		/* allows over the M2 divider of the DPLL. */
#define CM_DIV_M3_DPLL_ABE (CKGEN_CM1_BASE + 0xF4)		/* allows over the M2 divider of the DPLL. */

/******************************************/
/*********  CKGEN_CM2 Register     ********/
/******************************************/
#define CKGEN_CM2_BASE (0x4A008100)
#define CM_CLKSEL_MPU_M3_ISS_ROOT (CKGEN_CM2_BASE + 0x00)	/* MPU_A3/ISS	root clock selection 0*/
#define CM_CLKSEL_USB_60MHZ (CKGEN_CM2_BASE + 0x04)		/* Selects the configuration of the divider generating 60MHz clock for USB */
#define CM_SCALE_FCLK (CKGEN_CM2_BASE + 0x08)			/* scale PER_ABE_NC_FCLK, 96M_FCLK, 48M_FCLK, and 64M_FCLK */
#define CM_DIV_M2_DPLL_USB (CKGEN_CM2_BASE + 0x90)		/* provides controls over the M2 divider of the DPLL */
#define CM_CLKMODE_DPLL_PER	(CKGEN_CM2_BASE + 0x40)		/*  allows controlling the DPLL modes. 4 */
#define CM_CLKSEL_DPLL_PER  (CKGEN_CM2_BASE + 0x4C)		/* register provides controls over the DPLL. 0*/
#define CM_CLKMODE_DPLL_USB (CKGEN_CM2_BASE + 0x80)		/* allows controlling the DPLL modes 4*/
#define CM_DIV_M7_DPLL_PER  (CKGEN_CM2_BASE + 0x64)		/* control over the CLKOUT4 o/p of the HSDIVIDER */
#define CM_DIV_M6_DPLL_PER  (CKGEN_CM2_BASE + 0x60)		/* control over the CLKOUT3 o/p of the HSDIVIDER */
#define CM_DIV_M5_DPLL_PER  (CKGEN_CM2_BASE + 0x5C)		/* control over the CLKOUT2 o/p of the HSDIVIDER */
#define CM_DIV_M4_DPLL_PER	(CKGEN_CM2_BASE + 0x58)		/* control over the CLKOUT1 o/p of the HSDIVIDER */
#define CM_DIV_M3_DPLL_PER	(CKGEN_CM2_BASE + 0x54)		/* provides controls over the M3 divider of the DPLL */
#define CM_DIV_M2_DPLL_PER  (CKGEN_CM2_BASE + 0x50)		/* provides controls over the M2 divider of the DPLL */

/******************************************/
/*********  ABE_CM1 Register     ********/
/******************************************/
#define ABE_CM1_BASE  (0x4A004500)
#define CM1_ABE_CLKSTCTRL (ABE_CM1_BASE + 0x0)
#define CM1_ABE_DMIC_CLKCTRL  (ABE_CM1_BASE + 0x38)		/* manage DMIC clocks */
#define CM1_ABE_GPTIMER5_CLKCTRL  (ABE_CM1_BASE + 0x68) /* manage TIMER5 clocks */
#define CM1_ABE_GPTIMER6_CLKCTRL (ABE_CM1_BASE + 0x70) /* manage TIMER    6 clocks */
#define CM1_ABE_GPTIMER7_CLKCTRL (ABE_CM1_BASE + 0x78) /* manage TIMER7 clocks */
#define CM1_ABE_GPTIMER8_CLKCTRL (ABE_CM1_BASE + 0x80) /* manage TIMER8 clocks */
#define CM1_ABE_MCASP_CLKCTRL  (ABE_CM1_BASE + 0x40) /* manages the MCASP clocks. */
#define CM1_ABE_MCBSP1_CLKCTRL  (ABE_CM1_BASE + 0x48) /* manages the MCBSP1 clocks. */
#define CM1_ABE_MCBSP2_CLKCTRL  (ABE_CM1_BASE + 0x50) /* manages the MCBSP2 clocks. */
#define CM1_ABE_MCBSP3_CLKCTRL  (ABE_CM1_BASE + 0x58) /* manages the MCBSP3 clocks. */

/******************************************/
/*********  L3INT_CM2 Register     ********/
/******************************************/
#define L3INT_CM2_BASE (0x4A009300)
#define CM_L3INIT_HSMMC1_CLKCTRL	(L3INT_CM2_BASE + 0x28) /* manage the MMC1 clocks */
#define CM_L3INIT_HSMMC2_CLKCTRL	(L3INT_CM2_BASE + 0x30) /* manage the MMC2 clocks */
#define CM_L3INIT_HSUSBHOST_CLKCTRL (L3INT_CM2_BASE + 0x58) /* manages the USB_HOST_HS clocks. */
#define CM_L3INIT_HSUSBOTG_CLKCTRL  (L3INT_CM2_BASE + 0x60) /* manages the SB_OTG_HS clocks */
#define CM_L3INIT_HSI_CLKCTRL		(L3INT_CM2_BASE + 0x38) /* manages the HSI clocks */

/******************************************/
/*****       GPMC Registers			  *****/
/******************************************/
#define GPMC_BASE (0x50000000)
#define GPMC_REVISION (GPMC_BASE + 0x0)		/* contains the IP revision code */
#define GPMC_SYSCONFIG (GPMC_BASE + 0x10)	/* control the various parameters of the Interconnect */
#define GPMC_CONFIG7_BASE	(GPMC_BASE + 0X78)	/* CS address mapping configuration */


/******************************************/
/*****      Interrupt Registers		  *****/
/******************************************/
#define INTER_DISTRIBUTOR_BASE (0x48241000)
#define INTER_CONTR_TYPE  (INTER_DISTRIBUTOR_BASE + 0x4)


typedef struct gpmc {
	u32 gpmc_sysconfig;	/* control the various parameters of the Interconnect */
	u32 gpmc_config7[8]; /* CS address mapping configuration */
}gpmc_t;

typedef struct ckgen_prm {
	u32 cm_abe_dss_sys_clkse;	/* Select the SYS CLK for ABE and DSS subsystems */
	u32 cm_abe_dss_sys_clksel;	/* control the functional clock source of L4_WKUP, PRM and Smart Reflex functional clock. */
	u32 cm_abe_pll_ref_clksel;	/* Control the source of the reference clock for DPLL_ABE */
	u32 cm_sys_clksel;			/* Software sets the SYS_CLK configuration */
} ckgen_prm_t;

typedef struct l3int_cm2 {
	u32 cm_l3init_hsmmc1_clkctrl;		/* manage the MMC1 clocks 0x70 */
	u32 cm_l3init_hsmmc2_clkctrl;		/* manage the MMC2 clocks 0x70 */
	u32 cm_l3init_hsusbhost_clkctrl;	/* manages the USB_HOST_HS clocks. */
	u32 cm_l3init_hsusbotg_clkctrl;		/* manages the SB_OTG_HS clocks */
	u32 cm_l3init_hsi_clkctrl;			/* manages the HSI clocks */
}l3int_cm2_t;

typedef struct abe_cm1 {
	u32 cm1_abe_dmic_clkctrl;  		/* manange DMIC clocks */
	u32 cm1_abe_gptimer5_clkctrl;   /* manage TIMER5 clocks */
	u32 cm1_abe_gptimer6_clkctrl;	/* manage TIMER6 clocks */
	u32 cm1_abe_gptimer7_clkctrl;	/* manage TIMER6 clocks */
	u32 cm1_abe_gptimer8_clkctrl;	/* manage TIMER6 clocks */
	u32 cm1_abe_mcasp_clkctrl;		/* manages the MCASP clocks. */
	u32 cm1_abe_mcbsp1_clkctrl;		/* manages the MCBSP1 clocks. */
	u32 cm1_abe_mcbsp2_clkctrl;		/* manages the MCBSP2 clocks. */
	u32 cm1_abe_mcbsp3_clkctrl;		/* manages the MCBSP3 clocks. */
}abe_cm1_t;

typedef struct ckgen_cm1 {
	u32 cm_clksel_core;			/* CORE module clock selection. */
	u32 cm_clkmode_dpll_core;   /* allows controlling the DPLL modes */
    u32 cm_clksel_dpll_core;    /* provides controls over the DPLL */
    u32 cm_clkmode_dpll_mpu;    /* allows controlling the DPLL modes */
    u32 cm_clkmode_dpll_iva;    /* allows controlling the DPLL modes */
    u32 cm_clkmode_dpll_abe;    /* allows controlling the DPLL modes */
	u32 cm_div_m2_dpll_abe;		/* allows controlling over the M2 divider of the DPLL */
	u32 cm_div_m3_dpll_abe;		/* allows controlling over the M2 divider of the DPLL */
}ckgen_cm1_t;

typedef struct ckgen_cm2 {
	u32 cm_clksel_mpu_m3_iss_root; /* MPU_A3/ISS	root clock selection 0*/
	u32 cm_clksel_usb_60mhz;		/* Selects the configuration of the divider generating 60MHz clock for USB */
	u32 cm_scale_fclk;				/* scale PER_ABE_NC_FCLK, 96M_FCLK, 48M_FCLK, and 64M_FCLK */
	u32 cm_div_m2_dpll_usb;			/* provides controls over the M2 divider of the DPLL */
	u32 cm_clkmode_dpll_per;		/*  allows controlling the DPLL modes. 4 */
	u32 cm_clksel_dpll_per;         /* register provides controls over the DPLL. 0*/
	u32 cm_clkmode_dpll_usb;        /* allows controlling the DPLL modes 4*/
	u32 cm_div_m7_dpll_per;			/* control over the CLKOUT4 o/p of the HSDIVIDER */
	u32 cm_div_m6_dpll_per;			/* control over the CLKOUT3 o/p of the HSDIVIDER */
	u32 cm_div_m5_dpll_per;			/* control over the CLKOUT2 o/p of the HSDIVIDER */
	u32 cm_div_m4_dpll_per;			/* control over the CLKOUT1 o/p of the HSDIVIDER */
	u32 cm_div_m3_dpll_per;		    /* provides controls over the M3 divider of the DPLL */
	u32 cm_div_m2_dpll_per;			/* provides controls over the M2 divider of the DPLL */
}ckgen_cm2_t;

typedef struct omap4460x_io {
	/* CKGEN_PRM Register */
	ckgen_prm_t ckgen_prm;
	/* CKGEN_CM1 Register */
	ckgen_cm1_t ckgen_cm1;
	/* CKGEN_CM2 Register */
	ckgen_cm2_t ckgen_cm2;
	/* ABE_CM1 Register*/
	abe_cm1_t abe_cm1;
	/* L3INT_CM2 Register */
	l3int_cm2_t l3int_cm2;
	/* GMPC Registers */
	gpmc_t gpmc;
}omap4460x_io_t;

void omap4460x_mach_init(void *arch_instance, machine_config_t *this_mach);
#endif /* __OMAP4460X_H__ */
