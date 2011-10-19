/* 
This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2, or (at your option)
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.  */

/*
 * author gbf0871 <gbf0871@126.com>
 */
#ifndef __DEV_NANDFLASH_S3C6410_H_
#define __DEV_NANDFLASH_S3C6410_H_

#if 0
#define S3C6410_NFCONF_EN          (1<<15)
#define S3C6410_NFCONF_512BYTE     (1<<14)
#define S3C6410_NFCONF_4STEP       (1<<13)
#define S3C6410_NFCONF_INITECC     (1<<12)
#define S3C6410_NFCONF_nFCE        (1<<11)
#define S3C6410_NFCONF_TACLS(x)    ((x)<<8)
#define S3C6410_NFCONF_TWRPH0(x)   ((x)<<4)
#define S3C6410_NFCONF_TWRPH1(x)   ((x)<<0)
//#define NFCONF 0x4E000000	/* NAND flash configuration */
#define NFCONF 0x70200000 /* NAND flash configuration */
//#define NFCMD 0x4E000004	/* NAND flash command set register */
#define NFCMD 0x70200004 /* NAND flash command set register */
//#define NFADDR   0x4E000008  /*NAND flash address set register*/
#define NFADDR   0x70200008 /*NAND flash address set register*/
//#define NFDATA  0x4E00000C	/* NAND flash data register */
#define NFDATA  0x7020000C	/* NAND flash data register */
//#define NFSTAT   0x4E000010    /*NAND flash operation status*/
#define NFSTAT		0x70200010
//#define NFECC1   0x4E000014 	/* NAND flash ECC (Error Correction Code) register */
#define NFECC1   0x70200014 	/* NAND flash ECC (Error Correction Code) register */
#define NFECC2   0x70200015
#define NFECC3   0x70200016
#endif

#define S3C6410_NFCONF_BUSWIDTH_8   (0<<0)
#define S3C6410_NFCONF_BUSWIDTH_16  (1<<0)
#define S3C6410_NFCONF_ADVFLASH     (1<<3)
#define S3C6410_NFCONF_TACLS(x)     ((x)<<12)
#define S3C6410_NFCONF_TWRPH0(x)    ((x)<<8)
#define S3C6410_NFCONF_TWRPH1(x)    ((x)<<4)

#define S3C6410_NFCONT_LOCKTIGHT    (1<<13)
#define S3C6410_NFCONT_SOFTLOCK     (1<<12)
#define S3C6410_NFCONT_ILLEGALACC_EN    (1<<10)
#define S3C6410_NFCONT_RNBINT_EN    (1<<9)
#define S3C6410_NFCONT_RN_FALLING   (1<<8)
#define S3C6410_NFCONT_SPARE_ECCLOCK    (1<<6)
#define S3C6410_NFCONT_MAIN_ECCLOCK (1<<5)
#define S3C6410_NFCONT_INITECC      (1<<4)
#define S3C6410_NFCONT_nFCE     (1<<1)
#define S3C6410_NFCONT_ENABLE       (1<<0)

#define S3C6410_NFSTAT_READY        (1<<0)
#define S3C6410_NFSTAT_nCE      (1<<1)
#define S3C6410_NFSTAT_RnB_CHANGE   (1<<2)
#define S3C6410_NFSTAT_ILLEGAL_ACCESS   (1<<3)


#define NFCONF   0x70200000
#define NFCONT   0x70200004
#define NFCMD    0x70200008
#define NFADDR   0x7020000C
#define NFDATA   0x70200010
#define NFECCD0  0x70200014
#define NFECCD1  0x70200018
#define NFECCD   0x7020001C
#define NFSTAT   0x70200020
#define NFESTAT0 0x70200024
#define NFESTAT1 0x70200028
#define NFMECC0  0x7020002C
#define NFMECC1  0x70200030
#define NFSECC   0x70200034
#define NFSBLK   0x70200038
#define NFEBLK   0x7020003C

typedef struct nandflash_s3c6410_io
{
	uint32 nfconf;
	uint32 nfcont;
	uint32 nfcmd;
	uint32 nfaddr;
	uint32 nfdata;
	uint32 nfeccd0;
	uint32 nfeccd1;
	uint32 nfeccd;
	uint32 nfstat;
	uint32 nfestat0;
	uint32 nfestat1;
	uint32 nfmecc0;
	uint32 nfmecc1;
	uint32 nfsblk;
	uint32 nfeblk;
} nandflash_s3c6410_io_t;
#if 0
typedef struct nandflash_s3c6410_io
{
	uint32 nfconf;
	uint32 nfcmd;
	uint32 nfaddr;
	uint32 nfdata;
	uint32 nfstat;
	uint32 nfecc;
} nandflash_s3c6410_io_t;
#endif


#endif //_DEV_NANDFLASH_S3C6410_H_

