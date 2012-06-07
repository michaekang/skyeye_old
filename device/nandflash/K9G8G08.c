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
//#include "armdefs.h"
#include "skyeye_device.h"
#include "skyeye_nandflash.h"
#include "K9G8G08.h"
#include "nandflash_K9G8G08.h"
#include <stdlib.h>
static void
nandflash_s3c6410_fini (struct device_desc *dev)
{
	struct nandflash_device *nandflashdev=(struct nandflash_device*)dev->dev;
	struct nandflash_s3c6410_io *io = (struct nandflash_s3c2410_io *) dev->data;
	//nandflash_sb_uninstall(nandflashdev);
	nandflashdev->uinstall(nandflashdev);
	if (!dev->dev)
		free (dev->dev);
	if (!io)
		free (io);
}

static void
nandflash_s3c6410_reset (struct device_desc *dev)
{
	struct nandflash_device *nandflash_dev = (struct nandflash_device *) dev->dev;
	struct nandflash_s3c6410_io *io = (struct nandflash_s3c2410_io *) dev->data;

	io->nfconf=0x00001000;
	io->nfcont=0x0384;
	io->nfcmd=0;
	io->nfaddr=0;
	io->nfdata=0;

	io->nfeccd0=0;
	io->nfeccd1=0;
	io->nfeccd=0;
	io->nfstat=0;
	io->nfestat0=0;
	io->nfestat1=0;
	io->nfmecc0=0;
	io->nfmecc1=0;
	io->nfsblk=0;
	io->nfeblk=0;

	nandflash_dev->setCE(nandflash_dev,NF_HIGH);
	//nandflash_sb_setCE(NF_HIGH,nf);
}

static void
nandflash_s3c6410_update (struct device_desc *dev)
{
	struct nandflash_device *nandflash_dev = (struct nandflash_device *) dev->dev;
	struct nandflash_s3c6410_io *io = (struct nandflash_s3c2410_io *) dev->data;
	struct machine_config *mc = (struct machine_config *) dev->mach;

}

#if 0
int iomach(uint32 addr)
{
	if((addr>=NFCONF)&&(addr<=NFECC3))
		return 1;
	else
		return 0;
}
#endif
static int nandflash_s3c6410_read_word (struct device_desc *dev, uint32 addr, uint32 * data)
{
	struct nandflash_device *nandflash_dev = (struct nandflash_device *) dev->dev;
	struct nandflash_s3c6410_io *io = (struct nandflash_s3c2410_io *) dev->data;
	//struct nandflash_sb_status *nf = (struct nandflash_sb_status*)nandflash_dev->priv;
	int ret = ADDR_HIT;
	int ctrenable=io->nfcont&S3C6410_NFCONT_ENABLE;
	//if (iomach(addr))
	//	NANDFLASH_DBG("%s:addr %x\n", __FUNCTION__, addr);
	*data = 0;
	switch (addr) {
#if 0
	case NFCONF:
		*data = io->nfconf;
		break;
	case NFCMD:
		*data = io->nfcmd;
		break;
	case NFADDR:
		*data = io->nfaddr;
		break;
	case NFDATA:
		if(ctrenable)
			//*data=(uint32)nandflash_sb_readdata(nf);
			*data=(uint32)nandflash_dev->readdata(nandflash_dev);
		break;
	case NFSTAT:
		if(ctrenable)
		 	*data=nandflash_dev->readRB(nandflash_dev);
			//*data = nandflash_sb_readRB(nf);
		break;
	case NFECC1:
		*data = 0xFF;
		break;
	case NFECC2:
		*data = 0xFF;
		break;
	case NFECC3:
		*data = 0xFF;
		break;
	default:
		ret= ADDR_NOHIT;
		break;
	}
#endif

	case NFCONF:
		*data = io->nfconf;
		break;
	case NFCONT:
		*data = io->nfcont;
		break;
	case NFCMD:
		*data = io->nfcmd;
		break;
	case NFADDR:
		*data = io->nfaddr;
		break;
	case NFDATA:
		if(ctrenable)
			//*data=(uint32)nandflash_sb_readdata(nf);
			*data=(uint32)nandflash_dev->readdata(nandflash_dev);
		break;
	case NFECCD0:
		*data = 0xFF;
		break;
	case NFECCD1:
		*data = 0xFF;
		break;
	case NFECCD:
		*data = 0xFF;
		break;
	case NFSTAT:
		if(ctrenable)
		 	*data=nandflash_dev->readRB(nandflash_dev);
			//*data = nandflash_sb_readRB(nf);
		break;
	case NFESTAT0:
		if(ctrenable)
		 	*data=nandflash_dev->readRB(nandflash_dev);
			//*data = nandflash_sb_readRB(nf);
		break;
	case NFESTAT1:
		if(ctrenable)
		 	*data=nandflash_dev->readRB(nandflash_dev);
			//*data = nandflash_sb_readRB(nf);
		break;

	case NFMECC0:
		*data = 0xFF;
		break;

	case NFMECC1:
		*data = 0xFF;
		break;

	case NFSECC:
		*data = 0xFF;
		break;

	case NFSBLK:
		*data = 0xFF;
		break;
	case NFEBLK:
		*data = 0xFF;
		break;


	default:
		ret= ADDR_NOHIT;
		break;
	}
 	//if (iomach(addr))
	//  NANDFLASH_DBG("%s:readdata: %x\n", __FUNCTION__, *data);
	return ret;

}
static int nandflash_s3c6410_read_byte(struct device_desc *dev, uint32 addr, uint8 * data)
{
	return nandflash_s3c6410_read_word(dev,addr,(uint32*)data);
}


static int nandflash_s3c6410_write_word (struct device_desc *dev, uint32 addr, uint32 data)
{
	struct nandflash_device *nandflash_dev = (struct nandflash_device *) dev->dev;
	struct nandflash_s3c6410_io *io = (struct nandflash_s3c2410_io *) dev->data;
	//struct nandflash_sb_status *nf = (struct nandflash_sb_status*)nandflash_dev->priv;
	int ret = ADDR_HIT;
	int ctrenable=io->nfcont&S3C6410_NFCONT_ENABLE;
	//if (iomach(addr))
		
	switch (addr) {
	case NFCONF:
		io->nfconf=data;
		break;
	case NFCONT:
		io->nfcont = data;
		ctrenable=io->nfcont& S3C6410_NFCONT_ENABLE;
		if ((data&S3C6410_NFCONT_nFCE)&&(ctrenable))
		{
			nandflash_dev->setCE(nandflash_dev,NF_HIGH);
		}
		else
		{	
			nandflash_dev->setCE(nandflash_dev,NF_LOW);
		}

		break;
	case NFCMD:
		if(ctrenable)
		{
			nandflash_dev->sendcmd(nandflash_dev,(uint8)data);
		}
		break;
	case NFADDR:
		if(ctrenable)
			nandflash_dev->sendaddr(nandflash_dev,(uint8)data);
		break;
	case NFDATA:
		if(ctrenable)
			nandflash_dev->senddata(nandflash_dev,(uint8)data);
			//nandflash_sb_senddata((uint8)data,nf);
		break;
	case NFECCD0:
		break;
	case NFECCD1:
		break;
	case NFECCD:
		break;
	case NFSTAT:
		break;
	case NFESTAT0:
		break;
	case NFESTAT1:
		break;

	case NFMECC0:
		break;

	case NFMECC1:
		break;

	case NFSECC:
		break;

	case NFSBLK:
		break;
	case NFEBLK:
		break;


	default:
		ret = ADDR_NOHIT;
		break;
	}

	return ret;
}

static int
nandflash_s3c6410_write_byte (struct device_desc *dev, uint32 addr, uint8 data)
{
	return nandflash_s3c6410_write_word(dev,addr,(uint32)data);
}

static int
nandflash_s3c6410_setup (struct device_desc *dev)
{
	int i;
	struct nandflash_s3c6410_io *io;
	struct device_interrupt *intr = &dev->intr;
	struct nandflash_device *nandflashdev=(struct nandflash_device*)dev->dev;
	dev->fini = nandflash_s3c6410_fini;
	dev->reset = nandflash_s3c6410_reset;
	dev->update = nandflash_s3c6410_update;
	dev->read_word = nandflash_s3c6410_read_word;
	dev->write_word = nandflash_s3c6410_write_word;
	dev->read_byte= nandflash_s3c6410_read_byte;
	dev->write_byte= nandflash_s3c6410_write_byte;
	//nandflash_sb_setup(nandflashdev);
	if (nandflash_module_setup(nandflashdev,dev->name)==-1)
		return 1;
	nandflashdev->install(nandflashdev);
	io = (struct nandflash_s3c6410_io *)
		malloc (sizeof (struct nandflash_s3c6410_io));
	if (io == NULL)
		return 1;
	memset (io, 0, sizeof (struct nandflash_s3c6410_io));
	dev->data = (void *) io;
	nandflash_s3c6410_reset (dev);
	return 0;
}

void
nandflash_K9G8G08_init (struct device_module_set *mod_set)
{
	int i;
	register_device_module ("s3c6410x", mod_set, &nandflash_s3c6410_setup);
}

