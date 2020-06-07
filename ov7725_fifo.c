/*
 * ov7725 with fifo camera driver
 *
 * This file is subject to the terms and conditions of version 2 of
 * the GNU General Public License.  See the file COPYING in the main
 * directory of this archive for more details.
 *
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/vmalloc.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>

#define REG_GAIN      0x00
#define REG_BLUE      0x01
#define REG_RED       0x02
#define REG_GREEN     0x03
#define REG_BAVG      0x05
#define REG_GAVG      0x06
#define REG_RAVG      0x07
#define REG_AECH      0x08
#define REG_COM2      0x09
#define REG_PID       0x0A
#define REG_VER       0x0B
#define REG_COM3      0x0C
#define REG_COM4      0x0D
#define REG_COM5      0x0E
#define REG_COM6      0x0F
#define REG_AEC       0x10
#define REG_CLKRC     0x11
#define REG_COM7      0x12
#define REG_COM8      0x13
#define REG_COM9      0x14
#define REG_COM10     0x15
#define REG_REG16     0x16
#define REG_HSTART    0x17
#define REG_HSIZE     0x18
#define REG_VSTRT     0x19
#define REG_VSIZE     0x1A
#define REG_PSHFT     0x1B
#define REG_MIDH      0x1C
#define REG_MIDL      0x1D
#define REG_LAEC      0x1F
#define REG_COM11     0x20
#define REG_BDBase    0x22
#define REG_BDMStep   0x23
#define REG_AEW       0x24
#define REG_AEB       0x25
#define REG_VPT       0x26
#define REG_REG28     0x28
#define REG_HOutSize  0x29
#define REG_EXHCH     0x2A
#define REG_EXHCL     0x2B
#define REG_VOutSize  0x2C
#define REG_ADVFL     0x2D
#define REG_ADVFH     0x2E
#define REG_YAVE      0x2F
#define REG_LumHTh    0x30
#define REG_LumLTh    0x31
#define REG_HREF      0x32
#define REG_DM_LNL    0x33
#define REG_DM_LNH    0x34
#define REG_ADoff_B   0x35
#define REG_ADoff_R   0x36
#define REG_ADoff_Gb  0x37
#define REG_ADoff_Gr  0x38
#define REG_Off_B     0x39
#define REG_Off_R     0x3A
#define REG_Off_Gb    0x3B
#define REG_Off_Gr    0x3C
#define REG_COM12     0x3D
#define REG_COM13     0x3E
#define REG_COM14     0x3F
#define REG_COM16     0x41
#define REG_TGT_B     0x42
#define REG_TGT_R     0x43
#define REG_TGT_Gb    0x44
#define REG_TGT_Gr    0x45
#define REG_LC_CTR    0x46
#define REG_LC_XC     0x47
#define REG_LC_YC     0x48
#define REG_LC_COEF   0x49
#define REG_LC_RADI   0x4A
#define REG_LC_COEFB  0x4B 
#define REG_LC_COEFR  0x4C
#define REG_FixGain   0x4D
#define REG_AREF1     0x4F
#define REG_AREF6     0x54
#define REG_UFix      0x60
#define REG_VFix      0x61
#define REG_AWBb_blk  0x62
#define REG_AWB_Ctrl0 0x63
#define REG_DSP_Ctrl1 0x64
#define REG_DSP_Ctrl2 0x65
#define REG_DSP_Ctrl3 0x66
#define REG_DSP_Ctrl4 0x67
#define REG_AWB_bias  0x68
#define REG_AWBCtrl1  0x69
#define REG_AWBCtrl2  0x6A
#define REG_AWBCtrl3  0x6B
#define REG_AWBCtrl4  0x6C
#define REG_AWBCtrl5  0x6D
#define REG_AWBCtrl6  0x6E
#define REG_AWBCtrl7  0x6F
#define REG_AWBCtrl8  0x70
#define REG_AWBCtrl9  0x71
#define REG_AWBCtrl10 0x72
#define REG_AWBCtrl11 0x73
#define REG_AWBCtrl12 0x74
#define REG_AWBCtrl13 0x75
#define REG_AWBCtrl14 0x76
#define REG_AWBCtrl15 0x77
#define REG_AWBCtrl16 0x78
#define REG_AWBCtrl17 0x79
#define REG_AWBCtrl18 0x7A
#define REG_AWBCtrl19 0x7B
#define REG_AWBCtrl20 0x7C
#define REG_AWBCtrl21 0x7D 
#define REG_GAM1      0x7E
#define REG_GAM2      0x7F
#define REG_GAM3      0x80
#define REG_GAM4      0x81
#define REG_GAM5      0x82
#define REG_GAM6      0x83
#define REG_GAM7      0x84
#define REG_GAM8      0x85
#define REG_GAM9      0x86
#define REG_GAM10     0x87
#define REG_GAM11     0x88
#define REG_GAM12     0x89
#define REG_GAM13     0x8A
#define REG_GAM14     0x8B
#define REG_GAM15     0x8C
#define REG_SLOP      0x8D
#define REG_DNSTh     0x8E
#define REG_EDGE0     0x8F
#define REG_EDGE1     0x90
#define REG_DNSOff    0x91
#define REG_EDGE2     0x92
#define REG_EDGE3     0x93
#define REG_MTX1      0x94
#define REG_MTX2      0x95
#define REG_MTX3      0x96
#define REG_MTX4      0x97
#define REG_MTX5      0x98
#define REG_MTX6      0x99
#define REG_MTX_Ctrl  0x9A
#define REG_BRIGHT    0x9B
#define REG_CNST      0x9C
#define REG_UVADJ0    0x9E
#define REG_UVADJ1    0x9F
#define REG_SCAL0     0xA0
#define REG_SCAL1     0xA1
#define REG_SCAL2     0xA2
#define REG_SDE       0xA6
#define REG_USAT      0xA7
#define REG_VSAT      0xA8
#define REG_HUECOS    0xA9
#define REG_HUESIN    0xAA
#define REG_SIGN      0xAB
#define REG_DSPAuto   0xAC

#define VERSION(pid, ver) ((pid << 8) | (ver & 0xFF))
#define OV7725  0x7721
#define DSPAUTO     0xAC /* DSP auto function ON/OFF control */
#define PID         0x0A /* Product ID Number MSB */
#define VER         0x0B /* Product ID Number LSB */
#define MIDH        0x1C /* Manufacturer ID byte - high */
#define MIDL        0x1D /* Manufacturer ID byte - low  */

typedef struct Reg
{
	uint8_t Address;
	uint8_t Value;
}Reg_Info;


Reg_Info Sensor_Config[] =
{
	{REG_CLKRC,     0x00}, /*clock config*/
	{REG_COM7,      0x46}, /*QVGA RGB565 */
	{REG_HSTART,    0x3f},
	{REG_HSIZE,     0x50},
	{REG_VSTRT,     0x03},
	{REG_VSIZE,     0x78},
	{REG_HREF,      0x00},
	{REG_HOutSize,  0x50},
	{REG_VOutSize,  0x78},
	{REG_EXHCH,     0x00},
	

	/*DSP control*/
	{REG_TGT_B,     0x7f},
	{REG_FixGain,   0x09},
	{REG_AWB_Ctrl0, 0xe0},
	{REG_DSP_Ctrl1, 0xff},
	{REG_DSP_Ctrl2, 0x20},
	{REG_DSP_Ctrl3,	0x00},
	{REG_DSP_Ctrl4, 0x00},

	/*AGC AEC AWB*/
	{REG_COM8,		0xf0},
	{REG_COM4,		0x81},
	{REG_COM6,		0xc5},
	{REG_COM9,		0x21},
	{REG_BDBase,	0xFF},
	{REG_BDMStep,	0x01},
	{REG_AEW,		0x34},
	{REG_AEB,		0x3c},
	{REG_VPT,		0xa1},
	{REG_EXHCL,		0x00},
	{REG_AWBCtrl3,  0xaa},
	{REG_COM8,		0xff},
	{REG_AWBCtrl1,  0x5d},

	{REG_EDGE1,		0x0a},
	{REG_DNSOff,	0x01},
	{REG_EDGE2,		0x01},
	{REG_EDGE3,		0x01},

	{REG_MTX1,		0x5f},
	{REG_MTX2,		0x53},
	{REG_MTX3,		0x11},
	{REG_MTX4,		0x1a},
	{REG_MTX5,		0x3d},
	{REG_MTX6,		0x5a},
	{REG_MTX_Ctrl,  0x1e},

	{REG_BRIGHT,	0x00},
	{REG_CNST,		0x25},
	{REG_USAT,		0x65},
	{REG_VSAT,		0x65},
	{REG_UVADJ0,	0x81},
	//{REG_SDE,		  0x20},
	{REG_SDE,		  0x06},
	
    /*GAMMA config*/
	{REG_GAM1,		0x0c},
	{REG_GAM2,		0x16},
	{REG_GAM3,		0x2a},
	{REG_GAM4,		0x4e},
	{REG_GAM5,		0x61},
	{REG_GAM6,		0x6f},
	{REG_GAM7,		0x7b},
	{REG_GAM8,		0x86},
	{REG_GAM9,		0x8e},
	{REG_GAM10,		0x97},
	{REG_GAM11,		0xa4},
	{REG_GAM12,		0xaf},
	{REG_GAM13,		0xc5},
	{REG_GAM14,		0xd7},
	{REG_GAM15,		0xe8},
	{REG_SLOP,		0x20},

	{REG_HUECOS,	0x80},
	{REG_HUESIN,	0x80},
	{REG_DSPAuto,	0xff},
	{REG_DM_LNL,	0x00},
	{REG_BDBase,	0x99},
	{REG_BDMStep,	0x03},
	{REG_LC_RADI,	0x00},
	{REG_LC_COEF,	0x13},
	{REG_LC_XC,		0x08},
	{REG_LC_COEFB,  0x14},
	{REG_LC_COEFR,  0x17},
	{REG_LC_CTR,	0x05},
	
	{REG_COM3,		0xd0},

	{REG_COM5,		0xf5},
	//{REG_COM5,		0x31},
};

uint8_t OV7725_REG_NUM = sizeof(Sensor_Config)/sizeof(Sensor_Config[0]);

enum {
	ov7725,
};

enum {
	OV7725_SPEC_EFF,
	OV7725_LIGHT_MODE,
	OV7725_COLOR_SAT,
	OV7725_BRIGHT,
	OV7725_CONTRAST,
	OV7725_WINDOW,
};

struct cam_mode {		
	/*QVGA:sx + width <= 320 ,sy+height <= 240*/
	uint16_t cam_sx;
	uint16_t cam_sy;
	
	uint16_t cam_width;
	uint16_t cam_height;
	
	uint8_t light_mode;
	int8_t saturation;
	int8_t brightness;
	int8_t contrast;
	uint8_t effect;
};

struct ov7725_priv {
	struct i2c_client *i2c;
	u8				   vsync_counter;
	u16				  *vbuf;
	struct gpio_desc  *gpio_vsync_int;
	struct gpio_desc  *gpio_fifo_wrst;
	struct gpio_desc  *gpio_fifo_rrst;
	struct gpio_desc  *gpio_fifo_oe;
	struct gpio_desc  *gpio_fifo_wen;
	struct gpio_desc  *gpio_fifo_rclk;
	struct gpio_descs *gpio_fifo_data;
	struct cam_mode    cam_mode;
};

static struct ov7725_priv *priv;

static int ov7725_read_i2c(struct i2c_client *client, unsigned char reg,
		unsigned char *value)
{
	u8 data = reg;
	struct i2c_msg msg;
	int ret;

	/*
	 * Send out the register address...
	 */
	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = 1;
	msg.buf = &data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret < 0) {
		printk(KERN_ERR "Error %d on register write\n", ret);
		return ret;
	}
	/*
	 * ...then read back the result.
	 */
	msg.flags = client->flags | I2C_M_RD;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret >= 0) {
		*value = data;
		ret = 0;
	}
	return ret;
}

static int ov7725_write_i2c(struct i2c_client *client, unsigned char reg,
		unsigned char value)
{
	struct i2c_msg msg;
	unsigned char data[2] = { reg, value };
	int ret;

	msg.addr = client->addr;
	msg.flags = client->flags;
	msg.len = 2;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);
	if (ret > 0)
		ret = 0;
	if (reg == REG_COM7 && (value & 0x80))
		msleep(5);  /* Wait for reset to run */
	return ret;
}

static int ov7725_read(struct ov7725_priv *priv, unsigned char reg,
		unsigned char *value)
{
	struct i2c_client *client = priv->i2c;
	return ov7725_read_i2c(client, reg, value);
}

static int ov7725_write(struct ov7725_priv *priv, unsigned char reg,
		unsigned char value)
{
	struct i2c_client *client = priv->i2c;
	return ov7725_write_i2c(client, reg, value);
}

int ov7725_init(struct ov7725_priv *priv)
{
	uint16_t i = 0;
	int		    ret;
	unsigned char pid, ver, midh, midl;
	uint8_t Sensor_IDCode = 0;	
	const char         *devname;
	//DEBUG("ov7725 Register Config Start......");
	
	if(ov7725_write(priv, 0x12, 0x80))
	{
		//DEBUG("sccb write data error");		
		return -EIO ;
	}	

	/* Check and show product ID and manufacturer ID. */
	ret = ov7725_read(priv, PID, &pid);
	if (ret < 0)
		return ret;
	ret = ov7725_read(priv, VER, &ver);
	if (ret < 0)
		return ret;

	switch (VERSION((int)pid, (int)ver)) {
	case OV7725:
		devname     = "ov7725";
		break;
	default:
		dev_err(&priv->i2c->dev,
			"Product ID error %x:%x\n", pid, ver);
		ret = -ENODEV;
		return ret;
	}
	ret = ov7725_read(priv, MIDH, &midh);
	if (ret < 0)
		return ret;
	ret = ov7725_read(priv, MIDL, &midl);
	if (ret < 0)
		return ret;

	dev_info(&priv->i2c->dev,
		 "%s Product ID %0x:%0x Manufacturer ID %x:%x\n",
		 devname, pid, ver, midh, midl);

	
	for( i = 0 ; i < OV7725_REG_NUM ; i++ )
	{
		if(ov7725_write(priv, Sensor_Config[i].Address, Sensor_Config[i].Value))
		{                
			//DEBUG("write reg faild", Sensor_Config[i].Address);
			return -EIO;
		}
	}

	//DEBUG("ov7725 Register Config Success");
	
	return 0;
}

static int ov7725_set_spec_eff(struct ov7725_priv *priv, u8 readin)
{
	
	switch(readin) {
	case 0:
		ov7725_write(priv, 0xa6, 0x06);
		ov7725_write(priv, 0x60, 0x80);
		ov7725_write(priv, 0x61, 0x80);
	break;
	
	case 1:
		ov7725_write(priv, 0xa6, 0x26);
		ov7725_write(priv, 0x60, 0x80);
		ov7725_write(priv, 0x61, 0x80);
	break;	
	
	case 2:
		ov7725_write(priv, 0xa6, 0x1e);
		ov7725_write(priv, 0x60, 0xa0);
		ov7725_write(priv, 0x61, 0x40);	
	break;	
	
	case 3:
		ov7725_write(priv, 0xa6, 0x1e);
		ov7725_write(priv, 0x60, 0x40);
		ov7725_write(priv, 0x61, 0xa0);	
	break;	
	
	case 4:
		ov7725_write(priv, 0xa6, 0x1e);
		ov7725_write(priv, 0x60, 0x80);
		ov7725_write(priv, 0x61, 0xc0);		
	break;	
	
	case 5:
		ov7725_write(priv, 0xa6, 0x1e);
		ov7725_write(priv, 0x60, 0x60);
		ov7725_write(priv, 0x61, 0x60);		
	break;	
	
	case 6:
		ov7725_write(priv, 0xa6, 0x46);
	break;	
			
	default:
		return -EINVAL;
		break;
	}
	
	return 0;
}

static int ov7725_set_light_mode(struct ov7725_priv *priv, u8 readin)
{
	switch(readin) {
	case 0:
		ov7725_write(priv, 0x13, 0xff);
		ov7725_write(priv, 0x0e, 0x65);
		ov7725_write(priv, 0x2d, 0x00);
		ov7725_write(priv, 0x2e, 0x00);
		break;
	case 1:
		ov7725_write(priv, 0x13, 0xfd);
		ov7725_write(priv, 0x01, 0x5a);
		ov7725_write(priv, 0x02, 0x5c);
		ov7725_write(priv, 0x0e, 0x65);
		ov7725_write(priv, 0x2d, 0x00);
		ov7725_write(priv, 0x2e, 0x00);
		break;	
	case 2:
		ov7725_write(priv, 0x13, 0xfd);
		ov7725_write(priv, 0x01, 0x58);
		ov7725_write(priv, 0x02, 0x60);
		ov7725_write(priv, 0x0e, 0x65);
		ov7725_write(priv, 0x2d, 0x00);
		ov7725_write(priv, 0x2e, 0x00);
		break;	
	case 3:
		ov7725_write(priv, 0x13, 0xfd);
		ov7725_write(priv, 0x01, 0x84);
		ov7725_write(priv, 0x02, 0x4c);
		ov7725_write(priv, 0x0e, 0x65);
		ov7725_write(priv, 0x2d, 0x00);
		ov7725_write(priv, 0x2e, 0x00);
		break;	
	case 4:
		ov7725_write(priv, 0x13, 0xfd);
		ov7725_write(priv, 0x01, 0x96);
		ov7725_write(priv, 0x02, 0x40);
		ov7725_write(priv, 0x0e, 0x65);
		ov7725_write(priv, 0x2d, 0x00);
		ov7725_write(priv, 0x2e, 0x00);
		break;	
	
	case 5:
		ov7725_write(priv, 0x13, 0xff);
		ov7725_write(priv, 0x0e, 0xe5);
		break;	
	
	default:
		return -EINVAL; 
		break;
	}

	return 0;
}

static int ov7725_set_color_sat(struct ov7725_priv *priv, int8_t readin)
{
 	if(readin >=-4 && readin<=4) {	
		ov7725_write(priv, REG_USAT, (readin+4)<<4); 
		ov7725_write(priv, REG_VSAT, (readin+4)<<4);
	}
	else {
		return -EINVAL;
	}
	return 0;
}

static int ov7725_set_bright(struct ov7725_priv *priv, int8_t readin)
{
	uint8_t BRIGHT_Value,SIGN_Value;	
	
	switch(readin) {
	case 4:
			BRIGHT_Value = 0x48;
			SIGN_Value = 0x06;
		break;
	
	case 3:
			BRIGHT_Value = 0x38;
			SIGN_Value = 0x06;		
	break;	
	
	case 2:
			BRIGHT_Value = 0x28;
			SIGN_Value = 0x06;			
	break;	
	
	case 1:
			BRIGHT_Value = 0x18;
			SIGN_Value = 0x06;			
	break;	
	
	case 0:
			BRIGHT_Value = 0x08;
			SIGN_Value = 0x06;			
	break;	
	
	case -1:
			BRIGHT_Value = 0x08;
			SIGN_Value = 0x0e;		
	break;	
	
	case -2:
			BRIGHT_Value = 0x18;
			SIGN_Value = 0x0e;		
	break;	
	
	case -3:
			BRIGHT_Value = 0x28;
			SIGN_Value = 0x0e;		
	break;	
	
	case -4:
			BRIGHT_Value = 0x38;
			SIGN_Value = 0x0e;		
	break;	
	
	default:
		return -EINVAL;
		break;
	}

	ov7725_write(priv, REG_BRIGHT, BRIGHT_Value);
	ov7725_write(priv, REG_SIGN, SIGN_Value);

	return 0;
}

static int ov7725_set_contrast(struct ov7725_priv *priv, int8_t readin)
{
	if(readin >= -4 && readin <=4) {
		ov7725_write(priv, REG_CNST, (0x30-(4-readin)*4));
	}
	else {
		return -EINVAL;
	}
	return 0;
}

int ov7725_set_window(struct ov7725_priv *priv, uint16_t sx,uint16_t sy,uint16_t width,uint16_t height)
{
	unsigned char reg_raw,cal_temp;

	ov7725_write(priv, REG_COM7,0x46); 
	ov7725_read(priv, REG_HSTART, &reg_raw);

	cal_temp = (reg_raw + (sx>>2));

	ov7725_write(priv, REG_HSTART,cal_temp ); 
	ov7725_write(priv, REG_HSIZE,width>>2);
	ov7725_read(priv, REG_VSTRT, &reg_raw);	

	cal_temp = (reg_raw + (sy>>1));	
	
	ov7725_write(priv, REG_VSTRT,cal_temp);
	ov7725_write(priv, REG_VSIZE,height>>1);
	ov7725_read(priv, REG_HREF, &reg_raw);	

	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2)|((sx&0x03)<<4)|((sy&0x01)<<6));	
	
	ov7725_write(priv, REG_HREF,cal_temp);
	ov7725_write(priv, REG_HOutSize,width>>2);
	ov7725_write(priv, REG_VOutSize,height>>1);
	ov7725_read(priv, REG_EXHCH, &reg_raw);

	cal_temp = (reg_raw |(width&0x03)|((height&0x01)<<2));	

	ov7725_write(priv, REG_EXHCH,cal_temp);

	return 0;	
}

/*------------------------------------------------------------------------------------------*/

static int ov7725_file_open(struct inode *inode, struct file *file)
{
	if(ov7725_init(priv))
		return -EIO;

	ov7725_set_spec_eff(priv, priv->cam_mode.effect);
	ov7725_set_light_mode(priv, priv->cam_mode.light_mode);
	ov7725_set_color_sat(priv, priv->cam_mode.saturation);
	ov7725_set_bright(priv, priv->cam_mode.brightness);
	ov7725_set_contrast(priv, priv->cam_mode.contrast);
	ov7725_set_window(priv, priv->cam_mode.cam_sx, priv->cam_mode.cam_sy,
								priv->cam_mode.cam_width, priv->cam_mode.cam_height);	

	gpiod_set_value(priv->gpio_fifo_oe, 0);
	priv->vsync_counter = 0;

	return 0;
}

static ssize_t
ov7725_file_read(struct file *file, char __user *user_buf, size_t count, loff_t *ptr)
{
	int vmem_size;
	unsigned long flags;
	int i,j,k;

	vmem_size = priv->cam_mode.cam_height * priv->cam_mode.cam_width * 2;
	if (*ptr >= vmem_size){
		vfree(priv->vbuf);
		priv->vsync_counter = 0;
		return 0;
	}

	if (*ptr + count > vmem_size) count = vmem_size - *ptr;

	if(!priv->vsync_counter) {
		priv->vbuf = vzalloc(vmem_size);
		if (!priv->vbuf)
			return -1;

		local_irq_save(flags);

		while(1) {
			while(gpiod_get_value(priv->gpio_vsync_int)) {
				if(!gpiod_get_value(priv->gpio_vsync_int)) {
					if(priv->vsync_counter < 1) {
						priv->vsync_counter = 1;
						gpiod_set_value(priv->gpio_fifo_wrst, 0);
						gpiod_set_value(priv->gpio_fifo_wen, 1);
						gpiod_set_value(priv->gpio_fifo_wrst, 1);
					} else if(priv->vsync_counter >= 1) {
						gpiod_set_value(priv->gpio_fifo_wen, 0);
						goto read_frame;
					}
				}
			}
		}
read_frame:
		local_irq_restore(flags);

		DECLARE_BITMAP(values, priv->gpio_fifo_data->ndescs);

		gpiod_set_value(priv->gpio_fifo_rrst, 0);
		gpiod_set_value(priv->gpio_fifo_rclk, 0);
		gpiod_set_value(priv->gpio_fifo_rclk, 1);
		gpiod_set_value(priv->gpio_fifo_rrst, 1);
		gpiod_set_value(priv->gpio_fifo_rclk, 0);
		gpiod_set_value(priv->gpio_fifo_rclk, 1);
	
		for(i = 0; i < priv->cam_mode.cam_height; i++) {
			for(j = 0; j < priv->cam_mode.cam_width; j++) {
				gpiod_set_value(priv->gpio_fifo_rclk, 0);
				gpiod_get_array_value(priv->gpio_fifo_data->ndescs,
								priv->gpio_fifo_data->desc, priv->gpio_fifo_data->info, values);
				priv->vbuf[k] = ((u16)values[0]) << 8;
				gpiod_set_value(priv->gpio_fifo_rclk, 1);
				gpiod_set_value(priv->gpio_fifo_rclk, 0);
				gpiod_get_array_value(priv->gpio_fifo_data->ndescs,
								priv->gpio_fifo_data->desc, priv->gpio_fifo_data->info, values);
				priv->vbuf[k] |= ((u16)values[0]) & 0xff;
				gpiod_set_value(priv->gpio_fifo_rclk, 1);
				k++;
			}
		}
	}

	if(copy_to_user(user_buf, (void *)((unsigned long)priv->vbuf + (unsigned long)*ptr), count))
		return -EFAULT;

	*ptr += count;
	return count;
}

static const struct file_operations ov7725_fops = {
	.open		= ov7725_file_open,
	.read		= ov7725_file_read,
	.llseek		= no_llseek,
};

static struct miscdevice ov7725_miscdev = {
	MISC_DYNAMIC_MINOR,
	"ov7725",
	&ov7725_fops
};

static int ov7725_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	client->flags |= I2C_CLIENT_SCCB;

	priv = devm_kzalloc(&client->dev, sizeof(struct ov7725_priv), GFP_KERNEL);
	priv->i2c = client;

	priv->vsync_counter = 0;
	priv->cam_mode.cam_sx = 0;
	priv->cam_mode.cam_sy = 0;
	priv->cam_mode.cam_width = 320;
	priv->cam_mode.cam_height = 240;
	priv->cam_mode.light_mode = 0;
	priv->cam_mode.saturation = 0;
	priv->cam_mode.brightness = 0;
	priv->cam_mode.contrast   = 0;
	priv->cam_mode.effect     = 0;

	priv->gpio_vsync_int = gpiod_get(&client->dev, "vsync_int", GPIOD_IN);
	priv->gpio_fifo_wrst = gpiod_get(&client->dev, "fifo_wrst", GPIOD_OUT_LOW);
	priv->gpio_fifo_rrst = gpiod_get(&client->dev, "fifo_rrst", GPIOD_OUT_LOW);
	priv->gpio_fifo_oe = gpiod_get(&client->dev, "fifo_oe", GPIOD_OUT_HIGH);
	priv->gpio_fifo_wen = gpiod_get(&client->dev, "fifo_wen", GPIOD_OUT_LOW);
	priv->gpio_fifo_rclk = gpiod_get(&client->dev, "fifo_rclk", GPIOD_OUT_LOW);
	priv->gpio_fifo_data = gpiod_get_array(&client->dev, "fifo_data", GPIOD_IN);

	i2c_set_clientdata(client, priv);

	if(misc_register(&ov7725_miscdev) < 0) {
		return -1;
	}

	return 0;
}

static int ov7725_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id ov7725_dt_ids[] = {
	{ .compatible = "atk,ov7725fifo", },
	{}
};
MODULE_DEVICE_TABLE(of, ov7725_dt_ids);

static const struct i2c_device_id ov7725_id[] = {
	{ "ov7725", ov7725 },
	{}
};
MODULE_DEVICE_TABLE(i2c, ov7725_id);

static struct i2c_driver ov7725_driver = {
	.driver = {
		.name = "ov7725",
		.of_match_table = of_match_ptr(ov7725_dt_ids),
	},
	.probe = ov7725_probe,
	.remove = ov7725_remove,
	.id_table = ov7725_id,
};
module_i2c_driver(ov7725_driver);

MODULE_AUTHOR("hctang");
MODULE_DESCRIPTION("ov7725 driver");
MODULE_LICENSE("GPL v2");
