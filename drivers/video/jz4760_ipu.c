/********************** BEGIN LICENSE BLOCK ************************************
 *
 * INGENIC CONFIDENTIAL--NOT FOR DISTRIBUTION IN SOURCE CODE FORM
 * Copyright (c) Ingenic Semiconductor Co. Ltd 2005. All rights reserved.
 *
 * This file, and the files included with this file, is distributed and made
 * available on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER
 * EXPRESS OR IMPLIED, AND REALNETWORKS HEREBY DISCLAIMS ALL SUCH WARRANTIES,
 * INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT.
 *
 * http://www.ingenic.cn
 *
 ********************** END LICENSE BLOCK **************************************
 *
 *  Author:   <jbyu@ingenic.cn>
 *
 *  Create:   2008-10-15, by jbyu
 *
 *******************************************************************************
 */


#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/irq.h>
#include <linux/signal.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/delay.h>


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/proc_fs.h>

#include <asm/irq.h>
#include <asm/pgtable.h>
#include <asm/system.h>
#include <asm/uaccess.h>
#include <asm/processor.h>
#include <asm/jzsoc.h>



#ifndef printf
#define printf printk
#endif


//#include "jz_ipu.h"
#if defined(CONFIG_FB_JZ4750_LCD)
#include "jz4750_lcd.h"
#elif defined(CONFIG_FB_JZ4760_LCD)
#include "jz4760_lcd.h"
#endif
//#include "jz4760_android_ipu.h"


#if defined CONFIG_JZ4750
#include <asm/mach-jz4750/jz4750.h>
#include <asm/mach-jz4750/regs.h>
#include <asm/mach-jz4750/ops.h>
#elif defined CONFIG_SOC_JZ4760
#include <asm/mach-jz4760/jz4760lcdc.h>
#include "jz4760_ipu.h"
#include <asm/mach-jz4760/jz4760ipu.h>
#elif defined CONFIG_SOC_JZ4760B
#include <asm/mach-jz4760b/jz4760blcdc.h>
#include "jz4760_ipu.h"
#include <asm/mach-jz4760b/jz4760bipu.h>

#elif defined CONFIG_SOC_JZ4770
#include <mach/jz4770.h>
#endif

#include <asm/uaccess.h>




//#undef printf
//#define IPU_LUT_LEN							(32)
#define FB_LCD_WIDTH						5000//lcd_get_width()
#define FB_LCD_HEIGHT						3000//lcd_get_height()
//#define IPU_OPEN							(1 << 0)
//#define IPU_INIT							(1 << 1)
//#define IPU_SET_STATE_BIT					(25)		// 7 bit
//#define IPU_SET_STATE_MASK					(0x7f << IPU_SET_STATE_BIT)
//#define SET_BIT(x)							(1 << (IPU_SET_STATE_BIT+x))
//#define IPU_CHANGE_BUF						(SET_BIT(0))
//#define IPU_SET_CTRL						(SET_BIT(1))
//#define IPU_SET_DAT_FMT						(SET_BIT(2))
//#define IPU_SET_CSC							(SET_BIT(3))
#define IPU_RATIO_MUL						(100000)
// 不定义
// #define IPU_FUNC_EX

#define IPU_INTC_DISABLE()					OUTREG32(INTC_ICMSR(0), 1 << IRQ_IPU);
#define IPU_INTC_ENABLE()					OUTREG32(INTC_ICMCR(0), 1 << IRQ_IPU);


#ifndef PHYS
#define PHYS(n)  virt_to_phys(n)
#endif

#if defined(CONFIG_JZ_CIM)
extern unsigned int cim_read_dma(); 
#endif


img_param_t g_img_native_data = 
{
};

//struct ipu_driver_priv *ipu_priv = &g_ipu_native_data;
img_param_t *ipu_def_img = &g_img_native_data;

struct YuvStride def_stride = 
{
};

extern unsigned char * lcd_frame0;
extern unsigned char * lcd_frame1;


static int ipu_inited = 0, ipu_size_cfg = 0;
static int ipu_rtable_len;

unsigned int fb_w, fb_h;
unsigned int rsize_w2 = 0, rsize_h2 = 0;

static img_param_t *img;
//static struct Ration2m *ipu_ratio_table;
static struct Ration2m ipu_ratio_table[IPU_LUT_LEN*IPU_LUT_LEN];
static rsz_lut h_lut[IPU_LUT_LEN];
static rsz_lut v_lut[IPU_LUT_LEN];
static int ipu_change_buf = 0;
static unsigned int *py_stride, *pu_stride, *pv_stride, *pout_stride;


/* jz4760's ipu HRSZ_COEF_LUT*/
static int hoft_table[IPU_LUT_LEN+1]; 
static int hcoef_table[IPU_LUT_LEN+1];
static int hcoef_real_heiht;
/* jz4760's ipu VRSZ_COEF_LUT*/
static int voft_table[IPU_LUT_LEN+1];
static int vcoef_table[IPU_LUT_LEN+1];
static int vcoef_real_heiht;



//cchen 09-11-16
static int OutStride =480*4 ;

void set_outstride(int stride)
{
       OutStride = stride ;
}
//cchen

/*----------------------------------------------------------------------------------*/

extern int get_lcd_width();
extern int get_lcd_hight();
unsigned int lcd_get_width()
{
	return get_lcd_width();
}

unsigned int lcd_get_height()
{
	return get_lcd_hight();
}


static void get_fbaddr_info()  //cchen 09-12-4
{
	fb_w = FB_LCD_WIDTH;
	fb_h = FB_LCD_HEIGHT;
}
/*******************************************************
 * 配置寄存器
 *******************************************************/
static int jz47_set_ipu_csc_cfg(int outW, int outH, int Wdiff, int Hdiff)
{
	unsigned int in_fmt = img->ipu_d_fmt & IN_FMT_MASK;
	unsigned int out_fmt = img->ipu_d_fmt & OUT_FMT_MASK;

	switch (in_fmt)
	{
		case IN_FMT_YUV420:
			Hdiff = (Hdiff + 1) & ~1;
			Wdiff = (Wdiff + 1) & ~1;
			break;

		case IN_FMT_YUV422:
		printf("[%s][%d]\n",__FUNCTION__,__LINE__);
			Wdiff = (Wdiff + 1) & ~1;
			break;

		case IN_FMT_YUV444:
		case IN_FMT_YUV411:
			break;
		default:
			printk("Error: Input data format isn't support\n");
			return (-1);
	}

	switch(out_fmt)
	{
		case OUT_FMT_RGB888:
			outW = outW << 2;
			break;
		case OUT_FMT_RGB555:
		printf("[%s][%d]\n",__FUNCTION__,__LINE__);
			outW = outW << 1;
			break;
		case OUT_FMT_RGB565:
			outW = outW << 1;
			break;
	}
	//printf("\n=====outW=[%d]=outH=[%d]===\n",outW,outH);
	//printf("=====img->in_width=[%d]====img->in_height=[%d]=====\n",img->in_width,img->in_height);
	// 配置输入输出几何图形寄存器
	OUTREG32(IPU_IN_FM_GS, IN_FM_W(img->in_width) | IN_FM_H((img->in_height - Hdiff) & ~0x1));
	//printf("=====IPU_IN_FM_GS=[0x%x]=====\n",REG32(IPU_IN_FM_GS));
	OUTREG32(IPU_OUT_GS, OUT_FM_W(outW) | OUT_FM_H(outH));
	//printf("[%s][%d]\n",__FUNCTION__,__LINE__);

	// Set out stirde
#if 1
	if (img->stride != 0)
	{
		printf("[%s][%d]\n",__FUNCTION__,__LINE__);

		OUTREG32(IPU_OUT_STRIDE, img->stride->out);
		//OUTREG32(A_IPU_OUT_STRIDE, fb_w * img->in_bpp >> 3);
	}
	else
	{
		//printf("[%s][%d]\n",__FUNCTION__,__LINE__);

		//OUTREG32(A_IPU_OUT_STRIDE, outW);
		OUTREG32(IPU_OUT_STRIDE, fb_w * img->in_bpp >> 3);
	}
	//printf("stride->out=0x%x\n",REG32(IPU_OUT_STRIDE));
#else
	/***wyang***/
	//OUTREG32(IPU_OUT_STRIDE, 800*4);
#endif
	// 设置数据格式寄存器
	OUTREG32(IPU_D_FMT, img->ipu_d_fmt);

	// set CSC parameter
	if ((in_fmt != IN_FMT_YUV444) && (out_fmt != OUT_FMT_YUV422))
	{
		if (in_fmt != IN_FMT_PKG_RGB565){

		printk("set csc !\n");
		img->ipu_ctrl |= CSC_EN;

		OUTREG32(IPU_CSC_C0_COEF, YUV_CSC_C0);
		OUTREG32(IPU_CSC_C1_COEF, YUV_CSC_C1);
		OUTREG32(IPU_CSC_C2_COEF, YUV_CSC_C2);
		OUTREG32(IPU_CSC_C3_COEF, YUV_CSC_C3);
		OUTREG32(IPU_CSC_C4_COEF, YUV_CSC_C4);
		OUTREG32(IPU_CSC_OFSET_PARA, CHROM(YUV_CSC_CHROM) | LUMA(YUV_CSC_LUMA));
		}
	}

	return (0);
}

/************************************************
 *  初始化缩放比率列表
 ************************************************/
static int init_ipu_ratio_table(void)
{
#if 0
	unsigned int i, j, cnt;
	unsigned int r, min;
	int diff;

	ipu_ratio_table=(void *)alloc( (IPU_LUT_LEN) * (IPU_LUT_LEN)*sizeof(struct Ration2m) );

	//ipu_ratio_table= (struct Ration2m *)img->ratio_table;
	for (i = 1; i <= (IPU_LUT_LEN); i++)
	{
		for (j = 1; j <= (IPU_LUT_LEN); j++)
		{
			ipu_ratio_table[(i - 1) * IPU_LUT_LEN + j - 1].ratio = i * IPU_RATIO_MUL / j;
			ipu_ratio_table[(i - 1) * IPU_LUT_LEN + j - 1].n = i;
			ipu_ratio_table[(i - 1) * IPU_LUT_LEN + j - 1].m = j;
		}
	}

	// Eliminate the ratio greater than 1:2
	#if 1
	for (i = 0; i < (IPU_LUT_LEN) * (IPU_LUT_LEN); i++)
	{
		if (ipu_ratio_table[i].ratio < IPU_RATIO_MUL / 2)
		{
	    	ipu_ratio_table[i].n = ipu_ratio_table[i].m = -1;
	    }
    }
	#endif
	// eliminate the same ratio
	for (i = 0; i < (IPU_LUT_LEN) * (IPU_LUT_LEN); i++)
	{
		for (j = i + 1; j < (IPU_LUT_LEN) * (IPU_LUT_LEN); j++)
		{
			diff = ipu_ratio_table[i].ratio - ipu_ratio_table[j].ratio;
			if (diff > -100 && diff < 100)
			{
				ipu_ratio_table[j].n = -1;
				ipu_ratio_table[j].m = -1;
			}
		}
	}

	// reorder ipu_ratio_table
	cnt = 0;
	for (i = 0; i < (IPU_LUT_LEN) * (IPU_LUT_LEN); i++)
	{
		if (ipu_ratio_table[i].n != -1)
		{
			if (cnt != i)
			{
				ipu_ratio_table[cnt] = ipu_ratio_table[i];
			}
			cnt++;
		}
	}
	ipu_rtable_len = cnt;

	return (0);
#else
	unsigned int i, j, cnt;
	int diff;

	//ipu_ratio_table=(void *)alloc( (IPU_LUT_LEN) * (IPU_LUT_LEN)*sizeof(struct Ration2m) );

	//ipu_ratio_table= (struct Ration2m *)img->ratio_table;
	
	memset(ipu_ratio_table,0,IPU_LUT_LEN*IPU_LUT_LEN*sizeof(struct Ration2m));
	
	for (i = 1; i <= (IPU_LUT_LEN); i++)
	{
		for (j = 1; j <= (IPU_LUT_LEN); j++)
		{
			ipu_ratio_table[(i - 1) * IPU_LUT_LEN + j - 1].ratio = i * IPU_RATIO_MUL / j;
			ipu_ratio_table[(i - 1) * IPU_LUT_LEN + j - 1].n = i;
			ipu_ratio_table[(i - 1) * IPU_LUT_LEN + j - 1].m = j;
		}
	}

	// Eliminate the ratio greater than 1:2
	#if 1
	for (i = 0; i < (IPU_LUT_LEN) * (IPU_LUT_LEN); i++)
	{
		if (ipu_ratio_table[i].ratio < IPU_RATIO_MUL / 2)
		{
	    	ipu_ratio_table[i].n = ipu_ratio_table[i].m = -1;
	    }
    }
	#endif
	// eliminate the same ratio
	for (i = 0; i < (IPU_LUT_LEN) * (IPU_LUT_LEN); i++)
	{
		for (j = i + 1; j < (IPU_LUT_LEN) * (IPU_LUT_LEN); j++)
		{
			diff = ipu_ratio_table[i].ratio - ipu_ratio_table[j].ratio;
			if (diff > -100 && diff < 100)
			{
				ipu_ratio_table[j].n = -1;
				ipu_ratio_table[j].m = -1;
			}
		}
	}

	// reorder ipu_ratio_table
	cnt = 0;
	for (i = 0; i < (IPU_LUT_LEN) * (IPU_LUT_LEN); i++)
	{
		if (ipu_ratio_table[i].n != -1)
		{
			if (cnt != i)
			{
				ipu_ratio_table[cnt] = ipu_ratio_table[i];
			}
			cnt++;
		}
	}
	ipu_rtable_len = cnt;

	return (0);

#endif
}
/************************************************
 *	查找IPU缩放比率表
 ************************************************/
static int find_ipu_ratio_factor(unsigned int ratio, unsigned int up)
{
	unsigned int i, sel;
	unsigned int diff, min = IPU_RATIO_MUL;

	sel = ipu_rtable_len;

	for (i = 0; i < ipu_rtable_len; i++)
	{
		if ((up == 0) && ((ipu_ratio_table[i].n & 1) != 0))
		{
			continue;
		}

		if (ratio > ipu_ratio_table[i].ratio)
		{
			diff = ratio - ipu_ratio_table[i].ratio;
		}
		else
		{
			diff = ipu_ratio_table[i].ratio - ratio;
		}

		if (diff < min || i == 0)
		{
			min = diff;
			sel = i;
		}
	}

	return (sel);
}
#ifndef IPU_FUNC_EX
static int resize_lut_cal(int srcN, int dstM, int upScale, rsz_lut lut[]);
static int resize_out_cal(int insize, int outsize, int srcN, int dstM, int upScale, int *diff);
static void caculate_h_lut(rsz_lut *h_lut0, int * hoft_table, int* hcoef_table,  int H_MAX_LUT);
static void caculate_v_lut(rsz_lut *h_lut0, int * hoft_table, int* hcoef_table,  int H_MAX_LUT);
#else
static int (*resize_lut_cal)(int srcN, int dstM, int upScale, rsz_lut lut[]);
static int (*resize_out_cal)(int insize, int outsize, int srcN, int dstM, int upScale, int *diff);

void set_ipu_cal_func(void *lut_cal, void *out_cal)
{
	resize_lut_cal = lut_cal;
	resize_out_cal = out_cal;
}
#endif
/****************************************************
 *
 ****************************************************/
static int jz47_set_ipu_resize(int *outWp, int *outHp,int *Wdiff, int *Hdiff)
{
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
	int W = 0, H = 0, Hsel = 0, Wsel = 0;
	int srcN, dstM, width_up, height_up;
	int Height_lut_max, Width_lut_max;
	int i;
	// 条件不满足resize = img->out_width
	rsize_w2 = ((img->out_width + img->out_x) > fb_w) ? (fb_w  - img->out_x) : img->out_width;
	rsize_h2 = ((img->out_height + img->out_y) > fb_h) ? (fb_h - img->out_y) : img->out_height;
	// 输出尺寸不得大于输入两倍
	rsize_w2 = (rsize_w2 > 2 * img->in_width) ? (2 * img->in_width) : rsize_w2;
	rsize_h2 = (rsize_h2 > 2 * img->in_height) ? (2 * img->in_height) : rsize_h2;
	*Wdiff = *Hdiff = 0;

	printk("img-> out_widht = %d, height = %d in_width = %d in_height = %d rsize_w2 = %d rsize_h2 = %d\n",img->out_width,img->out_height,img->in_width,img->in_height,rsize_w2,rsize_h2);

	// 先非使能放大
	img->ipu_ctrl &= ~(VRSZ_EN | HRSZ_EN);
	// 如果不放大
	if ((img->in_width == rsize_w2) && (img->in_height == rsize_h2))
	{
		img->out_height = *outHp = rsize_h2;
		img->out_width = *outWp = rsize_w2;

		return (0);
	}
	else
	{
		//如果需要水平放大
		if (img->in_width != rsize_w2)
			img->ipu_ctrl |= HRSZ_EN;
		//如果需要垂直放大
		if (img->in_height != rsize_h2)
			img->ipu_ctrl |= VRSZ_EN;
	}
//printf("%s,%d\n",__FUNCTION__,__LINE__);

	// 初始化放缩比率表
 	//ipu_ratio_table = img->ratio_table;
	init_ipu_ratio_table();
//printf("%s,%d\n",__FUNCTION__,__LINE__);
	// 将缩放信息存储到局部变量中
	width_up = (rsize_w2 >= img->in_width);
	height_up = (rsize_h2 >= img->in_height);

	// get the resize factor
	if (W != rsize_w2)
	{
		Wsel = find_ipu_ratio_factor(img->in_width * IPU_RATIO_MUL /rsize_w2, 1);
		// printf ("horizontal resize: sel = %d, n=%d, m=%d\n", Wsel, ipu_ratio_table[Wsel].n,ipu_ratio_table[Wsel].m);
		W = rsize_w2;
	}
	if (H != rsize_h2)
	{
		Hsel = find_ipu_ratio_factor(img->in_height * IPU_RATIO_MUL /rsize_h2, height_up);
		// printf ("Vertical resize: sel = %d, n=%d, m=%d\n", Hsel, ipu_ratio_table[Hsel].n, ipu_ratio_table[Hsel].m);
		H = rsize_h2;
	}

	// 如果缩放---*.m
	Width_lut_max  = width_up  ? ipu_ratio_table[Wsel].m : ipu_ratio_table[Wsel].n;
	Height_lut_max = height_up ? ipu_ratio_table[Hsel].m : ipu_ratio_table[Hsel].n;
	//printf("ipu_ratio_table[Wsel].m=0x%d : ipu_ratio_table[Wsel].n=0x%d ,ipu_ratio_table[Hsel].m=0x%d  : ipu_ratio_table[Hsel].n=0x%d \n",ipu_ratio_table[Wsel].m ,ipu_ratio_table[Wsel].n,ipu_ratio_table[Hsel].m , ipu_ratio_table[Hsel].n);

	// 计算输出图形尺寸以及LUT
	srcN = ipu_ratio_table[Wsel].n;
	dstM = ipu_ratio_table[Wsel].m;
	*outWp = resize_out_cal( img->in_width, rsize_w2, srcN, dstM, width_up, Wdiff);
	resize_lut_cal(srcN, dstM, width_up, h_lut);

	srcN = ipu_ratio_table[Hsel].n;
	dstM = ipu_ratio_table[Hsel].m;
	*outHp = resize_out_cal( img->in_height, rsize_h2, srcN, dstM, height_up, Hdiff);
	resize_lut_cal(srcN, dstM, height_up, v_lut);

	img->out_height = *outHp;
	img->out_width = *outWp;

	caculate_h_lut(h_lut, hoft_table, hcoef_table, Width_lut_max);
	caculate_v_lut(v_lut, voft_table, vcoef_table, Height_lut_max);
	OUTREG32(IPU_RSZ_COEF_INDEX, ((vcoef_real_heiht- 1) << VE_IDX_H_BIT) | ((hcoef_real_heiht- 1)  << HE_IDX_W_BIT));
//printf("%s,%d\n",__FUNCTION__,__LINE__);
	// 配置LUT寄存器
	#if 1
	SETREG32(IPU_VRSZ_LUT_BASE, 1 << V_CONF_BIT);
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
	for (i = 0; i < vcoef_real_heiht; i++)
	{
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
		OUTREG32(IPU_VRSZ_LUT_BASE, ((vcoef_table[i+1]&W_COEF0_MSK)<< W_COEF0_BIT)
			| ((voft_table[i+1]&V_OFT_MSK) << V_OFT_BIT));
	}
//printf("%s,%d\n",__FUNCTION__,__LINE__);
	SETREG32(IPU_HRSZ_LUT_BASE, 1 << H_CONF_BIT);
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
	for (i = 0; i < hcoef_real_heiht; i++)
	{
	printk("%s,%d\n",__FUNCTION__,__LINE__);
		OUTREG32(IPU_HRSZ_LUT_BASE, ((hcoef_table[i+1]&W_COEF0_MSK)<< W_COEF0_BIT)
			| ((hoft_table[i+1]&H_OFT_MSK) << H_OFT_BIT));
	}
	#else
	for (i = 0; i < Height_lut_max; i++)
	{
		if ((height_up == 0) && ((i & 1) == 0))
		{
			if ((v_lut[i].out_n == 0) && (v_lut[i+1].out_n == 1))
			{
				v_lut[i].out_n = 1;
				v_lut[i+1].out_n = 0;
			}
	 	}

		OUTREG32(IPU_VRSZ_LUT_BASE, (v_lut[i].coef << W_COEF0_BIT)
			/*| (v_lut[i].in_n << IN_EN_BIT) | (v_lut[i].out_n  << OUT_EN_BIT)*/);
	}

	SETREG32(IPU_HRSZ_LUT_BASE, 1 << H_CONF_BIT);
	for (i = 0; i < Width_lut_max; i++)
	{
		OUTREG32(IPU_HRSZ_LUT_BASE, (h_lut[i].coef << W_COEF0_BIT)
		/*| (h_lut[i].in_n << IN_EN_BIT) | (h_lut[i].out_n  << OUT_EN_BIT)*/);
	}
	#endif

	return (0);
}
/**************************************************
 * 设置输入输出寄存器
 **************************************************/
static int jz47_set_ipu_buf(void)
{
	int ret;
	unsigned int py_buf;
	unsigned int pu_buf;
	unsigned int pv_buf;
	unsigned int py_t_buf;
	unsigned int pu_t_buf;
	unsigned int pv_t_buf;
	unsigned int out_buf;
	// 定义IPU_CONTROL数据结构变量
	PIPU_CTRL2 pipu_ctrl = (PIPU_CTRL2)(&img->ipu_ctrl);

#if 0
	py_buf = PHYS((unsigned int)img->y_buf);
	pu_buf = PHYS((unsigned int)img->u_buf);
	pv_buf = PHYS((unsigned int)img->v_buf);
#else
	py_buf = ((unsigned int)img->y_buf);
	pu_buf = ((unsigned int)img->u_buf);
	pv_buf = ((unsigned int)img->v_buf);
#endif
	//printf("=====py__buf=[0x%x],pu__buf=[0x%x],pv__buf=[0x%x]\n",py_buf,pu_buf,pv_buf);

	if (pipu_ctrl->spage_map != 0) // 如果允许页面地址映射
	{
		printk("spage map != 0 \n");
		// 数据有效性判断
		if ((py_t_buf == 0) || (pu_t_buf == 0) || (pv_t_buf == 0))
		{
			printk(" Can not found source map table, use no map now!\r\n");
			pipu_ctrl->spage_map = 0;
		}
		else
		{
			py_t_buf = PHYS((unsigned int)img->y_t_addr);
			pu_t_buf = PHYS((unsigned int)img->u_t_addr);
			pv_t_buf = PHYS((unsigned int)img->v_t_addr);

			//printf("=====py_t_buf=[0x%x],pu_t_buf=[0x%x],pv_t_buf=[0x%x]\n",py_t_buf,pu_t_buf,pv_t_buf);

			py_buf = py_t_buf & 0xfff;
			pu_buf = pu_t_buf & 0xfff;
			pv_buf = pv_t_buf & 0xfff;

			// Input source TLB base address
			OUTREG32(IPU_Y_PHY_T_ADDR, py_t_buf);
			OUTREG32(IPU_Y_PHY_T_ADDR, pu_t_buf);
			OUTREG32(IPU_Y_PHY_T_ADDR, pv_t_buf);

			//printf("=====IPU_SRC_TLB_ADDR=[0x%x]===\n",REG32(IPU_SRC_TLB_ADDR));
		}
	}
	else
	{
		if ((py_buf == 0) || (pu_buf == 0) || (pv_buf == 0))
		{
			printk ("++ Can not found buffer(0x%x,0x%x,0x%x) physical addr since addr errors +++\n",
		          (unsigned int)img->y_buf, (unsigned int)img->u_buf, (unsigned int)img->v_buf);
			return (-1);
		}
	}

	// 输入Y信号数据地址寄存器
	OUTREG32(IPU_Y_ADDR, py_buf);
	OUTREG32(IPU_U_ADDR, pu_buf);
	OUTREG32(IPU_V_ADDR, pv_buf);

	// 如果允许地址映射,以及不是直通模式
	if ((pipu_ctrl->dpage_map != 0) && (pipu_ctrl->lcdc_sel == 0))
	{
		if (PHYS(img->out_t_addr) == 0)
		{
			//printf(" Can not found destination map table, use no map now!\r\n");
			pipu_ctrl->dpage_map = 0;

			if (PHYS(img->out_buf) == 0)
			{
				printk("Can not found the destination buf[%#x]\r\n",  img->out_buf);
				return (-1);
			}
			else
			{
				OUTREG32(IPU_OUT_ADDR, PHYS(img->out_buf));
			}
		}
		else
		{
			// 配置数据输出寄存器、输出地址映射表寄存器
			OUTREG32(IPU_OUT_ADDR, PHYS(img->out_t_addr) & 0xfff);
			OUTREG32(IPU_U_PHY_T_ADDR, PHYS(img->out_t_addr));
		}
	}
	else
	{
		pipu_ctrl->dpage_map = 0;
		if (pipu_ctrl->lcdc_sel == 0)
		{
			if (PHYS(img->out_buf) == 0)
			{
				printk("Can not found the destination buf[%#x]\r\n",  img->out_buf);
				return (-1);
			}
			else
			{
                               out_buf = (unsigned int)img->out_buf + (img->out_y * lcd_get_width()+ img->out_x) * img->in_bpp / 8;//sxyzhang 2010-12-2
//				out_buf = (unsigned int)img->out_buf + (img->out_y * fb_w+ img->out_x) * img->in_bpp / 8;
				OUTREG32(IPU_OUT_ADDR, PHYS(out_buf));
				//OUTREG32(IPU_OUT_ADDR, (out_buf));
				//printf("===")
			}
		}else{
			printk("\n\n\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		}
	}
	return (0);
}

#ifndef IPU_FUNC_EX
/************************************************
 *	计算输出尺寸
 ************************************************/
static int resize_out_cal(int insize, int outsize, int srcN, int dstM,
                    	int upScale, int *diff)
{
	unsigned int calsize, delta;
	unsigned int tmp, tmp2;

	delta = IPU_RATIO_MUL;
	insize *= IPU_RATIO_MUL;

	do
	{
		tmp = ((insize - delta) * dstM / srcN) & (~(IPU_RATIO_MUL - 1));
		tmp2 = tmp  * srcN / dstM;
		if (upScale)// 如果图像放大
		{
			if (tmp2 == insize - delta)
			{
				calsize = tmp / IPU_RATIO_MUL + 1;
			}
			else
			{
				calsize = tmp / IPU_RATIO_MUL + 2;
			}
		}
		else   		// 缩小
		{
			if (tmp2 == insize - delta)
			{
				calsize = tmp / IPU_RATIO_MUL;
			}
			else
			{
		    	calsize = tmp / IPU_RATIO_MUL + 1;
		    }
		}

		delta += IPU_RATIO_MUL;

	} while (calsize > outsize);

	*diff = delta / IPU_RATIO_MUL - 2;

	return (calsize);
}

/************************************************
 *	计算LUT
 ************************************************/
static int resize_lut_cal(int srcN, int dstM, int upScale, rsz_lut lut[])
{
	int i, t, x;
	unsigned int w_coef, factor, factor2;

	if (upScale)
	{
		for (i = 0, t = 0; i < dstM; i++)
		{
			factor = (i * srcN * IPU_RATIO_MUL) / dstM;
			factor2 = factor - factor / IPU_RATIO_MUL * IPU_RATIO_MUL;
			w_coef = IPU_RATIO_MUL  - factor2;
			lut[i].coef = (unsigned int)(512 * w_coef / IPU_RATIO_MUL) & W_COEF0_MSK;
			// calculate in & out
			lut[i].out_n = 1;
			if (t <= factor)
			{
				lut[i].in_n = 1;
				t += IPU_RATIO_MUL;
			}
			else
			{
				lut[i].in_n = 0;
			}
		} // end for
	}
	else
	{
		for (i = 0, t = 0, x = 0; i < srcN; i++)
		{
			factor = (t * srcN + 1) * IPU_RATIO_MUL / dstM;
			if (dstM == 1)
			{
				// calculate in & out
				lut[i].coef = (i == 0) ? 256 : 0;
				lut[i].out_n = (i == 0) ? 1  : 0;
			}
			else
			{
				if (((t * srcN + 1) / dstM - i) >= 1)
				{
					lut[i].coef = 0;
				}
				else
				{
					if (factor - i * IPU_RATIO_MUL == 0)
					{
						lut[i].coef = 512;
						t++;
					}
					else
					{
						factor2 = (t * srcN ) / dstM * IPU_RATIO_MUL;
						factor = factor - factor2;
						w_coef = IPU_RATIO_MUL  - factor;
						lut[i].coef = (unsigned int)(512 * w_coef / IPU_RATIO_MUL) & W_COEF0_MSK;
						t++;
					}
				}
			}
			// calculate in & out
			lut[i].in_n = 1;
			if (dstM != 1)
			{
				if (((x * srcN + 1) / dstM - i) >= 1)
				{
					lut[i].out_n = 0;
				}
				else
				{
					lut[i].out_n = 1;
					x++;
				}
			}

		}	// for end down Scale
	}	// else upScale

	return (0);
}

static void caculate_h_lut(rsz_lut *h_lut0, int * hoft_table, int* hcoef_table,  int H_MAX_LUT)
{
	int j ;
	int i;
	int in_oft_tmp = 0;
	int coef_tmp = 0;
	rsz_lut *h_lut;

	h_lut = h_lut0;

	j = 0 ;
	for (i=0; i<H_MAX_LUT; i++)
	{
		if ( h_lut[i].out_n ) {
			hoft_table[j] = (h_lut[i].in_n == 0)? 0: in_oft_tmp;
			hcoef_table[j] = coef_tmp;
			coef_tmp = h_lut[i].coef;
			in_oft_tmp = h_lut[i].in_n==0? in_oft_tmp : h_lut[i].in_n ;
			j++;
		}
		else
			in_oft_tmp = h_lut[i].in_n + in_oft_tmp;
	}
	if ( h_lut[0].out_n ){
		hoft_table[j] = (h_lut[0].in_n == 0)? 0: in_oft_tmp;
		hcoef_table[j] = coef_tmp;
	}

	hcoef_real_heiht = j;
}

static void caculate_v_lut(rsz_lut *v_lut0, int * voft_table, int* vcoef_table, int V_MAX_LUT)
{
	int j;
	int i;
	int in_oft_tmp = 0;
	int coef_tmp = 0;
	rsz_lut *v_lut;

	v_lut = v_lut0;


	j = 0 ;
	for (i=0; i<V_MAX_LUT; i++)
	{
		if ( v_lut[i].out_n ){
			voft_table[j] = (v_lut[i].in_n == 0)? 0: in_oft_tmp;
			vcoef_table[j] = coef_tmp;
			coef_tmp = v_lut[i].coef;
			in_oft_tmp = v_lut[i].in_n==0? in_oft_tmp : v_lut[i].in_n ;
			j++;
		}
		else
			in_oft_tmp = v_lut[i].in_n + in_oft_tmp;
	}
	if ( v_lut[0].out_n ){
		voft_table[j] = (v_lut[0].in_n == 0)? 0: in_oft_tmp;
		vcoef_table[j] = coef_tmp;
	}

	vcoef_real_heiht = j;
}
#endif
/**********************************************************
 *	IPU中断处理函数----没有使用
 *  Camera.c文件中使用的查询
 **********************************************************/
static void ipu_interrupt_handler(void)
{
	//printf("_______[%s]_[%d]______\n",__FUNCTION__,__LINE__);
	//	if (ipu_change_buf != 0)
	{
		if ((INREG32(IPU_STATUS) & OUT_END) == 0)
		{
			return;
		}

		CLRREG32(IPU_STATUS, OUT_END);

		// jz47_set_ipu_buf();
		OUTREG32(IPU_Y_ADDR, PHYS((unsigned int)img->y_buf));
		OUTREG32(IPU_U_ADDR, PHYS((unsigned int)img->u_buf));
		OUTREG32(IPU_V_ADDR, PHYS((unsigned int)img->v_buf));
		// ipu_change_buf = 0;
		OUTREG32(IPU_Y_STRIDE, *py_stride);
		OUTREG32(IPU_UV_STRIDE, U_STRIDE(*pu_stride) | V_STRIDE(*pv_stride));

		SETREG32(IPU_CTRL, IPU_RUN);
		// printf(" %s\n", __FUNCTION__);
		IPU_INTC_DISABLE();
	}
}
/************************************************
 * 打印数据参数
 ************************************************/
int print_img(void)
{
	printk ("ipu_ctrl[%#x]\r\n" , img->ipu_ctrl);
	printk ("ipu_d_fmt[%#x]\r\n" , img->ipu_d_fmt);
	printk ("in_width[%#x]\r\n" , img->in_width);
	printk ("in_height[%#x]\r\n" , img->in_height);
	printk ("in_bpp[%#x]\r\n" , img->in_bpp);
	printk ("out_width[%#x]\r\n" , img->out_width);
	printk ("out_height[%#x]\r\n" , img->out_height);
	printk ("y_buf[%#x]\r\n" , img->y_buf);
	printk ("u_buf[%#x]\r\n" , img->u_buf);
	printk ("v_buf[%#x]\r\n" , img->v_buf);
	printk ("out_buf[%#x]\r\n" , img->out_buf);
	printk ("y_t_addr[%#x]\r\n" , img->y_t_addr);
	printk ("u_t_addr[%#x]\r\n" , img->u_t_addr);
	printk ("v_t_addr[%#x]\r\n" , img->v_t_addr);
	printk ("out_t_addr[%#x]\r\n" , img->out_t_addr);
}
/************************************************
 * 打印寄存器
 ************************************************/
int print_reg(void)
{
	return 0;
	printk("=1==IPU_CTRL=[0x%8x]===\n",REG32(IPU_CTRL));
	printk("=2==IPU_STATUS=[0x%8x]===\n",REG32(IPU_STATUS));
	printk("=3==IPU_D_FMT=[0x%8x]===\n",REG32(IPU_D_FMT));
	printk("=4==IPU_Y_ADDR=[0x%8x]===\n",REG32(IPU_Y_ADDR));
	printk("=5==IPU_U_ADDR=[0x%8x]===\n",REG32(IPU_U_ADDR));
	printk("=6==IPU_V_ADDR=[0x%8x]===\n",REG32(IPU_V_ADDR));
	printk("=7==IPU_IN_FM_GS=[0x%8x]===\n",REG32(IPU_IN_FM_GS));
	printk("=8==IPU_Y_STRIDE=[0x%8x]===\n",REG32(IPU_Y_STRIDE));
	printk("=9==IPU_UV_STRIDE=[0x%8x]===\n",REG32(IPU_UV_STRIDE));
	printf("=10==IPU_OUT_ADDR=[0x%8x]===\n",REG32(IPU_OUT_ADDR));
	printf("=11==IPU_OUT_GS=[0x%8x]===\n",REG32(IPU_OUT_GS));
	printf("=12==IPU_OUT_STRIDE=[0x%8x]===\n",REG32(IPU_OUT_STRIDE));
	printf("=13==IPU_RSZ_COEF_INDEX=[0x%8x]===\n",REG32(IPU_RSZ_COEF_INDEX));
	printf("=14==IPU_CSC_C0_COEF=[0x%8x]===\n",REG32(IPU_CSC_C0_COEF));
	printf("=15==IPU_CSC_C1_COEF=[0x%8x]===\n",REG32(IPU_CSC_C1_COEF));
	printf("=16==IPU_CSC_C2_COEF=[0x%8x]===\n",REG32(IPU_CSC_C2_COEF));
	printf("=17==IPU_CSC_C3_COEF=[0x%8x]===\n",REG32(IPU_CSC_C3_COEF));
	printf("=18==IPU_CSC_C4_COEF=[0x%8x]===\n",REG32(IPU_CSC_C4_COEF));
	printf("=19==IPU_HRSZ_LUT_BASE=[0x%8x]===\n",REG32(IPU_HRSZ_LUT_BASE));
	printf("=20==IPU_VRSZ_LUT_BASE=[0x%8x]===\n",REG32(IPU_VRSZ_LUT_BASE));
	printf("=21==IPU_CSC_OFSET_PARA=[0x%8x]===\n",REG32(IPU_CSC_OFSET_PARA));
	printf("=22==IPU_SRC_TLB_ADDR=[0x%8x]===\n",REG32(IPU_Y_PHY_T_ADDR));
	printf("=23==IPU_DEST_TLB_ADDR=[0x%8x]===\n",REG32(IPU_U_PHY_T_ADDR));
#if 0
	printf("=24==IPU_TLB_MONITOR=[0x%8x]===\n",REG32(IPU_TLB_MONITOR));
	printf("=25==IPU_ADDR_CTRL=[0x%8x]===\n",REG32(IPU_ADDR_CTRL));
	printf("=26==IPU_Y_ADDR_N=[0x%8x]===\n",REG32(IPU_Y_ADDR_N));
	printf("=27==IPU_U_ADDR_N=[0x%8x]===\n",REG32(IPU_U_ADDR_N));
	printf("=28==IPU_V_ADDR_N=[0x%8x]===\n",REG32(IPU_V_ADDR_N));
	printf("=29==IPU_OUT_ADDR_N=[0x%8x]===\n",REG32(IPU_OUT_ADDR_N));
	printf("=30==IPU_SRC_TLB_ADDR_N=[0x%8x]===\n",REG32(IPU_SRC_TLB_ADDR_N));
	printf("=31==IPU_DEST_TLB_ADDR_N=[0x%8x]===\n",REG32(IPU_DEST_TLB_ADDR_N));
	printf("=32==IPU_TLB_CTRL=[0x%8x]===\n",REG32(IPU_TLB_CTRL));
#endif
}

/************************************************
 * IPU初始化函数2
 ************************************************/
int ipu_init2(img_param_t *pimg)
{
	int ret = -1;
	int in_fmt;
	int out_fmt;
	int outW, outH, Wdiff, Hdiff;

	printf("Debug=== +++IPU init +++....\r\n");

#ifdef IPU_FUNC_EX // 没有定义
	if (!pimg || !resize_lut_cal || !resize_out_cal)
#else
	if (!pimg)
#endif
	{
		printf("ipu_init: parameter error\r\n");
		return (ret);
	}
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
 	get_fbaddr_info();
	// printf("reg_ctrl=0x%x,reg_ctrl=0x%x\n",img->ipu_ctrl,INREG32(IPU_CTRL));
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
	// 把传递过来的数据赋给全局变量
	img = pimg;
	outW 	= img->out_width;
	outH 	= img->out_height;
	in_fmt 	= img->ipu_d_fmt & IN_FMT_MASK;
	out_fmt = img->ipu_d_fmt & OUT_FMT_MASK;

//	ipu_ratio_table = img->ratio_table;  //sxyzhang 2010-12-2 close
	//printf("iputable=0x%x\n",ipu_ratio_table );

	py_stride 	= &(img->stride->y);
	pu_stride 	= &(img->stride->u);
	pv_stride 	= &(img->stride->v);
	pout_stride = &(img->stride->out);

	//printf("img->in_widht=%d\n",img->in_width);
	//printf("img->in_height=%d\n",img->in_height);
	//printf("img->in_bpp=%d\n",img->in_bpp);
	printf("img->out_widht=%d\n",img->out_width);
	printf("img->out_height%d\n",img->out_height);
	//printf("img->out-y=%d\n",img->stride->y);
	//printf("img->out-u=%d\n",img->stride->u);
	//printf("img->out-v=%d\n",img->stride->v);
	//printf("img->out-out=%d\n",img->stride->out);
	// img->ipu_ctrl &= (LCDC_SEL | FM_IRQ_EN | ADDR_SEL);
	// 如果没有进行初始化
	if (ipu_inited == 0)
	{
		/***wyang***/
		CLRREG32(CPM_CLKGR0, CLKGR0_IPU); // run ipu clock
		//使用查询法，没有使用中断
		//request_irq(IRQ_IPU, ipu_interrupt_handler, 0);
		IPU_INTC_DISABLE();

		CLRREG32(IPU_CTRL, IPU_RUN);
		SETREG32(IPU_CTRL, CHIP_EN);
		SETREG32(IPU_CTRL, IPU_RST);// reset controller
		CLRREG32(IPU_CTRL, IPU_RST);
		while ((INREG32(IPU_STATUS) & OUT_END) == 0){
			printk("time out!\n");; // wait the end flag
		}
	}
	else
	{
		while ((INREG32(IPU_CTRL) & IPU_RUN) && ((INREG32(IPU_STATUS) & OUT_END) == 0)); // wait the end flag
		SETREG32(IPU_CTRL, IPU_RST);				// reset controller
		CLRREG32(IPU_CTRL, IPU_RST);
		while ((INREG32(IPU_STATUS) & OUT_END) == 0); // wait the end flag
	}
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
	// 条件不满足(camera下)
	if ((in_fmt == IN_FMT_YUV444) && (out_fmt != OUT_FMT_YUV422))
	{
		img->ipu_ctrl &= ~SPKG_SEL; //Separated Frame
	}
	//printf("%s,%d\n",__FUNCTION__,__LINE__);

	// 计算缩放因子并返回
	jz47_set_ipu_resize(&outW, &outH, &Wdiff, &Hdiff);

	printf("===outW=[%d],outH=[%d],Wdiff=[%d],Hdiff=[%d]===\n",outW,outH,Wdiff,Hdiff);
	// 配置寄存器
	//printf("%s,%d\n",__FUNCTION__,__LINE__);
	ret = jz47_set_ipu_csc_cfg(outW, outH, Wdiff, Hdiff);
	if (ret != 0)  return (ret);
	//printf("%s,%d\n",__FUNCTION__,__LINE__);

 	//printf("reg_ctrl=0x%x,reg_ctrl=0x%x\n",img->ipu_ctrl,INREG32(IPU_CTRL));

	// 配置输入输出寄存器
	if (jz47_set_ipu_buf() < 0) // 正常返回0
	{
		//printf("%s,%d\n",__FUNCTION__,__LINE__);

		ret = -1;
		return (ret);
	}
	//printf("%s,%d\n",__FUNCTION__,__LINE__);

	// set  stride
	if (img->stride == 0)
	{
	//printf("[%s][%d]\n",__FUNCTION__,__LINE__);
	if((img->ipu_ctrl) |SPKG_SEL)
	//	if (img->ipu_ctrl & SPKG_SEL)
		{
			OUTREG32(IPU_Y_STRIDE, img->in_width * 2);
		}
		else
		{

			OUTREG32(IPU_Y_STRIDE, img->in_width);
		}

		switch(((PIPU_DFMT2)&(img->ipu_d_fmt))->in_fmt)
		{
			case IN_FMT_YUV420:
			case IN_FMT_YUV422:
				OUTREG32(IPU_UV_STRIDE, U_STRIDE(img->in_width / 2) | V_STRIDE(img->in_width / 2));
				break;

			case IN_FMT_YUV444:
				OUTREG32(IPU_UV_STRIDE, U_STRIDE(img->in_width) | V_STRIDE(img->in_width));
				break;

			case IN_FMT_YUV411:
				OUTREG32(IPU_UV_STRIDE, U_STRIDE(img->in_width / 4) | V_STRIDE(img->in_width / 4));
				break;

			default:
				printf("Error: Input data format isn't support\n");
				return (-1);
				break;
		}
	}
	else
	{
		printf("[%s][%d]\n",__FUNCTION__,__LINE__);
#if 1
		OUTREG32(IPU_Y_STRIDE, *py_stride);
		OUTREG32(IPU_UV_STRIDE, U_STRIDE(*pu_stride) | V_STRIDE(*pv_stride));
#else
		OUTREG32(IPU_Y_STRIDE, 640*2);
		OUTREG32(IPU_UV_STRIDE, U_STRIDE(320) | V_STRIDE(320));
#endif
		//printf("[%s][%d]\n",__FUNCTION__,__LINE__);

	}
	// IPU使能
	OUTREG32(IPU_CTRL, img->ipu_ctrl | CHIP_EN);
	//printf("[%s][%d]\n",__FUNCTION__,__LINE__);

	ipu_inited = 1;

	//print_img();
	print_reg();
	return (0);
}
/*************************************************
 *
 *************************************************/
int ipu_deinit2(void)
{
    //free_irq(IRQ_IPU);
    CLRREG32(IPU_CTRL, CHIP_EN);

    /***wyang***/
    //cpm_stop_clock(CGM_IPU);
    SETREG32(CPM_CLKGR0, CLKGR0_IPU);

    ipu_inited = 0;
    return (0);
}

static int ipu_run;

void ipu_driver_set_src_stride(int src_w, int dst_w)
{
	img_param_t *img;
	img = ipu_def_img;

	img->stride = &def_stride;
	img->stride->out = dst_w*2;
	img->stride->y = src_w*2;
	img->stride->u = src_w/2;
	img->stride->v = src_w/2;
}

#if defined(CONFIG_JZ_CIM)

void ipu_driver_init_test(int src_w,int src_h,int dst_w,int dst_h)
{
	img_param_t *img;
	img = ipu_def_img;
	//img->output_mode = IPU_OUTPUT_TO_FRAMEBUFFER;
	img->in_width = src_w;
	img->in_height = src_h;
	img->in_bpp = 16;
	img->out_width = dst_w;
	img->out_height = dst_h;
	if (dst_w > 2*src_w)
	{
		img->out_x = (dst_w - 2*src_w)/2;
	}else
		img->out_x = 0;

	if (dst_h > 2*src_h)
	{
		img->out_y = (dst_h - 2*src_h)/2;
	}else
		img->out_y = 0;

	img->ipu_ctrl = ADDR_SEL | SPKG_SEL ;
	//img->ipu_ctrl = ADDR_SEL | LCDC_SEL;
	//img->ipu_ctrl =  SPKG_SEL;
	//img->zoom_mode = ZOOM_MODE_BILINEAR;
	//img->in_fmt  = IN_FMT_YUV422;
	//img->out_fmt = OUT_FMT_RGB565;
	img->ipu_d_fmt = IN_FMT_YUV422 | OUT_FMT_RGB565 | IPU_INDATA_PACKAGE_OFFSIZE;
	//img->ipu_d_fmt = IN_FMT_YUV422 | OUT_FMT_RGB565 ;
	//img->out_buf = (unsigned int)(lcd_frame0 + 480*272*2);
	img->out_buf = (unsigned int)(lcd_frame0);
	img->y_buf = (unsigned int)cim_read_dma();
	img->u_buf = (unsigned int)cim_read_dma();
	img->v_buf = (unsigned int)cim_read_dma();
#if 0
	img->stride = &def_stride;
	img->stride->out = dst_w*2;
	img->stride->y = src_w*2;
	img->stride->u = src_w/2;
	img->stride->v = src_w/2;
#else
#endif
	ipu_init2(img);
#if 0
		__lcd_clr_ena();
		__lcd_enable_f1();
		//__lcd_disable_f0();
		__lcd_fg1_use_ipu();
		REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
		REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
		__lcd_set_ena();
#endif

}
EXPORT_SYMBOL(ipu_driver_init_test);
#endif

void ipu_driver_start(void);
void ipu_driver_stop(void);
void ipu_driver_open_tv(int src_w,int src_h,int dst_w,int dst_h)
{
	img_param_t *img;
	img = ipu_def_img;
	//img->output_mode = IPU_OUTPUT_TO_FRAMEBUFFER;
	img->in_width = src_w;
	img->in_height = src_h;
	img->in_bpp = 16;
	img->out_width = dst_w;
	img->out_height = dst_h;

	if (dst_w > 2*src_w)
	{
		img->out_x = (dst_w - 2*src_w)/2;
	}else
		img->out_x = 0;

	if (dst_h > 2*src_h)
	{
		img->out_y = (dst_h - 2*src_h)/2;
	}else
		img->out_y = 0;


	img->ipu_ctrl = ADDR_SEL | SPKG_SEL;
	//img->ipu_ctrl = ADDR_SEL | LCDC_SEL;
	//img->ipu_ctrl =  SPKG_SEL;
	//img->zoom_mode = ZOOM_MODE_BILINEAR;
	//img->in_fmt  = IN_FMT_YUV422;
	//img->out_fmt = OUT_FMT_RGB565;
	img->ipu_d_fmt = IN_FMT_PKG_RGB565 | OUT_FMT_RGB565 | 0x1 ;
	//img->ipu_d_fmt = IN_FMT_YUV422 | OUT_FMT_RGB565 ;
	//img->out_buf = (unsigned int)(lcd_frame0 + 480*272*2);
	img->out_buf = (unsigned int)(lcd_frame0);
	img->y_buf = (unsigned int)PHYS(lcd_frame0);
	img->u_buf = (unsigned int)PHYS(lcd_frame0);
	img->v_buf = (unsigned int)PHYS(lcd_frame0);
	img->stride = &def_stride;
	img->stride->out = dst_w*2;
	img->stride->y = src_w*2;
	img->stride->u = src_w/2;
	img->stride->v = src_w/2;
	ipu_init2(img);
	//ipu_driver_start();

}
extern unsigned int frame_yoffset;
void ipu_update_address(void)
{
	REG32(IPU_V_BASE + REG_Y_ADDR) = (unsigned int)PHYS(lcd_frame0 + frame_yoffset);
}

EXPORT_SYMBOL(ipu_update_address);
//EXPORT_SYMBOL(ipu_driver_stop);
//EXPORT_SYMBOL(ipu_driver_start);

void ipu_driver_flush_tv(void)
{
	if (!ipu_inited)
		return;
	//printk("flush tv start\n");
	while(((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0);
	REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
	if (!REG32(IPU_V_BASE + REG_ADDR_CTRL)){

		REG32(IPU_V_BASE + REG_Y_ADDR) = (unsigned int)PHYS(lcd_frame0);
		REG32(IPU_V_BASE + REG_ADDR_CTRL) = YUV_READY;
		REG32(IPU_V_BASE + REG_OUT_ADDR) = (unsigned int)ipu_def_img->out_buf;
		REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
		REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
	}
	//printk("flush tv end\n");

}

void ipu_driver_close_tv(void)
{
	ipu_driver_stop();
}

#if defined(CONFIG_JZ_CIM)
void ipu_driver_update_test(void)
{

	//printk("comming medive update cim data!\n");
	while(((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0);
	REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
	if (!REG32(IPU_V_BASE + REG_ADDR_CTRL)){

		//REG32(IPU_V_BASE + REG_Y_ADDR) = virt_to_phys(vaddr_yuv);
#if 1
		REG32(IPU_V_BASE + REG_Y_ADDR) = (unsigned int)cim_read_dma();
		REG32(IPU_V_BASE + REG_ADDR_CTRL) = YUV_READY;
		REG32(IPU_V_BASE + REG_OUT_ADDR) = (unsigned int)ipu_def_img->out_buf;
		REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
		REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
#else
		REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
#endif
	}
	//printk("go out medive update cim data!\n");

}
EXPORT_SYMBOL(ipu_driver_update_test);
#endif

void ipu_driver_start(void)
{
		REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
		REG32(IPU_V_BASE + REG_CTRL) |= IPU_RUN;
}
void ipu_driver_stop(void)
{
	if (!ipu_inited)
		return;
	ipu_inited = 0;
	while(((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0);
	REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
	REG32(IPU_V_BASE + REG_CTRL) &= ~IPU_RUN;
	memset((unsigned char *)ipu_def_img->out_buf,0,ipu_def_img->out_width*ipu_def_img->out_height*2);
}
void ipu_driver_stop_new(void)
{
	if (!ipu_inited)
		return;
	ipu_inited = 0;
	while(((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0);
	REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
	REG32(IPU_V_BASE + REG_CTRL) &= ~IPU_RUN;
	//memset((unsigned char *)ipu_def_img->out_buf,0,ipu_def_img->out_width*ipu_def_img->out_height*2);
}


void ipu_driver_wait_end(void)
{
	while(((REG32(IPU_V_BASE + REG_STATUS)) & OUT_END) == 0);
	REG32(IPU_V_BASE + REG_STATUS) &= ~OUT_END;
}

int ipu_get_status(void)
{
	return ipu_inited;
}

EXPORT_SYMBOL(ipu_get_status);

EXPORT_SYMBOL(ipu_driver_wait_end);
EXPORT_SYMBOL(ipu_driver_stop);
EXPORT_SYMBOL(ipu_driver_stop_new);
EXPORT_SYMBOL(ipu_driver_open_tv);
EXPORT_SYMBOL(ipu_driver_close_tv);
EXPORT_SYMBOL(ipu_driver_flush_tv);
EXPORT_SYMBOL(ipu_driver_set_src_stride);



