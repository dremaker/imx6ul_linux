/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/io.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/irq.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/clk.h>
#include <linux/types.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <asm/system.h>
static uint32_t track1 = 0;
static uint32_t track2 = 1;
static uint32_t track3 = 2;

static uint32_t track_mask;
#define TRACK_N_USED(n)		(track_mask&(1<<n))

#define TRACK_MASK_INIT 0x07

#define STX1 0x45
#define ETX1 0x1f
#define STX2 0x0b
#define ETX2 0x1f
#define INVALID_DATA  -1
#define NOT_GET_ETX   -2
#define LRC_ERROR     -3
#define NOT_GET_STX   -4

wait_queue_head_t inq;


#define MAG_MAX_BITS    1024
#define MAG_MAX_LONG    ((MAG_MAX_BITS + 31) / 32)
typedef struct {
        uint32_t data[MAG_MAX_LONG];
        uint32_t bits_count;
}track_data_t;


#define MAG_PUSLES_LEN (MAG_MAX_BITS + 376)
typedef struct {
	uint16_t count;
	uint16_t data[MAG_PUSLES_LEN];
	uint16_t last_data;
	uint16_t (*get_track_data)(void);
}track_pulse_t;

static track_pulse_t pulseBuf[3];
static volatile uint32_t read_pulse = 0;


static track_data_t tdata[3];

static volatile uint32_t track_flags;

#define TRACK_N_READY(n)	(0x001 << n)
#define TRACK_MASK_READY	(0x007)
#define TRACK_N_LOGIC1(n)	(0x010 << n)
#define TRACK_MASK_LOGIC1	(0x070)
#define TRACK_N_START(n)	(0x100 << n)
#define TRACK_MASK_START	(0x700)
#define TRACK_N_MASK(n)		(0x111<<n)
#define TRACK_OK(flag)		((flag&track_mask) && ((flag&0x07)==((flag>>8)&0x07)))
#define TRACK_N_OK(n, flag) ((flag&TRACK_N_READY(n)) && (flag&TRACK_N_START(n)))
#define TRACK_N_VALID(n)	TRACK_N_OK(n, track_flags)
#define TRACK_SET_ONE(tr, n)   \
        (tr)->data[(n) >> 5] |= (1 << ((n) & 31))
#define TRACK_DATA_CLEAR(pTrack) \
        	  memset(pTrack, 0x00, sizeof (track_data_t))

#define DRIVER_NAME "mxc_magc"

struct magc_t
{
	s32 track_gpio;
	s32 track_irq;

};

static struct magc_t magc_data[3];



#define MAGC_ENTER_CRITICAL()
#define MAGC_EXIT_CRITICAL()

uint8_t guide_count_lower_limit[3] = {10, 5, 10};

#define FINISH_IDLE_COUNT_MAX 10000
#define IDLE_COUNT_MAX 65000


#define MAGC_GPIO_CFG() do{     \
                              	gpio_direction_input(magc_data[0].track_gpio); \
								gpio_direction_input(magc_data[1].track_gpio); \
								gpio_direction_input(magc_data[2].track_gpio); \
	                          }while(0)


/* 台式POS只有2 3两个磁道，关闭自动调整1 3磁道功能 */
static uint32_t trackAdaptDone = 1; /* 磁道1 3 自动调整，磁道2需手动对准 */
//static uint32_t track23AdaptDone = 0; /* 磁道2 3 自动调整，依据磁道3数据比磁道2数据长特征 */

#define IRQ_DISABLE 0
#define IRQ_ENABLE 1
static volatile u32 irq_enable_flag = IRQ_ENABLE;
#define MAGC_INT_CLR()     do{ }while(0)

/* 由于disable_irq，enable_irq 只是简单的计数加减，当enable_irq调用次数多于disable_irq,
内核报unbalance错误，所以加入 irq_enale_flag来判断是否已经enable_irq.
注意irq_enable_flag的互斥问题。打开中断之前应该先完成赋值*/
#define MAGC_INT_ENABLE()  do{	if(irq_enable_flag == IRQ_DISABLE){ \
								irq_enable_flag = IRQ_ENABLE;	\
								enable_irq(magc_data[0].track_irq); \
								enable_irq(magc_data[1].track_irq); \
								enable_irq(magc_data[2].track_irq); \
								 }    \
								}while(0)
								
#define MAGC_INT_DISABLE() do{  if(irq_enable_flag == IRQ_ENABLE){  \
								disable_irq_nosync(magc_data[0].track_irq); \
								disable_irq_nosync(magc_data[1].track_irq); \
								disable_irq_nosync(magc_data[2].track_irq); \
								irq_enable_flag = IRQ_DISABLE;            \
								}    \
								}while(0)

#define MAGC_INT_CFG()     do{}while(0)



uint16_t track0_get_data(void)
{
	
	if(gpio_get_value(magc_data[0].track_gpio))
		return 1;
	return 0;	
}	

uint16_t track1_get_data(void)
{


	if(gpio_get_value(magc_data[1].track_gpio))
		return 1;
	return 0;	
}	

uint16_t track2_get_data(void)
{


	if(gpio_get_value(magc_data[2].track_gpio))
		return 1;
	return 0;	

}



static const struct of_device_id magc_imx_dt_ids[] = {
	{ .compatible = "fsl,imx6ul-magc", },
	{ /* sentinel */ }
};




static irqreturn_t magc_irq_handler(int irq, void *dev_id)
{
  	static unsigned long flags=0;
	uint16_t pulseCount[3] = {0};
	uint8_t guideCount[3] = {0};
	uint8_t i;
	static DEFINE_SPINLOCK(magc_emu_lock);


	pr_debug("enter %s \n", __FUNCTION__);
	MAGC_INT_CLR();
	MAGC_INT_DISABLE();	
	spin_lock_irqsave(&magc_emu_lock, flags); /* MAGC_EXIT_CRITICAL() */


	pulseBuf[0].last_data = pulseBuf[0].get_track_data();
	pulseBuf[1].last_data = pulseBuf[1].get_track_data();
	pulseBuf[2].last_data = pulseBuf[2].get_track_data();

	pulseBuf[0].count = 0;
	pulseBuf[1].count = 0;
	pulseBuf[2].count = 0;
	track_flags = 0;
	
	while (1) {
		
		for (i=0; i<3; i++) {
			pulseCount[i] += 2;
			
			if (track_flags & TRACK_N_START(i)) { /* magnetic card swiping */              
				if (pulseBuf[i].last_data != pulseBuf[i].get_track_data()) {
					pulseBuf[i].last_data = pulseBuf[i].get_track_data();
					if (pulseBuf[i].count < MAG_PUSLES_LEN) {
						pulseBuf[i].data[pulseBuf[i].count++] = pulseCount[i];
					}
					pulseCount[i] = 0;
				}
		
				if (pulseCount[i] >= FINISH_IDLE_COUNT_MAX) { /* magnetic card swipe finish */ 
					if (track_flags & TRACK_N_START(i)) {
						track_flags |= TRACK_N_READY(i); 
					}
				}
			}
			else {                
				if (guideCount[i] >= guide_count_lower_limit[i]) {
					track_flags |= TRACK_N_START(i); /* magnetic card swipe start */
					guideCount[i] = 0;
          			pulseCount[i] = 0;
				}
				else {
					if (pulseBuf[i].last_data != pulseBuf[i].get_track_data()) {                
						pulseBuf[i].last_data = pulseBuf[i].get_track_data();
						guideCount[i]++;
					}				
				}			
				
				if ((pulseCount[i] >= IDLE_COUNT_MAX) && (!track_flags)) { /* didnot swipe card in the limit time */
						MAGC_INT_ENABLE();  /*无效的中断进来，再次打开中断*/
						spin_unlock_irqrestore(&magc_emu_lock, flags); /* MAGC_EXIT_CRITICAL() */
						
						return IRQ_HANDLED;
				}			
			}
		}
		
		if (((track_flags&0x07)==((track_flags>>8)&0x07)) && track_flags) {
			break;
		}
		
	}
	
	
	spin_unlock_irqrestore(&magc_emu_lock, flags); /* MAGC_EXIT_CRITICAL() */
	wake_up(&inq);
	pr_debug("IRP_handle wake up 0x%X \n", track_flags);
	return IRQ_HANDLED;

}




#define ABANDON_BITS    5 /* note: ABANDON_BITS > 2*/
#define LONG_PULSE(n)   ((n) > tresholdVaule)
#define CRISIS_PULSE(n) ((n) == tresholdVaule)
#define NARROW_PULSE(n) ((n) < tresholdVaule)
void magc_bits_decode(void)
{
	uint32_t i, j, index;
	uint16_t Tc; /* Time of the current pusle cycle */
	uint16_t Tn; /* Time of the next pusle cycle */
	uint16_t Tt; /* Time of the third cycle */
	uint16_t T0; /* Time of the logic 0 pulse cycle */
	uint16_t tresholdVaule;
	uint32_t bit;

	for (i = 0; i < 3; i++) {
		if (TRACK_N_OK(i, track_flags)) {
			memset(tdata+i, 0x00, sizeof (track_data_t));
			tdata[i].bits_count = 1;

			if (pulseBuf[i].count < 100){ /*  */
next_track:
				continue;
			}

			/* caculate T0 */
			T0 = 0;
			for (j = 0; j < 8; j++) {
				Tc = pulseBuf[i].data[j+1];
				Tn = pulseBuf[i].data[j+2];
				if (Tc > (Tn<<1)) {
					Tc = (pulseBuf[i].data[j] + Tn) >> 1;
				}
				T0 += Tc;
			}
			T0 = T0 >> 3;
			tresholdVaule = T0 - (T0 >> 2);
			
			index = ABANDON_BITS;
			pulseBuf[i].count -= ABANDON_BITS + 2;	// 2bit reserved 
			
			for ( ; index < pulseBuf[i].count; index++) {
				Tc = pulseBuf[i].data[index];
				Tn = pulseBuf[i].data[index+1]; /* only when ABANDON_BITS > 0 */
				
				if (LONG_PULSE(Tc)) {
					bit = 0;
					
					for (j=0; j+1<Tc/T0; j++) {
						/* set logic 0 */
						tdata[i].bits_count++;
						if(tdata[i].bits_count >= MAG_MAX_BITS) {
//							pr_debug(  "pulse count: %d, Tc=%d, T0=%d, Tn=%d\n",
//								tdata[i].bits_count, Tc, T0, Tn);
							pr_err("ERR: track %d data invalid,discard\n", i);
							goto next_track;// break;
						}
					}
				}
				else {
					bit = 1;
					
					if (CRISIS_PULSE(Tc)) {
						if (LONG_PULSE(Tn)||CRISIS_PULSE(Tn)) {
							bit = 0;
						}
					}
					else {
						if (NARROW_PULSE(Tc+Tn) || CRISIS_PULSE(Tc+Tn)) {
							Tt = pulseBuf[i].data[index+2]; /* only when ABANDON_BITS > 2 */
							if (NARROW_PULSE(Tt)) {
								bit = 0;
								index += 2;
								Tc += Tn + Tt;
							}
						}
						
//						if ( abs(Tc + Tn - T0) >= (T0>>1)) { /*  */
						if ((Tc+Tn) > (T0+(T0>>1)) || (Tc+Tn) <= (T0>>1)) {
			               bit = 0;
			            }
					}
				}
				
				if (bit==1) {
					if((tdata[i].bits_count >> 5) >= MAG_MAX_LONG)
						pr_debug( "TRACK_SET_ONE set: %d-%d\n", tdata[i].bits_count, tdata[i].bits_count >> 5);
					TRACK_SET_ONE(tdata + i, tdata[i].bits_count);
					index++;
					T0 = (T0 + Tc + Tn)>>1;
				}
				else {
					T0 = (T0 + Tc)>>1;
				}
				tdata[i].bits_count ++;
				if(tdata[i].bits_count >= MAG_MAX_BITS) {
					pr_debug("ERR: track[%d] data invalid,discard\n", i);
					goto next_track;// break;
				}
				
				tresholdVaule = T0 - (T0 >> 2);
			}
			
		}
	}
	
}


/*
** BRIEF:
**		Reversal byte
** PARAM:
**    data[in] data to reversal
** RETURN:
**	  the reversaled data
*/
uint32_t reversal_32bit(uint32_t data)
{
	uint32_t i;
	uint32_t tmp;

	tmp = 0;
	for (i=0x80000000; i>0; i>>=1) {
		tmp >>= 1;
		if ((data&i)!=0) {
			tmp |= 0x80000000;
		}
	}
	return (tmp);
}

/*
** BRIEF:
**		decode track1 data
** PARAM:
**    trackBuf[in] src data
**		trackOut[out] decoded data to be stored;
**		trackOutLen[in] the length of trackOut
** RETURN:
**   >0 decode success, return the length of data
**   -1 invalid data
**   -2 does not get ETX
**   -3 LRC error
**   -4 does not get STX
*/
int track1_decode(uint32_t *trackIn, uint8_t trackInLen,\
                   uint8_t *trackOut, uint8_t trackOutLen)
{
	uint16_t indexBit;
	uint8_t  indexTrackOut;
	uint32_t tmp;
	uint8_t  i;
	uint32_t xorLrc;
	uint8_t  getETX;
	uint8_t  reverse_done = 0;
	uint16_t indexBitSTX;

repeat:

	indexBit = 0;
	indexTrackOut = 0;
	xorLrc = 0;
	getETX = 0;
	i = 0;

	while ( trackIn[i]== 0) { /*find logic 1 */
		i++;
		indexBit += 32;
		if (i >= trackInLen) {
			return (INVALID_DATA);
		}
	}

	tmp = trackIn[i];
	for (i=0; i<32; i++) {
		if ((tmp&0x01) != 0) {
			indexBit += i;
			break;
		}
		tmp >>= 1;
	}

	tmp = trackIn[indexBit>>5]>>(indexBit&31);
	if ((indexBit&31) > 25) {
		tmp += trackIn[(indexBit>>5) + 1]<<(32-(indexBit&31));
	}

	while ((tmp&0x7f) != STX1){ /* find out STX1 */
		indexBit++;
		if (indexBit >= 20) {
			goto reverse;
		}
		tmp = trackIn[indexBit>>5]>>(indexBit&31);
		if ((indexBit&31) > 25) {
			tmp += trackIn[(indexBit>>5) + 1]<<(32-(indexBit&31));
		}
	}

	indexBitSTX = indexBit;
	for ( ; indexBit<(trackInLen<<5); indexBit+=7) {
		tmp = trackIn[indexBit>>5]>>(indexBit&31);
		if ((indexBit&31) > 25) {
			tmp += trackIn[(indexBit>>5) + 1]<<(32-(indexBit&31));
		}

		if (getETX) {
			if ((indexBit - indexBitSTX) == 14) {
				goto reverse;
			}
			if ((tmp&0x3f) == xorLrc) {
				return (indexTrackOut);
			}
			else {
				if (!reverse_done) {
					goto reverse;
				}
				return (LRC_ERROR);
			}
		}

		if (indexTrackOut < trackOutLen) {
			trackOut[indexTrackOut++] = (unsigned char)(tmp&0x3f) + ' '; /* turn into ASCII */
		}
		xorLrc ^= (tmp&0x3f);

		if ((tmp&0x7f) == ETX1) {
			getETX = 1;
		}
	}
	if (reverse_done) {
		return (NOT_GET_ETX);
	}

reverse:	
	if (!reverse_done) { /* reverse direction */
		for (i=0; i<(trackInLen>>1); i++) {
			tmp = trackIn[i];
			trackIn[i] = reversal_32bit(trackIn[trackInLen-1-i]);
			trackIn[trackInLen-1-i] = reversal_32bit(tmp);
		}
		if ((trackInLen&1) == 1) {
			tmp = trackIn[trackInLen>>1];
			trackIn[trackInLen>>1] = reversal_32bit(tmp);
		}
		reverse_done = 1;
		goto repeat;
	}
	else {
		return (NOT_GET_STX);
	}
}

/*
** BRIEF:
**		decode track2 or track3 data
** PARAM:
**    trackBuf[in] src data
**		trackOut[out] decoded data to be stored
**		trackOutLen[in] the length of trackOut
** RETURN:
**   >0 decode success, return the length of data
**   -1 invalid data
**   -2 does not get ETX
**   -3 LRC error
**   -4 does not get STX
*/
int track23_decode(uint32_t *trackIn, uint8_t trackInLen,\
                   uint8_t *trackOut, uint8_t trackOutLen)
{
	uint16_t indexBit;
	uint8_t indexTrackOut;
	uint32_t tmp;
	uint8_t i;
	uint32_t xorLrc;
	uint8_t getETX;
	uint8_t reverse_done = 0;
	uint16_t indexBitSTX;
	
repeat:

	indexBit = 0;
	indexTrackOut = 0;
	xorLrc = 0;
	getETX = 0;
	i = 0;

	while ( trackIn[i]== 0) { /* find logic 1 */
		i++;
		indexBit += 32;
		if (i >= trackInLen) {
			return (INVALID_DATA);
		}
	}

	tmp = trackIn[i];
	for (i=0; i<32; i++) {
		if ((tmp&0x01) != 0) {
			indexBit += i;
			break;
		}
		tmp >>= 1;
	}

	tmp = trackIn[indexBit>>5]>>(indexBit&31);
	if ((indexBit&31) > 27) {
		tmp += trackIn[(indexBit>>5) + 1]<<(32-(indexBit&31));
	}

	while ((tmp&0x1f) != STX2){ /* find out STX2 */
		indexBit++;
		if (indexBit >= 40) {
			goto reverse;
		}
		tmp = trackIn[indexBit>>5]>>(indexBit&31);
		if ((indexBit&31) > 27) {
			tmp += trackIn[(indexBit>>5) + 1]<<(32-(indexBit&31));
		}
	}

	/* forward direction */
	indexBitSTX = indexBit;
	for ( ; indexBit<(trackInLen<<5); indexBit+=5) {
		tmp = trackIn[indexBit>>5]>>(indexBit&31);
		if ((indexBit&31) > 27) {
			tmp += trackIn[(indexBit>>5) + 1]<<(32-(indexBit&31));
		}

		if (getETX) {
			if ((indexBit - indexBitSTX) == 10) {
				goto reverse;
			}
			if ((tmp&0x0f) == xorLrc) {
				return (indexTrackOut);
			}
			else {
				if (!reverse_done) {
					goto reverse;
				}
				return (LRC_ERROR);
			}
		}

		if (indexTrackOut < trackOutLen) {
			trackOut[indexTrackOut++] = (unsigned char)(tmp&0x0f) + '0'; /* turn into ASCII */
		}
		xorLrc ^= (tmp&0x0f);

		if ((tmp&0x1f) == ETX2) {
			getETX = 1;
		}
	}
	if (reverse_done) {
		return (NOT_GET_ETX);
	}
	
reverse:
	if (!reverse_done) { /* reverse direction */
			for (i=0; i<(trackInLen>>1); i++) {
			tmp = trackIn[i];
			trackIn[i] = reversal_32bit(trackIn[trackInLen-1-i]);
			trackIn[trackInLen-1-i] = reversal_32bit(tmp);
		}
		if ((trackInLen&1) == 1) {
			tmp = trackIn[trackInLen>>1];
			trackIn[trackInLen>>1] = reversal_32bit(tmp);
		}
		reverse_done = 1;
		goto repeat;
	}
	else {
		return (NOT_GET_STX);
	}
}

/*
**	trackBuf[out] where decode data to be stored;
**		trackBuf[0]: track validate flag,
**			bit0: set 1 means that track1 decode succuss
**			bit1: set 1 means that track2 decode succuss
**			bit2: set 1 means that track3 decode succuss**			bit4: set 1 means that track1 data presents
**			bit5: set 1 means that track2 data presents
**			bit6: set 1 means that track3 data presents
**		trackBuf[1]: valid data length amount in bytes for track1
**		trackBuf[2]: valid data length amount in bytes for track2
**		trackBuf[3]: valid data length amount in bytes for track3
**		trackBuf[4...]: valid data, stored with (track1 + track2 + track3)**    trackBuf: trackBuf[0] + trackBuf[1] + trackBuf[2] + trackBuf[3] + '%' + track1 + '?' + ';' + track2 + '?' + ';' + track3 + '?' **    note: the length of trackBuf >= 10
** RETURN:
**   the length of trackBuf
**   
*/
#define DECODE_AGAIN	0xff
uint32_t magc_decode(uint8_t *trackBuf, uint32_t trackBufLen)
{
	int32_t retVal;
	uint32_t index=4;
	uint32_t temp = 0;
	uint8_t i;

decode_again:	index = 4;
	for (i=0; i<4; i++) {
		trackBuf[i] = 0;
	}

	if (TRACK_N_VALID(track1)) {		trackBuf[0] |= 0x10;
		retVal = track1_decode(&tdata[track1].data[0], tdata[track1].bits_count/32 + 1, trackBuf+index, trackBufLen);
		if (retVal < 0) {
			trackBuf[1] = 2;
			trackBuf[index++] = '%';
			trackBuf[index++] = '?';
		}
		else {
			index += retVal;
			trackBuf[0] |= 0x01;
			trackBuf[1] = retVal;
		}
	}
	else {
		trackBuf[1] = 2;
		trackBuf[index++] = '%';
		trackBuf[index++] = '?';
	}

	if (TRACK_N_VALID(track2)) {		trackBuf[0] |= 0x20;
		retVal = track23_decode(&tdata[track2].data[0], tdata[track2].bits_count/32 + 1, trackBuf+index, trackBufLen-index);
		if (retVal < 0) {
		trackBuf[2] = 2;
			trackBuf[index++] = ';';
			trackBuf[index++] = '?';
		}
		else {
			index += retVal;
			trackBuf[0] |= 0x02;
			trackBuf[2] = retVal;
		}
	}
	else {
	trackBuf[2] = 2;
		trackBuf[index++] = ';';
		trackBuf[index++] = '?';
	}

	if (TRACK_N_VALID(track3)) {		trackBuf[0] |= 0x40;
		retVal = track23_decode(&tdata[track3].data[0], tdata[track3].bits_count/32 + 1, trackBuf+index, trackBufLen-index);
		if (retVal < 0) {
		trackBuf[3] = 2;
			trackBuf[index++] = ';';
			trackBuf[index++] = '?';
		}
		else {
			index += retVal;
			trackBuf[0] |= 0x04;
			trackBuf[3] = retVal;
		}
	}
	else {
		trackBuf[3] =2;
		trackBuf[index++] = ';';
		trackBuf[index++] = '?';
	}
	if (trackBuf[0]&0x05)	/* track1 or track3 decode successful */
		trackAdaptDone = 1;

	if ((trackAdaptDone!=1)
		&& (track_mask&(1<<track1))
		&& (track_mask&(1<<track3)))
	{
		pr_debug(  "switch track 1&3[%d]\n", track_mask);
		temp = track1;
		track1 = track3;
		track3 = temp;
//		track_mask	// update?
		trackAdaptDone = DECODE_AGAIN;
		goto decode_again;
	}
	track_flags = 0;
	return index;
}


static ssize_t magc_read(struct file *filp, char *buf, size_t count, loff_t *ppos)
{
#define TRACK_LEN 250
	unsigned char trackBuf[TRACK_LEN];
	int len;
	int ret;
	while (!TRACK_OK(track_flags))  {  
		if(filp->f_flags & O_NONBLOCK)  
			return -EAGAIN;  
		pr_debug("wait_event_interruptible before\n");
		wait_event_interruptible(inq, TRACK_OK(track_flags));
		pr_debug("wait_event_interruptible \n");
		if(!TRACK_OK(track_flags)) {
			pr_debug(  "track data does not ready - 0x%04x!\n", track_flags);
			return 0;
		}
	}
	if(read_pulse) {
		size_t n;
		char *p = buf;
		read_pulse = 0;	// restored
		if(TRACK_N_OK(track1, track_flags)) {
			n = (pulseBuf[track1].count+1)*sizeof(uint16_t);
		} else {
			n = 2;
			pulseBuf[track1].count = 0;
		}
		if(count<n) return 0;
		ret = copy_to_user(p, (void*)&pulseBuf[track1], n);
		count -= n;
		p += n;
		if(TRACK_N_OK(track2, track_flags)) {
			n = (pulseBuf[track2].count+1)*sizeof(uint16_t);
		} else {
			n = 2;
			pulseBuf[track2].count = 0;
		}
		if(count<n) return 0;
		ret = copy_to_user(p, (void*)&pulseBuf[track2], n);
		count -= n;
		p += n;
		if(TRACK_N_OK(track3, track_flags)) {
			n = (pulseBuf[track3].count+1)*sizeof(uint16_t);
		} else {
			n = 2;
			pulseBuf[track3].count = 0;
		}
		if(count<n) return 0;
		ret = copy_to_user(p, (void*)&pulseBuf[track3], n);
		//count -= n;
		p += n;
		len = p - buf;
		track_flags = 0;
	} else {

		uint32_t i, j;

		for (j=0; j<3; j++) {
		  pr_debug( "pulseBuf[%d]: ", j);
			for (i=0; i<pulseBuf[j].count; i++) {
			 pr_debug( "%d ", pulseBuf[j].data[i]);   
			}
			pr_debug( "\n");
		}
			  
		magc_bits_decode();
		for (j=0; j<3; j++) {
			pr_debug( "tdata[%d]: ", j);
			for (i=0; i<tdata[j].bits_count/32 + 1; i++) {
			 pr_debug( "0x%x ", tdata[j].data[i]);   
			}
			pr_debug( "\n");
		}

		len = magc_decode(trackBuf, TRACK_LEN);
		pr_debug( "Track decoded %02x, [%d, %d, %d]\n", trackBuf[0],
					trackBuf[1], trackBuf[2], trackBuf[3]);

		if (count < len) len = count;
		ret = copy_to_user(buf, (void*)trackBuf, len);
	}
	pr_debug("end read \n");
	return len;
}


unsigned int magc_poll(struct file * filp, poll_table * wait) 
{
	poll_wait(filp, &inq,  wait);

	if (TRACK_OK(track_flags)) {
		return POLLIN | POLLRDNORM;
	}
	return 0;
}
#define MAG_IOC_MAGIC		10
#define MAGC_IOC_START		_IO(MAG_IOC_MAGIC, 1)
#define MAGC_IOC_SET_GAIN 	_IO(MAG_IOC_MAGIC, 2)
#define MAGC_IOC_SET_REF 	_IO(MAG_IOC_MAGIC, 3)





static long magc_ioctl(struct file *file,
		     unsigned int cmd, unsigned long arg)
{
	switch (cmd) {
		case MAGC_IOC_START:
			pr_debug(  "magc start: %x\n", track_flags);
			if(track_flags && !TRACK_OK(track_flags)) {
				pr_debug( "magc busy: %x\n", track_flags);
				return -EBUSY;
			}
#define READ_PULSE_MARK		0x5aa5
#define READ_PULSE_MARK1	0xa55a5aa5
			read_pulse = (uint32_t)arg == READ_PULSE_MARK ? 1 : 0;
			if((uint32_t)arg == READ_PULSE_MARK1)
				read_pulse = 1;
			else
				track_flags = 0;
			
			MAGC_INT_CLR();
			MAGC_INT_ENABLE();
			break;
		default:
			return -ENOTTY;
	}
	return 0;
}

static int magc_close(struct inode* inode, struct file* filp)
{
    track_flags = 0;
    MAGC_INT_CLR();
    MAGC_INT_DISABLE();	
    return 0;
}

static struct file_operations magc_fops =  
{  
	.owner	= THIS_MODULE,
	.read	= magc_read,
	.poll	= magc_poll,
	.unlocked_ioctl	= magc_ioctl,
	.release = magc_close,
};  
  
  
 static struct miscdevice magc_dev = {
	  MISC_DYNAMIC_MINOR,
	  "mxc_magc",
	  &magc_fops
  };

static int magc_get_gpio(struct device *dev, const char* track_gpio_name, struct magc_t * magc_type)
{
	int retVal = EIO;
	int track_gpio;
	u32 track_irq;
	struct device_node *magc_node = dev->of_node;
	
	track_gpio = of_get_named_gpio(magc_node, track_gpio_name, 0);
	
	pr_debug("track_gpio %d\n",  track_gpio);
	if(!gpio_is_valid(track_gpio)){
		pr_err("error get magc gpio\n");
		return retVal;
	}

	magc_type->track_gpio = track_gpio;
	
	if(devm_gpio_request(dev, magc_type->track_gpio, track_gpio_name) < 0){
		pr_err("gpio %d request failed!\n",track_gpio); 
		return retVal; 
	}
	if(gpio_direction_input(magc_type->track_gpio) < 0){
		pr_err("gpio %d direction output failed!\n", track_gpio); 
		return retVal; 
	}
	
	if ((track_irq = gpio_to_irq(magc_type->track_gpio)) < 0 ){
		pr_err("GPIO to IRQ mapping failure \n");    
		return retVal;  
	}
	magc_type->track_irq = track_irq;
	
	irq_set_irq_type(magc_type->track_irq,  IRQF_TRIGGER_RISING|IRQF_TRIGGER_FALLING);
	if (devm_request_irq(dev, magc_type->track_irq , magc_irq_handler , IRQF_DISABLED, "magc",  NULL) < 0) {    
		pr_err("Irq Request failure\n");    
		return retVal; 
	}
	
	return 0;
}

static int magc_probe(struct platform_device *pdev)
{
	int retVal = EIO;
	
	
	const struct of_device_id *of_id =
			of_match_device(magc_imx_dt_ids, &pdev->dev);
	pr_debug("enter %s\n",  __FUNCTION__);
	if (!of_id)
		return 0;
	pulseBuf[0].get_track_data = track0_get_data;
	pulseBuf[1].get_track_data = track1_get_data;
	pulseBuf[2].get_track_data = track2_get_data;

	
	if(magc_get_gpio(&(pdev->dev), "track1-gpios", &magc_data[0]) < 0){

		pr_err("get %s gpio failure\n", "track1-gpios");
		return retVal;
	}
	if(magc_get_gpio(&(pdev->dev), "track2-gpios", &magc_data[1]) < 0){

		pr_err("get %s gpio failure\n", "track2-gpios");
		return retVal;
	}
	if(magc_get_gpio(&(pdev->dev), "track3-gpios", &magc_data[2]) < 0){

		pr_err("get %s gpio failure\n", "track3-gpios");
		return retVal;
	}
	MAGC_INT_CLR();
	MAGC_INT_DISABLE();
	init_waitqueue_head(&inq);
	track_mask = TRACK_MASK_INIT;
	magc_dev.parent = &(pdev->dev);
	misc_register(&magc_dev);
	return 0;
}

static int magc_remove(struct platform_device *pdev)
{

	misc_deregister(&magc_dev);
	return 0;
}



static struct platform_driver magc_driver = {
	.driver = {
			.name = DRIVER_NAME,
			.owner = THIS_MODULE,
			.of_match_table = magc_imx_dt_ids,
			},
	.probe = magc_probe,
	.remove = magc_remove,
};

static int __init magc_drv_init(void)
{
	return platform_driver_register(&magc_driver);
}

static void __exit magc_drv_exit(void)
{
	platform_driver_unregister(&magc_driver);
}

module_init(magc_drv_init);
module_exit(magc_drv_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MXC magc Driver");
MODULE_LICENSE("GPL");

