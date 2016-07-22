#include "stm32f10x_it.h"
#include "trx_rf_spi.h"
#include "cc1120.h"
#include "hal_types.h"
#include "stm32f10x.h" 
#include <stdio.h>
#include <string.h>
#include "cc112x_config.h"

#include "compiler.h"
#include "hal.h"
#include "halStack.h"
#include "console.h"
#include "debug.h"
#include "ieee_lrwpan_defs.h"

/***********LXfor6top***************************/
/*
 *470 -- 480MHz，0--9，10 channels
 */
#define CARRIER_FREQUENCY  470      /* Channel 0: 470Mhz */
#define CHANNEL_SPACING    1        /* Channel spacing: 100kHz  */
#define FREQ_OSC           32       /*  */
#define LO_DIVIDER         8

static uint8_t packet_tx[CC112X_MAX_PAYLOAD];
static uint8_t packet_rx[1 + CC112X_MAX_PAYLOAD + 2];

/******************************************/
//清空接收
static void flushrx(void)
{
  UINT32 send_timer;
  //restart_input();

  LOCK_SPI();
  if(state() == CC11xx_STATE_RXFIFO_OVERFLOW) {
    strobe(CC112X_SFRX);
  }
  strobe(CC112X_SIDLE);
  send_timer = halGetMACTimer();
  while ((state() == CC11xx_STATE_IDLE) 
      && (halMACTimerNowDelta(send_timer) < MSECS_TO_MACTICKS(100)));
  //BUSYWAIT_UNTIL((state() == CC11xx_STATE_IDLE), RTIMER_SECOND / 10);
  strobe(CC11xx_SFRX);
  strobe(CC11xx_SRX);
  RELEASE_SPI();
}

/******************************************/
int
radio_loadPacket(const void *payload, unsigned short len)
{
  if(state() == CC11xx_STATE_RXFIFO_OVERFLOW) {
    flushrx();
  }

  if(len > CC112X_MAX_PAYLOAD) {
#if LOWSN_DEBUG
    printf("CC11xx DEBUG: Too big packet, aborting prepare %d\n", len);
#endif /* DEBUG */
    return RADIO_TX_ERR;
  }

  memcpy(packet_tx, payload, len);
  return RADIO_TX_OK;
}

/**********************************************************/
int
radio_txNow(unsigned char len)
{
  UINT32 send_timer;
  if(state() == CC11xx_STATE_RXFIFO_OVERFLOW) {
    flushrx();
  }

#if LOWSN_DEBUG
  printf("tx len %d\n", len);
#endif /* DEBUG */
  if(len > CC112X_MAX_PAYLOAD) {
#if LOWSN_DEBUG || 1
    printf("cc11xx: too big tx %d\n", len);
#endif /* DEBUG */
    return RADIO_TX_ERR;
  }

  //RIMESTATS_ADD(lltx);

  LOCK_SPI();
  strobe(CC112X_SIDLE);

  cc112xSpiWriteTxFifo((unsigned char *)packet_tx, len);
  send_timer = halGetMACTimer();
  while((state() != CC11xx_STATE_TX)//等待进入发送状态
    && (halMACTimerNowDelta(send_timer) < MSECS_TO_MACTICKS(100)));

  if(state() != CC11xx_STATE_TX) {
#if DEBUG
    printf("didn't tx (in %d)\n", state());
#endif /* DEBUG */
    //check_txfifo();
    flushrx();
    RELEASE_SPI();
    return RADIO_TX_ERR;
  }
  
  RELEASE_SPI();
  send_timer = halGetMACTimer();
  while((state() == CC11xx_STATE_TX)//等待跳出发送状态
        && (halMACTimerNowDelta(send_timer) < MSECS_TO_MACTICKS(100)));
  //BUSYWAIT_UNTIL((state() != CC11xx_STATE_TX), RTIMER_SECOND / 10);

  if(state() == CC11xx_STATE_TX) {
#if DEBUG
    printf("didn't end tx (in %d, txbytes %d)\n", state(), txbytes());
#endif /* DEBUG */
    //check_txfifo();
    flushrx();
    return RADIO_TX_ERR;
  }

  return RADIO_TX_OK;
}
/**********************************************************/
int radio_On(void)
{
  if(SPI_IS_LOCKED()) {
    return 0;
  }
  LOCK_SPI();
  flushrx();
  strobe(CC112X_SRX);
  RELEASE_SPI();
  
  return 1;
}
/**********************************************************/
int radio_Off(void)
{
  UINT32 send_timer;
  
  if(SPI_IS_LOCKED()) {
    return 0;
  }

  LOCK_SPI();
  strobe(CC112X_SIDLE);
  send_timer = halGetMACTimer();
  while((state() != CC11xx_STATE_IDLE)//空闲则跳出
        && (halMACTimerNowDelta(send_timer) < MSECS_TO_MACTICKS(100)));
  //BUSYWAIT_UNTIL((state() == CC11xx_STATE_IDLE), RTIMER_SECOND / 10);
  strobe(CC112X_SPWD);
  RELEASE_SPI();

  return 1;
}
/**********************************************************/
/* XXX Placeholder awaiting new radio_driver API */
signed char radio_readRssi(void)
{
  return read_rssi();
}
/**********************************************************/
/**
 * read RSSI
 */
static signed char read_rssi(void)
{
  uint8_t raw;

  burst_read(CC112X_RSSI1, &raw, 1);
  return rssi_dbm(raw);
}
/**********************************************************/
/* XXX Placeholder awaiting new radio_driver API */
signed char radio_readLqi(void)
{
  return read_lqi();
}
/**********************************************************/
/**
 * rf1a read LQI
 */
static unsigned char read_lqi(void)
{
  unsigned char temp;

  burst_read(CC112X_LQI_VAL, &temp, 1);
  return temp;
}
/**********************************************************/
/**
 * Return CCA bit in packet status register.
 */
static int channel_clear(void)
{
  uint8_t radio_was_off, cca, val;
  UINT32 sendtimer;

  if(SPI_IS_LOCKED()) {
    return 1;
  }

  if(state() == CC11xx_STATE_IDLE ||
     state() == CC11xx_STATE_SLEEP) {
    radio_was_off = 1;
    radio_On();
    send_timer = halGetMACTimer();
    while((state() != CC11xx_STATE_IDLE)//空闲则跳出
          && (halMACTimerNowDelta(send_timer) < MSECS_TO_MACTICKS(100)));
    //BUSYWAIT_UNTIL((state() == CC11xx_STATE_RX), RTIMER_SECOND / 100);
  } else {
    radio_was_off = 0;
  }

  cca = 1; /* clear if no carrier detected */

#define CARRIER_SENSE_VALID   BV(1)
#define CARRIER_SENSE         BV(2)
  
  burst_read(CC112X_RSSI0, &val, 1);
  if(!(val & CARRIER_SENSE_VALID)) {
    cca = 1;
  } else if (val & CARRIER_SENSE) {
    cca = 0;
  } else {
    cca = 1;
  }

  if(radio_was_off) {
    radio_Off();
  }
  
  return cca;
}
/**********************************************************/
void channel_set(uint8_t c)
{
	uint32 freq, f_vco, freq_regs;
	uint8 writeByte;
	UINT16 FREQ_REG[]={CC112X_FREQ0,CC112X_FREQ1,CC112X_FREQ2};
	
	if(c>9) return; 
	
	freq = CARRIER_FREQUENCY + c*CHANNEL_SPACING; /* Channel -> radio frequency */
	f_vco = freq*LO_DIVIDER; /* Radio frequency -> VCO frequency */
	freq_regs = f_vco<<16; /* Multiply by 2^16 */
	freq_regs /= FREQ_OSC; /* Divide by oscillator frequency -> Frequency registers */
	
	for(int i=0; i<3; i++){
		writeByte = ((unsigned char*)&freq_regs)[i];
		cc112xSpiWriteReg(FREQ_REG[i], &writeByte, 1);
	}
	delay_nms(1);
	
	for(int i=0; i<3; i++){
		writeByte = ((unsigned char*)&freq_regs)[i];
		cc112xSpiWriteReg(FREQ_REG[i], &writeByte, 1);
	}
}

#if 0

void radio_txNow() {

  Enable_IRQ_ISR();
  
  trxSpiCmdStrobe(CC112X_STX);
  
  Disable_IRQ_ISR();
}

void radio_rxEnable() {
  
  Enable_IRQ_ISR();
  
  trxSpiCmdStrobe(CC112X_STX);
  //SFD、完成中断

  CC1120_EXTI_Enable();

  Disable_IRQ_ISR();

}

void getRcvData(uint8_t* ptr,uint8_t *pFlen,INT8* pRssi,BOOL* pCrc){


}
#endif