/****************************************
* \file     LoraSX1276_Driver.h
* \brief    Header file for the Lora SX1276 Chipset
* \date     Jan 2021
* \author   RJAM
****************************************/

#ifndef LORA_SX1276_H
#define LORA_SX1276_H

/********* Includes ********************/
#include "esp_types.h"
#include "esp_err.h"
#include "sdkconfig.h"

#include "./sx_fsk_definitions.h"
#include "./sx_lora_definitions.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
/********* Definitions *****************/

/** registers have overlapping addresses as function changes 
 *  between LoRa / fsk/ook mode  
 **/

#include "CommandAPI.h"
#define LORA_PERIPH_LEN 26

extern const parameter_t lora_parameter_map[LORA_PERIPH_LEN];
extern const peripheral_t lora_peripheral_template;



/** Driver defines **/

#define SX_READWRITE_BIT (1 << 7)
#define SX_WRITE 1
#define SX_READ  0
#define SX_SPI_TIMEOUT_DEFAULT 500

#define SX1276_LORA_FREQ_EURO 868000000UL
// this is the highest possible frq value based on 
#define SX_LORA_MAX_FREQUENCY 1024000000UL

#define SX_FRQ_HERTZ_PER_REGCOUNT   61.035



/** TODONE: put these in a couple of arrays or prefix lora/fsk 
 ***/ 

#define SX1276_REGADDR_REGFIFO 0x00
#define SX1276_REGADDR_OPMODE  0x01
#define SX1276_REGADDR_BITRATE_MSB 0x02
#define SX1276_REGADDR_BITRATE_LSB 0x03
#define SX1276_REGADDR_DEV_MSB 0x04
#define SX1276_REGADDR_DEV_LSB  0x05
#define SX1276_REGADDR_CARRFREQ_MSB 0x06
#define SX1276_REGADDR_CARRFREQ_MIDB 0x07
#define SX1276_REGADDR_CARRFREQ_LSB 0x08
#define SX1276_REGADDR_PA_CONFIG 0x09
#define SX1276_REGADDR_PA_RAMP   0x0A
#define SX1276_REGADDR_OCURRENT_PROT 0x0B
#define SX1276_REGADDR_LNA_CONFIG 0x0C
#define SX1276_REGADDR_RX_CONFIG 0x0D
#define SX1276_REGADDR_FIFOADDR_PTR 0x0D
#define SX1276_REGADDR_RSSI_CONFIG 0x0E
#define SX1276_REGADDR_FIFO_TXBASE 0x0E
#define SX1276_REGADDR_RSSI_COLLSN 0x0F
#define SX1276_REGADDR_FIFO_RXBASE 0x0F
#define SX1276_REGADDR_RSSI_THRESH 0x10
#define SX1276_REGADDR_FIFO_RXCURR 0x10
#define SX1276_REGADDR_RSSI_VALUE 0x11
#define SX1276_REGADDR_IRQFLAGS_MASK  0x11
#define SX1276_REGADDR_RX_BW 0x12
#define SX1276_REGADDR_IRQ_FLAGS 0x12
#define SX1276_REGADDR_AFC_BW 0x13
#define SX1276_REGADDR_RX_N_BYTES 0x13
#define SX1276_REGADDR_OOK_PEAK 0x14
#define SX1276_REGADDR_RXHDR_CNT_MSB 0x14
#define SX1276_REGADDR_OOK_FIX 0x15
#define SX1276_REGADDR_RXHDR_CNT_LSB 0x15
#define SX1276_REGADDR_OOK_AVG 0x16
#define SX1276_REGADDR_RXPKT_CNT_MSB 0x16
#define SX1276_REGADDR_RXPKT_CNT_LSB 0x17
#define SX1276_REGADDR_MODEM_STAT 0x18
#define SX1276_REGADDR_PKT_SNR_VAL 0x19
#define SX1276_REGADDR_PKT_RSSI_VAL 0x1A
#define SX1276_REGADDR_AFC_MSB  0x1B
#define SX1276_REGADDR_RSSI_VAL 0x1B
#define SX1276_REGADDR_AFC_LSB  0x1C
#define SX1276_REGADDR_HOPCHANNEL 0x1C
#define SX1276_REGADDR_FEI_MSB_FKSMODE  0x1D
#define SX1276_REGADDR_MODEM_CONFIG1 0x1D
#define SX1276_REGADDR_FEI_LSB_FSKMODE  0x1E
#define SX1276_REGADDR_MODEM_CONFIG2 0x1E
#define SX1276_REGADDR_PREAMBLE_DETECT 0x1F
#define SX1276_REGADDR_SYMB_TIMEOUT_LSB 0x1F
#define SX1276_REGADDR_RX_TIMEOUT1  0x20
#define SX1276_REGADDR_PREAMBLE_MSB_LORAMODE 0x20
#define SX1276_REGADDR_RX_TIMEOUT2 0x21
#define SX1276_REGADDR_PREAMBLE_LSB_LORAMODE 0x21
#define SX1276_REGADDR_RX_TIMEOUT3 0x22
#define SX1276_REGADDR_PAYLOAD_LEN 0x22
#define SX1276_REGADDR_RX_DELAY 0x23
#define SX1276_REGADDR_MAX_PAYLOAD_LEN 0x23
#define SX1276_REGADDR_OSC      0x24
#define SX1276_REGADDR_HOP_PERIOD 0x24
#define SX1276_REGADDR_PREAMBLE_MSB_FSKMODE 0x25
#define SX1276_REGADDR_FIFO_RXBYTE 0x25
#define SX1276_REGADDR_PREAMBLE_LSB_FSKMODE 0x26
#define SX1276_REGADDR_MODEM_CONFIG3 0x26
#define SX1276_REGADDR_SYNC_CONFIG 0x27
#define SX1276_REGADDR_SYNC_VAL1 0x28
#define SX1276_REGADDR_FEI_MSB_LORAMODE 0x28
#define SX1276_REGADDR_SYNC_VAL2 0x29
#define SX1276_REGADDR_FEI_MIDB 0x29
#define SX1276_REGADDR_SYNC_VAL3 0x2A
#define SX1276_REGADDR_FEI_LSB_LORAMODE 0x2A
#define SX1276_REGADDR_SYNC_VAL4 0x2B
#define SX1276_REGADDR_SYNC_VAL5 0x2C
#define SX1276_REGADDR_RSSI_WIDEBAND 0x2C
#define SX1276_REGADDR_SYNC_VAL6 0x2D
#define SX1276_REGADDR_SYNC_VAL7 0x2E
#define SX1276_REGADDR_SYNC_VAL8 0x2F
#define SX1276_REGADDR_PKT_CONFIG1 0x30
#define SX1276_REGADDR_PKT_CONFIG2 0x31
#define SX1276_REGADDR_DETECT_OPMZ 0x31
#define SX1276_REGADDR_FSK_PAYLOAD_LEN 0x32
#define SX1276_REGADDR_NODE_ADDR 0x33
#define SX1276_REGADDR_INVERT_IQ 0x33
#define SX1276_REGADDR_BROADCAST_ADDR 0x34
#define SX1276_REGADDR_FSKFIFO_THRESH 0x35
#define SX1276_REGADDR_SEQ_CONFIG1 0x36
#define SX1276_REGADDR_SEQ_CONFIG2 0x37
#define SX1276_REGADDR_DETECT_THRESH 0x37
#define SX1276_REGADDR_TIMER_RES 0x38
#define SX1276_REGADDR_TIMER1_COEF 0x39
#define SX1276_REGADDR_SYNC_WORD 0x39
#define SX1276_REGADDR_TIMER2_COEF 0x3A
#define SX1276_REGADDR_IMAGE_CAL 0x3B
#define SX1276_REGADDR_TEMP 0x3C
#define SX1276_REGADDR_LOWBAT 0x3D
#define SX1276_REGADDR_IRQFLAGS1 0x3E
#define SX1276_REGADDR_IRQFLAGS2 0x3F
#define SX1276_REGADDR_DIO_MAP1 0x40
#define SX1276_REGADDR_DIO_MAP2 0x41
#define SX1276_REGADDR_VERSION 0x42
#define SX1276_REGADDR_PLL_HOP 0x44
#define SX1276_REGADDR_TXCO 0x4B
#define SX1276_REGADDR_PA_DAC 0x4D
#define SX1276_REGADDR_FORMER_TEMP 0x5B
#define SX1276_REGADDR_BITRATE_FRAC 0x5D
#define SX1276_REGADDR_AGC_REF 0x61
#define SX1276_REGADDR_AGC_THRESH1 0x62
#define SX1276_REGADDR_AGC_THRESH2 0x63
#define SX1276_REGADDR_AGC_THRESH3 0x64
#define SX1276_REGADDR_PLL  0x70



#define SX1276_REGADDR_REGFIFO_DEFAULT      0x00
#define SX1276_REGADDR_OPMODE_DEFAULT       0x01
#define SX1276_REGADDR_BITRATE_MSB_DEFAULT  0x1A
#define SX1276_REGADDR_BITRATE_LSB_DEFAULT  0x0B
#define SX1276_REGADDR_DEV_MSB_DEFAULT      0x00
#define SX1276_REGADDR_DEV_LSB_DEFAULT      0x52
#define SX1276_REGADDR_CARRFREQ_MSB_DEFAULT 0x6C
#define SX1276_REGADDR_CARRFREQ_MIDB_DEFAULT    0x80
#define SX1276_REGADDR_CARRFREQ_LSB_DEFAULT     0x00
#define SX1276_REGADDR_PA_CONFIG_DEFAULT        0x4F
#define SX1276_REGADDR_PA_RAMP_DEFAULT          0x09
#define SX1276_REGADDR_OCURRENT_PROT_DEFAULT    0x2B
#define SX1276_REGADDR_LNA_CONFIG_DEFAULT       0x20
#define SX1276_REGADDR_RX_CONFIG_DEFAULT        0x0E
#define SX1276_REGADDR_FIFOADDR_PTR_DEFAULT     0x08
#define SX1276_REGADDR_RSSI_CONFIG_DEFAULT      0xFF
#define SX1276_REGADDR_FIFO_TXBASEADDR_DEFAULT  0x02
#define SX1276_REGADDR_RSSI_COLLSN_DEFAULT      0x0A
#define SX1276_REGADDR_FIFO_RXBASEADDR_DEFAULT  0x0A
#define SX1276_REGADDR_RSSI_THRESH_DEFAULT      0xFF
#define SX1276_REGADDR_FIFO_RXCURR_DEFAULT      0xFF
#define SX1276_REGADDR_RSSI_VALUE_DEFAULT       0x00
#define SX1276_REGADDR_IRQFLAGS_MASK_DEFAULT    0x00
#define SX1276_REGADDR_RX_BW_DEFAULT            0x15
#define SX1276_REGADDR_IRQ_FLAGS_DEFAULT        0x15
#define SX1276_REGADDR_AFC_BW_DEFAULT           0x0B
#define SX1276_REGADDR_RX_N_BYTES_DEFAULT       0x0B
#define SX1276_REGADDR_OOK_PEAK_DEFAULT         0x28
#define SX1276_REGADDR_RXHDR_CNT_MSB_DEFAULT    0x28




/** REG MODE BITS **/
#define SX1276_LORA_MODE_BIT (1 << 7)
#define SX1276_ACCESS_SHARED_REGS_BIT (1 << 6)
#define SX1276_LOWFREQ_MODE_ON_BIT (1 << 3)

/** REG OCP BITS **/
#define SX1276_OCP_ON_BIT (1 << 5)

/** PA CONFIG BITS **/
#define SX1276_PA_SELECT_BIT (1 << 7)

/** IRQ FLAGS MASK **/
#define SX1276_RXTO_MASK_BIT (1 << 7)
#define SX1276_RXDONE_MASK_BIT (1 << 6)
#define SX1276_PAYLOAD_CRC_ERR_MASK_BIT (1 << 5)
#define SX1276_VALID_HDR_MASK_BIT (1 << 4)
#define SX1276_TXDONE_MASK_BIT (1 << 3)
#define SX1276_CALDONE_MASK_BIT (1 << 2)
#define SX1276_FHSS_CHG_CHAN_MASK_BIT (1 << 1)
#define SX1276_CAD_DETECT_MASK_BIT (1)

/** IRQ FLAGS LORA BITS **/
#define SX1276_RXTO_BIT (1 << 7)
#define SX1276_RXDONE_BIT (1 << 6)
#define SX1276_PAYLOADCRC_ERR_BIT (1 << 5)
#define SX1276_VALID_HDR_BIT (1 << 4)
#define SX1276_TXDONE_BIT (1 << 3)
#define SX1276_CAD_DONE_BIT (1 << 2)
#define SX1276_FHSS_CHG_CHANNEL_BIT (1 << 1)
#define SX1276_CAD_DETECT_BIT (1)

/** MODEM STATUS BITS **/
#define SX1276_MODEM_CLR_BIT (1 << 4)
#define SX1276_HDR_INFO_VALID_BIT (1 << 3)
#define SX1276_RX_ONGOING_BIT (1 << 2)
#define SX1276_SIGNAL_SYNCR_BIT (1 << 1)
#define SX1276_SIGNAL_DETECT_BIT (1)

/** HOP CHANNEL BITS **/
#define SX1276_PLL_TIMEOUT_BIT (1 << 7)
#define SX1276_CRC_ON_PAYLOAD_BIT (1 << 6)

/** MODEM CONFIG1 BITS **/
#define SX1276_IMPLICIT_HDR_MODE_ON_BIT (1)

/** MODEM CONFIG2 BITS **/
#define SX1276_TX_CONT_MODE_BIT (1 << 3)
#define SX1276_RX_PAYLOAD_CRC_ON_BIT (1 << 2)

/** MODEM CONFIG3 BITS **/
#define SX1276_LOW_DATARATE_OPT_BIT (1 << 3)
#define SX1276_AGC_AUTO_ON_BIT (1 << 2)

/** INVERT IQ **/
#define SX1276_INVERT_IQ_BIT (1 << 6)

/** RX_CONFIG BITS **/
#define SX1276_RST_RX_ON_COLL_BIT (1 << 7)
#define SX1276_RST_RX_WO_PLL_BIT (1 << 6)
#define SX1276_RST_RX_W_PLL_BIT (1 << 5)
#define SX1276_AFC_AUTO_BIT (1 << 4)
#define SX1276_AGC_AUTO_BIT (1 << 3)

/** AFC FEI BITS **/
#define SX1276_AGC_START_BIT (1 << 4)
#define SX1276_AFC_CLR_BIT (1 << 1)
#define SX1276_AFC_AUTOCLR_BIT (1)

/** OSC BITS **/
#define SX1276_RC_CAL_START_BIT (1 << 3)

/** SYNC CONFIG BITS **/
#define SX1276_PTRAMBLE_POL_BIT (1 << 5)
#define SX1276_SYNC_ON_BIT      (1 << 4)

/** PKT CONFIG BITS **/
#define SX1276_PKT_FORMAT_BIT (1 << 7)
#define SX1276_CRC_ON_BIT (1 << 4)
#define SX1276_CRC_AUTOCLR_OFF_BIT (1 << 3)
#define SX1276_CRC_WHITENING_BIT (1)

/** PKT CONFIG 2 BITS **/
#define SX1276_DATA_MODE_BIT (1 << 6)
#define SX1276_IO_HOMECTRL_ON_BIT (1 << 5)
#define SX1276_BEACON_ON_BIT (1 << 3)

/** FIFO THRESH BITS **/
#define SX1276_TX_STARTCOND_BIT (1 << 7)

/** SEQ CONFIG1 BITS **/
#define SX1276_SEQ_START_BIT (1 << 7)
#define SX1276_SEQ_STOP_BIT (1 << 6)
#define SX1276_IDLE_MODE_BIT (1 << 5)
#define SX1276_LOWPOW_SLECT_BIT (1 << 2)
#define SX1276_FROM_IDLE_BIT (1 << 1)
#define SX1276_FROM_TRANSMIT_BIT (1)

/** IMAGE CAL BITS **/
#define SX1276_AUTO_IMG_CAL_ON_BIT (1 << 7)
#define SX1276_IMG_CAL_START_BIT (1 << 6)
#define SX1276_IMG_CAL_RUNNING_BIT (1 << 5)
#define SX1276_TEMP_CHANGE_BIT  (1 << 3)
#define SX1276_TEMP_MON_OFF_BIT (1)

/** LOW BATT BITS **/
#define SX1276_LOW_BAT_ON_BIT (1 << 3)

/** IRQ FLAGS1 BITS **/
#define SX1276_MODE_READY_BIT (1 << 7)
#define SX1276_RX_READY_BIT (1 <<  6)
#define SX1276_TX_READY_BIT (1 << 5)
#define SX1276_PLL_LOCK_BIT (1 << 4)
#define SX1276_RSSI_BIT (1 << 3)
#define SX1276_TIMEOUT_BIT (1 << 2)
#define SX1276_PREAMBLE_DETECT_BIT (1 << 1)
#define SX1276_SYNCADDR_MATCH_BIT (1)

/** IRQ FLAGS2 BITS **/
#define SX1276_FIFO_FULL_BIT (1 << 7)
#define SX1276_FIFO_EMPTY_BIT (1 << 6)
#define SX1276_FIFO_LVL_BIT (1 << 5)
#define SX1276_FIFO_OVR_BIT (1 << 4)
#define SX1276_PKT_SENT_BIT (1 << 3)
#define SX1276_PAYLOAD_RDY_BIT (1 << 2)
#define SX1276_CRC_OK_BIT (1 << 1)
#define SX1276_LOWBAT_BIT (1)

/** DIO MAPPING2 BITS **/
#define SX1276_MAP_PREAMBLE_DETECT_BIT (1)

/** PLL HOP BITS **/
#define SX1276_FAST_HOP_ON_BIT (1 << 7)

/** TXCO BITS **/
#define SX1276_TXCO_INPUTON_BIT (1 << 4)

/** LNA BITS **/
#define SX1276_LNA_BOOST_EN 0x3

/** HIGH PWR **/
#define RE_PA_STD_PWR 0x84
#define RE_PA_HP_ENABLE 0x87


/********** Types **********************/



typedef enum {
    SX_DEVICE_FSK_MODE,
    SX_DEVICE_LORA_MODE,
} sx_device_mode_t;

typedef enum {
    SX1276_TRXMODE_SLEEP,  /** clears fifo, switch lora/fsk **/
    SX1276_TRXMODE_STDBY,  /** large bits off **/
    SX1276_TRXMODE_FS_TX,  /** freq synth tx, pll on, rf off **/
    SX1276_TRXMODE_TX,     /** rf on, tx, then standby **/
    SX1276_TRXMODE_FS_RX,  /** above, but rx **/
    SX1276_TRXMODE_RX_CONT, /** rx until told to stop **/
    SX1276_TRXMODE_RX_SGL,  /** rx a single packet, then stdby **/
    SX1276_TRXMODE_CAD,    /** check chan for lora preamble **/
} sx_trxmode_t;            

typedef enum {
    SX1276_PA_OUTPUT_RFO,
    SX1276_PA_OUTPUT_PABOOST,
} pa_select_t;

typedef enum {
    SX1276_MODSHAPING_NONE,
    SX1276_MODSHAPING_GFBT_1,
    SX1276_MODSHAPING_GFBT_05,
    SX1276_MODSHAPING_GFBT_03
} fsk_modshaping_t;

typedef enum {
    SX1276_MODSHAPING_ZERO,
    SX1276_MODSHAPING_FCUTOFF_BR,
    SX1276_MODSHAPING_FCUTOFF_2BR,
} ook_modshaping_t;

typedef enum {
    SX1276_MAX_GAIN = 0x01,
    SX1276_6DB_GAIN,
    SX1276_12DB_GAIN,
    SX1276_24DB_GAIN,
    SX1276_36DB_GAIN,
    SX1276_48DB_GAIN,
} lna_gain_t;

typedef enum {
    SX1276_RSSI_SMOOTH_2SAMPLES,
    SX1276_RSSI_SMOOTH_4SAMPLES,
    SX1276_RSSI_SMOOTH_8SAMPLES,
    SX1276_RSSI_SMOOTH_16SAMPLES,
    SX1276_RSSI_SMOOTH_32SAMPLES,
    SX1276_RSSI_SMOOTH_64SAMPLES,
    SX1276_RSSI_SMOOTH_128SAMPLES,
    SX1276_RSSI_SMOOTH_256SAMPLES,
} rssi_smoothing_t;


typedef enum {
    SX1276_LORA_BW_7_8KHZ,
    SX1276_LORA_BW_10_4KHZ,
    SX1276_LORA_BW_15_6KHZ,
    SX1276_LORA_BW_20_8KHZ,
    SX1276_LORA_BW_31_25KHZ,
    SX1276_LORA_BW_41_7KHZ,
    SX1276_LORA_BW_62_5KHZ,
    SX1276_LORA_BW_125KHZ,
    SX1276_LORA_BW_250KHZ,
    SX1276_LORA_BW_500KHZ,
    SX1276_LORA_BW_MAX,
} lora_bw_t;

typedef enum {
    SX1276_SPREADF_MIN = 0x05,
    SX1276_SPREADF_64_CS,
    SX1276_SPREADF_128_CS,
    SX1276_SPREADF_256_CS,
    SX1276_SPREADF_512_CS,
    SX1276_SPREADF_1024_CS,
    SX1276_SPREADF_2048_CS,
    SX1276_SPREADF_4096_CS,
    SX1276_SPREADF_MAX,
} lora_spread_fac_t;


typedef enum {
    SX1276_CODING_RATE_4_5 = 0x01,
    SX1276_CODING_RATE_4_6,
    SX1276_CODING_RATE_4_7,
    SX1276_CODING_RATE_4_8,
    SX1276_CODING_RATE_MAX,
} lora_coding_rate_t;

typedef enum {
    SX1276_RX_BW_MANT_16,
    SX1276_RX_BW_MANT_20,
    SX1276_RX_BW_MANT_24,
} channel_filter_bw_t;


typedef enum {
    SX1276_OOK_THRESH_FIXED,
    SX1276_OOK_THRESH_AVG,
    SX1276_OOK_THRESH_PEAK,
} ook_threshold_t;


typedef enum {
    SX1276_PEAK_THRESH_STEP_05DB,
    SX1276_PEAK_THRESH_STEP_1DB,
    SX1276_PEAK_THRESH_STEP_1_5DB,
    SX1276_PEAK_THRESH_STEP_2DB,
    SX1276_PEAK_THRESH_STEP_3DB,
    SX1276_PEAK_THRESH_STEP_4DB,
    SX1276_PEAK_THRESH_STEP_5DB,
    SX1276_PEAK_THRESH_STEP_6DB,
} ook_peak_thresh_step_t;

typedef enum {
    SX1276_OOK_PEAK_THRESH_DEC_1,
    SX1276_OOK_PEAK_THRESH_DEC_0_5,
    SX1276_OOK_PEAK_THRESH_DEC_0_25,
    SX1276_OOK_PEAK_THRESH_DEC_0_125,
    SX1276_OOK_PEAK_THRESH_DEC_2,
    SX1276_OOK_PEAK_THRESH_DEC_4,
    SX1276_OOK_PEAK_THRESH_DEC_8,
    SX1276_OOK_PEAK_THRESH_DEC_16,
} ook_peak_thresh_dec_t;

typedef enum {
    SX1276_OOK_AVG_THRESH_FIT_CR_32PI,
    SX1276_OOK_AVG_THRESH_FIT_CR_8PI,
    SX1276_OOK_AVG_THRESH_FIT_CR_4PI,
    SX1276_OOK_AVG_THRESH_FIT_CR_2PI,
} ook_avg_thresh_fit_t;


typedef enum {
    SX1276_PREAMBLE_DETECT_1B,
    SX1276_PREAMBLE_DETECT_2B,
    SX1276_PREAMBLE_DETECT_3B,
} preamble_detect_size_t;

typedef enum {
    SX1276_CLKOUT_FRQ_FXOSC,
    SX1276_CLKOUT_FRQ_FXOSC_DIV2,
    SX1276_CLKOUT_FRQ_FXOSC_DIV4,
    SX1276_CLKOUT_FRQ_FXOSC_DIV8,
    SX1276_CLKOUT_FRQ_FXOSC_DIV16,
    SX1276_CLKOUT_FRQ_FXOSC_DIV32,
} osc_clkout_freq_t;


typedef enum {
    SX1276_AUTORST_RX_MODE_OFF,
    SX1276_AUTORST_RX_MODE_ON_NOWAIT,
    SX1276_AUTORST_RX_MODE_ON_WAITPLL
} autorst_rx_mode_t;


typedef enum {
    SX1276_DCFREE_NONE,
    SX1276_DCFREE_MANCHESTER,
    SX1276_DCFREE_WHITENING,
} dcfree_mode_t;

typedef enum {
    SX1276_ADDR_FILTER_NONE,
    SX1276_ADDR_FILTER_NODE_ONLY,
    SX1276_ADDR_FILTER_NODE_OR_BROADCAST,
} address_filtering_t;


typedef enum {
    SX1276_SEQ_FROM_START_LOWPOWSEL,
    SX1276_SEQ_FROM_START_RX_STATE,
    SX1276_SEQ_FROM_START_TX_STATE,
    SX1276_SEQ_FROM_START_TX_FIFO_IRQ,
} seq_from_start_t;

typedef enum {
    SX1276_SEQ_FROM_IDLE_RX_STATE,
    SX1276_SEQ_FROM_IDLE_TX_STATE,
} seq_from_idle_t;

typedef enum {
    SX1276_SEQ_FROM_TX_RX_STATE,
    SX1276_SEQ_FROM_TX_TX_STATE,
} seq_from_tx_t;

typedef enum {
    SX1276_SEQ_FROM_RXRCV_PKTRCVD_PLRDY_IRQ = 0x01,
    SX1276_SEQ_FROM_RXRCV_LOWPOW_SEL_PLRDY_IRQ,
    SX1276_SEQ_FROM_RXRCV_PKTRCVD_CRCOK,
    SX1276_SEQ_FROM_RXRCV_SEQOFF_RSSI_IRQ,
    SX1276_SEQ_FROM_RXRCV_SEQOFF_SYNCADDR_IRQ,
    SX1276_SEQ_FROM_RXRCV_SEQOFF_PREAMB_IRQ,
} seq_from_rxrcv_t;


typedef enum {
    SX1276_SEQ_FROM_RXTO_RECVSTATE,
    SX1276_SEQ_FROM_RXTO_TXSTATE,
    SX1276_SEQ_FROM_RXTO_LOWPOW_SEL,
    SX1276_SEQ_FROM_RXTO_SEQOFF
} seq_from_rx_tmout_t;


typedef enum {
    SX1276_SEQ_FROM_PKTRCV_SEQOFF,
    SX1276_SEQ_FROM_PKTRCV_TXSTATE_FIFOEMPTY_IRQ,
    SX1276_SEQ_FROM_PKTRCV_LOWPOW_SEL,
    SX1276_SEQ_FROM_PKTRCV_RCV_FSMODE,
    SX1276_SEQ_FROM_PKTRCV_RXSTATE,
} seq_from_pktrcv_t;

typedef enum {
    SX1276_TIMER_RES_DISABLED,
    SX1276_TIMER_RES_64MICRO,
    SX1276_TIMER_RES_4_1MILLIS,
    SX1276_TIMER_RES_262MILLIS,
} timer_resolution_t;


typedef enum {
    SX1276_TEMP_THRESH_5DEG,
    SX1276_TEMP_THRESH_10DEG,
    SX1276_TEMP_THRESH_15DEG,
    SX1276_TEMP_THRESH_20DEG,
} temp_thresh_t;


typedef enum {
    SX1276_LOWBAT_TRIM_1_7V,
    SX1276_LOWBAT_TRIM_1_76V,
    SX1276_LOWBAT_TRIM_1_84V,
    SX1276_LOWBAT_TRIM_1_9V,
    SX1276_LOWBAT_TRIM_1_98V,
    SX1276_LOWBAT_TRIM_2V,
    SX1276_LOWBAT_TRIM_2_1V,
    SX1276_LOWBAT_TRIM_2_2V,
} low_batt_trim_t;


typedef enum {
    DIO_FUNC_NONE,
    DIO_FUNC_RX_DONE,   /** DIO 0 **/
    DIO_FUNC_RX_TIMEOUT, /** DIO 1 **/
    DIO_FUNC_TX_DONE,    /** DIO 0 **/
    DIO_FUNC_CAD_DONE,   /** DIO 0 **/
    DIO_FUNC_CAD_DETECT, /** DIO 1 **/
    DIO_FUNC_MODE_RDY,   /** DIO 5 **/
    DIO_FUNC_CLK_OUT,    /** DIO 5 **/
    DIO_FUNC_PLL_LOCK,   /** DIO 4 **/
    DIO_FUNC_VALID_HDR,  /** DIO 3 **/
    DIO_FUNC_FHSS_CHAN_CHG,  /** DIO 2/1 **/
    DIO_FUNC_PAYLOAD_CRC_ERR, /** DIO 3 **/
    DIO_FUNC_INVALID,
} sx_dio_func_t;

typedef struct {
    gpio_num_t rst_pin;
    gpio_num_t cs_pin;
    gpio_num_t data0;
    gpio_num_t irq_pin;
    uint8_t spi_bus;
} sx1276_init_t;


typedef struct SX1276_Device_Settings
{
    uint32_t  frequency;
    sx_device_mode_t current_mode;
    lna_gain_t gain;
    lora_bw_t bw;
    lora_spread_fac_t sf;
    bool agc_auto;
    bool lna_boost;
    bool lora_mode;

} sx1276_settings_t;



typedef struct SX1276_Driver
{
    sx_device_mode_t device_mode;

    /** internal register maps **/
    union {
        FSK_Register_Map_t fsk_reg;
        Lora_Register_Map_t lora_reg;
    } registers;

    gpio_num_t rst_pin; 
    gpio_num_t cs_pin; 
    gpio_num_t irq_pin;
    spi_device_handle_t spi_handle;

    TaskHandle_t task;

} sx1276_driver_t;


typedef sx1276_driver_t * SX1276_DEV;  /** found this approach in the VL53L0X driver,
                                        *  seems quite nice, less stars everywhere  
                                        **/

/******** Function Definitions *********/

/**
 *  \brief Initialises the sx1276 device 
 *  \param init ptr to populated sx1276_init_t struct
 *  \return SX1276 handle or NULL on error
 *  **/
#ifdef CONFIG_DRIVERS_USE_HEAP
SX1276_DEV sx1276_init(sx1276_init_t *init);
#else
SX1276_DEV sx1276_init(SX1276_DEV dev_handle, sx1276_init_t *init);
#endif

/**
 *  \brief Gets the device radio mode
 *  \param dev device handle
 *  \param mode ptr to value storage
 *  \return ESP_OK or error code
 *  **/
status_t sx_get_device_mode(SX1276_DEV dev, sx_device_mode_t *mode);

/**
 *  \brief Sets the device radio mode
 *  \param dev device handle
 *  \param mode ptr to value - one of sx_device_mode_t
 *  \return ESP_OK or error code
 *  **/
status_t sx_set_device_mode(SX1276_DEV dev, sx_device_mode_t *mode);

/**
 *  \brief Gets the device Version number
 *  \param dev device handle
 *  \param ver ptr to value storage
 *  \return ESP_OK or error code
 *  **/
status_t sx_get_version(SX1276_DEV dev, uint8_t *ver);

/**
 *  \brief Gets the device transaction mode
 *  \param dev device handle
 *  \param mode ptr to value storage
 *  \return ESP_OK or error code
 *  **/
status_t sx_get_trx_mode(SX1276_DEV dev, uint8_t *mode);

/**
 *  \brief Sets the device transaction mode
 *  \param dev device handle
 *  \param mode ptr to value - one of sx_trxmode_t
 *  \return ESP_OK or error code
 *  **/
status_t sx_set_trx_mode(SX1276_DEV dev, sx_trxmode_t *mode);

/**
 *  \brief Gets the device frequency (in Hertz)
 *  \param dev device handle
 *  \param mode ptr to value storage
 *  \return ESP_OK or error code
 *  **/
status_t sx_get_frequency(SX1276_DEV dev, uint32_t *frq);

/**
 *  \brief Sets the device frequency (in Hertz)
 *  \param dev device handle
 *  \param mode ptr to value (Max = SX_LORA_MAX_FREQUENCY-1)
 *  \return ESP_OK or error code
 * **/
status_t sx_set_frequency(SX1276_DEV dev, uint32_t *frq);

/**
 *  \brief Gets the device power amp output pin
 *         - 0 RFO Pin (max +14dB)
 *         - 1 PA BOOST Pin (max +20dB) 
 *  \param dev device handle
 *  \param mode ptr to value storage
 *  \return ESP_OK or error code
 *  **/
status_t sx_get_pa_sel(SX1276_DEV dev, bool *pa_sel);

/**
 *  \brief Sets the device power amp output pin
 *         - 0 RFO Pin (max +14dB)
 *         - 1 PA BOOST Pin (max +20dB) 
 *  \param dev device handle
 *  \param mode ptr to value storage
 *  \return ESP_OK or error code
 *  **/
status_t sx_set_pa_sel(SX1276_DEV dev, bool *pa_sel);

/**
 *  \brief Gets the lora header mode
 *         - 0 Explict
 *         - 1+ Implicit
 *  \param dev device handle
 *  \param mode ptr to value 
 *  \return ESP_OK or error code
 * **/
status_t sx_get_lora_headermode(SX1276_DEV dev, uint8_t *val);

/**
 *  \brief Sets the lora header mode
 *  \param dev device handle
 *  \param mode ptr to value (0 = Explict, 1+ = implicit)
 *  \return ESP_OK or error code
 * **/
status_t sx_set_lora_headermode(SX1276_DEV dev, uint8_t *val);


/**
 *  \brief Gets the device Overcurrent Protection
 *          enabled status
 *         - 0 disabled
 *         - 1 enabled
 *  \param dev device handle
 *  \param mode ptr to value storage 
 *  \return ESP_OK or error code
 * **/
status_t sx_get_ocp_en(SX1276_DEV dev, bool *en);

/**
 *  \brief Sets the device Overcurrent Protection
 *          enabled status
 *         - 0 disabled
 *         - 1 enabled
 *  \param dev device handle
 *  \param mode ptr to value
 *  \return ESP_OK or error code
 * **/
status_t sx_set_ocp_en(SX1276_DEV dev, bool *en);

/**
 *  \brief Gets the overcurrent protection trim
 *  \param dev device handle
 *  \param trim ptr to value storage
 *  \return ESP_OK or error code
 * **/
status_t sx_get_ocp_trim(SX1276_DEV dev, uint8_t *trim);

/**
 *  \brief Sets the overcurrent protection trim
 *          Trimming of OCP current:
 *          Maximum register value: 15
 *          I max = 45+5*OcpTrim [mA] if OcpTrim <= 15 (120 mA) /
 *          I max = -30+10*OcpTrim [mA] if 15 < OcpTrim <= 27 (130 to 240
 *          mA)
 *          I max = 240mA for higher settings
 *          Default I max = 100m
 *          
 *  \param dev device handle
 *  \param trim ptr to value
 *  \return ESP_OK or error code
 * **/
status_t sx_set_ocp_trim(SX1276_DEV dev, uint8_t *trim);

// status_t sx1276_get_modtype(SX1276_DEV dev, uint8_t *mode);

// status_t sx1276_get_lowfreq_mode(SX1276_DEV dev, uint8_t *mode);

/**
 *  \brief Gets the device Low Noise Amplifier gain setting
 *  \param dev device handle
 *  \param mode ptr to value storage 
 *  \return ESP_OK or error code
 * **/
status_t sx_get_lna_gain(SX1276_DEV dev, uint8_t *gain);

/**
 *  \brief Sets device LNA Gain
 *          Should be one of lna_gain_t
 *          (max: 6)
 *  \param dev device handle
 *  \param mode ptr to value 
 *  \return ESP_OK or error code
 * **/
status_t sx_set_lna_gain(SX1276_DEV dev, uint8_t *gain);

/**
 *  \brief Gets the device LNA HF boost
 *          status
 *         - 0 disabled
 *         - 1 enabled
 *  \param dev device handle
 *  \param mode ptr to value storage 
 *  \return ESP_OK or error code
 * **/
status_t sx_get_lna_boost_hf(SX1276_DEV dev, bool *io);

/**
 *  \brief Sets the device LNA HF boost
 *          status
 *         - 0 disabled
 *         - 1 enabled
 *  \param dev device handle
 *  \param mode ptr to value 
 *  \return ESP_OK or error code
 * **/
status_t sx_set_lna_boost_hf(SX1276_DEV dev, bool *io);

/**
 *  \brief Gets the device lora bandwidth
 *         
 *  \param dev device handle
 *  \param mode ptr to value storage
 *  \return ESP_OK or error code
 * **/
status_t sx_get_signal_bandwidth(SX1276_DEV dev, lora_bw_t *bw);

/**
 *  \brief Sets the device lora bandwidth
 *          must be one of lora_bw_t
 *  \param dev device handle
 *  \param mode ptr to value 
 *  \return ESP_OK or error code
 * **/
status_t sx_set_signal_bandwidth(SX1276_DEV dev, lora_bw_t *bw);

/**
 * \brief gets the currently configured spreading 
 *        factor
 * \param dev device handle 
 * \param val ptr to value storage 
 * \return ESP_OK or error code
*/
status_t sx_get_lora_spreading_factor(SX1276_DEV dev, uint8_t *val);

/**
 * \brief sets the currently configured spreading 
 *        factor
 * \param dev device handle 
 * \param val ptr to value 
 * \return ESP_OK or error code
*/
status_t sx_set_lora_spreading_factor(SX1276_DEV dev, uint8_t *val);

/**
 * \brief gets the currently configured rx crc  
 *        check enabled setting
 * \param dev device handle 
 * \param en ptr to value storage 
 * \return ESP_OK or error code
*/
status_t sx_get_rx_payload_crc_en(SX1276_DEV dev,  bool *en);

/**
 * \brief sets the rx crc check enabled setting
 * \param dev device handle 
 * \param en ptr to value storage 
 * \return ESP_OK or error code
*/
status_t sx_set_rx_payload_crc_en(SX1276_DEV dev, bool *en);

status_t sx_get_low_datarate_optimise(SX1276_DEV dev,  bool *en);

status_t sx_set_low_datarate_optimise(SX1276_DEV dev,  bool *en);

/**
 *  \brief Gets the device AGC Auto Enabled
 *          status (LNA Gain controlled by AGC)
 *         - 0 disabled
 *         - 1 enabled
 *  \param dev device handle
 *  \param mode ptr to value 
 *  \return ESP_OK or error code
 * **/
status_t sx_get_agc_auto(SX1276_DEV dev,  bool *io);

/**
 *  \brief Gets the LoRa device AGC Auto Enabled
 *          status
 *         - 0 disabled
 *         - 1 enabled
 *  \param dev device handle
 *  \param mode ptr to value 
 *  \return ESP_OK or error code
 * **/
status_t sx_set_agc_auto(SX1276_DEV dev, bool *io);

/**
 * @brief get the estimated frequency error of
 *        the modem 
 * 
 * @param dev handle
 * @param frq value storage
 * @return status_t 
 */
status_t sx_get_frequency_err(SX1276_DEV dev, uint32_t *frq);

/**
 * @brief get the LoRa sync word
 * 
 * @param dev device handle
 * @param val value storage
 * @return status_t 
 */
status_t sx_get_lora_syncword(SX1276_DEV dev, uint8_t *val);

/**
 * @brief Set the LoRa sync word
 * 
 * @param dev device handle
 * @param val value storage
 * @return status_t 
 */
status_t sx_set_lora_syncword(SX1276_DEV dev, uint8_t *val);

/** TODO: Add to parameter map later 
 *  **/
status_t sx_set_tx_pwr(SX1276_DEV dev, uint16_t pwr);

/**
 * @brief Gets the valid header count
 * 
 * @param dev device handle
 * @param cnt number of valid headers received
 * @return ** status_t 
 */
status_t sx_get_valid_hdr_count(SX1276_DEV dev, uint32_t *cnt);

/**
 * @brief Get the valid packet count
 * 
 * @param dev device handle
 * @param cnt valid packets received
 * @return ** status_t 
 */
status_t sx_get_valid_pkt_count(SX1276_DEV dev, uint32_t *cnt);

/**
 * @brief get the length of last rx packet
 * 
 * @param dev device handle
 * @param len last rx length in bytes
 * @return ** status_t 
 */
status_t sx_get_last_rx_len(SX1276_DEV dev, uint8_t *len);


/**
 * @brief Get last received packet coding rate
 * 
 * @param dev device handle
 * @param cr  coding rate
 * @return status_t 
 */
status_t sx_get_last_rx_coding_rate(SX1276_DEV dev, uint8_t *cr);

/**
 * @brief Get last received packet signal-noise ratio
 * 
 * @param dev device handle
 * @param snr signal-noise ratio
 * @return status_t 
 */
status_t sx_get_last_pkt_snr(SX1276_DEV dev, int16_t *snr);

/**
 * @brief Get last packet RSSI
 * 
 * @param dev device handle
 * @param rssi rssi
 * @return status_t 
 */
status_t sx_get_last_pkt_rssi(SX1276_DEV dev, uint16_t *rssi);

/**
 * @brief get function assigned to the DI/O 0 pin
 * 
 * @param dev device handle
 * @param val value storage
 * @return status_t 
 */
status_t sx_get_lora_dio0_func(SX1276_DEV dev, sx_dio_func_t *val);

/**
 * @brief Set function assigned to the DI/O 0 pin
 * 
 * @param dev device handle
 * @param val value, one of sx_dio_func_t
 * @return status_t 
 */
status_t sx_set_lora_dio0_func(SX1276_DEV dev, sx_dio_func_t *val);

/**
 * @brief Send some data. Over LoRa. Cool!
 * 
 * @param dev device handle
 * @param data pointer to data
 * @param len  length of data to send
 * 
 * @return status_t
*/
status_t sx_lora_transmit_data(SX1276_DEV dev, uint8_t *data, uint8_t len);

/**
 *  \brief Sets the sx1276 device to LoRa mode and
 *          initialises several settings - 
 *          sets tx fifo ptr to 0
 *          sets the LNA Gain to max
 *          sets the AGC Auto enabled bit
 *          sets the TxPower to 17
 *          sets the lora header mode to explicit
 *          sets the device into standby mode
 *  \param dev device handle
 *  \return ESP_OK or error code
 *  **/
status_t sx_setup_lora(SX1276_DEV dev);

status_t sx_lora_set_fifo_tx_start(SX1276_DEV dev, uint8_t *val);

status_t sx_lora_set_fifo_rx_start(SX1276_DEV dev, uint8_t *val);

#endif /* LORA_SX1276_H */
