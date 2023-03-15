#ifndef __ALC5616_H__
#define __ALC5616_H__

#include "gd32f10x.h"

#define DEVICE_ID                           0x10EC

#define ALC5616_RESET				        0x00
/*				        5621 5622 5623  */
/* speaker output vol	2    2          */
/* line output vol      4    2          */
/* HP output vol		4    0    4     */
#define ALC5616_HP_OUT_VOL			        0x02

#define ALC5616_SPK_OUT_VOL1		        0x03
#define ALC5616_SPK_OUT_VOL2		        0x05
#define ALC5616_IN12_INPUT_CTRL             0x0D
#define ALC5616_INL_INR_VOL_CTRL            0x0F
// Digital Gain/Volume
#define ALC5616_DAC_L1_R1_DIG_VOL           0x19
#define ALC5616_STEREO_ADC_DIG_VOL          0x1C
#define ALC5616_ADC_DIG_BOOST_CTRL          0x1E
// Digital Mixer
#define ALC5616_STEREO_ADC_DIG_MIXER_CTRL   0x27
#define ALC5616_STEREO_ADC_DAC_MIXER_CTRL   0x29
#define ALC5616_STEREO_DAC_DIG_MIXER_CTRL   0x2A
#define ALC5616_MUTE_DACL1_2_STEREO_DACLMIXER (~(1 << 14))
#define ALC5616_MUTE_DACR1_2_STEREO_DACRMIXER (~(1 << 6))

// Output Mixer
#define ALC5616_HP_OUT_MIX_CTRL             0x45

#define ALC5616_OUT_MIXL_CTRL1              0x4D
#define ALC5616_OUT_MIXL_CTRL2              0x4E
#define ALC5616_OUT_MIXL_CTRL3              0x4F
#define ALC5616_MUTE_DACL1_2_OUTMIXL        (~(1 << 0))

#define ALC5616_OUT_MIXR_CTRL1              0x50
#define ALC5616_OUT_MIXR_CTRL2              0x51
#define ALC5616_OUT_MIXR_CTRL3              0x52
#define ALC5616_MUTE_DACR1_2_OUTMIXR        (~(1 << 0))

#define LINE_OUT_MIX_CTRL                   0x53
// PowerManagement
#define ALC5616_PWR_MANAG_CTRL1             0x61
#define ALC5616_PWR_DACL1_PWRON             (1 << 12)
#define ALC5616_PWR_DACR1_PWRON             (1 << 11)
#define ALC5616_PWR_I2S1_DI_EN              (1 << 15)

#define ALC5616_PWR_MANAG_CTRL2             0x62
#define ALC5616_PWR_STEREO1_DAC_FILTER_PWRON (1 << 11)

#define ALC5616_PWR_MANAG_CTRL3             0x63
#define ALC5616_PWR_MBIAS_BANDGAP_PWRON     (1 << 11)
#define ALC5616_PWR_VREF1_PWRON             (1 << 15)
#define ALC5616_PWR_MAIN_BIAS_PWRON         (1 << 13)
#define ALC5616_PWR_VREF2_PWRON             (1 << 4)    
#define ALC5616_PWR_FAST_VREF1              (~(1 << 14))
#define ALC5616_PWR_FAST_VREF2              (~(1 <<  3))
#define ALC5616_PWR_SLOW_VREF1              (1 << 14)
#define ALC5616_PWR_SLOW_VREF2              (1 <<  3)
#define ALC5616_HPL_AMP_PWRON               (1 <<  7)
#define ALC5616_HPR_AMP_PWRON               (1 <<  6)
#define ALC5616_LDO_OUTPUT_1V4              (3 <<  0)

#define ALC5616_PWR_MANAG_CTRL4             0x64
#define ALC5616_PWR_MANAG_CTRL5             0x65
#define ALC5616_OUT_MIXERL_PWRON            (1 << 15)
#define ALC5616_OUT_MIXERR_PWRON            (1 << 14)

#define ALC5616_PWR_MANAG_CTRL6             0x66
#define ALC5616_HPOVOLL_PWRON               (1 << 11)
#define ALC5616_HPOVOLR_PWRON               (1 << 10)

// PR Register
#define ALC5616_PR_REG_INDEX                0x6A
#define ALC5616_PR_REG_DATA                 0x6C
// Digital Interface
#define ALC5616_I2S1_DIG_INTERFALCE_CTRL    0x70
#define ALC5616_ADC_DAC_CLK_CTRL1           0x73
#define ALC5616_ADC_DAC_CLK_CTRL2           0x74
// Global Clock
#define ALC5616_GLOBAL_CLK_CTRL_REG         0x80
#define ALC5616_PLL_CTRL1                   0x81
#define ALC5616_PLL_CTRL2                   0x82
// HP Amp
#define ALC5616_HP_AMP_CTRL1                0x8E
#define HP_OUTPUT_ENABLE                    (1 << 4)
#define HP_CHARGE_PUMP_PWRON                (1 << 3)
#define HP_AMP_ALL_PWRON                    (1 << 0)
#define HP_SOFT_GEN_PWRDOWN                 (~(1 << 2))

#define ALC5616_HP_AMP_CTRL2                0x8F
#define ALC5616_HP_SEL_DEPOP_MODE2          (1 << 13)

// MICBIAS
#define  ALC5616_MIC_BIAS_CTRL              0x93
// JD
#define ALC5616_JACK_DET_CTRL               0x94
#define ALC5616_JACK_DET_CTRL1              0xBB
#define ALC5616_JACK_DET_CTRL2              0xBC
// EQ
#define ALC5616_EQ_CTRL1                    0xB0
#define ALC5616_EQ_CTRL2                    0xB1
// DRC/AGC
#define ALC5616_DRC_AGC_CTRL1               0xB4
#define ALC5616_DRC_AGC_CTRL2               0xB5
#define ALC5616_DRC_AGC_CTRL3               0xB6

// IRQ

// GPIO

// Wind Filter
#define ALC5616_WIND_FILTER_CTRL1           0xD3
#define ALC5616_WIND_FILTER_CTRL2           0xD4
// SVOL & ZCD
#define ALC5616_SOFT_VOL_ACD_CTRL           0xD9
// General Control
#define ALC5616_GEN_CTRL                    0xFA
#define ALC5616_MCLK_INPUT_EN               (1 << 0)
#define ALC5616_DET_SYS_CLK_EN              (1 << 3)


#define PR_3D_ALC5616_ADC_DAC_RESET_CTRL    0x3D
#define ALC5616_DAC_CLK1_GEN_EN             (1 << 10)
#define ALC5616_DAC_CLK2_GEN_EN             (1 <<  9)

// Vendor ID
#define ALC5616_VENDOR_ID			        0xFE


#define AUDIO_1            (0x8008000)
#define AUDIO_2            (0x8014000)
#define AUDIO_3            (0x801F000)
#define AUDIO_4            (0x802C000)

int i2s_transfer_dma(uint32_t addr);
int alc5616_init(void);

#endif
