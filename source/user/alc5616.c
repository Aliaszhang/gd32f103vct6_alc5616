#include "alc5616.h"
#include "spi_driver.h"
#include "i2c_driver.h"
#include "fmc_operation.h"
#include "systick.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//RIFF 块
typedef __packed struct
{
    uint32_t ChunkID;   //chunk id;这里固定为"RIFF",即 0X46464952
    uint32_t ChunkSize; //集合大小;文件总大小-8
    uint32_t Format;    //格式;WAVE,即 0X45564157
}ChunkRIFF ;

//fmt 块
typedef __packed struct
{
    uint32_t ChunkID;       //chunk id;这里固定为"fmt ",即 0X20746D66
    uint32_t ChunkSize ;    //子集合大小(不包括 ID 和 Size); 如：16.
    uint16_t AudioFormat;   //音频格式;0X01,表示线性 PCM;
    uint16_t NumOfChannels; //通道数量;1,表示单声道;2,表示双声道;
    uint32_t SampleRate;    //采样率;0X1F40,表示 8Khz
    uint32_t ByteRate;      //字节速率; (Sample Rate * BitsPerSample * Channels) / 8.
    uint16_t BlockAlign;    //块对齐(字节); 
    uint16_t BitsPerSample; //单个采样数据大小;4 位 ADPCM,设置为 4
//	uint16_t ByteExtraData;	//附加的数据字节;2个; 线性PCM,没有这个参数
}ChunkFMT;

//fact 块
typedef __packed struct 
{
    uint32_t ChunkID; //chunk id;这里固定为"fact",即 0X74636166;
    uint32_t ChunkSize ; //子集合大小(不包括 ID 和 Size);这里为:4.
    uint32_t DataFactSize; //数据转换为 PCM 格式后的大小
}ChunkFACT;
//LIST块 
typedef __packed struct 
{
    uint32_t ChunkID;		   	//chunk id;这里固定为"LIST",即0X74636166;
    uint32_t ChunkSize ;		   	//子集合大小(不包括ID和Size);这里为:4. 
}ChunkLIST;
//data 块
typedef __packed struct 
{
    uint32_t ChunkID;   //chunk id;这里固定为"data",即 0X61746164
    uint32_t ChunkSize; //子集合大小(不包括 ID 和 Size);
}ChunkDATA;

//wav头
typedef __packed struct
{ 
	ChunkRIFF riff;	//riff块
	ChunkFMT fmt;  	//fmt块
//	ChunkFACT fact;	//fact块 线性PCM,没有这个结构体
    ChunkLIST list;
	ChunkDATA data;	//data块		 
}__WaveHeader; 

//wav 播放控制结构体
typedef __packed struct
{ 
    uint16_t audioformat;			//音频格式;0X01,表示线性PCM;0X11表示IMA ADPCM
	uint16_t nchannels;				//通道数量;1,表示单声道;2,表示双声道; 
	uint16_t blockalign;				//块对齐(字节);  
	uint32_t datasize;				//WAV数据大小 

    uint32_t totsec ;				//整首歌时长,单位:秒
    uint32_t cursec ;				//当前播放时长

    uint32_t bitrate;	   			//比特率(位速)
	uint32_t samplerate;				//采样率 
	uint16_t bps;					//位数,比如16bit,24bit,32bit

	uint32_t datastart;				//数据帧开始的位置(在文件里面的偏移)
}__wavctrl;

int alc5616_i2c_write(uint8_t write_address, uint16_t value)
{
    uint8_t ret;
    uint8_t tmp[2] = {0};

    tmp[0] = ((value & 0xFF00) >> 8);
    tmp[1] = (value & 0x00FF);
    ret = i2c_bytes_write_timeout(I2CX, I2CX_AUDIO_ADDRESS, tmp, write_address, 2); 
    if (ret == I2C_OK) 
    {
        return 0;
    }
    return -1;
}

uint16_t alc5616_i2c_read(uint8_t write_address)
{
    uint8_t tmp[2] = {0};
    uint8_t ret;

    ret = i2c_bytes_read_timeout(I2CX, I2CX_AUDIO_ADDRESS, tmp, write_address, 2);
    if (ret == I2C_OK)
    {
        return (tmp[0] << 8 | tmp[1]);
    }
    return 0xFFFF;
}

static inline int alc5616_reset(void)
{
	return alc5616_i2c_write(ALC5616_RESET, 0);
}

#if 1
int show_alc5616_settings(void)
{
    uint16_t reg;

    reg = alc5616_i2c_read(ALC5616_RESET);
    printf("set alc5616 ALC5616_RESET read:0x%#04X(0x0000)!\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_ADC_DAC_CLK_CTRL1);
    printf("set alc5616 ALC5616_ADC_DAC_CLK_CTRL1 read:0x%#04X(0x1104)!\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_GEN_CTRL);
    printf("set alc5616 ALC5616_GEN_CTRL read:0x%#04X(0x0019)!\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL1);
    printf("set alc5616 ALC5616_PWR_MANAG_CTRL1 read:0x%#04X(0x9800)!\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL2);
    printf("set alc5616 ALC5616_PWR_MANAG_CTRL2 read:0x%#04X!(0x0800)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL3);
    printf("set alc5616 ALC5616_PWR_MANAG_CTRL3 read:0x%#04X!(0xE8F8)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL5);
    printf("set alc5616 ALC5616_PWR_MANAG_CTRL5 read:0x%#04X!(0xC000)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL6);
    printf("set alc5616 ALC5616_PWR_MANAG_CTRL6 read:0x%#04X!(0x0C00)\r\n", reg);
    
    reg = alc5616_i2c_read(ALC5616_PR_REG_DATA);
    printf("set alc5616 ALC5616_PR_REG_DATA read:0x%#04X!(0x2E00)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_HP_AMP_CTRL1);
    printf("set alc5616 ALC5616_HP_AMP_CTRL1 read:0x%#04X!(0x0019)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_HP_AMP_CTRL2);
    printf("set alc5616 ALC5616_HP_AMP_CTRL2 read:0x%#04X!(0x3100)\r\n", reg);

    reg = alc5616_i2c_read(ALC5616_STEREO_DAC_DIG_MIXER_CTRL);
    printf("set alc5616 ALC5616_STEREO_DAC_DIG_MIXER_CTRL read:0x%#04X!(0x1212)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_OUT_MIXL_CTRL3);
    printf("set alc5616 ALC5616_OUT_MIXL_CTRL3 read:0x%#04X!(0x0278)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_OUT_MIXR_CTRL3);
    printf("set alc5616 ALC5616_OUT_MIXR_CTRL3 read:0x%#04X!(0x0278)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_HP_OUT_MIX_CTRL);
    printf("set alc5616 ALC5616_HP_OUT_MIX_CTRL read:0x%#04X!(0x4000)\r\n", reg);
    reg = alc5616_i2c_read(ALC5616_HP_OUT_VOL);
    printf("set alc5616 ALC5616_HP_OUT_VOL read:0x%#04X!(0x0808)\r\n", reg);

    reg = alc5616_i2c_read(ALC5616_I2S1_DIG_INTERFALCE_CTRL);
    printf("set alc5616 ALC5616_I2S1_DIG_INTERFALCE_CTRL read:0x%#04X!(0x8000)\r\n", reg);

    reg = alc5616_i2c_read(0x91);
    printf("set alc5616 MX-91 read:0x%#04X!(0x0E00)\r\n\n", reg);

    return 0;
}
#endif

int alc5616_init(void)
{  
    uint16_t reg;

    reg = alc5616_i2c_read(ALC5616_VENDOR_ID);
	if (reg != DEVICE_ID) {
		printf("failed to read vendor ID: %#x(%#x)\n", reg, DEVICE_ID);
		return -1;
	}
    printf("Found codec id : alc5616\n");

    // reset all register to default value
    alc5616_reset();
    
    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL3);
    reg |= (ALC5616_PWR_MBIAS_BANDGAP_PWRON | ALC5616_PWR_VREF1_PWRON | ALC5616_PWR_MAIN_BIAS_PWRON | ALC5616_PWR_VREF2_PWRON | ALC5616_LDO_OUTPUT_1V4);
    reg &= (ALC5616_PWR_FAST_VREF1);
    reg &= (ALC5616_PWR_FAST_VREF2);
    // alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL3, reg);
    alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL3, 0xA8F0);
    delay_1ms(500);
    

    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL1);
    reg |= ALC5616_PWR_DACL1_PWRON | ALC5616_PWR_DACR1_PWRON | ALC5616_PWR_I2S1_DI_EN;
    alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL1, reg);

    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL2);
    reg |= ALC5616_PWR_STEREO1_DAC_FILTER_PWRON;
    alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL2, reg);

    reg = alc5616_i2c_read(ALC5616_GEN_CTRL);
    reg |= ALC5616_MCLK_INPUT_EN | ALC5616_DET_SYS_CLK_EN;
    alc5616_i2c_write(ALC5616_GEN_CTRL, reg);

    // 1000 0000 0000 0010
    // alc5616_i2c_write(ALC5616_I2S1_DIG_INTERFALCE_CTRL, 0x8000);

    alc5616_i2c_write(ALC5616_PR_REG_INDEX, 0x003D);
    alc5616_i2c_write(ALC5616_PR_REG_DATA, 0x2E00);

    alc5616_i2c_write(ALC5616_GLOBAL_CLK_CTRL_REG, 0x5000);
    alc5616_i2c_write(ALC5616_PLL_CTRL1, 0x5F0A);
    alc5616_i2c_write(ALC5616_PLL_CTRL2, 0x0800);

    alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL4, 0x0200);

    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL5);
    reg |= ALC5616_OUT_MIXERL_PWRON | ALC5616_OUT_MIXERR_PWRON;
    alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL5, reg);

    reg = alc5616_i2c_read(ALC5616_PWR_MANAG_CTRL6);
    reg |= ALC5616_HPOVOLL_PWRON | ALC5616_HPOVOLR_PWRON;
    alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL6, reg);

    // For good analog performance
    alc5616_i2c_write(ALC5616_PWR_MANAG_CTRL3, 0xE8F8);

    // 0000 0000 0001 1001
    reg = alc5616_i2c_read(ALC5616_HP_AMP_CTRL1);
    reg |= (HP_OUTPUT_ENABLE | HP_AMP_ALL_PWRON | HP_CHARGE_PUMP_PWRON);
    reg &= HP_SOFT_GEN_PWRDOWN;
    alc5616_i2c_write(ALC5616_HP_AMP_CTRL1, reg);

    // 001 100010 000 0000
    reg = alc5616_i2c_read(ALC5616_HP_AMP_CTRL2);
    reg |= ALC5616_HP_SEL_DEPOP_MODE2;
    alc5616_i2c_write(ALC5616_HP_AMP_CTRL2, reg);

    // 0001 0010 0001 0010
    reg = alc5616_i2c_read(ALC5616_STEREO_DAC_DIG_MIXER_CTRL);
    reg &= ALC5616_MUTE_DACL1_2_STEREO_DACLMIXER;
    reg &= ALC5616_MUTE_DACR1_2_STEREO_DACRMIXER;
    alc5616_i2c_write(ALC5616_STEREO_DAC_DIG_MIXER_CTRL, reg);

    reg = alc5616_i2c_read(ALC5616_OUT_MIXL_CTRL3);
    reg &= ALC5616_MUTE_DACL1_2_OUTMIXL;
    alc5616_i2c_write(ALC5616_OUT_MIXL_CTRL3, reg);

    reg = alc5616_i2c_read(ALC5616_OUT_MIXR_CTRL3);
    reg &= ALC5616_MUTE_DACR1_2_OUTMIXR;
    alc5616_i2c_write(ALC5616_OUT_MIXR_CTRL3, reg);
    
    // 0100 0000 0000 0000
    alc5616_i2c_write(ALC5616_HP_OUT_MIX_CTRL, 0x4000);

    // 00 001000 00 001000
    alc5616_i2c_write(ALC5616_HP_OUT_VOL, 0x0808);

    alc5616_i2c_write(0x91, 0x0E00);

    show_alc5616_settings();

    return 0;
}

__wavctrl wavctrl;          // WAV 控制结构体
uint8_t wavtransferend=0;   // sai 传输完成标志
uint8_t wavwitchbuf=0;      // saisbufx 指示标志
//WAV 解析初始化
//wavx:wav 信息存放结构体指针
//返回值:0,成功;1,打开文件失败;2,非 WAV 文件;3,DATA 区域未找到.
uint8_t wav_decode_init(__wavctrl* wavx, uint32_t addr)
{
    uint8_t *buf; 
    uint8_t res = 0; 
    ChunkRIFF *riff; 
    ChunkFMT *fmt;
    ChunkFACT *fact;
    ChunkDATA *data;

    buf = (uint8_t *)malloc(512);
    if(buf) //内存申请成功
    {
        fmc_read_data(addr, buf, 512); //读取 512 字节在数据
        riff = (ChunkRIFF *)buf; //获取 RIFF 块
        if( riff->Format == 0x45564157) //是 WAV 文件
        {
            fmt = (ChunkFMT *)(buf + sizeof(ChunkRIFF)); //获取 FMT 块
            fact = (ChunkFACT *)(buf + sizeof(ChunkRIFF) + 8 + fmt->ChunkSize); //读取 FACT 块
            if(fact->ChunkID == 0x74636166 || fact->ChunkID == 0x5453494C)
            {
                wavx->datastart = sizeof(ChunkRIFF) + 8 + fmt->ChunkSize + 8 + fact->ChunkSize;
                //具有 fact/LIST 块的时候(未测试)
            }
            else
            {
                wavx->datastart = sizeof(ChunkRIFF) + 8 + fmt->ChunkSize; 
            } 
            
            data=(ChunkDATA *)(buf + wavx->datastart); //读取 DATA 块
            
            if(data->ChunkID == 0X61746164) //解析成功!
            {
                wavx->audioformat = fmt->AudioFormat; //音频格式
                wavx->nchannels = fmt->NumOfChannels; //通道数
                wavx->samplerate = fmt->SampleRate;   //采样率
                wavx->bitrate = fmt->ByteRate * 8;    //得到位速
                wavx->blockalign = fmt->BlockAlign;   //块对齐
                wavx->bps = fmt->BitsPerSample;       //位数, 16/24/32 位
                wavx->datasize = data->ChunkSize;     //数据块大小
                wavx->datastart = wavx->datastart + 8;//数据流开始的地方. 
            }
            else 
            {
                res=3; //data 区域未找到.
            }
        }
        else
        {
            res=2; //非 wav 文件
        } 
    }
    free(buf); //释放内存

    return res;
}

#if 1

int i2s_transfer_dma(uint32_t addr)
{
    uint32_t remain_len;
    uint32_t rd_addr;
    uint8_t ret;
    // uint16_t *audio_buffer = NULL;
    // dma_parameter_struct dma_init_struct;


    ret = wav_decode_init(&wavctrl, addr);
    if (ret != 0)
    {
        printf("decoede wav fail, ret = %d\r\n", ret);
    }
    printf("wav.aduioformat = %d\r\n", wavctrl.audioformat);
    printf("wav.nchannels = %d\r\n", wavctrl.nchannels);
    printf("wav.samplerate = %d\r\n", wavctrl.samplerate);
    printf("wav.bitrate = %d\r\n", wavctrl.bitrate);
    printf("wav.blockalign = %d\r\n", wavctrl.blockalign);
    printf("wav.bps = %d\r\n", wavctrl.bps);
    printf("wav.datasize = %d\r\n", wavctrl.datasize);
    printf("wav.datastart = %d\r\n\n\n", wavctrl.datastart);

    i2s_disable(SPI1);
    // i2s_init(SPI1, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);
    i2s_psc_config(SPI1, wavctrl.samplerate, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_ENABLE);
    i2s_enable(SPI1);

    // audio_buffer = (uint16_t *)malloc(AUDIO_ALRAM_ONE_DATA_SIZE * sizeof(uint16_t));

    remain_len = wavctrl.datasize / 2;
    rd_addr = addr + wavctrl.datastart;
    
    /* DMA1 channel0 initialize */
    // dma_deinit(DMA0, DMA_CH0);
    // dma_struct_para_init(&dma_init_struct);
    
    // dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
    // dma_init_struct.memory_addr = (uint32_t)audio_buffer;
    // dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    // dma_init_struct.memory_width = DMA_MEMORY_WIDTH_16BIT;
    // dma_init_struct.number = AUDIO_ALRAM_ONE_DATA_SIZE;
    // dma_init_struct.periph_addr = (uint32_t)(rd_addr);
    // dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_ENABLE;
    // dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
    // dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;
    // dma_init(DMA0, DMA_CH0, &dma_init_struct);
    // /* DMA1 channel0 mode configuration */
    // dma_circulation_disable(DMA0, DMA_CH0);
    // dma_memory_to_memory_enable(DMA0, DMA_CH0);

    // i2s_dma_config(audio_buffer, AUDIO_ALRAM_ONE_DATA_SIZE);

    i2s_dma_config(rd_addr, remain_len);


    // while (remain_len > 0)
    // {
    //     rd_size = (remain_len > AUDIO_ALRAM_ONE_DATA_SIZE) ? AUDIO_ALRAM_ONE_DATA_SIZE : remain_len;
    //     memset(audio_buffer, 0, AUDIO_ALRAM_ONE_DATA_SIZE * sizeof(uint16_t));
        
    //     /* enable DMA1 transfer */
    //     dma_channel_enable(DMA0, DMA_CH0);
    //     while(!dma_flag_get(DMA0, DMA_CH0, DMA_FLAG_FTF)){
    //     }

    //     /* wait DMA transmit completed */
    //     while(!dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF)) {
    //     }

    //     remain_len -= rd_size;
    //     rd_addr += rd_size;
    //     ret ++;
    // }
    // printf("\r\n%d package\r\n\n", ret);
    //free(audio_buffer);
    // return remain_len;
    return 0;
}
#else
#define AUDIO_ALRAM_ONE_DATA_SIZE        (4096U)
static uint8_t audio_buffer[AUDIO_ALRAM_ONE_DATA_SIZE] = {0xFF};

int i2s_transfer_dma(uint32_t addr)
{
    uint32_t rd_size, remain_len;
    uint32_t rd_addr;
    uint8_t ret;

    ret = wav_decode_init(&wavctrl, addr);
    if (ret != 0)
    {
        printf("decoede wav fail, ret = %d\r\n", ret);
    }
    printf("wav.aduioformat = %d\r\n", wavctrl.audioformat);
    printf("wav.nchannels = %d\r\n", wavctrl.nchannels);
    printf("wav.samplerate = %d\r\n", wavctrl.samplerate);
    printf("wav.bitrate = %d\r\n", wavctrl.bitrate);
    printf("wav.blockalign = %d\r\n", wavctrl.blockalign);
    printf("wav.bps = %d\r\n", wavctrl.bps);
    printf("wav.datasize = %d\r\n", wavctrl.datasize);
    printf("wav.datastart = %d\r\n\n\n", wavctrl.datastart);

    i2s_disable(SPI1);
    // i2s_init(SPI1, I2S_MODE_MASTERTX, I2S_STD_PHILLIPS, I2S_CKPL_LOW);
    i2s_psc_config(SPI1, wavctrl.samplerate, I2S_FRAMEFORMAT_DT16B_CH16B, I2S_MCKOUT_ENABLE);
    i2s_enable(SPI1);

    remain_len = wavctrl.datasize;
    rd_addr = addr + wavctrl.datastart;
    
    ret = 0;
    while (remain_len > 0)
    {
        rd_size = (remain_len > AUDIO_ALRAM_ONE_DATA_SIZE) ? AUDIO_ALRAM_ONE_DATA_SIZE : remain_len;
        memset(audio_buffer, 0, sizeof(audio_buffer));

        fmc_read_data(rd_addr, audio_buffer, rd_size);

        i2s_dma_config((uint16_t *)audio_buffer, rd_size/2);
        /* wait DMA transmit completed */
        while(!dma_flag_get(DMA0, DMA_CH4, DMA_FLAG_FTF)) {
        }

        remain_len -= rd_size;
        // printf("i2s send %dbytes, remain %dbytes, total size %dbytes\r\n", rd_size, remain_len, wavctrl.datasize);
        rd_addr += rd_size;
        ret ++;
    }
    printf("\r\n%d package\r\n\n", ret);
    return remain_len;
}
#endif















