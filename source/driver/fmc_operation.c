#include "fmc_operation.h"
#include "gd32f10x_fmc.h"


/*!
    \brief      write 8 bit length data to a given address
    \param[in]  address: a given address(0x08000000~0x0803FFFF)
    \param[in]  length: data length
    \param[in]  data_8: data pointer
    \param[out] none
    \retval     none
*/

int fmc_erase_page(uint32_t start_addr, uint32_t end_addr)
{
    uint32_t erase_counter;
    int ret;
    uint32_t page_num = (end_addr - start_addr) / FMC_PAGE_SIZE;
    
    printf("\r\nstart erase flash part %#x\n", start_addr);
    /* unlock the flash program erase controller */
    fmc_unlock();

    /* clear pending flags */
    fmc_flag_clear(FMC_FLAG_BANK0_END);
    fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
    fmc_flag_clear(FMC_FLAG_BANK0_PGERR);

    /* erase the flash pages */
    for(erase_counter = 0; erase_counter < page_num; erase_counter++)
    {
        if (FMC_READY == fmc_page_erase(start_addr + (FMC_PAGE_SIZE * erase_counter)))
        {
            fmc_flag_clear(FMC_FLAG_BANK0_END);
            fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
            fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
        }
        else
        {
            ret = -1;
            printf("\r\nerase failed!\n");
            goto end;
        }
    }
    printf("\r\nerase complete!\n");
    printf("\r\n");
end:
    /* lock the main FMC after the erase operation */
    fmc_lock();

    return ret;

}


int fmc_write_word_data(uint32_t address, uint16_t length, uint8_t* data_8)
{
    uint16_t i, index = 0;
    int ret = 0;
    uint32_t data;
    uint16_t int_len = 0, remainder = 0;
    
    // printf("\r\nFMC program operation: %#x, len:%d\r\n", address, length);
    int_len = length / 4;
    remainder = length % 4;
    index = 0;

    /* unlock the flash program erase controller */
    fmc_unlock();

    /* program flash */
    for (i = 0; i < int_len; i++)
    {
        data = (data_8[index] | (data_8[index + 1] << 8) | (data_8[index + 2] << 16) | (data_8[index + 3] << 24));
        if (FMC_READY == fmc_word_program(address, data))
        {
            address += 4;
            index += 4;
            fmc_flag_clear(FMC_FLAG_BANK0_END);
            fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
            fmc_flag_clear(FMC_FLAG_BANK0_PGERR); 
        }
        else
        {
            ret = -1;
            goto end;
        }
    }

    if (remainder > 0)
    {
        if (remainder == 1)
        {
            data = data_8[index];
        }
        else if (remainder == 2)
        {
            data = (data_8[index] | (data_8[index + 1] << 8));
        }
        else if (remainder == 3)
        {
            data = (data_8[index] | (data_8[index + 1] << 8) | (data_8[index + 1] << 16));
        }
        if (FMC_READY == fmc_word_program(address, data))
        {
            address += 4;
            index += 4;
            fmc_flag_clear(FMC_FLAG_BANK0_END);
            fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
            fmc_flag_clear(FMC_FLAG_BANK0_PGERR); 
        }
        else
        {
            ret = -1;
            goto end;
        }
    }

    ret = length;
    // printf("\r\nWrite complete!\n");
    // printf("\r\n");
end:
    /* lock the flash program erase controller */
    fmc_lock();

    return ret;
}

int fmc_read_data(uint32_t addr, uint8_t *buf, size_t size)
{
    size_t i;
    
    // printf("\r\nfmc Read data from 0x%02X, read len %d\n", addr, size);
    for (i = 0; i < size; i++, addr++, buf++)
    {
        *buf = *(uint8_t *) addr;
    }

    return size;
}






