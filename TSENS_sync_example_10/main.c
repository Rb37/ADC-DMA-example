/**
 * \file
 *
 * \brief ADC TSENS Example
 *
 * Copyright (C) 2017 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */
#include "atmel_start.h"
#include "tsens_example_config.h"

#define NVM_TEMP_CAL_TLI_POS 0
#define NVM_TEMP_CAL_TLI_SIZE 8
#define NVM_TEMP_CAL_TLD_POS 8
#define NVM_TEMP_CAL_TLD_SIZE 4
#define NVM_TEMP_CAL_THI_POS 12
#define NVM_TEMP_CAL_THI_SIZE 8
#define NVM_TEMP_CAL_THD_POS 20
#define NVM_TEMP_CAL_THD_SIZE 4
#define NVM_TEMP_CAL_VPL_POS 40
#define NVM_TEMP_CAL_VPL_SIZE 12
#define NVM_TEMP_CAL_VPH_POS 52
#define NVM_TEMP_CAL_VPH_SIZE 12
#define NVM_TEMP_CAL_VCL_POS 64
#define NVM_TEMP_CAL_VCL_SIZE 12
#define NVM_TEMP_CAL_VCH_POS 76
#define NVM_TEMP_CAL_VCH_SIZE 12

int16_t temp_cal_tl, temp_cal_th;
int16_t temp_cal_vpl, temp_cal_vph, temp_cal_vcl, temp_cal_vch;
int16_t tp_result, tc_result;
int32_t temperature_result;

// Buffer length to transfer/receive
#define BUFFER_LEN 3

// DMA channel numbers for receive and transmit
#define ADC_RES_DMA_CH 0
#define ADC_CONFIG_DMA_CH 1

volatile static uint32_t ADCConfigBuf[BUFFER_LEN] = { 0x1800, 0x181D, 0x181C };
volatile static uint32_t ADCResBuf[BUFFER_LEN];
volatile static uint32_t ADCGoodResBuf[BUFFER_LEN];
    
volatile static uint8_t data_ready = 0;

// callbacks
static void dma_transfer_done_adc_res(struct _dma_resource *const resource)
{
    for (uint32_t i = 0; i < BUFFER_LEN; i++)
    {
        ADCGoodResBuf[i] = ADCResBuf[i];
    }
    _dma_enable_transaction(ADC_RES_DMA_CH, false);
    data_ready++;
}

static void dma_error_adc_res(struct _dma_resource *const resource)
{
    // write error handling code here
    for ( ; ;)
    {
        ; // error trap
    }
}

static void dma_transfer_done_adc_config(struct _dma_resource *const resource)
{
    _dma_enable_transaction(ADC_CONFIG_DMA_CH, false);
}

static void dma_error_adc_config(struct _dma_resource *const resource)
{
    // write error handling code here
    for ( ; ;)
    {
        ; // error trap
    }
}

// register callbacks
void Register_dma_adc_res_callback(void)
{
    struct _dma_resource *resource_adc_res;
    _dma_get_channel_resource(&resource_adc_res, ADC_RES_DMA_CH);
    resource_adc_res->dma_cb.transfer_done = dma_transfer_done_adc_res;
    resource_adc_res->dma_cb.error         = dma_error_adc_res;
}

void Register_dma_adc_config_callback(void)
{
    struct _dma_resource *resource_adc_config;
    _dma_get_channel_resource(&resource_adc_config, ADC_CONFIG_DMA_CH);
    resource_adc_config->dma_cb.transfer_done = dma_transfer_done_adc_config;
    resource_adc_config->dma_cb.error         = dma_error_adc_config;
}

// USART RX channel configuration
void Configure_Channel_ADC_Res()
{
    _dma_set_source_address(ADC_RES_DMA_CH, (void*)_adc_get_source_for_dma(&ADC_0.device)); //ADC_0.device.hw + ADC_RESULT_OFFSET
    _dma_set_destination_address(ADC_RES_DMA_CH, (void*)ADCResBuf); // (void*)ADCResBuf static_cast<void*>(const_cast<uint32_t*>(ADCResBuf))
    _dma_set_data_amount(ADC_RES_DMA_CH, (uint32_t)BUFFER_LEN);

    // callback
    Register_dma_adc_res_callback();

    // Enable DMA transfer complete interrupt
    _dma_set_irq_state(ADC_RES_DMA_CH, DMA_TRANSFER_COMPLETE_CB, true);
}

// USART TX channel configuration
void Configure_Channel_ADC_Config()
{
    _dma_set_source_address(ADC_CONFIG_DMA_CH, (void*)ADCConfigBuf); // (void*)ADCConfigBuf
    _dma_set_destination_address(ADC_CONFIG_DMA_CH, (void*)_adc_get_source_for_dma_dseqdata(&ADC_0.device)); //ADC_0.device.hw + ADC_DSEQDATA_OFFSET
    _dma_set_data_amount(ADC_CONFIG_DMA_CH, (uint32_t)BUFFER_LEN);

    // callback
    Register_dma_adc_config_callback();

    // Enable DMA transfer complete interrupt
    _dma_set_irq_state(ADC_CONFIG_DMA_CH, DMA_TRANSFER_COMPLETE_CB, true);
}

/**
 * Initialize ADC TSENS conversion
 */
void ADC_temperature_init(void)
{
	int8_t temp_cal_tli, temp_cal_thi, temp_cal_tld, temp_cal_thd;

	temp_cal_vpl = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VPL_POS / 32)) >> (NVM_TEMP_CAL_VPL_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VPL_SIZE) - 1);
	temp_cal_vph = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VPH_POS / 32)) >> (NVM_TEMP_CAL_VPH_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VPH_SIZE) - 1);
	temp_cal_vcl = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VCL_POS / 32)) >> (NVM_TEMP_CAL_VCL_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VCL_SIZE) - 1);
	temp_cal_vch = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_VCH_POS / 32)) >> (NVM_TEMP_CAL_VCH_POS % 32))
	               & ((1 << NVM_TEMP_CAL_VCH_SIZE) - 1);

	temp_cal_tli = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_TLI_POS / 32)) >> (NVM_TEMP_CAL_TLI_POS % 32))
	               & ((1 << NVM_TEMP_CAL_TLI_SIZE) - 1);
	temp_cal_tld = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_TLD_POS / 32)) >> (NVM_TEMP_CAL_TLD_POS % 32))
	               & ((1 << NVM_TEMP_CAL_TLD_SIZE) - 1);

	temp_cal_tl = ((uint16_t)temp_cal_tli) << 4 | ((uint16_t)temp_cal_tld);

	temp_cal_thi = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_THI_POS / 32)) >> (NVM_TEMP_CAL_THI_POS % 32))
	               & ((1 << NVM_TEMP_CAL_THI_SIZE) - 1);
	temp_cal_thd = (*((uint32_t *)(NVMCTRL_TEMP_LOG_W0) + (NVM_TEMP_CAL_THD_POS / 32)) >> (NVM_TEMP_CAL_THD_POS % 32))
	               & ((1 << NVM_TEMP_CAL_THD_SIZE) - 1);

	temp_cal_th = ((uint16_t)temp_cal_thi) << 4 | ((uint16_t)temp_cal_thd);

	hri_supc_set_VREF_ONDEMAND_bit(SUPC);
	hri_supc_set_VREF_TSEN_bit(SUPC);
	hri_supc_clear_VREF_VREFOE_bit(SUPC);   
}

int main(void)
{
	atmel_start_init();
	printf("-- ADC Sync DMA Example. Temperature and Pin --\r\n");
	ADC_temperature_init();
    
    // Configure DMA channels
    Configure_Channel_ADC_Res();
    Configure_Channel_ADC_Config();

    _dma_enable_transaction(ADC_CONFIG_DMA_CH, false);
    
    _dma_enable_transaction(ADC_RES_DMA_CH, false);
    
    hri_adc_set_DSEQCTRL_INPUTCTRL_bit(ADC_0.device.hw);
    hri_adc_set_DSEQCTRL_AUTOSTART_bit(ADC_0.device.hw);
    
    adc_sync_enable_channel(&ADC_0, CONF_ADC_CHANNEL);
    
    _adc_sync_convert(&ADC_0.device);
    
    
	for ( ; ; )
    {
        if(data_ready)
        {
            tc_result = ADCGoodResBuf[1];

            tp_result = ADCGoodResBuf[2];

            temperature_result = (int64_t)(temp_cal_tl * temp_cal_vph * tc_result - (int64_t)temp_cal_vpl * temp_cal_th * tc_result
            - (int64_t)temp_cal_tl * temp_cal_vch * tp_result
            + (int64_t)temp_cal_th * temp_cal_vcl * tp_result);
            temperature_result /= ((int32_t)temp_cal_vcl * tp_result - (int32_t)temp_cal_vch * tp_result
            - (int32_t)temp_cal_vpl * tc_result
            + (int32_t)temp_cal_vph * tc_result);
            temperature_result >>= 4;
            printf("Val %d, Temp %d\r\n", ADCGoodResBuf[0], temperature_result); 

            data_ready = 0;
        } 
	}
}