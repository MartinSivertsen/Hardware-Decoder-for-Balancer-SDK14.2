/**
 * Copyright (c) 2015 - 2017, Nordic Semiconductor ASA
 * 
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 * 
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 * 
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 * 
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 * 
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */
/**
 * @file
 * @brief File with example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * @sa twi_master_with_twis_slave_example
 */

/**
 * @defgroup twi_master_with_twis_slave_example Example code presenting usage of drivers for TWIS slave and TWI in master mode
 *
 * This code presents the usage of two drivers:
 * - @ref nrf_twi_drv (in synchronous mode)
 * - @ref nrf_twis_drv (in asynchronous mode)
 *
 * On the slave, EEPROM memory is simulated.
 * The size of the simulated EEPROM is configurable in the config.h file.
 * Default memory value of the device is 320 bytes. It is simulated using internal RAM.
 * This RAM area is accessed only by simulated EEPROM so the rest of the application can access it
 * only using TWI commands via hardware configured pins.
 *
 * The selected memory chip uses a 7-bit constant address. Word to access is selected during
 * a write operation: first byte sent is used as the current address pointer.
 *
 * A maximum of an 8-byte page can be written in a single access.
 * The whole memory can be read in a single access.
 *
 * When the slave (simulated EEPROM) is initialized, it copies the given part of flash
 * (see EEPROM_SIM_FLASH_ADDRESS in config.h) into RAM (enabling the use of the slave for
 * bootloader use cases).
 *
 * Many variables like length of sequential writes/reads, TWI instance to use, endianness of
 * of slave bype addressing can be configured in config.h file
 *
 * Differences between real chip and simulated one:
 * 1. Simulated chip has practically 0 ms write time.
 *    This example does not poll the memory for readiness.
 * 2. During sequential read, when memory end is reached, there is no support for roll-over.
 *    It is recommended for master to assure that it does not try to read more than the page limits.
 *    If the master is not tracking this and trying to read after the page ends, then the slave will start to NACK
 *    for trying to over-read the memory. The master should end the transaction when slave starts NACKing, which could
 *    mean that the master has read the end of the page.
 * 3. It is possible to write a maximum of EEPROM_SIM_SEQ_WRITE_MAX_BYTES bytes in a single
 *    sequential write. However, in simulated EEPROM the whole address pointer is incremented.
 *    In a real device, writing would roll-over in memory page.
 *
 * On the master side, we communicate with that memory and allow write and read.
 * Simple commands via UART can be used to check the memory.
 *
 * Pins to short:
 * - @ref TWI_SCL_M - @ref EEPROM_SIM_SCL_S
 * - @ref TWI_SDA_M - @ref EEPROM_SIM_SDA_S
 *
 * Supported commands will be listed after Tab button press.
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include "config.h"
#include "twi_send.h"
#include "nrf_drv_twi.h"
#include "nrf_drv_twis.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf.h"
#include "bsp.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"


#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "quad_decoder.h"
#include "printing.h"


/*Quadrature decoder*/
#define QUAD_DECODER_USING_NRF_QDEC     0

#define QUAD_DECODER_A_PIN_A            10
#define QUAD_DECODER_A_PIN_B             9
#define QUAD_DECODER_B_PIN_A            22
#define QUAD_DECODER_B_PIN_B            23

#define QUAD_DECODER_A_TICKS_PER_REV    400.0f       //1060.0f (comment left from Jan Tore)
#define QUAD_DECODER_B_TICKS_PER_REV    400.0f
#define QUAD_DECODER_A_DIAMETER         30.0f       // Diameter of wheel attached to the encoder [mm]
#define QUAD_DECODER_B_DIAMETER         30.0f

#define QUAD_DECODER_SDA                15          // SDA pin from nRF52832 on balancing shield to nRF52832 on Development Kit
#define QUAD_DECODER_SCL                16          // SCL pin from nRF52832 on balancing shield to nRF52832 on Development Kit

#define M_PI                            3.14159265358f




// Configuring decoders
static quad_decoder_t quad_decoder_a =
    {
        .pin_a = QUAD_DECODER_A_PIN_A,
        .pin_b = QUAD_DECODER_A_PIN_B,
        .ticks_per_revolution = QUAD_DECODER_A_TICKS_PER_REV,
        .diameter = QUAD_DECODER_A_DIAMETER / 1000.0f
    };

static quad_decoder_t quad_decoder_b =
    {
        .pin_a = QUAD_DECODER_B_PIN_B,
        .pin_b = QUAD_DECODER_B_PIN_A,
        .ticks_per_revolution = QUAD_DECODER_B_TICKS_PER_REV,
        .diameter = QUAD_DECODER_B_DIAMETER / 1000.0f
    };



/**@brief Handler for quadrature decoder event
 *
 */

void quad_decoder_evt_handler_a(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint8_t state_a = nrf_gpio_pin_read(quad_decoder_a.pin_a);
    uint8_t state_b = nrf_gpio_pin_read(quad_decoder_a.pin_b);

    if((state_a != 0) && (state_b != 0))
        quad_decoder_a.counter++;
    else
        quad_decoder_a.counter--;
    

    #if PRINT_QUAD_DECODER_VALUES_ON_EVENT
        NRF_LOG_RAW_INFO("Quadrature A counter: %5d", quad_decoder_a.counter);
        //NRF_LOG_FLUSH();
    #endif
}



/**@brief Handler for quadrature decoder event
 *
 */

void quad_decoder_evt_handler_b(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint8_t state_a = nrf_gpio_pin_read(quad_decoder_b.pin_a);
    uint8_t state_b = nrf_gpio_pin_read(quad_decoder_b.pin_b);

    if((state_a != 0) && (state_b != 0))
        quad_decoder_b.counter++;
    else
        quad_decoder_b.counter--;
    
    #if PRINT_QUAD_DECODER_VALUES_ON_EVENT
        NRF_LOG_RAW_INFO("     Quadrature B counter: %5d\n", quad_decoder_b.counter);
        //NRF_LOG_FLUSH();
    #endif
}
#if 1
static uint32_t twi_send_handler(uint8_t ** p_data, uint32_t * p_len)
{

    int32_t quad_data_a = quad_decoder_a.counter;
    int32_t quad_data_b = quad_decoder_b.counter;
    //static int32_t quad_data = 0;

    static uint8_t quad_dec_buf[8];
    quad_dec_buf[0] = quad_data_a;
    quad_dec_buf[1] = quad_data_a >> 8;
    quad_dec_buf[2] = quad_data_a >> 16;
    quad_dec_buf[3] = quad_data_a >> 24;
    quad_dec_buf[4] = quad_data_b;
    quad_dec_buf[5] = quad_data_b >> 8;
    quad_dec_buf[6] = quad_data_b >> 16;
    quad_dec_buf[7] = quad_data_b >> 24;

    *p_data = quad_dec_buf;
    *p_len = 8;

    #if PRINT_TWI_TRANSMIT_BUFFER
        NRF_LOG_RAW_INFO("Dec A: %5d Buf0: %3d Buf1: %3d Buf2: %3d Buf3: %3d\n", 
        quad_data_a, quad_dec_buf[0], quad_dec_buf[1], quad_dec_buf[2], quad_dec_buf[3]);
    #endif

    #if PRINT_TWI_TRANSMIT_QUAD_COUNTER_VALUES
        NRF_LOG_RAW_INFO("Dec A: %5d Dec B: %5d \n", quad_data_a, quad_data_b);
    #endif


    return NRF_SUCCESS;
}
#endif

/**
 *  The beginning of the journey
 */
int main(void)
{
    ret_code_t err_code;
    bool epprom_error = 0;
    /* Initialization of UART */

    bsp_board_leds_init();

    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    /* Initializing simulated EEPROM */
    err_code = eeprom_simulator_init(twi_send_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);

    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    quad_decoder_init(&quad_decoder_a, quad_decoder_evt_handler_a);
    quad_decoder_init(&quad_decoder_b, quad_decoder_evt_handler_b);


    NRF_LOG_RAW_INFO("This has now become firmware for decoding encoder values on the balancing robot.\n\n");
    /* Welcome message */



    /* Main loop */
    while (1)
    {
        NRF_LOG_PROCESS();

        if (epprom_error != eeprom_simulator_error_check())
        {
            epprom_error = eeprom_simulator_error_check();
            if (epprom_error != 0)
            {
                NRF_LOG_RAW_INFO(
                        "WARNING: EEPROM transmission error detected.\r\n"
                        "Use 'x' command to read error word.\r\n"
                );
            }
        }
    }
}
