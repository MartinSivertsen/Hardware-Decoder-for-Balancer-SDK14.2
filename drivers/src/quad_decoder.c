
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_gpiote.h"
#include "quad_decoder.h"
#include "nrf_log.h"

#define M_PI    3.14159f

void quad_decoder_init(quad_decoder_t * decoder, quad_decoder_evt_handler_t evt_handler)
{
    uint32_t err_code;
    decoder->evt_handler = evt_handler;

    if(!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        APP_ERROR_CHECK(err_code);
    }

    // Configuring pins
    nrf_drv_gpiote_in_config_t config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
    config.pull = NRF_GPIO_PIN_NOPULL;
    err_code = nrf_drv_gpiote_in_init(decoder->pin_a, &config, decoder->evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_gpio_cfg_input(decoder->pin_b, NRF_GPIO_PIN_NOPULL);
    nrf_drv_gpiote_in_event_enable(decoder->pin_a, true);

    //NRF_LOG_RAW_INFO("Quadrature decoder initialized.\n");
}
