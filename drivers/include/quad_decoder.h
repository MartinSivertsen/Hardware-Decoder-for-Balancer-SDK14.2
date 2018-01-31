
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_drv_gpiote.h"


typedef void (*quad_decoder_evt_handler_t)(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

typedef struct
{
    uint8_t pin_a;
    uint8_t pin_b;
    uint8_t prev_state_a;
    uint8_t prev_state_b;
    int32_t counter;
    float distance;
    float speed;
    float diameter;                        /**< Diameter of wheel attached to the encoder [m] */
    uint32_t ticks_per_revolution;
    quad_decoder_evt_handler_t evt_handler;
} quad_decoder_t;


/*  @brief Initializes a quadrature decoder, using one GPIOTE channel (per decoder)
 */
void quad_decoder_init(quad_decoder_t * decoder, quad_decoder_evt_handler_t evt_handler) ;
