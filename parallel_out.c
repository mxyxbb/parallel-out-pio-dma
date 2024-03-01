/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "stdint.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"
#include "hardware/interp.h"
#include "hardware/vreg.h"
#include "hardware/pll.h"
#include "hardware/clocks.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/clocks.h"
#include "parallel_out.pio.h"



const uint CAPTURE_PIN_BASE = 0;
const uint CAPTURE_PIN_COUNT = 16;
const uint CLOCK_INPUT_PIN = 16;
const uint CAPTURE_N_SAMPLES = 65535;
const uint LED_PIN = 25;

int dma_chan;
int dma_chan2;

uint16_t *capture_buf;

static inline uint bits_packed_per_word(uint pin_count) {
    // If the number of pins to be sampled divides the shift register size, we
    // can use the full SR and FIFO width, and push when the input shift count
    // exactly reaches 32. If not, we have to push earlier, so we use the FIFO
    // a little less efficiently.
    const uint SHIFT_REG_WIDTH = 32;
    return SHIFT_REG_WIDTH - (SHIFT_REG_WIDTH % pin_count);
}

void fill_buff(uint16_t *buf, uint buf_size){
    for (uint i = 0; i < buf_size; i++)
    {
        buf[i]=i%0xffff;
    }
}

static inline void parallel_out_put_dma_circular(PIO pio, uint sm, uint dma_chan, uint16_t *capture_buf, size_t capture_size_halfwords, uint dma_chan2) {
    pio_sm_set_enabled(pio, sm, false);
    // Need to clear _input shift counter_, as well as FIFO, because there may be
    // partial ISR contents left over from a previous run. sm_restart does this.
    pio_sm_clear_fifos(pio, sm);
    pio_sm_restart(pio, sm);

    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, true));
    channel_config_set_chain_to(&c, dma_chan2); //trigger the dma_chan2 after trans finished
    dma_channel_configure(dma_chan, &c,
        &pio->txf[sm],        // Destination pointer
        capture_buf,      // Source pointer
        capture_size_halfwords, // Number of transfers
        false                // Start immediately
    );
    
    //pio_sm_set_enabled(pio, sm, true);
}

int main() {

    vreg_set_voltage(VREG_VOLTAGE_MAX);
    set_sys_clock_khz(200*1000, true);
    stdio_init_all();
    gpio_init(CLOCK_INPUT_PIN);
    gpio_set_dir(CLOCK_INPUT_PIN, false);
    sleep_ms(3000);
    printf("starting...\n");
    uint total_sample_bits = CAPTURE_N_SAMPLES * CAPTURE_PIN_COUNT;
    total_sample_bits += bits_packed_per_word(CAPTURE_PIN_COUNT) - 1;
    uint buf_size_halfwords = 2 * total_sample_bits / bits_packed_per_word(CAPTURE_PIN_COUNT);
    uint16_t *capture_buf = malloc(buf_size_halfwords * sizeof(uint16_t));
    hard_assert(capture_buf);

    fill_buff(capture_buf, buf_size_halfwords);

    PIO pio = pio0;
    uint sm = 0;
    dma_chan = dma_claim_unused_channel(true);
    dma_chan2 = dma_claim_unused_channel(true);
    
    uint offset = pio_add_program(pio, &parallel_out_program);
    parallel_out_program_init(pio, sm, offset, CAPTURE_PIN_BASE, CAPTURE_PIN_COUNT, CLOCK_INPUT_PIN, 1.f);
    // parallel_out_put(pio, sm, 0xffff);
    // sleep_ms(100);
    // parallel_out_put(pio, sm, 0);
    // sleep_ms(100);
    // parallel_out_put(pio, sm, 0xffff);
    // sleep_ms(100);
    // parallel_out_put(pio, sm, 0);
    // sleep_ms(100);
    
    // inter chain the two channel
    parallel_out_put_dma_circular(pio, sm, dma_chan, capture_buf, buf_size_halfwords, dma_chan2);
    parallel_out_put_dma_circular(pio, sm, dma_chan2, capture_buf, buf_size_halfwords, dma_chan);
    //start one channel
    dma_channel_start(dma_chan2);
    //start the pio sm
    pio_sm_set_enabled(pio, sm, true);

    //CPU退休
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while(1)
    {
        dma_channel_wait_for_finish_blocking(dma_chan2);
        dma_channel_set_read_addr(dma_chan2, capture_buf, false);
        dma_channel_wait_for_finish_blocking(dma_chan);
        dma_channel_set_read_addr(dma_chan, capture_buf, false);
         
        
        // gpio_put(LED_PIN, 1);
        // sleep_ms(500);
        // gpio_put(LED_PIN, 0);
        // sleep_ms(500);
    }

}
