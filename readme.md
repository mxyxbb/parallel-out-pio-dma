# 【树莓派Pico】16位100MHz并口高速输出数据|输出并口数据|PIO|DMA

![pic](https://github.com/mxyxbb/parallel-out-pio-dma/assets/53026754/26171de4-fc1b-4eba-8eeb-e06e9d1557f9)
![pic1](https://github.com/mxyxbb/parallel-out-pio-dma/assets/53026754/a62d74bf-eb1e-465c-a52d-400801682afc)
![pic2](https://github.com/mxyxbb/parallel-out-pio-dma/assets/53026754/6d573e20-e375-42a4-814d-536ba762aea1)

# 主要内容

本文实现的功能实际上十分类似于example中的逻辑分析仪logic_analyser例程。主要是将输入修改为了输出。

## pio程序内容

通过1个wait命令读取时钟信号
通过out pins, 16指令来向16个引脚输出信号

```
.program parallel_out

; This is just a simple clocked parallel TX. At 200 MHz system clock we can
; sustain up to 100 * 32 Mbps (16 used here).
; Data on OUT pin 0~15
; Clock input on GPIO pin 16

.wrap_target
    wait 1 gpio,16  ; stall here if no clockin (trigger high)
    out pins, 16
.wrap
```

## pio初始化函数

提供了两个函数，parallel_out_program_init函数用于初始化
pin_base和pin_count表示设置从标号pin_base开始的pin_count个引脚为输出引脚。本例中输出引脚为pin0~15.
初始化函数内部依序进行了
- 1.使用pio_gpio_init对用到的引脚进行**初始化**
- 2.使用pio_sm_set_consecutive_pindirs对用到的引脚设置**方向**
- 3.设置state machine的基本参数: out_pins、clkdiv、out_shift、fifo_join
- 其中out_shift的参数表示：true输出移位寄存器右移, true自动装载, pin_count移位个数
- 4.使用pio_sm_init进行状态机初始化
函数parallel_out_get用于测试，功能为堵塞式写入tx fifo（实际中采用dma方式，不使用该函数）
- 使用pio_sm_is_tx_fifo_full获取tx fifo状态
- 满时返回true，等待autopull。
- 空时返回false，后面就将数据x写入到tx fifo内。
```
% c-sdk {

// clock_pin should be a real pin number
static inline void parallel_out_program_init(PIO pio, uint sm, uint offset, uint pin_base, uint pin_count, uint clock_pin, float div) {
    
    for (uint16_t i = 0; i < pin_count; i++)
        pio_gpio_init(pio, pin_base+i);
    pio_gpio_init(pio, clock_pin); //wait gpio should be inited. 
    pio_sm_set_consecutive_pindirs(pio, sm, pin_base, pin_count, true);
    pio_sm_config c = parallel_out_program_get_default_config(offset);
    sm_config_set_out_pins(&c, pin_base, pin_count);
    sm_config_set_clkdiv(&c, div);
    sm_config_set_out_shift(&c, true, true, pin_count);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

static inline void parallel_out_put(PIO pio, uint sm, uint16_t x) {
    while (pio_sm_is_tx_fifo_full(pio, sm))
        ;
    *(volatile uint16_t*)&pio->txf[sm] = x;
}

%}
```

## 主程序内容

在main.c中，在开头使用下面两个函数将rp2040的主频超频到200MHz。

```
    vreg_set_voltage(VREG_VOLTAGE_MAX);
    set_sys_clock_khz(200*1000, true);
```

准备部分是申请ram空间，创建一个待发送的数组，并向数组内写入测试数据。
```
uint16_t *capture_buf = malloc(buf_size_halfwords * sizeof(uint16_t));
fill_buff(capture_buf, buf_size_halfwords);
```

**关键程序**如下。主要行为依序为
- 1.获取两个没用到的dma通道(互相触发，循环发送)
- 2.向pio指令空间中加载pio程序
- 3.初始化pio程序与相关引脚，本例中数据输出引脚为0~15，时钟读取引脚为16.
- 4.初始化dma传输的源，目的，和数量。并设置一个dma传输完后触发另外一个dma。
- 5.启动其中一个dma传输
- 6.启动pio程序

```
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
```

相关函数如下。该函数用于初始化dma参数
- 1.设置传输宽度为16位
- 2.read_increment设置为是（从capture_buf为首地址的一系列空间中读取）
- 3.write_increment设置为否（固定向sm的txfifo中写入）
- 4.set_dreq表示设置dma请求源，pio_get_dreq(pio, sm, true)表示pio的tx请求。
- 就是说tx空了就立马叫dma搬来。
- 5.set_chain_to表示本dma通道传输结束后触发另一个dma通道chan2开始传输。
- 6.最后是简单设置目的地址、源地址、传输数量，并设置是否立马开始传输（否）。

```
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
```

## 测试程序

下面是接续在**关键程序**之后的测试程序，主要功能为
- 等待dma_chan2传输完成
- 重新设置一下（重置）dma_chan2的读地址
- 等待dma_chan传输完成
- 重新设置一下（重置）dma_chan的读地址
- 这样dma_chan2传输完成后就会触发dma_chan的传输。dma_chan传输完成后就会触发dma_chan2的传输。达到循环传输的目的。

```
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
```
