
/* Author, Copyright: Oleg Borodin <onborodin@gmail.com> 2018 */

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/i2c.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/exti.h>


#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <wctype.h>
#include <ctype.h>
#include <locale.h>
#include <wchar.h>
#include <time.h>

#include <syscall.h>

#include <buffer.h>
#include <uastdio.h>

#include <ds3231.h>
#include <datetime.h>

#include <st7735.h>
#include <console.h>

#define LED_PORT    GPIOC
#define LED_PIN     GPIO13

#define BUTTON_PORT GPIOA
#define BUTTON_PIN  GPIO0


void delay(uint32_t n) {
    for (volatile int i = 0; i < n; i++)
        __asm__("nop");
}

static void clock_setup(void) {
    rcc_clock_setup_in_hse_8mhz_out_72mhz();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    rcc_periph_clock_enable(RCC_TIM2);
    rcc_periph_clock_enable(RCC_TIM3);
    rcc_periph_clock_enable(RCC_TIM3);

    rcc_periph_clock_enable(RCC_AFIO);

    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_I2C1);
    rcc_periph_clock_enable(RCC_SPI1);
    rcc_periph_clock_enable(RCC_SPI2);
    rcc_periph_clock_enable(RCC_ADC1);

    rcc_periph_clock_enable(RCC_DMA1);

}

void uart_setup(void) {

    usart_disable(USART1);
    nvic_enable_irq(NVIC_USART1_IRQ);

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO_USART1_RX);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_enable_rx_interrupt(USART1);

    usart_enable(USART1);
}

inline bool usart_recv_is_ready(uint32_t usart) {
    return (USART_SR(usart) & USART_SR_RXNE);
}

inline bool usart_rx_int_is_enable(uint32_t usart) {
    return (USART_CR1(USART1) & USART_CR1_RXNEIE);
}

void usart1_isr(void) {
    static uint8_t data = 0;

    if (usart_rx_int_is_enable(USART1) && usart_recv_is_ready(USART1)) {

        gpio_toggle(GPIOC, GPIO13);
        data = usart_recv(USART1);
        buffer_put_byte(&stdin_buffer, data);
        buffer_put_byte(&stdout_buffer, data);
        if (data == '\r')
            buffer_put_byte(&stdout_buffer, '\n');
        //usart_enable_tx_interrupt(USART1);
    }
}

static void tim2_setup(void) {

    nvic_enable_irq(NVIC_TIM2_IRQ);

    timer_reset(TIM2);

    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_direction_up(TIM2);
    timer_set_prescaler(TIM2, 16);

    timer_disable_preload(TIM2);
    timer_continuous_mode(TIM2);
    timer_set_period(TIM2, 0xFFFF);

    timer_set_oc_value(TIM2, TIM_OC1, 1);
    timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    timer_enable_counter(TIM2);
}

void tim2_isr(void) {
    if (timer_get_flag(TIM2, TIM_SR_CC1IF)) {
        timer_clear_flag(TIM2, TIM_SR_CC1IF);
        uint8_t c;
        while ((c = buffer_get_byte(&stdout_buffer)) > 0)
           usart_send_blocking(USART1, c);
    }
}

void tim3_setup(void) {

    nvic_enable_irq(NVIC_TIM3_IRQ);

    timer_reset(TIM3);

    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    timer_direction_up(TIM3);
    timer_set_prescaler(TIM3, 16);
    timer_disable_preload(TIM3);
    timer_continuous_mode(TIM3);
    timer_set_period(TIM3, 0xFFFF);
    timer_set_oc_value(TIM3, TIM_OC1, 1);
    timer_enable_irq(TIM3, TIM_DIER_CC1IE);

    timer_enable_counter(TIM3);
}

volatile uint32_t tim3_counter = 0;
volatile uint32_t tim3_oc1_counter = 0;

void tim3_isr(void) {

    if (timer_get_flag(TIM3, TIM_DIER_UIE)) {
        timer_clear_flag(TIM3, TIM_DIER_UIE);
        tim3_counter++;
    }

    if (timer_get_flag(TIM3, TIM_SR_CC1IF)) {
        timer_clear_flag(TIM3, TIM_SR_CC1IF);
        tim3_oc1_counter++;
    }
}


volatile static uint32_t systick_counter_ms = 0;

static void systick_setup(void) {

    nvic_enable_irq(NVIC_SYSTICK_IRQ);
    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
    systick_set_reload(9000 - 1);
    systick_interrupt_enable();
    systick_counter_enable();
}

void sys_tick_handler(void) {
    systick_counter_ms++;
}

void delay_ms(uint32_t t) {
    systick_counter_ms = 0;
    while (systick_counter_ms < t);
}

void rtc_setup(void) {
    rtc_auto_awake(RCC_LSE, 0x7FFF);
    nvic_enable_irq(NVIC_RTC_IRQ);
    nvic_set_priority(NVIC_RTC_IRQ, 1);
    rtc_interrupt_enable(RTC_SEC);
}

volatile uint32_t loop_counter = 0;
volatile uint32_t rate_value = 0;


void rtc_isr(void) {
    rtc_clear_flag(RTC_SEC);
    rate_value = loop_counter;
    loop_counter = 0;
}

volatile uint16_t adc_res[4];

void dma_setup(void) {
    dma_disable_channel(DMA1, DMA_CHANNEL1);

    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);

    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);

    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t) &ADC_DR(ADC1));

    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t) &adc_res);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, 2);

    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);
}

void adc_dma_setup(void) {
    static uint8_t channel_seq[16];

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO2);
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO1);

    adc_power_off(ADC1);

    adc_enable_scan_mode(ADC1);
    adc_set_continuous_conversion_mode(ADC1);
    adc_disable_discontinuous_mode_regular(ADC1);

    adc_enable_external_trigger_regular(ADC1, ADC_CR2_EXTSEL_SWSTART);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_7DOT5CYC);

    adc_power_on(ADC1);
    delay(10);

    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
    adc_enable_temperature_sensor();

    channel_seq[0] = 1;
    channel_seq[1] = 2;
    channel_seq[2] = 16;
    adc_set_regular_sequence(ADC1, 2, channel_seq);

    adc_enable_dma(ADC1);
    delay(100);
    adc_start_conversion_regular(ADC1);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);
}

void dma1_channel1_isr(void) {
    dma_clear_interrupt_flags(DMA1, DMA_CHANNEL1, DMA_IFCR_CGIF1);
}

void nvic_setup(void) {
    //nvic_enable_irq(NVIC_HARD_FAULT_IRQ);
}


uint16_t exti_line_state;

static void exti_setup(void) {
    nvic_enable_irq(NVIC_EXTI0_IRQ);

    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);
    //gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, GPIO6);
    //gpio_set(GPIOA, GPIO6);

    exti_select_source(EXTI0, GPIOA);
    exti_set_trigger(EXTI0, EXTI_TRIGGER_BOTH);
    exti_enable_request(EXTI0);
}

void exti0_isr(void) {
    exti_line_state = GPIOA_IDR;

    if ((exti_line_state & GPIO0) != 0) {
        gpio_clear(LED_PORT, LED_PIN);
    } else {
        gpio_set(LED_PORT, LED_PIN);
    }

    //gpio_toggle(LED_PORT, LED_PIN);

    printf("INT\r\n");

    exti_reset_request(EXTI0);
}


int16_t get_mcu_temp(void) {
    float V_25 = 1.45;
    float Slope = 4.3e-3;
    float Vref = 1.78;
    float V_sense = adc_res[3]/4096.0 * Vref;
    float temp = (V_25 - V_sense)/Slope + 25.0;
    return (int16_t)temp;
}

int main(void) {

    delay(10000);
    clock_setup();
    io_setup();
    uart_setup();

    tim2_setup();
    tim3_setup();

    dma_setup();
    adc_dma_setup();
    nvic_setup();
    systick_setup();
    rtc_setup();

    uint32_t i = 1;

    lcd_spi_setup();
    console_setup();

    lcd_setup();
    lcd_clear();

    exti_setup();

    console_puts(&console, "STM32 CONSOLE\r\n");
    console_puts(&console, "READY>");


    gpio_set_mode(LED_PORT, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, LED_PIN);
    //gpio_set_mode(BUTTON_PORT, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, BUTTON_PIN);
    //gpio_set(BUTTON_PORT, BUTTON_PIN);

    while (1) {

        #define STR_LEN 16
        uint8_t str[STR_LEN + 1];

        //snprintf(str, STR_LEN, "Cnt %4lu",  timer_get_counter(TIM3));
        //console_xyputs(&console, 2, 0, str);

        snprintf(str, STR_LEN, "Rat %4d",  rate_value);
        console_xyputs(&console, 2, 0, str);

        snprintf(str, STR_LEN, "Tmp %4d", get_mcu_temp());
        console_xyputs(&console, 3, 0, str);

        int16_t x = adc_res[0] - 2048;
        int16_t y = adc_res[1] - 2048;

        snprintf(str, STR_LEN, "%5d %5d", x, y);
        console_xyputs(&console, 5, 0, str);

#if 0
        if (gpio_get(BUTTON_PORT, BUTTON_PIN)) {
            gpio_set(LED_PORT, LED_PIN);
            console_xyputc(&console, 2, 8, 'X');
        } else {
            gpio_clear(LED_PORT, LED_PIN);
            console_xyputc(&console, 2, 8, ' ');
        }
#endif
        snprintf(str, STR_LEN, "0x%08X", i);
        console_xyputs(&console, 8, 0, str);

        uint16_t y_value = i % (127 - 0);
        uint16_t y_prev;

        uint16_t x_value = i % (127 - 10);
        uint16_t x_prev;

        if ((x_prev != x_value) && (y_prev != y_value)) {
            lcd_write_rect(20, y_prev, 10, 10, LCD_BLUE);
            lcd_write_rect(20, y_value, 10, 10, LCD_GREEN);
        }
        y_prev = y_value;
        x_prev = x_value;

        //gpio_toggle(LED_PORT, LED_PIN);
        //printf("W%12u\r\n", i++);

        //delay_ms(10);
        loop_counter++;
        i++;
    }
    return 0;
}

/* EOF */
