#include "gd32f10x.h"
#include "tsm_rtc.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"


uint32_t RTCSRC_FLAG = 0;

void usart_config(void);

int main(void)
{

    /* configure systick */
    systick_config();
    rcu_periph_clock_enable(RCU_GPIOC);
    //RCU = REset Clock Unit
    rcu_periph_clock_enable(RCU_AF);

    //SWD remap
    gpio_pin_remap_config(GPIO_SWJ_SWDPENABLE_REMAP, ENABLE);
    //gpio init
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);

    //uart0
    usart_config();
    // printf("MCU START");

    // RTC
    nvic_configuration();

    if((bkp_data_read(BKP_DATA_0) != 0xA5A5) || (0x00 == RTCSRC_FLAG)){
        printf("\r\n this is a RTC Demo \r\n");
        printf("\r\n RTC not yet configured..");

        rtc_configuration();
        printf("\r\n RTC configured..");

        time_adjust();
        bkp_data_write(BKP_DATA_0, 0xA5A5);
    } else {
        if (rcu_flag_get(RCU_FLAG_PORRST) != RESET){
            printf("\r\n\n Power on reset occured..");
        } else if (rcu_flag_get(RCU_FLAG_SWRST) != RESET){
            printf("\r\n\n External reset occurred.. ");
        }

        rcu_periph_clock_enable(RCU_PMU);
        pmu_backup_write_enable();

        printf("\r\n no need to configure RTC .." );
        rtc_register_sync_wait();
        rtc_interrupt_enable(RTC_INT_SECOND);

        rtc_lwoff_wait();
    }
#ifdef RTCCLOCKOUTPUT_ENABLE
    /* enable PMU and BKPI clocks */
    rcu_periph_clock_enable(RCU_BKPI);
    rcu_periph_clock_enable(RCU_PMU);
    /* allow access to BKP domain */
    pmu_backup_write_enable();

    /* disable the tamper pin */
    bkp_tamper_detection_disable();

    /* enable RTC clock output on tamper Pin */
    bkp_rtc_calibration_output_enable();
#endif

    //clear reset flags
    rcu_all_reset_flag_clear();

    time_show();

    //WWDGT
    if(RESET != rcu_flag_get(RCU_FLAG_WWDGTRST)){
        printf("mcu restarted");
        rcu_all_reset_flag_clear();
        // while(1){
        // }
    }
    // enable wwdgt clock
    rcu_periph_clock_enable(RCU_WWDGT);
    
    /*
    *  set WWDGT clock = (PCLK1 (72MHz)/4096)/8 = 2197 (~45.5 us) in real test < 59 ms then MCU will restart 
    *  set counter value to 127
    *  set window value to 127
    */
    wwdgt_config(127, 127, WWDGT_CFG_PSC_DIV8);
    wwdgt_enable();

    while(1)
    {
        // gpio_bit_set(GPIOC, GPIO_PIN_13);
        // delay_1ms(500);
        // gpio_bit_reset(GPIOC, GPIO_PIN_13);
        // delay_1ms(500);
        gpio_bit_write(GPIOC,GPIO_PIN_13,(SET));
        delay_1ms(20);
        gpio_bit_write(GPIOC,GPIO_PIN_13,(RESET));
        delay_1ms(35);
        // uint8_t status_wdt = wwdgt_flag_get();
        printf("No restart");
        wwdgt_counter_update(127);

    }
}



void usart_config(void)
{
    rcu_periph_clock_enable(RCU_USART0);

    #if defined USART_REMAP
        rcu_periph_clock_enable(RCU_GPIOB);
        rcu_periph_clock_enable(RCU_AF);
        gpio_pin_remap_config(GPIO_USART0_REMAP,ENABLE);
        gpio_init(GPIOB, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
        gpio_init(GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    #else
        rcu_periph_clock_enable(RCU_GPIOA);
        gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);       
        gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
    #endif

    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200U);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);        
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE); 
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);       
}

int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART0, (uint8_t)ch);
    while (RESET == usart_flag_get(USART0, USART_FLAG_TBE));
    return ch;
    
}


