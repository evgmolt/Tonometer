#include "main.h"
#include "Timers.h"

void nvic_config_1(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER1_IRQn, 1, 1);
}

void nvic_config_2(void)
{
    nvic_priority_group_set(NVIC_PRIGROUP_PRE1_SUB3);
    nvic_irq_enable(TIMER2_IRQn, 1, 1);
}

void timer_config_1(void)
{
    /* ----------------------------------------------------------------------------
    TIMER1 Configuration: 
    TIMER1CLK = SystemCoreClock/(12000*10000) = 1s).
    ---------------------------------------------------------------------------- */
    timer_parameter_struct timer_initpara;
    rcu_periph_clock_enable(RCU_TIMER1);
    timer_deinit(TIMER1);
    /* initialize TIMER init parameter struct */
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 119;//99;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = 9999;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER1, &timer_initpara);

    timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    timer_enable(TIMER1);
}

void timer_config_2(void)
{
//     ----------------------------------------------------------------------------
//    TIMER2 Configuration: 
//    TIMER1CLK = SystemCoreClock/(1000*f) = ... s).
//    ---------------------------------------------------------------------------- 
    timer_parameter_struct timer_initpara;

    rcu_periph_clock_enable(RCU_TIMER2);

    timer_deinit(TIMER2);
//  initialize TIMER init parameter struct 
    timer_struct_para_init(&timer_initpara);
    /* TIMER1 configuration */
    timer_initpara.prescaler         = 999;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = (120000/frequency)-1;//= 599; 479
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_init(TIMER2, &timer_initpara);

    //timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    //timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    //timer_enable(TIMER2);
}

void timer_2_start(void){
		timer_interrupt_flag_clear(TIMER2, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER2, TIMER_INT_UP);
    timer_enable(TIMER2);
}

void timer_2_stop(void){
	timer_disable(TIMER2);			
}

void timer_1_start(void){
		timer_interrupt_flag_clear(TIMER1, TIMER_INT_FLAG_UP);
    timer_interrupt_enable(TIMER1, TIMER_INT_UP);
    timer_enable(TIMER1);
}
void timer_1_stop(void){
	timer_disable(TIMER1);			
}
