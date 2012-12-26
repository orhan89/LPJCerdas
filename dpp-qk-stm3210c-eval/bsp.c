/*****************************************************************************
* Product: DPP example, STM3210C-EVAL board, QK kernel
* Last Updated for Version: 4.5.02
* Date of the Last Update:  Jul 26, 2012
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) 2002-2012 Quantum Leaps, LLC. All rights reserved.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 2 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
v* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Contact information:
* Quantum Leaps Web sites: http://www.quantum-leaps.com
*                          http://www.state-machine.com
* e-mail:                  info@quantum-leaps.com
*****************************************************************************/
#include "qp_port.h"
#include "bsp.h"
#include "lpj.h"
#include <stdio.h>

Q_DEFINE_THIS_FILE

#include "stm32f10x.h"

static unsigned short header = 0x42;

typedef struct MessageTag {
    uint32_t msg;
    uint16_t address;
    uint8_t data;
    uint8_t checksum;
} Message;

static Message Tx;
static Message Rx;

unsigned char calcChecksum(Message * message) {
    /* printf("Message = %X\r\n\n", (*message).msg); */
    /* printf("Checksum = %X\r\n", header + (*message).address + (*message).data); */
    return (uint8_t) (header + (*message).address + (*message).data);
}

extern void PWMInit(void);
extern void I2C_Setup(void);
extern void Init_BH1750(void);
extern uint16_t Receive(void);
extern void PIR_Setup(void);
extern void Com_Init(void);

enum ISR_Priorities {   /* ISR priorities starting from the highest urgency */
    SYSTICK_PRIO,
    PIR_PRIO,
    USART2_PRIO,
    USART3_PRIO
    /* ... */
};

#ifdef Q_SPY
    QSTimeCtr QS_tickTime_;
    QSTimeCtr QS_tickPeriod_;
    static uint8_t l_SysTick_Handler;

    #define QS_BUF_SIZE   (2*1024)
    #define QS_BAUD_RATE  115200

    enum AppRecords {                 /* application-specific trace records */
        PHILO_STAT = QS_USER
    };
#endif

/*..........................................................................*/
void SysTick_Handler(void) __attribute__((__interrupt__));
void SysTick_Handler(void) {
    QK_ISR_ENTRY();                       /* inform QK-nano about ISR entry */
#ifdef Q_SPY
    uint32_t dummy = SysTick->CTRL;        /* clear NVIC_ST_CTRL_COUNT flag */
    QS_tickTime_ += QS_tickPeriod_;       /* account for the clock rollover */
#endif
    QF_TICK(&l_SysTick_Handler);           /* process all armed time events */

    QK_ISR_EXIT();                         /* inform QK-nano about ISR exit */
}

void USART2_IRQHandler(void) __attribute__((__interrupt__));
void USART2_IRQHandler(void) {
  QK_ISR_ENTRY();

  if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    USART_SendData(USART2, USART_ReceiveData(USART2));

    int c;
    uint32_t message;

    c = USART_ReceiveData(USART2);
//    fprintf(3,"test\r\n");

    if(c == header) {
	//printf("sesuai header\r\n");
	read(0,&message,3);
	//printf("asdasdasdasdasd\r\n");
	setbuf(stdout,NULL);
	setbuf(stdin,NULL);

	Rx.msg = 0x00000000;
	Rx.msg = message;
	Rx.address = (Rx.msg & 0x00FFC000)>>14;
	Rx.data = (Rx.msg & 0x00003F00)>>8;
	Rx.checksum = (Rx.msg & 0x000000FF);

	//message = 0x11001100;
	if(Rx.data == 0x02){
            static QEvent const lowEvt = {LowLight_SIG, 0};
            QActive_postFIFO(AO_LPJ, &lowEvt);
	}
	if(Rx.data == 0x03){
            static QEvent const highEvt = {HighLight_SIG, 0};
            QActive_postFIFO(AO_LPJ, &highEvt);
	}
	if(Rx.data == 0x04){
	    reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG);
	    ope->presence = (uint8_t) ADA;
	    QActive_postFIFO(AO_LPJ, (QEvent *)ope);
	}
	if(Rx.data == 0x05){
	    reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG);
	    ope->presence = (uint8_t) TIDAK_ADA;
	    QActive_postFIFO(AO_LPJ, (QEvent *)ope);
	}
    }

	/* if(c == 'a'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG); */
	/*     ope->presence = (uint8_t) ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

	/* if(c == 'b'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG); */
	/*     ope->presence = (uint8_t) TIDAK_ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

	/* if(c == 'c'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportAfter_SIG); */
	/*     ope->presence = (uint8_t) ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

	/* if(c == 'd'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportAfter_SIG); */
	/*     ope->presence = (uint8_t) TIDAK_ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

        /* if(c == 'e'){ */
	/*     printf("message = %X\r\n", message); */
        /*     static QEvent const lowEvt = {LowLight_SIG, 0}; */
        /*     QActive_postFIFO(AO_LPJ, &lowEvt); */
        /* } */
        /* if(c == 'f'){ */
        /*     static QEvent const highEvt = {HighLight_SIG, 0}; */
        /*     QActive_postFIFO(AO_LPJ, &highEvt); */
        /* } */
  }
  
  if(USART_GetITStatus(USART2, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART2, 0x41);

    /* Disable the USARTy Transmit interrupt */
    USART_ITConfig(USART2, USART_IT_TXE, DISABLE);
  }

  QK_ISR_EXIT();
}

void USART3_IRQHandler(void) __attribute__((__interrupt__));
void USART3_IRQHandler(void) {
  QK_ISR_ENTRY();
  //fprintf(stdout, "Ada Interupsi\r\n");
  if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
  {
    /* Read one byte from the receive data register */
    USART_SendData(USART2, USART_ReceiveData(USART3));

    int c;
    uint32_t message;

    c = USART_ReceiveData(USART3);

    if(c == header)
	//printf("sesuai header\r\n");
	//printf("sesuai header\r\n");
	read(3,&message,3);
	//printf("asdasdasdasdasd\r\n");
	setbuf(stdout,NULL);
	setbuf(stdin,NULL);

	Rx.msg = 0x00000000;
	Rx.msg = message;
	Rx.address = (Rx.msg & 0x00FFC000)>>14;
	Rx.data = (Rx.msg & 0x00003F00)>>8;
	Rx.checksum = (Rx.msg & 0x000000FF);

	//message = 0x11001100;
	if(Rx.data == 0x02){
            static QEvent const lowEvt = {LowLight_SIG, 0};
            QActive_postFIFO(AO_LPJ, &lowEvt);
	}
	if(Rx.data == 0x03){
            static QEvent const highEvt = {HighLight_SIG, 0};
            QActive_postFIFO(AO_LPJ, &highEvt);
	}
	if(Rx.data == 0x04){
	    reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG);
	    ope->presence = (uint8_t) ADA;
	    QActive_postFIFO(AO_LPJ, (QEvent *)ope);
	}
	if(Rx.data == 0x05){
	    reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG);
	    ope->presence = (uint8_t) TIDAK_ADA;
	    QActive_postFIFO(AO_LPJ, (QEvent *)ope);
	}

	/* Rx.address = (Rx.msg & 0x00FFC000)>>14; */
	/* Rx.data = (Rx.msg & 0x00003F00)>>8; */
	/* Rx.checksum = (Rx.msg & 0x000000FF); */
	printf("Sisi Penerima\r\n");
	printf("-------------\r\n");
	printf("Message = %X\r\n", (Rx.msg & 0x00FFFFFF));
	printf("Address = %X\r\n", Rx.address);
	printf("Data = %X\r\n", Rx.data);
	printf("Checksum = %X\r\n", Rx.checksum);

	/* if(calcChecksum(&Rx) == Rx.checksum) {  */
	/*     printf("Checksum sesuai\r\n"); */
	/* } */
	/* else */
	/*     printf("Checksum tidak sesuai"); */

	/* if(c == 'a'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG); */
	/*     ope->presence = (uint8_t) ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

	/* if(c == 'b'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportBefore_SIG); */
	/*     ope->presence = (uint8_t) TIDAK_ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

	/* if(c == 'c'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportAfter_SIG); */
	/*     ope->presence = (uint8_t) ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

	/* if(c == 'd'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportAfter_SIG); */
	/*     ope->presence = (uint8_t) TIDAK_ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */

	/* if(c == 'd'){ */
	/*     reportEvt *ope = Q_NEW(reportEvt, reportAfter_SIG); */
	/*     ope->presence = (uint8_t) TIDAK_ADA; */
	/*     QActive_postFIFO(AO_LPJ, (QEvent *)ope); */
	/* } */
        /* if(c == 'e'){ */
        /*     static QEvent const lowEvt = {LowLight_SIG, 0}; */
        /*     QActive_postFIFO(AO_LPJ, &lowEvt); */
        /* } */
        /* if(c == 'f'){ */
        /*     static QEvent const lowEvt = {LowLight_SIG, 0}; */
        /*     QActive_postFIFO(AO_LPJ, &lowEvt); */
        /* } */
  }
  
  if(USART_GetITStatus(USART3, USART_IT_TXE) != RESET)
  {   
    /* Write one byte to the transmit data register */
    USART_SendData(USART3, 0x41);

    /* Disable the USARTy Transmit interrupt */
    USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
  }

  QK_ISR_EXIT();
}

void EXTI0_IRQHandler(void) __attribute__ ((__interrupt__));
void EXTI0_IRQHandler(void)
{
  QK_ISR_ENTRY();

  if(EXTI_GetITStatus(EXTI_Line0) != RESET){
      periksaPIR();
      EXTI_ClearITPendingBit(EXTI_Line0);
  }

  QK_ISR_EXIT();
}


/*..........................................................................*/
void BSP_init(void) {
//    uint32_t i,j=50;

    SystemInit();         /* initialize STM32 system (clock, PLL and Flash) */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, DISABLE);

    PWMInit(); /* PWM Initialization */

//I2C_Setup();

//Init_BH1750();
    
    PIR_Setup();

    Com_Init();

    setbuf(stdout,NULL);
    setbuf(stdin,NULL);

    fprintf(stdout,"Hello"); /* (*fd)._file); */

    /* if (QS_INIT((void *)0) == 0) {    /\* initialize the QS software tracing *\/ */
    /* 	Q_ERROR(); */
    /* } */

    QS_OBJ_DICTIONARY(&l_SysTick_Handler);

}

void turnOff(void) {
    TIM_SetCompare1(TIM1,5000);
}

void turnOn(void) {
    TIM_SetCompare1(TIM1, 0);
}

void Dimm(uint16_t dimm) {
    TIM_SetCompare1(TIM1, dimm);
}

void Bright(void) {
    TIM_SetCompare1(TIM1, 0);
}

void periksaLux(void) {
    static uint8_t lux;
    lux = Receive();

    if(lux<50){
         static QEvent const lowEvt = {LowLight_SIG, 0};
         QActive_postFIFO(AO_LPJ, &lowEvt);
    }
    else{
         static QEvent const highEvt = {HighLight_SIG, 0};
         QActive_postFIFO(AO_LPJ, &highEvt);
    }
}

void periksaPIR(void) {
    uint8_t tempPIR;

    tempPIR = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_0);
    
    /* if(tempPIR == 0) { */
    /*     static QEvent const pirEvt = {PIRIntr_SIG, 0}; */
    /*     QActive_postFIFO(AO_LPJ, (QEvent *)&pirEvt); */
    /* } */
    //printf("PIR = %d\r\n", tempPIR);
    pirEvt *pe = Q_NEW(pirEvt, PIRIntr_SIG);
    pe->pir = tempPIR;
    QActive_postFIFO(AO_LPJ, (QEvent *)pe);

}

void kirimReport(uint8_t data) {

    /* static Message tx; */
    /* static Message * ptx; */

    Tx.msg = 0x00000000;
    Tx.address = 0x0250;
    Tx.data = data;
    Tx.checksum = calcChecksum(&Tx);
    Tx.msg |= (Tx.address << 14);
    Tx.msg |= (Tx.data << 8);
    Tx.msg |=  Tx.checksum;
    //Tx.msg |= (header << 24);

    //ptx = &(tx.msg);

    //char a = pir+48;
    //printf("MSG = %X", Tx.msg);
    //write(1,&a,1);
    write(1,&(header),1);
    write(1,&(Tx.msg),4);

    /* Rx = Tx; */

    /* 	Rx.address = (Rx.msg & 0x00FFC000)>>14; */
    /* 	Rx.data = (Rx.msg & 0x00003F00)>>8; */
    /* 	Rx.checksum = (Rx.msg & 0x000000FF); */
    /* 	printf("Sisi Penerima\r\n"); */
    /* 	printf("-------------\r\n"); */
    /* 	printf("Message = %X\r\n", (Rx.msg & 0x00FFFFFF)); */
    /* 	printf("Address = %X\r\n", Rx.address); */
    /* 	printf("Data = %X\r\n", Rx.data); */
    /* 	printf("Checksum = %X\r\n", Rx.checksum); */

    /* 	if(calcChecksum(&Rx) == Rx.checksum) {  */
    /* 	    printf("Checksum sesuai\r\n"); */
    /* 	} */
    /* 	else */
    /* 	    printf("Checksum tidak sesuai"); */

}

/*..........................................................................*/
void QF_onStartup(void) {

    /* Set up and enable the SysTick timer.  It will be used as a reference
    * for delay loops in the interrupt handlers.  The SysTick timer period
    * will be set up for BSP_TICKS_PER_SEC.
    */
    SysTick_Config(720000);/* SystemFrequency_SysClk / BSP_TICKS_PER_SEC); */


                       /* set priorities of all interrupts in the system... */
    NVIC_SetPriority(SysTick_IRQn, SYSTICK_PRIO);
    NVIC_SetPriority(USART2_IRQn, USART2_PRIO);
    NVIC_SetPriority(USART3_IRQn, USART3_PRIO);
    NVIC_SetPriority(EXTI15_10_IRQn, PIR_PRIO);
    /* ... */
}
/*..........................................................................*/
void QF_onCleanup(void) {
}
/*..........................................................................*/
void QK_onIdle(void) {

#ifdef Q_SPY

    if ((USART2->SR & USART_FLAG_TXE) != 0) {              /* is TXE empty? */
        uint16_t b;

        QF_INT_DISABLE();
        b = QS_getByte();
        QF_INT_ENABLE();

        if (b != QS_EOD) {                              /* not End-Of-Data? */
           USART2->DR = (b & 0xFF);             /* put into the DR register */
        }
    }

#elif defined NDEBUG
    __WFI();                                          /* wait for interrupt */
#endif
}

/*..........................................................................*/
/* error routine that is called if the STM32 library encounters an error    */
void assert_failed(char const *file, int line) {
    Q_onAssert(file, line);
}

/*..........................................................................*/
void Q_onAssert(char const * const file, int line) {
    (void)file;                                   /* avoid compiler warning */
    (void)line;                                   /* avoid compiler warning */
    QF_INT_DISABLE();         /* make sure that all interrupts are disabled */

    printf("Assertion %s, line %d\n", file, line);

    for (;;) {       /* NOTE: replace the loop with reset for final version */
    }
}

/*--------------------------------------------------------------------------*/
#ifdef Q_SPY
/*..........................................................................*/
uint8_t QS_onStartup(void const *arg) {
    static uint8_t qsBuf[QS_BUF_SIZE];            /* buffer for Quantum Spy */
    QS_initBuf(qsBuf, sizeof(qsBuf));

                                     /* enable USART2 and GPIOA/AFIO clocks */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO,
                           ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);

                                       /* configure GPIOD.5 as push-pull... */
    GPIO_InitTypeDef  gpio_init;
    gpio_init.GPIO_Pin   = GPIO_Pin_5;
    gpio_init.GPIO_Speed = GPIO_Speed_50MHz;
    gpio_init.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &gpio_init);
                                  /* configure GPIOD.6 as input floating... */
    gpio_init.GPIO_Pin   = GPIO_Pin_6;
    gpio_init.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &gpio_init);

    USART_InitTypeDef usart_init;
    usart_init.USART_BaudRate            = QS_BAUD_RATE;
    usart_init.USART_WordLength          = USART_WordLength_8b;
    usart_init.USART_StopBits            = USART_StopBits_1;
    usart_init.USART_Parity              = USART_Parity_No ;
    usart_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart_init.USART_Mode                = USART_Mode_Tx;
    USART_Init(USART2, &usart_init);

    USART_ClockInitTypeDef usart_clk_init;
    usart_clk_init.USART_Clock               = USART_Clock_Disable;
    usart_clk_init.USART_CPOL                = USART_CPOL_Low;
    usart_clk_init.USART_CPHA                = USART_CPHA_2Edge;
    usart_clk_init.USART_LastBit             = USART_LastBit_Disable;
    USART_ClockInit(USART2, &usart_clk_init);

    USART_Cmd(USART2, ENABLE);                             /* enable USART2 */

    QS_tickPeriod_ = (QSTimeCtr)(SystemFrequency_SysClk / BSP_TICKS_PER_SEC);
    QS_tickTime_ = QS_tickPeriod_;        /* to start the timestamp at zero */

                                                 /* setup the QS filters... */
    QS_FILTER_ON(QS_ALL_RECORDS);

//    QS_FILTER_OFF(QS_QEP_STATE_EMPTY);
//    QS_FILTER_OFF(QS_QEP_STATE_ENTRY);
//    QS_FILTER_OFF(QS_QEP_STATE_EXIT);
//    QS_FILTER_OFF(QS_QEP_STATE_INIT);
//    QS_FILTER_OFF(QS_QEP_INIT_TRAN);
//    QS_FILTER_OFF(QS_QEP_INTERN_TRAN);
//    QS_FILTER_OFF(QS_QEP_TRAN);
//    QS_FILTER_OFF(QS_QEP_IGNORED);

    QS_FILTER_OFF(QS_QF_ACTIVE_ADD);
    QS_FILTER_OFF(QS_QF_ACTIVE_REMOVE);
    QS_FILTER_OFF(QS_QF_ACTIVE_SUBSCRIBE);
    QS_FILTER_OFF(QS_QF_ACTIVE_UNSUBSCRIBE);
    QS_FILTER_OFF(QS_QF_ACTIVE_POST_FIFO);
    QS_FILTER_OFF(QS_QF_ACTIVE_POST_LIFO);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET);
    QS_FILTER_OFF(QS_QF_ACTIVE_GET_LAST);
    QS_FILTER_OFF(QS_QF_EQUEUE_INIT);
    QS_FILTER_OFF(QS_QF_EQUEUE_POST_FIFO);
    QS_FILTER_OFF(QS_QF_EQUEUE_POST_LIFO);
    QS_FILTER_OFF(QS_QF_EQUEUE_GET);
    QS_FILTER_OFF(QS_QF_EQUEUE_GET_LAST);
    QS_FILTER_OFF(QS_QF_MPOOL_INIT);
    QS_FILTER_OFF(QS_QF_MPOOL_GET);
    QS_FILTER_OFF(QS_QF_MPOOL_PUT);
    QS_FILTER_OFF(QS_QF_PUBLISH);
    QS_FILTER_OFF(QS_QF_NEW);
    QS_FILTER_OFF(QS_QF_GC_ATTEMPT);
    QS_FILTER_OFF(QS_QF_GC);
//    QS_FILTER_OFF(QS_QF_TICK);
    QS_FILTER_OFF(QS_QF_TIMEEVT_ARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_AUTO_DISARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM_ATTEMPT);
    QS_FILTER_OFF(QS_QF_TIMEEVT_DISARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_REARM);
    QS_FILTER_OFF(QS_QF_TIMEEVT_POST);
    QS_FILTER_OFF(QS_QF_CRIT_ENTRY);
    QS_FILTER_OFF(QS_QF_CRIT_EXIT);
    QS_FILTER_OFF(QS_QF_ISR_ENTRY);
    QS_FILTER_OFF(QS_QF_ISR_EXIT);

//    QS_FILTER_OFF(QS_QK_MUTEX_LOCK);
//    QS_FILTER_OFF(QS_QK_MUTEX_UNLOCK);
    QS_FILTER_OFF(QS_QK_SCHEDULE);

    return (uint8_t)1;                                    /* return success */
}
/*..........................................................................*/
void QS_onCleanup(void) {
}
/*..........................................................................*/
QSTimeCtr QS_onGetTime(void) {            /* invoked with interrupts locked */
    if ((SysTick->CTRL & 0x00010000) == 0) {               /* COUNT no set? */
        return QS_tickTime_ - (QSTimeCtr)SysTick->VAL;
    }
    else {     /* the rollover occured, but the SysTick_ISR did not run yet */
        return QS_tickTime_ + QS_tickPeriod_ - (QSTimeCtr)SysTick->VAL;
    }
}
/*..........................................................................*/
void QS_onFlush(void) {
    uint16_t b;
    while ((b = QS_getByte()) != QS_EOD) {      /* while not End-Of-Data... */
        while ((USART2->SR & USART_FLAG_TXE) == 0) { /* while TXE not empty */
        }
        USART2->DR = (b & 0xFF);                /* put into the DR register */
    }
}
#endif                                                             /* Q_SPY */
/*--------------------------------------------------------------------------*/

/*****************************************************************************
* NOTE01:
* The blue LED is used to visualize the idle loop activity. The brightness
* of the LED is proportional to the frequency of invcations of the idle loop.
* Please note that the LED is toggled with interrupts locked, so no interrupt
* execution time contributes to the brightness of the User LED.
*/
