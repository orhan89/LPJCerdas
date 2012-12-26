/**
  ******************************************************************************
  * @file    src/bh1750.c 
  * @author  Ricky Hariady
  * @version V0.1
  * @date    12/08/2010
  * @brief   BH1750 (Light Intensity Sensor) Configuration & Function
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_conf.h"

/** @addtogroup Lampu Penerangan Jalan Cerdas
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define I2C1_SLAVE_ADDRESS7 0x23
#define I2C_SPEED 100000

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* Private functions ---------------------------------------------------------*/
void I2C_Setup(void)
{
	I2C_InitTypeDef		I2C_InitStructure;
	GPIO_InitTypeDef	GPIO_InitStructure;
	
	/* Enable the clock for I2C1, GPIO B, GPIO C, and AFIO Peripheral */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOC| RCC_APB2Periph_AFIO, ENABLE);
		
	/* Disable the GPIO B Peripheral */
	GPIO_DeInit(GPIOB);

	/* Initialise GPIO B Port 6 (SDA) and Port 7 (SCL)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialise GPIO C Port 1 (ADDR)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

    /* Write low to ADDR (GPIOC.01)*/
	GPIO_WriteBit(GPIOC, GPIO_Pin_1, Bit_RESET);

	/* Disable the reset for I2C1 Peripheral */
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_I2C1, DISABLE);

	/* Disable the 12C1 Peripheral */
	I2C_DeInit(I2C1);

	
	/* Initialise the I2C1 Peripheral*/
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED;
	I2C_Init(I2C1, &I2C_InitStructure);

	/* Enable the I2C1 Peripheral*/
	I2C_Cmd(I2C1, ENABLE);
	
	/* Enable acknowledge for I2C1 Peripheral*/
	I2C_AcknowledgeConfig(I2C1, ENABLE);

}

void Init_BH1750(void)
{
	I2C_GenerateSTART(I2C1, ENABLE);
	//while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2C1, I2C1_SLAVE_ADDRESS7<<1, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	I2C_SendData(I2C1, 0x10);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));
}

u16 Receive(void)
{
	uint8_t Temp1, Temp2;
	uint16_t Value;
	u8 Address;

	Address = I2C1_SLAVE_ADDRESS7<<1;
	I2C_AcknowledgeConfig(I2C1, ENABLE);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_GetFlagStatus(I2C1, I2C_FLAG_SB));
	I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	Temp1 = I2C_ReceiveData(I2C1);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
	Temp2 = I2C_ReceiveData(I2C1);

	//I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C1, DISABLE);

	I2C_GenerateSTOP(I2C1, ENABLE);
	while(I2C_GetFlagStatus(I2C1, I2C_FLAG_STOPF));

	Value = (uint16_t)(Temp1<<8);
	Value |= Temp2;
	Value /= 1.2;

	return Value;
}
