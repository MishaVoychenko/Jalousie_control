#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_i2c.h"
#include "misc.h"
#include <string.h>
#include <stdio.h>

#define SYSCLK 72000000
#define PRESCALER 72
#define RX_BUF_SIZE 80

volatile char RX_FLAG_END_LINE = 0;
volatile char RXi;
volatile char RXc;
char RX_BUF[RX_BUF_SIZE] = {'\0'};
const int MIN_SERVO_STATE = 1000;
const int MAX_SERVO_STATE = 2000;
const uint16_t MIN_LUMINOSITY = 100;
const uint16_t MAX_LUMINOSITY = 2000;

enum status {
    ON,
    OFF
};

enum status jalousie = OFF;

// Sets System clock frequency to 72MHz
void SetSysClockTo72(void)
{
    ErrorStatus HSEStartUpStatus;
    // RCC system reset
    RCC_DeInit();

    // Enable HSE
    RCC_HSEConfig( RCC_HSE_ON);

    // Wait till HSE is ready
    HSEStartUpStatus = RCC_WaitForHSEStartUp();

    if (HSEStartUpStatus == SUCCESS)
    {

        RCC_HCLKConfig( RCC_SYSCLK_Div1);
        RCC_PCLK2Config( RCC_HCLK_Div1);
        RCC_PCLK1Config( RCC_HCLK_Div2);

        // PLLCLK = 8MHz * 9 = 72 MHz
        RCC_PLLConfig(0x00010000, RCC_PLLMul_9);

        // Enable PLL
        RCC_PLLCmd( ENABLE);

        // Wait till PLL is ready
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

        // Select PLL as system clock source
        RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK);

        // Wait till PLL is used as system clock source
        while (RCC_GetSYSCLKSource() != 0x08);
    }
}


// Initialization of USART1
void USART1_init(void)
{
    // Enable USART1 and GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // NVIC Configuration
    NVIC_InitTypeDef NVIC_InitStructure;
    // Enable the USARTs Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure the GPIOs
    GPIO_InitTypeDef GPIO_InitStructure;

    // Configure USART1 Tx as alternate function push-pull
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure USART1 Rx as input floating
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Configure the USART1
    USART_InitTypeDef USART_InitStructure;

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

    USART_Init(USART1, &USART_InitStructure);

    // Enable USART1
    USART_Cmd(USART1, ENABLE);

    // Enable the USART1 Receive interrupt
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
}

void clear_RXBuffer(void) {
    for (RXi=0; RXi<RX_BUF_SIZE; RXi++)
        RX_BUF[RXi] = '\0';
    RXi = 0;
}

// interrupt USART1
void USART1_IRQHandler(void)
{
    if ((USART1->SR & USART_FLAG_RXNE) != (u16)RESET)
    {
        RXc = USART_ReceiveData(USART1);
        RX_BUF[RXi] = RXc;
        RXi++;

        if (RXc != 13) {
            if (RXi > RX_BUF_SIZE-1) {
                clear_RXBuffer();
            }
        }
        else {
            RX_FLAG_END_LINE = 1;
        }
    }
}

void USART1_send(char *pucBuffer)
{
    while (*pucBuffer)
    {
        USART_SendData(USART1, *pucBuffer++);
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }
    }
}

// Initialization of I2C2
void I2C2_init(void)
{
    I2C_InitTypeDef I2C_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    // Enable I2C2 and GPIOB clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure I2C pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // I2C configuration
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x38;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = 100000;

    // I2C Peripheral Enable
    I2C_Cmd(I2C2, ENABLE);

    // Apply I2C configuration after enabling it
    I2C_Init(I2C2, &I2C_InitStructure);
}

// Read from light sensor
uint16_t BH1750_read(void)
{
    uint8_t buff[2];
    I2C_AcknowledgeConfig(I2C2, ENABLE);
    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    // Send device address
    I2C_Send7bitAddress(I2C2, 0x46, I2C_Direction_Transmitter);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
    // Send command
    I2C_SendData(I2C2, 0x10);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTING));
    I2C_GenerateSTART(I2C2, ENABLE);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));
    // Send device address
    I2C_Send7bitAddress(I2C2, 0x47, I2C_Direction_Receiver);
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
    // Read data
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    buff[0] = I2C_ReceiveData(I2C2);
    // Read data
    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
    buff[1] = I2C_ReceiveData(I2C2);
    I2C_GenerateSTOP(I2C2, ENABLE);
    I2C_AcknowledgeConfig(I2C2, DISABLE);
    uint16_t val = 0;
    val = ((buff[0] << 8) | buff[1]) / 1.2;
    return val;
}

// Initialization of servo motor
void SERVO_init(void) {
    GPIO_InitTypeDef port;
    TIM_TimeBaseInitTypeDef timer;
    TIM_OCInitTypeDef timerPWM;

    // Enable GPIOB and TIM4 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

    // Configure the GPIO pin
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_6;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &port);

    // Configure the timer TIM4
    TIM_TimeBaseStructInit(&timer);
    timer.TIM_Prescaler = PRESCALER;
    timer.TIM_Period = SYSCLK / PRESCALER / 50;
    timer.TIM_ClockDivision = 0;
    timer.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &timer);

    TIM_OCStructInit(&timerPWM);
    timerPWM.TIM_Pulse = 2000;
    timerPWM.TIM_OCMode = TIM_OCMode_PWM1;
    timerPWM.TIM_OutputState = TIM_OutputState_Enable;
    timerPWM.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM4, &timerPWM);

    TIM_Cmd(TIM4, ENABLE);
}

void delay(uint32_t _time)
{
    _time = _time * 420;
    while (_time--)
    {
    }
}

// Turning angle of the servo motor
int SERVO_angle(uint16_t x)
{
    int TIM_Pulse;
    if (jalousie == ON)
    {
        if (x < MIN_LUMINOSITY)
        {
            x = MIN_LUMINOSITY;
        }
        if (x > MAX_LUMINOSITY)
        {
            x = MAX_LUMINOSITY;
        }

        TIM_Pulse = MAX_SERVO_STATE + (MIN_SERVO_STATE - MAX_SERVO_STATE) * (x - MAX_LUMINOSITY) / (MIN_LUMINOSITY - MAX_LUMINOSITY);
    } else if (jalousie == OFF)
    {
        if (x < MIN_LUMINOSITY-50)
        {
            x = MIN_LUMINOSITY-50;
        }
        if (x > MIN_LUMINOSITY+50)
        {
            x = MIN_LUMINOSITY+50;
        }
        TIM_Pulse = MAX_SERVO_STATE + (MIN_SERVO_STATE - MAX_SERVO_STATE) * 0.5 * (x - MIN_LUMINOSITY - 50) / (-100);
    }
    delay(2500);
    return TIM_Pulse;
}

int main(void)
{
    int TIM_Pulse = 0;
    uint16_t lumens = 0;
    char str[10] = {'\0'};
    enum status info = OFF;

    // Set System clock
    SetSysClockTo72();
    // Initialize USART1
    USART1_init();
    // Initialize I2C2
    I2C2_init();
    // Initialize servo motor
    SERVO_init();

    while (1)
    {
        lumens = BH1750_read();
        // Processing commands
        if (RX_FLAG_END_LINE == 1) {
            // Reset END_LINE Flag
            RX_FLAG_END_LINE = 0;
            if (strncmp(RX_BUF, "on\r", 3) == 0)
            {
                USART1_send("Jalousie are open\r\n");
                jalousie = ON;
                info = ON;
            } else if (strncmp(RX_BUF, "off\r", 4) == 0)
            {
                USART1_send("Jalousie are closed\r\n");
                jalousie = OFF;
                info = ON;
            } else if (strncmp(RX_BUF, "status\r", 7) == 0)
            {
                info = ON;
            } else if (strncmp(RX_BUF, "help\r", 5) == 0)
            {
                USART1_send("on - open the jalousie\r\noff - close the jalousie\r\nstatus - the status of jalousie\r\n");
            } else
            {
                USART1_send("Send 'help' to see all commands\r\n");
            }
            clear_RXBuffer();
        }

        TIM_Pulse = SERVO_angle(lumens);
        TIM4->CCR1 = TIM_Pulse;

        // Status output
        if (info == ON)
        {
            info = OFF;
            sprintf(str, "lumens %d\r\n", lumens);
            USART1_send(str);
            sprintf(str, "angle %d%%\r\n", 100 - 100 * (TIM_Pulse - MIN_SERVO_STATE) / (MAX_SERVO_STATE - MIN_SERVO_STATE));
            USART1_send(str);
        }
    }
}
