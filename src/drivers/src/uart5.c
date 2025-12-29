#include "uart5.h"
#include "stm32f4xx.h"
#include <string.h>

#define UART5_TX_BUFFER_SIZE 128  // ??????

static uint8_t txBuffer[UART5_TX_BUFFER_SIZE];
static volatile uint16_t txHead = 0;
static volatile uint16_t txTail = 0;

void UART5_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_Init(GPIOC, &GPIO_InitStruct);
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStruct);

    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStruct.USART_Mode = USART_Mode_Tx;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;

    USART_Init(UART5, &USART_InitStruct);
    USART_Cmd(UART5, ENABLE);

    // ?? USART5 ??
    NVIC_InitStruct.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    USART_ITConfig(UART5, USART_IT_TXE, DISABLE); // ??????
}

uint8_t Calc_XOR(uint8_t *data, uint16_t len)
{
    uint8_t xor = 0;
    for (uint16_t i = 0; i < len; i++) xor ^= data[i];
    return xor;
}

// ????????
void UART5_SendBuffer(uint8_t* data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        uint16_t nextHead = (txHead + 1) % UART5_TX_BUFFER_SIZE;
        if (nextHead == txTail) {
            // ????,?????
            return;
        }
        txBuffer[txHead] = data[i];
        txHead = nextHead;
    }
    USART_ITConfig(UART5, USART_IT_TXE, ENABLE); // ????
}

// ?????(float????)
void UART5_Send_Float_Packet(float* data, uint8_t count)
{
    uint8_t packet[24];
    packet[0] = 0xAA;
    packet[1] = 0x55;
    memcpy(&packet[2], data, count * sizeof(float));
    packet[22] = Calc_XOR(&packet[2], 20);
    packet[23] = 0x0D;

    UART5_SendBuffer(packet, 24);
}

// USART5 ??????
void UART5_IRQHandler(void)
{
    if (USART_GetITStatus(UART5, USART_IT_TXE) != RESET) {
        if (txTail != txHead) {
            USART_SendData(UART5, txBuffer[txTail]);
            txTail = (txTail + 1) % UART5_TX_BUFFER_SIZE;
        } else {
            USART_ITConfig(UART5, USART_IT_TXE, DISABLE); // ????,????
        }
        USART_ClearITPendingBit(UART5, USART_IT_TXE);
    }
}
