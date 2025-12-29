#ifndef __UART5_H
#define __UART5_H

#include "sys.h"
#include <stdbool.h>

void UART5_Config(void);
void UART5_SendBuffer(uint8_t* data, uint16_t len);
uint8_t Calc_XOR(uint8_t *data, uint16_t len);
void UART5_Send_Float_Packet(float* data, uint8_t count);


#endif /* __UART5_H */
