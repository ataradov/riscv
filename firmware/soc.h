#ifndef _SOC_H_
#define _SOC_H_

/*- Includes ----------------------------------------------------------------*/
#include <stdint.h>

/*- Definitions -------------------------------------------------------------*/
#define MMIO_REG(addr, type)   (*(volatile type *)(addr))
#define MMIO_PER(addr, type)   ((volatile type *)(addr))

//-----------------------------------------------------------------------------
typedef struct
{
  uint32_t     CSR;
  uint32_t     BR;
  uint32_t     DATA;
} Uart;

#define UART_CSR_TX_READY  (1 << 0)
#define UART_CSR_RX_READY  (1 << 1)

#define UART MMIO_PER(0x00010000, Uart)

//-----------------------------------------------------------------------------
typedef struct
{
  uint32_t     WRITE;
  uint32_t     SET;
  uint32_t     CLR;
  uint32_t     TGL;
  uint32_t     READ;
} Gpio;

#define GPIO_BIT_0   (1 << 0)
#define GPIO_BIT_1   (1 << 1)
#define GPIO_BIT_2   (1 << 2)
#define GPIO_BIT_3   (1 << 3)
#define GPIO_BIT_4   (1 << 4)
#define GPIO_BIT_5   (1 << 5)
#define GPIO_BIT_6   (1 << 6)
#define GPIO_BIT_7   (1 << 7)
#define GPIO_BIT_8   (1 << 8)
#define GPIO_BIT_9   (1 << 9)
#define GPIO_BIT_10  (1 << 10)
#define GPIO_BIT_11  (1 << 11)
#define GPIO_BIT_12  (1 << 12)
#define GPIO_BIT_13  (1 << 13)
#define GPIO_BIT_14  (1 << 14)
#define GPIO_BIT_15  (1 << 15)
#define GPIO_BIT_16  (1 << 16)
#define GPIO_BIT_17  (1 << 17)
#define GPIO_BIT_18  (1 << 18)
#define GPIO_BIT_19  (1 << 19)
#define GPIO_BIT_20  (1 << 20)
#define GPIO_BIT_21  (1 << 21)
#define GPIO_BIT_22  (1 << 22)
#define GPIO_BIT_23  (1 << 23)
#define GPIO_BIT_24  (1 << 24)
#define GPIO_BIT_25  (1 << 25)
#define GPIO_BIT_26  (1 << 26)
#define GPIO_BIT_27  (1 << 27)
#define GPIO_BIT_28  (1 << 28)
#define GPIO_BIT_29  (1 << 29)
#define GPIO_BIT_30  (1 << 30)
#define GPIO_BIT_31  (1 << 31)

#define GPIO MMIO_PER(0x00020000, Gpio)

//-----------------------------------------------------------------------------
typedef struct
{
  uint32_t     CSR;
  uint32_t     COUNT;
  uint32_t     COMPARE;
} Timer;

#define TIMER_CSR_ENABLE   (1 << 0)
#define TIMER_CSR_DISABLE  (1 << 1)
#define TIMER_CSR_OVERFLOW (1 << 2)

#define TIMER MMIO_PER(0x00040000, Timer)

#endif // _SOC_H_

