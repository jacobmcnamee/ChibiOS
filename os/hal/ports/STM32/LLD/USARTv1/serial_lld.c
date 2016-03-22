/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    STM32/USARTv1/serial_lld.c
 * @brief   STM32 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/
#define USART1_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART1_RX_DMA_STREAM,                     \
                       STM32_USART1_RX_DMA_CHN)

#define USART1_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART1_TX_DMA_STREAM,                     \
                       STM32_USART1_TX_DMA_CHN)

#define USART2_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART2_RX_DMA_STREAM,                     \
                       STM32_USART2_RX_DMA_CHN)

#define USART2_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART2_TX_DMA_STREAM,                     \
                       STM32_USART2_TX_DMA_CHN)

#define USART3_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART3_RX_DMA_STREAM,                     \
                       STM32_USART3_RX_DMA_CHN)

#define USART3_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART3_TX_DMA_STREAM,                     \
                       STM32_USART3_TX_DMA_CHN)

#define UART4_RX_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_UART_UART4_RX_DMA_STREAM,                      \
                       STM32_UART4_RX_DMA_CHN)

#define UART4_TX_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_UART_UART4_TX_DMA_STREAM,                      \
                       STM32_UART4_TX_DMA_CHN)

#define UART5_RX_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_UART_UART5_RX_DMA_STREAM,                      \
                       STM32_UART5_RX_DMA_CHN)

#define UART5_TX_DMA_CHANNEL                                                \
  STM32_DMA_GETCHANNEL(STM32_UART_UART5_TX_DMA_STREAM,                      \
                       STM32_UART5_TX_DMA_CHN)

#define USART6_RX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART6_RX_DMA_STREAM,                     \
                       STM32_USART6_RX_DMA_CHN)

#define USART6_TX_DMA_CHANNEL                                               \
  STM32_DMA_GETCHANNEL(STM32_UART_USART6_TX_DMA_STREAM,                     \
                       STM32_USART6_TX_DMA_CHN)

#define dmaBufTail(sdp)                                                     \
  (STM32_SERIAL_DMA_BUFFER_SIZE - dmaStreamGetTransactionSize(sdp->dmarx))

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief USART1 serial driver identifier.*/
#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief USART2 serial driver identifier.*/
#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/** @brief USART3 serial driver identifier.*/
#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
SerialDriver SD3;
#endif

/** @brief UART4 serial driver identifier.*/
#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
SerialDriver SD4;
#endif

/** @brief UART5 serial driver identifier.*/
#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
SerialDriver SD5;
#endif

/** @brief USART6 serial driver identifier.*/
#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
SerialDriver SD6;
#endif

/** @brief UART7 serial driver identifier.*/
#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
SerialDriver SD7;
#endif

/** @brief UART8 serial driver identifier.*/
#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
SerialDriver SD8;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/** @brief Driver default configuration.*/
static const SerialConfig default_config =
{
  SERIAL_DEFAULT_BITRATE,
  0,
  USART_CR2_STOP1_BITS | USART_CR2_LINEN,
  0
};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/
static void send_dma(SerialDriver *sdp);

static void serve_dma_interrupt(SerialDriver *sdp) {
  osalSysLockFromISR();
  send_dma(sdp);
  osalSysUnlockFromISR();
}

/**
 * @brief   USART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void usart_init(SerialDriver *sdp, const SerialConfig *config) {
  USART_TypeDef *u = sdp->usart;

  /* Baud rate setting.*/
#if STM32_HAS_USART6
  if ((sdp->usart == USART1) || (sdp->usart == USART6))
#else
  if (sdp->usart == USART1)
#endif
    u->BRR = STM32_PCLK2 / config->speed;
  else
    u->BRR = STM32_PCLK1 / config->speed;

  sdp->rxbufhead = 0;
  dmaStreamAllocate(sdp->dmarx, 0, NULL, NULL);
  dmaStreamSetPeripheral(sdp->dmarx, &sdp->usart->DR);
  dmaStreamSetMemory0(sdp->dmarx, &sdp->rxbuf);
  dmaStreamSetTransactionSize(sdp->dmarx, STM32_SERIAL_DMA_BUFFER_SIZE);
  dmaStreamSetMode(sdp->dmarx, sdp->dmamode | STM32_DMA_CR_DIR_P2M |
                               STM32_DMA_CR_CIRC | STM32_DMA_CR_MINC);
  dmaStreamEnable(sdp->dmarx);

  dmaStreamSetPeripheral(sdp->dmatx, &sdp->usart->DR);
  dmaStreamSetMode(sdp->dmatx, sdp->dmamode | STM32_DMA_CR_DIR_M2P |
                               STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);

  /* Note that some bits are enforced.*/
  u->CR2 = config->cr2 | USART_CR2_LBDIE;
  u->CR3 = config->cr3 | USART_CR3_EIE | USART_CR3_DMAR | USART_CR3_DMAT;
  u->CR1 = config->cr1 | USART_CR1_UE | USART_CR1_PEIE |
                         USART_CR1_RXNEIE | USART_CR1_TE |
                         USART_CR1_RE;
  u->SR = 0;
  (void)u->SR;  /* SR reset step 1.*/
  (void)u->DR;  /* SR reset step 2.*/
}

/**
 * @brief   USART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an USART I/O block
 */
static void usart_deinit(USART_TypeDef *u) {

  u->CR1 = 0;
  u->CR2 = 0;
  u->CR3 = 0;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        USART SR register value
 */
static void set_error(SerialDriver *sdp, uint16_t sr) {
  eventflags_t sts = 0;

  if (sr & USART_SR_ORE)
    sts |= SD_OVERRUN_ERROR;
  if (sr & USART_SR_PE)
    sts |= SD_PARITY_ERROR;
  if (sr & USART_SR_FE)
    sts |= SD_FRAMING_ERROR;
  if (sr & USART_SR_NE)
    sts |= SD_NOISE_ERROR;
  chnAddFlagsI(sdp, sts);
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the USART
 */
static void serve_interrupt(SerialDriver *sdp) {
  USART_TypeDef *u = sdp->usart;
  uint16_t cr1 = u->CR1;
  uint16_t sr = u->SR;

  /* Special case, LIN break detection.*/
  if (sr & USART_SR_LBD) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    u->SR = ~USART_SR_LBD;
    osalSysUnlockFromISR();
  }

  /* Data available.*/
  osalSysLockFromISR();
  while ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) ||
         (sdp->rxbufhead != dmaBufTail(sdp))) {
    /* Error condition detection.*/
    if (sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE  | USART_SR_PE)) {
      set_error(sdp, sr);
      (void)u->DR;
    }
    if (sdp->rxbufhead != dmaBufTail(sdp)) {
      sdIncomingDataI(sdp, sdp->rxbuf[sdp->rxbufhead++]);
      sdp->rxbufhead %= STM32_SERIAL_DMA_BUFFER_SIZE;
    }
    sr = u->SR;
  }
  osalSysUnlockFromISR();

  /* Physical transmission end.*/
  if (sr & USART_SR_TC) {
    osalSysLockFromISR();
    if (oqIsEmptyI(&sdp->oqueue))
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
    u->CR1 = cr1 & ~USART_CR1_TCIE;
    u->SR = ~USART_SR_TC;
    osalSysUnlockFromISR();
  }
}

static void send_dma(SerialDriver *sdp) {
  unsigned i;
  for (i = 0; i < sizeof(sdp->txbuf); i++) {
    msg_t b = oqGetI(&sdp->oqueue);
    if (b < Q_OK)
      break;
    sdp->txbuf[i] = b;
  }

  if (i) {
    dmaStreamSetMemory0(sdp->dmatx, &sdp->txbuf);
    dmaStreamSetTransactionSize(sdp->dmatx, i);
    dmaStreamSetMode(sdp->dmatx, sdp->dmamode | STM32_DMA_CR_DIR_M2P |
                                 STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    dmaStreamEnable(sdp->dmatx);
    sdp->txbusy = true;
  } else {
    chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
    sdp->txbusy = false;
  }
}

#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  if (!SD1.txbusy)
    send_dma(&SD1);
}
#endif

#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  if (!SD2.txbusy)
    send_dma(&SD2);
}
#endif

#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
static void notify3(io_queue_t *qp) {

  (void)qp;
  if (!SD3.txbusy)
    send_dma(&SD3);
}
#endif

#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
static void notify4(io_queue_t *qp) {

  (void)qp;
  if (!SD4.txbusy)
    send_dma(&SD4);
}
#endif

#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
static void notify5(io_queue_t *qp) {

  (void)qp;
  if (!SD5.txbusy)
    send_dma(&SD5);
}
#endif

#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
static void notify6(io_queue_t *qp) {

  (void)qp;
  if (!SD6.txbusy)
    send_dma(&SD6);
}
#endif

#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
static void notify7(io_queue_t *qp) {

  (void)qp;
  if (!SD7.txbusy)
    send_dma(&SD7);
}
#endif

#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
static void notify8(io_queue_t *qp) {

  (void)qp;
  if (!SD8.txbusy)
    send_dma(&SD8);
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if STM32_SERIAL_USE_USART1 || defined(__DOXYGEN__)
#if !defined(STM32_USART1_HANDLER)
#error "STM32_USART1_HANDLER not defined"
#endif
/**
 * @brief   USART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_USART2 || defined(__DOXYGEN__)
#if !defined(STM32_USART2_HANDLER)
#error "STM32_USART2_HANDLER not defined"
#endif
/**
 * @brief   USART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART2_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_USART3 || defined(__DOXYGEN__)
#if !defined(STM32_USART3_HANDLER)
#error "STM32_USART3_HANDLER not defined"
#endif
/**
 * @brief   USART3 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART3_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD3);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_UART4 || defined(__DOXYGEN__)
#if !defined(STM32_UART4_HANDLER)
#error "STM32_UART4_HANDLER not defined"
#endif
/**
 * @brief   UART4 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART4_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD4);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_UART5 || defined(__DOXYGEN__)
#if !defined(STM32_UART5_HANDLER)
#error "STM32_UART5_HANDLER not defined"
#endif
/**
 * @brief   UART5 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART5_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD5);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_USART6 || defined(__DOXYGEN__)
#if !defined(STM32_USART6_HANDLER)
#error "STM32_USART6_HANDLER not defined"
#endif
/**
 * @brief   USART6 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_USART6_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD6);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_UART7 || defined(__DOXYGEN__)
#if !defined(STM32_UART7_HANDLER)
#error "STM32_UART7_HANDLER not defined"
#endif
/**
 * @brief   UART7 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART7_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD7);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if STM32_SERIAL_USE_UART8 || defined(__DOXYGEN__)
#if !defined(STM32_UART8_HANDLER)
#error "STM32_UART8_HANDLER not defined"
#endif
/**
 * @brief   UART8 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(STM32_UART8_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD8);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if STM32_SERIAL_USE_USART1
  sdObjectInit(&SD1, NULL, notify1);
  SD1.usart = USART1;
  SD1.dmarx = STM32_DMA_STREAM(STM32_SERIAL_USART1_RX_DMA_STREAM);
  SD1.dmatx = STM32_DMA_STREAM(STM32_SERIAL_USART1_TX_DMA_STREAM);
  dmaStreamAllocate(SD1.dmatx, STM32_SERIAL_USART1_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD1);
#endif

#if STM32_SERIAL_USE_USART2
  sdObjectInit(&SD2, NULL, notify2);
  SD2.usart = USART2;
  SD2.dmarx = STM32_DMA_STREAM(STM32_SERIAL_USART2_RX_DMA_STREAM);
  SD2.dmatx = STM32_DMA_STREAM(STM32_SERIAL_USART2_TX_DMA_STREAM);
  dmaStreamAllocate(SD2.dmatx, STM32_SERIAL_USART2_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD2);
#endif

#if STM32_SERIAL_USE_USART3
  sdObjectInit(&SD3, NULL, notify3);
  SD3.usart = USART3;
  SD3.dmarx = STM32_DMA_STREAM(STM32_SERIAL_USART3_RX_DMA_STREAM);
  SD3.dmatx = STM32_DMA_STREAM(STM32_SERIAL_USART3_TX_DMA_STREAM);
  dmaStreamAllocate(SD3.dmatx, STM32_SERIAL_USART3_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD3);
#endif

#if STM32_SERIAL_USE_UART4
  sdObjectInit(&SD4, NULL, notify4);
  SD4.usart = UART4;
  SD4.dmarx = STM32_DMA_STREAM(STM32_SERIAL_UART4_RX_DMA_STREAM);
  SD4.dmatx = STM32_DMA_STREAM(STM32_SERIAL_UART4_TX_DMA_STREAM);
  dmaStreamAllocate(SD4.dmatx, STM32_SERIAL_UART4_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD4);
#endif

#if STM32_SERIAL_USE_UART5
  sdObjectInit(&SD5, NULL, notify5);
  SD5.usart = UART5;
  SD5.dmarx = STM32_DMA_STREAM(STM32_SERIAL_UART5_RX_DMA_STREAM);
  SD5.dmatx = STM32_DMA_STREAM(STM32_SERIAL_UART5_TX_DMA_STREAM);
  dmaStreamAllocate(SD5.dmatx, STM32_SERIAL_UART5_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD5);
#endif

#if STM32_SERIAL_USE_USART6
  sdObjectInit(&SD6, NULL, notify6);
  SD6.usart = USART6;
  SD6.dmarx = STM32_DMA_STREAM(STM32_SERIAL_USART6_RX_DMA_STREAM);
  SD6.dmatx = STM32_DMA_STREAM(STM32_SERIAL_USART6_TX_DMA_STREAM);
  dmaStreamAllocate(SD6.dmatx, STM32_SERIAL_USART6_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD6);
#endif

#if STM32_SERIAL_USE_UART7
  sdObjectInit(&SD7, NULL, notify7);
  SD7.usart = UART7;
  SD7.dmarx = STM32_DMA_STREAM(STM32_SERIAL_UART7_RX_DMA_STREAM);
  SD7.dmatx = STM32_DMA_STREAM(STM32_SERIAL_UART7_TX_DMA_STREAM);
  dmaStreamAllocate(SD7.dmatx, STM32_SERIAL_UART7_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD7);
#endif

#if STM32_SERIAL_USE_UART8
  sdObjectInit(&SD8, NULL, notify8);
  SD8.usart = UART8;
  SD8.dmarx = STM32_DMA_STREAM(STM32_SERIAL_UART8_RX_DMA_STREAM);
  SD8.dmatx = STM32_DMA_STREAM(STM32_SERIAL_UART8_TX_DMA_STREAM);
  dmaStreamAllocate(SD8.dmatx, STM32_SERIAL_UART8_PRIORITY,
                    (stm32_dmaisr_t)serve_dma_interrupt, &SD8);
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if STM32_SERIAL_USE_USART1
    if (&SD1 == sdp) {
      rccEnableUSART1(FALSE);
      nvicEnableVector(STM32_USART1_NUMBER, STM32_SERIAL_USART1_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(USART1_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_USART1_DMA_PRIORITY);
    }
#endif
#if STM32_SERIAL_USE_USART2
    if (&SD2 == sdp) {
      rccEnableUSART2(FALSE);
      nvicEnableVector(STM32_USART2_NUMBER, STM32_SERIAL_USART2_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(USART2_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_USART2_DMA_PRIORITY);
    }
#endif
#if STM32_SERIAL_USE_USART3
    if (&SD3 == sdp) {
      rccEnableUSART3(FALSE);
      nvicEnableVector(STM32_USART3_NUMBER, STM32_SERIAL_USART3_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(USART3_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_USART3_DMA_PRIORITY);
    }
#endif
#if STM32_SERIAL_USE_UART4
    if (&SD4 == sdp) {
      rccEnableUART4(FALSE);
      nvicEnableVector(STM32_UART4_NUMBER, STM32_SERIAL_UART4_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(UART4_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_UART4_DMA_PRIORITY);
    }
#endif
#if STM32_SERIAL_USE_UART5
    if (&SD5 == sdp) {
      rccEnableUART5(FALSE);
      nvicEnableVector(STM32_UART5_NUMBER, STM32_SERIAL_UART5_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(UART5_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_UART5_DMA_PRIORITY);
    }
#endif
#if STM32_SERIAL_USE_USART6
    if (&SD6 == sdp) {
      rccEnableUSART6(FALSE);
      nvicEnableVector(STM32_USART6_NUMBER, STM32_SERIAL_USART6_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(USART6_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_USART6_DMA_PRIORITY);
    }
#endif
#if STM32_SERIAL_USE_UART7
    if (&SD7 == sdp) {
      rccEnableUART7(FALSE);
      nvicEnableVector(STM32_UART7_NUMBER, STM32_SERIAL_UART7_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(UART7_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_UART7_DMA_PRIORITY);
    }
#endif
#if STM32_SERIAL_USE_UART8
    if (&SD8 == sdp) {
      rccEnableUART8(FALSE);
      nvicEnableVector(STM32_UART8_NUMBER, STM32_SERIAL_UART8_PRIORITY);
      sdp->dmamode |= STM32_DMA_CR_CHSEL(UART8_RX_DMA_CHANNEL) |
                      STM32_DMA_CR_PL(STM32_SERIAL_UART8_DMA_PRIORITY);
    }
#endif
  }
  usart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    dmaStreamDisable(sdp->dmarx);
    usart_deinit(sdp->usart);
#if STM32_SERIAL_USE_USART1
    if (&SD1 == sdp) {
      rccDisableUSART1(FALSE);
      nvicDisableVector(STM32_USART1_NUMBER);
      return;
    }
#endif
#if STM32_SERIAL_USE_USART2
    if (&SD2 == sdp) {
      rccDisableUSART2(FALSE);
      nvicDisableVector(STM32_USART2_NUMBER);
      return;
    }
#endif
#if STM32_SERIAL_USE_USART3
    if (&SD3 == sdp) {
      rccDisableUSART3(FALSE);
      nvicDisableVector(STM32_USART3_NUMBER);
      return;
    }
#endif
#if STM32_SERIAL_USE_UART4
    if (&SD4 == sdp) {
      rccDisableUART4(FALSE);
      nvicDisableVector(STM32_UART4_NUMBER);
      return;
    }
#endif
#if STM32_SERIAL_USE_UART5
    if (&SD5 == sdp) {
      rccDisableUART5(FALSE);
      nvicDisableVector(STM32_UART5_NUMBER);
      return;
    }
#endif
#if STM32_SERIAL_USE_USART6
    if (&SD6 == sdp) {
      rccDisableUSART6(FALSE);
      nvicDisableVector(STM32_USART6_NUMBER);
      return;
    }
#endif
#if STM32_SERIAL_USE_UART7
    if (&SD7 == sdp) {
      rccDisableUART7(FALSE);
      nvicDisableVector(STM32_UART7_NUMBER);
      return;
    }
#endif
#if STM32_SERIAL_USE_UART8
    if (&SD8 == sdp) {
      rccDisableUART8(FALSE);
      nvicDisableVector(STM32_UART8_NUMBER);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
