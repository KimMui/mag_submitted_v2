/************************************************************************************************
 * arch/arm/src/sam34/chip/sam3u_vectors.h
 *
 *   Copyright (C) 2009-2010, 2013 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ************************************************************************************************/

/************************************************************************************************
 * Pre-processor Definitions
 ************************************************************************************************/
/* This file is included by sam_vectors.S.  It provides the macro VECTOR that
 * supplies ach SAM3U vector in terms of a (lower-case) ISR label and an
 * (upper-case) IRQ number as defined in arch/arm/include/sam/sam3u_irq.h.
 * sam_vectors.S will defined the VECTOR in different ways in order to generate
 * the interrupt vectors and handlers in their final form.
 */

/* If the common ARMv7-M vector handling is used, then all it needs is the following
 * definition that provides the number of supported vectors.
 */

#ifdef CONFIG_ARMV7M_CMNVECTOR

/* Reserve 30 interrupt table entries for I/O interrupts. */

# define ARMV7M_PERIPHERAL_INTERRUPTS 30

#else
  VECTOR(sam_supc, SAM_IRQ_SUPC)      /* Vector 16+0:  Supply Controller */
  VECTOR(sam_rstc, SAM_IRQ_RSTC)      /* Vector 16+1:  Reset Controller */
  VECTOR(sam_rtc, SAM_IRQ_RTC)        /* Vector 16+2:  Real Time Clock */
  VECTOR(sam_rtt, SAM_IRQ_RTT)        /* Vector 16+3:  Real Time Timer */
  VECTOR(sam_wdt, SAM_IRQ_WDT)        /* Vector 16+4:  Watchdog Timer */
  VECTOR(sam_pmc, SAM_IRQ_PMC)        /* Vector 16+5:  Power Management Controller */
  VECTOR(sam_eefc0, SAM_IRQ_EEFC0)    /* Vector 16+6:  Enhanced Embedded Flash Controller 0 */
  VECTOR(sam_eefc1, SAM_IRQ_EEFC1)    /* Vector 16+7:  Enhanced Embedded Flash Controller 1 */
  VECTOR(sam_uart0, SAM_IRQ_UART0)    /* Vector 16+8:  Universal Asynchronous Receiver Transmitter */
  VECTOR(sam_smc, SAM_IRQ_SMC)        /* Vector 16+9:  Static Memory Controller */
  VECTOR(sam_pioa, SAM_IRQ_PIOA)      /* Vector 16+10: Parallel I/O Controller A */
  VECTOR(sam_piob, SAM_IRQ_PIOB)      /* Vector 16+11: Parallel I/O Controller B */
  VECTOR(sam_pioc, SAM_IRQ_PIOC)      /* Vector 16+12: Parallel I/O Controller C */
  VECTOR(sam_usart0, SAM_IRQ_USART0)  /* Vector 16+13: USART 0 */
  VECTOR(sam_usart1, SAM_IRQ_USART1)  /* Vector 16+14: USART 1 */
  VECTOR(sam_usart2, SAM_IRQ_USART2)  /* Vector 16+15: USART 2 */
  VECTOR(sam_usart3, SAM_IRQ_USART3)  /* Vector 16+16: USART 3 */
  VECTOR(sam_hsmci, SAM_IRQ_HSMCI)    /* Vector 16+17: High Speed Multimedia Card Interface */
  VECTOR(sam_twi0, SAM_IRQ_TWI0)      /* Vector 16+18: Two-Wire Interface 0 */
  VECTOR(sam_twi1, SAM_IRQ_TWI1)      /* Vector 16+19: Two-Wire Interface 1 */
  VECTOR(sam_spi0, SAM_IRQ_SPI0)      /* Vector 16+20: Serial Peripheral Interface */
  VECTOR(sam_ssc, SAM_IRQ_SSC)        /* Vector 16+21: Synchronous Serial Controller */
  VECTOR(sam_tc0, SAM_IRQ_TC0)        /* Vector 16+22: Timer Counter 0 */
  VECTOR(sam_tc1, SAM_IRQ_TC1)        /* Vector 16+23: Timer Counter 1 */
  VECTOR(sam_tc2, SAM_IRQ_TC2)        /* Vector 16+24: Timer Counter 2 */
  VECTOR(sam_pwm, SAM_IRQ_PWM)        /* Vector 16+25: Pulse Width Modulation Controller */
  VECTOR(sam_adc12b, SAM_IRQ_ADC12B)  /* Vector 16+26: 12-bit ADC Controller */
  VECTOR(sam_adc, SAM_IRQ_ADC)        /* Vector 16+27: 10-bit ADC Controller */
  VECTOR(sam_dmac, SAM_IRQ_DMAC)      /* Vector 16+28: DMA Controller */
  VECTOR(sam_udphs, SAM_IRQ_UDPHS)    /* Vector 16+29: USB Device High Speed */
#endif
