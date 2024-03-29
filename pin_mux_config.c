//*****************************************************************************
// pin_mux_config.c
//
// configure the device pins for different signals
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// This file was automatically generated on 2/2/2016 at 7:49:30 PM
// by TI PinMux version 
//
//*****************************************************************************

#include "pin_mux_config.h" 
#include "hw_types.h"
#include "hw_memmap.h"
#include "hw_gpio.h"
#include "pin.h"
#include "gpio.h"
#include "prcm.h"

//*****************************************************************************
void PinMuxConfig(void)
{
    //
    // Enable Peripheral Clocks 
    //
    PRCMPeripheralClkEnable(PRCM_GPIOA0, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GPIOA3, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_GSPI, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA1, PRCM_RUN_MODE_CLK);
    PRCMPeripheralClkEnable(PRCM_UARTA0, PRCM_RUN_MODE_CLK);

    //
    // Configure PIN_58 for GPIO Output
    //
    PinTypeGPIO(PIN_58, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA0_BASE, 0x8, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_59 for GPIO Output
    //
    PinTypeGPIO(PIN_59, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA0_BASE, 0x10, GPIO_DIR_MODE_OUT);

    //
    // Configure PIN_18 for GPIO Input
    //
    PinTypeGPIO(PIN_18, PIN_MODE_0, false);
    GPIODirModeSet(GPIOA3_BASE, 0x10, GPIO_DIR_MODE_IN);

    //
    // Configure PIN_08 for SPI0 GSPI_CS
    //
    PinTypeSPI(PIN_08, PIN_MODE_7);

    //
    // Configure PIN_05 for SPI0 GSPI_CLK
    //
    PinTypeSPI(PIN_05, PIN_MODE_7);

    //
    // Configure PIN_06 for SPI0 GSPI_MISO
    //
    PinTypeSPI(PIN_06, PIN_MODE_7);

    //
    // Configure PIN_07 for SPI0 GSPI_MOSI
    //
    PinTypeSPI(PIN_07, PIN_MODE_7);

    //
    // Configure PIN_62 for UART1 UART1_RTS
    //
    PinTypeUART(PIN_62, PIN_MODE_3);

    //
    // Configure PIN_61 for UART1 UART1_CTS
    //
    PinTypeUART(PIN_61, PIN_MODE_3);

    //
    // Configure PIN_01 for UART1 UART1_TX
    //
    PinTypeUART(PIN_01, PIN_MODE_7);

    //
    // Configure PIN_02 for UART1 UART1_RX
    //
    PinTypeUART(PIN_02, PIN_MODE_7);

    //
    // Configure PIN_52 for UART0 UART0_RTS
    //
    PinTypeUART(PIN_52, PIN_MODE_6);

    //
    // Configure PIN_50 for UART0 UART0_CTS
    //
    PinTypeUART(PIN_50, PIN_MODE_12);

    //
    // Configure PIN_03 for UART0 UART0_TX
    //
    PinTypeUART(PIN_03, PIN_MODE_7);

    //
    // Configure PIN_04 for UART0 UART0_RX
    //
    PinTypeUART(PIN_04, PIN_MODE_7);
}
