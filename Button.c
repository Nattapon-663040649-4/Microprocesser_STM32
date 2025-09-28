/*
 * Button.c
 *
 *  Created on: Sep 28, 2025
 *      Author: NBODTKKU


#include <stdint.h>
#define STM32F411xE
#include "stm32f4xx.h"

static void delay(volatile uint32_t t) {
    while (t--) __NOP();
}

int main(void) {
    // --- Enable Clocks --- //
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;  // GPIOB

    // --- Setup LEDs (PA6 = Red, PB6 = Green) --- //
    GPIOA->MODER &= ~(3U << (2*6));
    GPIOA->MODER |=  (1U << (2*6)); // PA6 = output

    GPIOB->MODER &= ~(3U << (2*6));
    GPIOB->MODER |=  (1U << (2*6)); // PB6 = output

    // --- Setup Buttons (PA10, PB3, PB5, PB4) as input --- //
    GPIOA->MODER &= ~(3U << (2*10)); // PA10 input
    GPIOB->MODER &= ~(3U << (2*3));  // PB3 input
    GPIOB->MODER &= ~(3U << (2*5));  // PB5 input
    GPIOB->MODER &= ~(3U << (2*4));  // PB4 input

    // Optional: enable Pull-up (ถ้า board ปล่อยขาไว้ floating)
    GPIOA->PUPDR |= (1U << (2*10)); // PA10 pull-up
    GPIOB->PUPDR |= (1U << (2*3));  // PB3 pull-up
    GPIOB->PUPDR |= (1U << (2*5));  // PB5 pull-up
    GPIOB->PUPDR |= (1U << (2*4));  // PB4 pull-up


    while (1) {
        // --- Read buttons --- //
        uint8_t btn_D2 = (GPIOA->IDR & (1U << 10)) ? 1 : 0; // PA10
        uint8_t btn_D3 = (GPIOB->IDR & (1U << 3))  ? 1 : 0; // PB3
        uint8_t btn_D4 = (GPIOB->IDR & (1U << 5))  ? 1 : 0; // PB5
        uint8_t btn_D5 = (GPIOB->IDR & (1U << 4))  ? 1 : 0; // PB4

        // --- ถ้ากดคู่ปุ่มห้าม (PA10+PB3) หรือ (PB5+PB4) พร้อมกัน --- //
        if ( (!btn_D2 && !btn_D3) || (!btn_D4 && !btn_D5) ) {
            // ดับไฟทั้งหมด
            GPIOA->BSRR = GPIO_BSRR_BR6; // Red OFF
            GPIOB->BSRR = GPIO_BSRR_BR6; // Green OFF
            continue; // ข้ามไป loop ใหม่ ไม่ทำงานอื่น
        }

        // --- D2: Green ON when pressed --- //
        if (!btn_D2) { // Active LOW
            GPIOB->BSRR = GPIO_BSRR_BS6;   // Green ON
        } else {
            GPIOB->BSRR = GPIO_BSRR_BR6;   // Green OFF
        }

        // --- D3: Red ON when pressed --- //
        if (!btn_D3) {
            GPIOA->BSRR = GPIO_BSRR_BS6;   // Red ON
        } else {
            GPIOA->BSRR = GPIO_BSRR_BR6;   // Red OFF
        }

        // --- D4: Green Blink when pressed --- //
        if (!btn_D4) {
            GPIOB->BSRR = GPIO_BSRR_BS6;
            delay(300000);
            GPIOB->BSRR = GPIO_BSRR_BR6;
            delay(300000);
        }

        // --- D5: Red Blink when pressed --- //
        if (!btn_D5) {
            GPIOA->BSRR = GPIO_BSRR_BS6;
            delay(300000);
            GPIOA->BSRR = GPIO_BSRR_BR6;
            delay(300000);
        }
    }
}

*/
