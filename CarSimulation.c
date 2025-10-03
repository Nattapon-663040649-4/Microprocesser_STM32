/*
 * CarSimulation.c
 *
 * Author: Naphat
 * Date: Oct 2025
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define STM32F411xE
#include "stm32f4xx.h"

// --- Define constants --- //
#define VREF        3.3f
#define VCC         3.3f
#define ADC_MAXRES  4095.0f
#define RX          10000.0f
#define SLOPE       -0.6875f
#define OFFSET      5.1276f

// --- Delay --- //
static void delay(volatile uint32_t t) {
    while (t--) __NOP();
}

int main(void) {
    // --- Enable Clocks --- //
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // --- LED Outputs: PA6=Red, PA7=Yellow, PB6=Green --- //
    GPIOA->MODER &= ~((3U << (2*6)) | (3U << (2*7)));
    GPIOA->MODER |=  (1U << (2*6)) | (1U << (2*7));
    GPIOB->MODER &= ~(3U << (2*6));
    GPIOB->MODER |=  (1U << (2*6));

    // --- Button Inputs: PA10, PB3, PB5, PB4 --- //
    GPIOA->MODER &= ~(3U << (2*10));
    GPIOB->MODER &= ~((3U << (2*3)) | (3U << (2*5)) | (3U << (2*4)));
    GPIOA->PUPDR |= (1U << (2*10));
    GPIOB->PUPDR |= (1U << (2*3)) | (1U << (2*5)) | (1U << (2*4));

    // --- 7-seg outputs: PA5–PA9, PB10, PC7 --- //
    GPIOA->MODER &= ~(GPIO_MODER_MODER5 | GPIO_MODER_MODER6 |
                      GPIO_MODER_MODER7 | GPIO_MODER_MODER8 |
                      GPIO_MODER_MODER9);
    GPIOA->MODER |=  (0b01 << GPIO_MODER_MODER5_Pos) |
                     (0b01 << GPIO_MODER_MODER6_Pos) |
                     (0b01 << GPIO_MODER_MODER7_Pos) |
                     (0b01 << GPIO_MODER_MODER8_Pos) |
                     (0b01 << GPIO_MODER_MODER9_Pos);
    GPIOB->MODER &= ~(GPIO_MODER_MODER10);
    GPIOB->MODER |=  (0b01 << GPIO_MODER_MODER10_Pos);
    GPIOC->MODER &= ~(GPIO_MODER_MODER7);
    GPIOC->MODER |=  (0b01 << GPIO_MODER_MODER7_Pos);

    // --- ADC1 config: PA1 (LDR), PA4 (Potentiometer) --- //
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER1_Pos); // PA1 analog
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER4_Pos); // PA4 analog
    ADC1->CR2 = 0;
    ADC1->CR2 |= ADC_CR2_ADON;

    // --- Enable FPU --- //
    SCB->CPACR |= (0b1111 << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    uint16_t adc_val = 0;
    uint8_t speed = 0, number = 0;

    while (1) {
        // --- Read Buttons --- //
        uint8_t btn_D2 = (GPIOA->IDR & (1U << 10)) ? 1 : 0;
        uint8_t btn_D3 = (GPIOB->IDR & (1U << 3))  ? 1 : 0;
        uint8_t btn_D4 = (GPIOB->IDR & (1U << 5))  ? 1 : 0;
        uint8_t btn_D5 = (GPIOB->IDR & (1U << 4))  ? 1 : 0;

        if ((!btn_D2 && !btn_D3) || (!btn_D4 && !btn_D5)) {
            GPIOA->BSRR = GPIO_BSRR_BR6; // Red OFF
            GPIOB->BSRR = GPIO_BSRR_BR6; // Green OFF
        } else {
            if (!btn_D2) GPIOB->BSRR = GPIO_BSRR_BS6;
            else         GPIOB->BSRR = GPIO_BSRR_BR6;

            if (!btn_D3) GPIOA->BSRR = GPIO_BSRR_BS6;
            else         GPIOA->BSRR = GPIO_BSRR_BR6;

            if (!btn_D4) {
                GPIOB->BSRR = GPIO_BSRR_BS6;
                delay(300000);
                GPIOB->BSRR = GPIO_BSRR_BR6;
                delay(300000);
            }

            if (!btn_D5) {
                GPIOA->BSRR = GPIO_BSRR_BS6;
                delay(300000);
                GPIOA->BSRR = GPIO_BSRR_BR6;
                delay(300000);
            }
        }

        // --- ADC Read PA1 (LDR) --- //
        ADC1->SQR3 = 1; // channel 1
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC));
        float adc_voltage = (ADC1->DR * VREF) / ADC_MAXRES;
        float r_ldr = RX * adc_voltage / (VCC - adc_voltage);
        float lightintensity = powf(10, SLOPE * log10f(r_ldr) + OFFSET);

        if (lightintensity < 300.0f) GPIOA->ODR |= GPIO_ODR_OD7;
        else                         GPIOA->ODR &= ~(GPIO_ODR_OD7);

        // --- ADC Read PA4 (Potentiometer) --- //
        ADC1->SQR3 = 4; // channel 4
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while (!(ADC1->SR & ADC_SR_EOC));
        adc_val = ADC1->DR;

        // --- Map to Speed (0–99), clockwise = increase --- //
        speed = ((4095 - adc_val) * 100) / 4096;
        if (speed > 99) speed = 99;
        number = speed / 10; // 0–9

        // --- Clear 7-seg --- //
        GPIOA->ODR &= ~((1 << 8) | (1 << 9));
        GPIOB->ODR &= ~(1 << 10);
        GPIOC->ODR &= ~(1 << 7);

        // --- Display number --- //
        switch (number) {
            case 1: GPIOC->ODR |= (1 << 7); break;
            case 2: GPIOA->ODR |= (1 << 8); break;
            case 3: GPIOA->ODR |= (1 << 8); GPIOC->ODR |= (1 << 7); break;
            case 4: GPIOB->ODR |= (1 << 10); break;
            case 5: GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break;
            case 6: GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); break;
            case 7: GPIOA->ODR |= (1 << 8); GPIOB->ODR |= (1 << 10); GPIOC->ODR |= (1 << 7); break;
            case 8: GPIOA->ODR |= (1 << 9); break;
            case 9: GPIOA->ODR |= (1 << 9); GPIOC->ODR |= (1 << 7); break;
            default: break; // case 0
        }
    }
}
