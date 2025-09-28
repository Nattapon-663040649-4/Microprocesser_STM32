/*
 * CarSimulation
 *
 * Author: Naphat
 * Date: Oct 2025
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define STM32F411xE
#include "stm32f4xx.h"

/* --- Define constants --- */
#define VREF        3.3f
#define VCC         3.3f
#define ADC_MAXRES  4095.0f
#define RX          10000.0f
#define SLOPE       -0.6875f
#define OFFSET      5.1276f

/* --- Delay --- */
static void delay(volatile uint32_t t) {
    while (t--) __NOP();
}

/* --- UART TX String (optional, not used for Lux now) --- */
char stringOut[50];
void vdg_UART_TxString(char strOut[]) {
    for (uint8_t idx = 0; strOut[idx] != '\0'; idx++) {
        while((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = strOut[idx];
    }
}

int main(void) {
    /* --- Enable Clocks --- */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    /* --- LED Outputs: PA6=Red, PB6=Green, PA7=Yellow --- */
    GPIOA->MODER &= ~((3U << (2*6)) | (3U << (2*7)));
    GPIOA->MODER |=  (1U << (2*6)) | (1U << (2*7)); // output
    GPIOB->MODER &= ~(3U << (2*6));
    GPIOB->MODER |=  (1U << (2*6));

    /* --- Button Inputs: PA10, PB3, PB5, PB4 --- */
    GPIOA->MODER &= ~(3U << (2*10));
    GPIOB->MODER &= ~((3U << (2*3)) | (3U << (2*5)) | (3U << (2*4)));
    GPIOA->PUPDR |= (1U << (2*10));
    GPIOB->PUPDR |= (1U << (2*3)) | (1U << (2*5)) | (1U << (2*4));

    /* --- USART2 (PA2=TX, PA3=RX) for debug --- */
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos) | (0b10 << GPIO_MODER_MODER3_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL2_Pos) | (0b0111 << GPIO_AFRL_AFSEL3_Pos);

    USART2->CR1 |= USART_CR1_UE;
    USART2->CR1 &= ~USART_CR1_M;
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->BRR = 139;   // 115200 bps @16MHz
    USART2->CR1 |= USART_CR1_TE;

    /* --- ADC1, Channel 1 (PA1) --- */
    GPIOA->MODER &= ~(GPIO_MODER_MODER1);
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER1_Pos);
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SMPR2 |= ADC_SMPR2_SMP1;
    ADC1->SQR1 &= ~(ADC_SQR1_L);
    ADC1->SQR3 = 1;  // channel 1

    /* --- Enable FPU --- */
    SCB->CPACR |= (0b1111 << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    while (1) {
        /* --- Read buttons --- */
        uint8_t btn_D2 = (GPIOA->IDR & (1U << 10)) ? 1 : 0; // PA10
        uint8_t btn_D3 = (GPIOB->IDR & (1U << 3))  ? 1 : 0; // PB3
        uint8_t btn_D4 = (GPIOB->IDR & (1U << 5))  ? 1 : 0; // PB5
        uint8_t btn_D5 = (GPIOB->IDR & (1U << 4))  ? 1 : 0; // PB4

        /* --- ถ้ากดคู่ปุ่มห้าม ดับแดง+เขียว --- */
        if ((!btn_D2 && !btn_D3) || (!btn_D4 && !btn_D5)) {
            GPIOA->BSRR = GPIO_BSRR_BR6; // Red OFF
            GPIOB->BSRR = GPIO_BSRR_BR6; // Green OFF
        } else {
            /* --- D2: Green ON when pressed --- */
            if (!btn_D2) {
                GPIOB->BSRR = GPIO_BSRR_BS6;
            } else {
                GPIOB->BSRR = GPIO_BSRR_BR6;
            }

            /* --- D3: Red ON when pressed --- */
            if (!btn_D3) {
                GPIOA->BSRR = GPIO_BSRR_BS6;
            } else {
                GPIOA->BSRR = GPIO_BSRR_BR6;
            }

            /* --- D4: Green Blink when pressed --- */
            if (!btn_D4) {
                GPIOB->BSRR = GPIO_BSRR_BS6;
                delay(300000);
                GPIOB->BSRR = GPIO_BSRR_BR6;
                delay(300000);
            }

            /* --- D5: Red Blink when pressed --- */
            if (!btn_D5) {
                GPIOA->BSRR = GPIO_BSRR_BS6;
                delay(300000);
                GPIOA->BSRR = GPIO_BSRR_BR6;
                delay(300000);
            }
        }

        /* --- ADC Read LDR --- */
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while ((ADC1->SR & ADC_SR_EOC) == 0);

        float adc_voltage = (ADC1->DR * VREF) / ADC_MAXRES;
        float r_ldr = RX * adc_voltage / (VCC - adc_voltage);
        float lightintensity = powf(10, SLOPE * log10f(r_ldr) + OFFSET);

        /* --- Control Yellow LED --- */
        if (lightintensity < 300.0f) {
            GPIOA->ODR |= GPIO_ODR_OD7;   // Yellow ON
        } else {
            GPIOA->ODR &= ~(GPIO_ODR_OD7); // Yellow OFF
        }

        for (uint32_t iter = 0; iter < 133333; iter++) {}
    }
}
