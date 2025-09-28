/*
 * Light.c
 *
 *  Created on: Sep 28, 2025
 *      Author: NBODTKKU


#include <stdint.h>
#include <stdio.h>
#include <math.h>
#define STM32F411xE
#include "stm32f4xx.h"

#define VREF        3.3f
#define VCC         3.3f
#define ADC_MAXRES  4095.0f
#define RX          10000.0f
#define SLOPE       -0.6875f
#define OFFSET      5.1276f

char stringOut[50];

void vdg_UART_TxString(char strOut[]) {
    for (uint8_t idx = 0; strOut[idx] != '\0'; idx++) {
        while((USART2->SR & USART_SR_TXE) == 0);
        USART2->DR = strOut[idx];
    }
}

int main(void) {
    // --- Enable Clock --- //
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // --- USART2 (PA2=TX, PA3=RX) --- //
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3);
    GPIOA->MODER |= (0b10 << GPIO_MODER_MODER2_Pos) | (0b10 << GPIO_MODER_MODER3_Pos);
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFRL2 | GPIO_AFRL_AFRL3);
    GPIOA->AFR[0] |= (0b0111 << GPIO_AFRL_AFSEL2_Pos) | (0b0111 << GPIO_AFRL_AFSEL3_Pos);

    USART2->CR1 |= USART_CR1_UE;
    USART2->CR1 &= ~USART_CR1_M;
    USART2->CR2 &= ~USART_CR2_STOP;
    USART2->BRR = 139;               // 115200 bps @ 16MHz
    USART2->CR1 |= USART_CR1_TE;

    // --- LED Yellow (PA7) --- //
    GPIOA->MODER &= ~(GPIO_MODER_MODER7);
    GPIOA->MODER |= (0b01 << GPIO_MODER_MODER7_Pos);  // PA7 output
    GPIOA->OTYPER &= ~(GPIO_OTYPER_OT7);
    GPIOA->OSPEEDR &= ~(GPIO_OSPEEDR_OSPEED7);

    // --- ADC1, Channel 1 (PA1) --- //
    GPIOA->MODER &= ~(GPIO_MODER_MODER1);
    GPIOA->MODER |= (0b11 << GPIO_MODER_MODER1_Pos);  // Analog mode
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->SMPR2 |= ADC_SMPR2_SMP1;
    ADC1->SQR1 &= ~(ADC_SQR1_L);
    ADC1->SQR3 = 1;  // Channel 1

    // --- Enable FPU --- //
    SCB->CPACR |= (0b1111 << 20);
    __asm volatile("dsb");
    __asm volatile("isb");

    while(1) {
        // --- Start ADC conversion --- //
        ADC1->CR2 |= ADC_CR2_SWSTART;
        while((ADC1->SR & ADC_SR_EOC) == 0);

        float adc_voltage = (ADC1->DR * VREF) / ADC_MAXRES;
        float r_ldr = RX * adc_voltage / (VCC - adc_voltage);
        float lightintensity = powf(10, SLOPE * log10f(r_ldr) + OFFSET);

        // --- Control LED --- //
        if (lightintensity < 300.0f) {
            GPIOA->ODR |= GPIO_ODR_OD7;   // Yellow ON
        } else {
            GPIOA->ODR &= ~(GPIO_ODR_OD7); // Yellow OFF
        }

        for (uint32_t iter = 0; iter < 133333; iter++) {}
    }
}

*/
