#include "systick_time.h"
#include "lcd_1602_drive.h"
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include <stdio.h>

/*
I2C2
PB10 -> SCL
PB11 -> SDA

I2C1
PB6 -> SCL
PB7 -> SDA
*/

#define TRIG_PIN GPIO_Pin_0   
#define ECHO_PIN GPIO_Pin_1   
#define LED_PIN GPIO_Pin_6
#define ERROR_LED_PIN GPIO_Pin_7
#define MAX_DISTANCE_CM 400
#define MIN_DISTANCE_CM 2
#define TIMEOUT_US 100000

void GPIO_Config(void);
void Timer_Config(void);
void Delay_us(uint16_t us);
uint32_t Read_HCSR04(void);
void LEdRED(uint32_t _distance);

char distance_str[16], status[16];
uint32_t a = 1, b = 1;
uint32_t delay_time;

int main(void) {
    systick_init();
    lcd_i2c_init(2);

    GPIO_Config();
    Timer_Config();

    while (1) {
        static uint32_t last_update = 0;

        uint32_t distance = Read_HCSR04();
        if (distance == 0) {
            sprintf(distance_str, "Error: No signal");
            GPIO_SetBits(GPIOA, ERROR_LED_PIN); // Bật LED báo lỗi
        } else {
            sprintf(distance_str, "Distance:%02d", distance);
            GPIO_ResetBits(GPIOA, ERROR_LED_PIN); // Tắt LED báo lỗi
        }

        sprintf(status, "sys:%d-obs:%d", a, b);
        LEdRED(distance);

        // Cập nhật LCD mỗi 500 ms
        if (systick_get_ms() - last_update >= 500) {
            lcd_i2c_msg(2, 1, 0, distance_str);
            lcd_i2c_msg(2, 2, 0, status);
            last_update = systick_get_ms();
        }

        Delay_us(1000); // Tránh quá tải CPU
    }
}

void GPIO_Config(void) {
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    GPIO_InitTypeDef GPIO_InitStructure;

    // Cấu hình TRIG_PIN
    GPIO_InitStructure.GPIO_Pin = TRIG_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Cấu hình ECHO_PIN
    GPIO_InitStructure.GPIO_Pin = ECHO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Cấu hình LED_PIN
    GPIO_InitStructure.GPIO_Pin = LED_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // Cấu hình ERROR_LED_PIN
    GPIO_InitStructure.GPIO_Pin = ERROR_LED_PIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void Timer_Config(void) {
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Prescaler = 84 - 1; // 1 MHz
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF; // 65535 us
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
}

void Delay_us(uint16_t us) {
    uint16_t start = TIM_GetCounter(TIM2);
    while ((uint16_t)(TIM_GetCounter(TIM2) - start) < us);
}

uint32_t Read_HCSR04(void) {
    uint32_t duration, distance, timeout = TIMEOUT_US;

    // Kích hoạt tín hiệu TRIG
    GPIO_SetBits(GPIOA, TRIG_PIN);
    Delay_us(10);
    GPIO_ResetBits(GPIOA, TRIG_PIN);

    // Chờ ECHO_PIN lên mức cao
    while (!GPIO_ReadInputDataBit(GPIOA, ECHO_PIN)) {
        if (timeout-- == 0) return 0; // Lỗi do không có tín hiệu
    }

    // Đo thời gian tín hiệu ECHO
    TIM_SetCounter(TIM2, 0);
    timeout = TIMEOUT_US; // Reset timeout
    while (GPIO_ReadInputDataBit(GPIOA, ECHO_PIN)) {
        if (timeout-- == 0) return 0; // Lỗi do tín hiệu kéo dài
    }

    duration = TIM_GetCounter(TIM2);

    // Tính khoảng cách
    distance = (duration * 0.034) / 2;
    return (distance > MAX_DISTANCE_CM || distance < MIN_DISTANCE_CM) ? 0 : distance;
}

void LEdRED(uint32_t _distance) {
    if (_distance == 0) {
        delay_time = 0; // Không nhấp nháy nếu không có tín hiệu
    } else if (_distance <= 5) {
        delay_time = 50; // 10 Hz
    } else if (_distance >= 15) {
        delay_time = 250; // 2 Hz
    } else {
        delay_time = 250 - (_distance - 5) * 12.5;
    }

    if (delay_time > 0) {
        GPIO_SetBits(GPIOA, LED_PIN);
        Delay_us(delay_time * 1000);
        GPIO_ResetBits(GPIOA, LED_PIN);
        Delay_us(delay_time * 1000);
    }
}
