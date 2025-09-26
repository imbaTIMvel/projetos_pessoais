#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/adc.h"
#include "inc/ssd1306.h"
#include "neopixel.c"
#include "inc/mpu6050_i2c.h"

uint8_t count_btn;
uint8_t char_x = 60;
uint8_t char_y = 30;

// DISPLAY
#define I2C_SDA_ssd1306 2 // 14 para a versão 6, 2 para a versão 7
#define I2C_SCL_ssd1306 3 // 15 para a versão 6, 3 para a versão 7
// BOTÕES
#define BTN_A 5
#define BTN_B 6
#define BTN_C 10 // 22 para a versão 6, 10 para a versão 7
// JOYSTICK
#define VRX 27
#define VRY 26
// MATRIZ DE LEDS
#define LED_PIN 7
#define LED_COUNT 25
// ACELERÔMETRO
#define I2C_PORT i2c0
#define I2C_SDA 0
#define I2C_SCL 1
#define MPU6050_ADDR 0x68

void btn_handler(uint gpio, uint32_t events) { 
    switch(gpio) {
        case(BTN_A):    // Botão A foi pressionado
            if (events & GPIO_IRQ_EDGE_RISE) {
                count_btn += 1;
            }
        case(BTN_B):    // Botão A foi pressionado
            if (events & GPIO_IRQ_EDGE_RISE) {
                count_btn += 1;
            }
        case(BTN_C):    // Botão A foi pressionado
            if (events & GPIO_IRQ_EDGE_RISE) {
                count_btn += 1;
            }
    }
}

void init_buttons() {
    // Botão A
    gpio_init(BTN_A);
    gpio_set_dir(BTN_A, GPIO_IN);
    gpio_pull_up(BTN_A);
    // Botão B
    gpio_init(BTN_B);
    gpio_set_dir(BTN_B, GPIO_IN);
    gpio_pull_up(BTN_B);
    // Botão C
    gpio_init(BTN_C);
    gpio_set_dir(BTN_C, GPIO_IN);
    gpio_pull_up(BTN_C);
    // Habilitar interrupções - processar pressionamento dos Botões A, B e C
    gpio_set_irq_enabled_with_callback(BTN_A, GPIO_IRQ_EDGE_RISE, true, &btn_handler);
    gpio_set_irq_enabled(BTN_B, GPIO_IRQ_EDGE_RISE, true);
    gpio_set_irq_enabled(BTN_C, GPIO_IRQ_EDGE_RISE, true);
}

void init_joystick() {
    // Inicializa o ADC e os pinos de entrada analógica
    adc_init();         // Inicializa o módulo ADC
    adc_gpio_init(VRX); // Configura o pino VRX (eixo X) para entrada ADC
    adc_gpio_init(VRY); // Configura o pino VRY (eixo Y) para entrada ADC
}

void config_i2c() {
    // Display
    i2c_init(i2c1, 400 * 1000);
    gpio_set_function(I2C_SDA_ssd1306, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_ssd1306, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_ssd1306);
    gpio_pull_up(I2C_SCL_ssd1306);

    // Acelerômetro
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
}

void mpu6050_reset() {
    uint8_t buf[] = {0x6B, 0x80};
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    sleep_ms(100);
    buf[1] = 0x00;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
    sleep_ms(10);
}

uint8_t mpu6050_get_accel_range() {
    uint8_t reg = 0x1C;
    uint8_t val;
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, &val, 1, false);
    return (val >> 3) & 0x03; // bits 4:3
}

// 0=±2g, 1=±4g, 2=±8g, 3=±16g
void mpu6050_set_accel_range(uint8_t range) {
    uint8_t buf[2];
    buf[0] = 0x1C; // ACCEL_CONFIG register
    buf[1] = range << 3; // bits 3 e 4
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6];
    uint8_t reg = 0x3B; //MPU6050_REG_ACCEL_XOUT_H
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    for (int i=0; i<3; i++)
        accel[i] = (buffer[2*i]<<8) | buffer[2*i+1];

    reg = 0x43; //MPU6050_REG_GYRO_XOUT_H
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 6, false);
    for (int i=0; i<3; i++)
        gyro[i] = (buffer[2*i]<<8) | buffer[2*i+1];

    reg = 0x41; //MPU6050_REG_TEMP_OUT_H
    i2c_write_blocking(I2C_PORT, MPU6050_ADDR, &reg, 1, true);
    i2c_read_blocking(I2C_PORT, MPU6050_ADDR, buffer, 2, false);
    *temp = (buffer[0]<<8) | buffer[1];
}

int main(void) {
    stdio_init_all();
    init_buttons();
    init_joystick();
    config_i2c();
    mpu6050_reset();
    ssd1306_init();
    npInit(LED_PIN, LED_COUNT);
    sleep_ms(1000);
    struct render_area frame_area = {
        start_column : 0,
        end_column : ssd1306_width - 1,
        start_page : 0,
        end_page : ssd1306_n_pages - 1
    };
    calculate_render_area_buffer_length(&frame_area);
    uint8_t ssd[ssd1306_buffer_length];
    memset(ssd, 0, ssd1306_buffer_length);
    render_on_display(ssd, &frame_area);
    npClear();
    npSetLED(0, 80, 0, 0);
    npSetLED(12, 0, 80, 0); // acende LED 12 com valor 0 R, 0 G e 80 B
    npSetLED(24, 0, 0, 80);
    npWrite();
    int16_t accel[3], gyro[3], temp;
    while(true) {
        adc_select_input(1);
        uint adc_x_raw = adc_read();
        if (adc_x_raw < 800) {
            if (char_x > 6) {char_x -= 1;}
        }
        else if (adc_x_raw > 3300) {
            if (char_x < 122) {char_x += 1;}
        }
        mpu6050_read_raw(accel, gyro, &temp);
        printf("Accel X: %d, Y: %d, Z: %d\n", accel[0], accel[1], accel[2]);
        if (count_btn % 2 == 1) {
            memset(ssd, 0, ssd1306_buffer_length);
            ssd1306_draw_string(ssd, char_x, char_y, "Y");
            render_on_display(ssd, &frame_area);
        } else {
            memset(ssd, 0, ssd1306_buffer_length);
            ssd1306_draw_string(ssd, char_x, char_y, "N");
            render_on_display(ssd, &frame_area);
        }
        sleep_ms(200);
    }
}