/**
 * Projeto: Sistema IoT de Detecção de Queda para Idosos
 * Placa: BitDogLab (RP2040 / Pico W)
 * Linguagem: C++ com FreeRTOS
 */

#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// Definições de Pinos
#define I2C_SDA_EXTERNAL 0
#define I2C_SCL_EXTERNAL 1
#define I2C_SDA_OLED 14
#define I2C_SCL_OLED 15
#define BUZZER_PIN 21

// Limiares para Detecção de Queda (Valores experimentais)
#define FALL_THRESHOLD 2.5f  // G's (Aceleração total)
#define ORIENTATION_CHANGE 0.5f
#define INACTIVITY_TIME 2000 // ms

// Estrutura para dados do sensor
typedef struct {
    float ax, ay, az;
} sensor_data_t;

QueueHandle_t sensorQueue;

// Task: Leitura do Sensor (MPU6050)
void vTaskSensor(void *pvParameters) {
    sensor_data_t data;
    // Inicialização fictícia do I2C e MPU6050
    // No código real, incluiria mpu6050_init()
    
    while (true) {
        // Simulação de leitura (Substituir por mpu6050_read_raw)
        data.ax = 0.0f; data.ay = 0.0f; data.az = 1.0f; 
        
        xQueueSend(sensorQueue, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10)); // 100Hz
    }
}

// Task: Algoritmo de Detecção de Queda
void vTaskDetection(void *pvParameters) {
    sensor_data_t data;
    while (true) {
        if (xQueueReceive(sensorQueue, &data, portMAX_DELAY)) {
            // Cálculo do Vetor Resultante: R = sqrt(ax^2 + ay^2 + az^2)
            float total_accel = sqrt(data.ax*data.ax + data.ay*data.ay + data.az*data.az);
            
            if (total_accel > FALL_THRESHOLD) {
                printf("ALERTA: Queda detectada!\n");
                // Aciona Buzzer e envia MQTT
                gpio_put(BUZZER_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_put(BUZZER_PIN, 0);
            }
        }
    }
}

// Task: Comunicação MQTT
void vTaskMQTT(void *pvParameters) {
    // Inicializa Wi-Fi
    if (cyw43_arch_init()) {
        printf("Erro ao inicializar Wi-Fi\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    
    // Loop de conexão e publicação
    while (true) {
        // Lógica de conexão Wi-Fi e MQTT
        // cyw43_arch_wifi_connect_timeout_ms(...)
        vTaskDelay(pdMS_TO_TICKS(5000));
    }
}

int main() {
    stdio_init_all();
    
    // Configuração de GPIOs
    gpio_init(BUZZER_PIN);
    gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    
    sensorQueue = xQueueCreate(10, sizeof(sensor_data_t));
    
    xTaskCreate(vTaskSensor, "SensorTask", 256, NULL, 3, NULL);
    xTaskCreate(vTaskDetection, "DetectTask", 256, NULL, 2, NULL);
    xTaskCreate(vTaskMQTT, "MQTTTask", 512, NULL, 1, NULL);
    
    vTaskStartScheduler();
    
    while (true);
}
