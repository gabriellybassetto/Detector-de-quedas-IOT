# Guia de Implementação: Sistema IoT de Detecção de Queda para Idosos


Este guia detalhado oferece um roteiro completo para a implementação de um sistema IoT de detecção de queda, utilizando a placa **BitDogLab** baseada no microcontrolador Raspberry Pi Pico W. O objetivo é fornecer uma solução acessível e confiável para monitorar idosos e pacientes em recuperação, alertando cuidadores em caso de incidentes.

## 1. Visão Geral do Projeto
O sistema proposto integra hardware, firmware e comunicação em nuvem para detectar quedas e emitir alertas. A placa BitDogLab, equipada com um acelerômetro MPU6050, monitora continuamente os movimentos do usuário. Em caso de detecção de queda, um alerta local é acionado (buzzer e display OLED) e uma notificação é enviada via protocolo MQTT para um dashboard remoto.

## 2. Lista de Materiais
Para a montagem e operação deste projeto, você precisará dos seguintes componentes:

| Componente | Quantidade | Descrição |
| :--- | :--- | :--- |
| Placa BitDogLab | 1 | Placa de desenvolvimento baseada no Raspberry Pi Pico W (RP2040 com Wi-Fi). |
| Sensor Acelerômetro MPU6050 | 1 | Sensor de 6 eixos (acelerômetro e giroscópio) para detecção de movimento e quedas. |
| Display OLED SSD1306 | 1 | Display de 0.96 polegadas, 128x64 pixels, interface I2C (geralmente integrado na BitDogLab). |
| Buzzer | 1 | Componente para emissão de alertas sonoros (geralmente integrado na BitDogLab). |
| Cabos Jumper | Opcional | Para conexões adicionais, caso o MPU6050 não esteja integrado ou seja externo. |

## 3. Conexões de Hardware
A placa BitDogLab já integra a maioria dos periféricos essenciais. As conexões I2C para o display OLED e o buzzer são internas. Caso você utilize um sensor MPU6050 externo, siga as seguintes conexões para a interface I2C0 da BitDogLab [1]:

| Pino do MPU6050 | Pino da BitDogLab (RP2040) |
| :--- | :--- |
| VCC | 3.3V |
| GND | GND |
| SDA | GPIO 0 |
| SCL | GPIO 1 |

Para os componentes internos da BitDogLab, a pinagem identificada é a seguinte [2, 3]:

| Componente | Pino/Interface |
| :--- | :--- |
| Display OLED (SSD1306) | I2C1 (SDA: GPIO 14, SCL: GPIO 15) |
| Buzzer | GPIO 21 |

## 4. Configuração do Ambiente de Desenvolvimento
Para compilar e carregar o firmware na sua BitDogLab, siga os passos abaixo para configurar o ambiente de desenvolvimento:

1.  **Instale o Visual Studio Code (VS Code)**: Faça o download e instale a versão mais recente do VS Code para o seu sistema operacional [4].
2.  **Instale a Extensão Raspberry Pi Pico**: No VS Code, vá para a aba de Extensões (Ctrl+Shift+X) e procure por "Raspberry Pi Pico". Instale a extensão oficial.
3.  **Instale o CMake e o Pico SDK**: Siga as instruções oficiais da Raspberry Pi para instalar o CMake e o Pico SDK. Isso geralmente envolve a instalação de ferramentas de linha de comando e a configuração de variáveis de ambiente [5].
4.  **Configure o FreeRTOS**: O FreeRTOS é um sistema operacional de tempo real (RTOS) que será utilizado para gerenciar as tarefas do projeto. O Pico SDK possui exemplos de integração com FreeRTOS. Certifique-se de que o caminho para o kernel do FreeRTOS esteja configurado corretamente no seu ambiente de desenvolvimento [6].

## 5. Estrutura do Código (Firmware)
O firmware é desenvolvido em C++ utilizando o Pico SDK e o FreeRTOS. A arquitetura do software é baseada em tarefas (tasks) para garantir a execução concorrente e eficiente das funcionalidades do sistema:

-   `vTaskSensor`: Responsável pela leitura contínua dos dados do acelerômetro MPU6050. Esta tarefa é executada em alta frequência (100Hz) para capturar movimentos detalhados. Os dados são então colocados em uma fila (Queue) para serem processados pela tarefa de detecção.
-   `vTaskDetection`: Esta tarefa consome os dados da fila de sensores e aplica o algoritmo de detecção de queda. Se uma queda for identificada, ela aciona o buzzer local e sinaliza a tarefa MQTT para enviar um alerta.
-   `vTaskDisplay`: (A ser implementada) Responsável por atualizar o display OLED com informações de status, como "Monitorando..." ou "ALERTA DE QUEDA!".
-   `vTaskMQTT`: Gerencia a conexão Wi-Fi da BitDogLab e a comunicação com o broker MQTT. Em caso de alerta de queda, esta tarefa publica uma mensagem em um tópico MQTT específico.

## 6. Algoritmo de Detecção de Queda
O algoritmo de detecção de queda baseia-se na análise da **Magnitude da Aceleração Total** (vetor resultante) capturada pelo MPU6050. A magnitude é calculada pela fórmula:

> $Magnitude = \sqrt{Ax^2 + Ay^2 + Az^2}$

Onde Ax, Ay e Az são as acelerações nos eixos X, Y e Z, respectivamente. Em condições normais (repouso), a magnitude da aceleração é aproximadamente 1G (devido à gravidade). Durante uma queda, o algoritmo busca dois eventos principais:

1.  **Queda Livre**: Um período de baixa aceleração (magnitude próxima a 0G) quando o corpo está em queda.
2.  **Impacto**: Um pico de alta aceleração (magnitude significativamente maior que 1G, por exemplo, > 2.5G) no momento do impacto com o solo.

O código identifica esse pico de impacto para disparar o alarme, minimizando falsos positivos.

## 7. Configuração MQTT e Dashboard
Para receber e visualizar os alertas de queda remotamente, você precisará configurar um serviço MQTT e um dashboard:

1.  **Crie uma Conta no Adafruit IO**: Acesse [Adafruit IO](https://io.adafruit.com/) e crie uma conta gratuita. O Adafruit IO oferece um broker MQTT e a capacidade de criar dashboards personalizados.
2.  **Crie um Feed MQTT**: No Adafruit IO, navegue até "Feeds" e crie um novo feed. Sugira o nome `queda-alerta` para este feed. Este será o tópico onde a BitDogLab publicará os alertas.
3.  **Obtenha suas Credenciais MQTT**: No Adafruit IO, você encontrará seu **AIO Username** e **AIO Key**. Essas credenciais serão necessárias para a BitDogLab se conectar ao broker MQTT.
4.  **Configure o Código**: No arquivo `main.cpp`, você precisará inserir suas credenciais Wi-Fi (SSID e PASSWORD) e as credenciais do Adafruit IO (AIO_USERNAME e AIO_KEY) nos locais indicados.
5.  **Crie um Dashboard**: No Adafruit IO, crie um novo dashboard e adicione blocos (por exemplo, um bloco de texto ou indicador) para exibir o status do feed `queda-alerta`. Você pode configurar notificações para serem enviadas para seu celular quando uma nova mensagem for publicada neste feed.

## 8. Código Fonte (main.cpp)

```cpp
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
    // TODO: Inicializar I2C0 e MPU6050 aqui
    // Exemplo: mpu6050_init(i2c0, I2C_SDA_EXTERNAL, I2C_SCL_EXTERNAL);
    
    while (true) {
        // TODO: Substituir por leitura real do MPU6050
        // Exemplo: mpu6050_read_raw(&data.ax, &data.ay, &data.az);
        data.ax = 0.0f; data.ay = 0.0f; data.az = 1.0f; // Simulação
        
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
                // TODO: Acionar Display OLED
                gpio_put(BUZZER_PIN, 1); // Aciona Buzzer
                vTaskDelay(pdMS_TO_TICKS(500));
                gpio_put(BUZZER_PIN, 0); // Desliga Buzzer
                // TODO: Enviar mensagem MQTT (sinalizar vTaskMQTT)
            }
        }
    }
}

// Task: Comunicação MQTT
void vTaskMQTT(void *pvParameters) {
    // TODO: Definir suas credenciais Wi-Fi e Adafruit IO
    const char *WIFI_SSID = "SEU_SSID";
    const char *WIFI_PASSWORD = "SUA_SENHA_WIFI";
    const char *AIO_USERNAME = "SEU_USUARIO_ADAFRUIT_IO";
    const char *AIO_KEY = "SUA_CHAVE_ADAFRUIT_IO";
    const char *MQTT_FEED = "SEU_USUARIO_ADAFRUIT_IO/feeds/queda-alerta";

    if (cyw43_arch_init()) {
        printf("Erro ao inicializar Wi-Fi\n");
        return;
    }
    cyw43_arch_enable_sta_mode();
    
    while (true) {
        if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
            printf("Falha na conexão Wi-Fi!\n");
        } else {
            printf("Conectado ao Wi-Fi.\n");
            // TODO: Implementar conexão e publicação MQTT usando paho-mqtt ou similar
            // Exemplo: mqtt_client_connect(AIO_USERNAME, AIO_KEY);
            // mqtt_publish(MQTT_FEED, "queda_detectada");
        }
        vTaskDelay(pdMS_TO_TICKS(60000)); // Tenta reconectar/publicar a cada 1 minuto
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
    // TODO: Criar vTaskDisplay
    
    vTaskStartScheduler();
    
    while (true);
}
```

## 9. Próximos Passos e Melhorias
-   **Implementação Completa do MPU6050**: Integrar o driver `mpu6050_i2c.c` e `mpu6050_i2c.h` fornecidos no repositório da BitDogLab para leituras reais do sensor.
-   **Implementação do Display OLED**: Utilizar os drivers `ssd1306_i2c.c` e `ssd1306_i2c.h` para exibir mensagens no display OLED.
-   **Integração MQTT**: Adicionar uma biblioteca MQTT (como `paho-mqtt` ou `lwip_mqtt`) e integrá-la à `vTaskMQTT` para enviar alertas de queda ao Adafruit IO.
-   **Algoritmo de Detecção Avançado**: Refinar o algoritmo de detecção de queda para incluir análise de inatividade e mudança de orientação, reduzindo falsos positivos.
-   **Gerenciamento de Energia**: Implementar modos de baixo consumo para prolongar a vida útil da bateria, se o dispositivo for portátil.
-   **Notificações Mobile**: Configurar o Adafruit IO para enviar notificações push para dispositivos móveis.

## 10. Referências
[1] BitDogLab/BitDogLab-C - `Basic_peripherals/sensor_acelerometro_giroscopio/mpu6050` (https://github.com/BitDogLab/BitDogLab-C/tree/main/Basic_peripherals/sensor_acelerometro_giroscopio/mpu6050)
[2] BitDogLab/BitDogLab-C - `PWM+Display+Buzzer/BitDogLab.c` (https://github.com/BitDogLab/BitDogLab-C/blob/main/PWM+Display+Buzzer/BitDogLab.c)
[3] BitDogLab/BitDogLab-C - `PWM+Display+Buzzer/play_audio.c` (https://github.com/BitDogLab/BitDogLab-C/blob/main/PWM+Display+Buzzer/play_audio.c)
[4] Visual Studio Code - Download (https://code.visualstudio.com/download)
[5] Raspberry Pi Pico C/C++ SDK - Getting Started (https://datasheets.raspberrypi.com/pico/raspberry-pi-pico-c-sdk.pdf)
[6] raspberrypi/pico-examples - FreeRTOS Networking (https://github.com/raspberrypi/pico-examples)
