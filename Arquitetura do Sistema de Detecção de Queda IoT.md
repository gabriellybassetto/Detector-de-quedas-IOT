# Arquitetura do Sistema de Detecção de Queda IoT

O sistema será composto por três camadas principais: Hardware (BitDogLab), Firmware (C++/FreeRTOS) e Nuvem (MQTT/Dashboard).

## 1. Camada de Hardware
A placa **BitDogLab** atuará como o núcleo de processamento. O sensor **MPU6050** será responsável por capturar os dados de aceleração em três eixos (X, Y, Z).
- **Processamento Local**: O RP2040 processará os dados brutos para identificar picos de aceleração seguidos de uma mudança brusca de orientação ou imobilidade.
- **Alertas Locais**: Em caso de queda, o **Buzzer** emitirá um sinal sonoro e o **Display OLED** mostrará uma mensagem de alerta.

## 2. Camada de Firmware (Software Embarcado)
Utilizaremos o **FreeRTOS** para gerenciar a execução paralela de tarefas críticas:
- **Task_Sensor**: Leitura do MPU6050 a cada 10ms (100Hz).
- **Task_Detection**: Algoritmo de detecção de queda (análise de vetor resultante).
- **Task_Display**: Atualização da interface do usuário no OLED.
- **Task_MQTT**: Gerenciamento da conexão Wi-Fi e publicação de alertas no tópico MQTT.

## 3. Camada de Comunicação e Nuvem
- **Protocolo**: MQTT (Message Queuing Telemetry Transport).
- **Broker**: Utilizaremos um broker público gratuito (como o HiveMQ ou Mosquitto) para facilitar os testes iniciais.
- **Dashboard**: Sugestão de uso do **TagoIO** ou **Adafruit IO** para visualização dos dados e recebimento de notificações no celular.

| Componente | Função | Pino/Interface |
| :--- | :--- | :--- |
| MPU6050 | Acelerômetro | I2C0 (SDA: 0, SCL: 1) |
| SSD1306 | Display OLED | I2C1 (SDA: 14, SCL: 15) |
| Buzzer | Alerta Sonoro | GPIO 21 |
| Pico W | Processamento e Wi-Fi | Integrado |
