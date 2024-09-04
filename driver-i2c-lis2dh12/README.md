# Driver LIS2DH12 I2C

Este projeto fornece um driver para o acelerômetro LIS2DH12 usando a interface I2C. O driver é implementado em C e projetado para rodar na ESP32.

## Funcionalidades

- Inicializar e desinicializar o sensor LIS2DH12
- Ler o ID do dispositivo
- Configurar as configurações do sensor
- Ler dados de aceleração dos eixos X, Y e Z
- Ler dados de temperatura (se habilitado)

## Requisitos

- Framework ESP-IDF
- Compilador C
- Biblioteca I2C
- FreeRTOS (para funções de delay em tarefas)

## Uso

### Inicialização

Para inicializar o sensor LIS2DH12, chame a função `lis2dh12_test_init`. Esta função configura o barramento I2C e configura o sensor.

### Leitura de Dados

Para ler dados do sensor, chame a função `lis2dh12_test_get_data`. Esta função lê o ID do dispositivo, as configurações e os dados de aceleração do sensor.

### Desinicialização

Para desinicializar o sensor e liberar recursos, chame a função `lis2dh12_test_deinit`.

## Exemplo

Aqui está um exemplo de como usar o driver em sua aplicação:

```c
void app_main(void)
{
    lis2dh12_test_init();
    vTaskDelay(1000 / portTICK_RATE_MS);
    lis2dh12_test_get_data();
    lis2dh12_test_deinit();
}
