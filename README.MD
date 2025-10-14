# Sign_Gloves_ESP32

Projeto TCC – Luva tradutora de Libras baseada em ESP32 utilizando PlatformIO.

## Ambiente de desenvolvimento
- VS Code + extensão PlatformIO IDE
- Framework: Arduino
- Porta serial: 115200 bps

## Como compilar e gravar
1. Conecte o ESP32 DevKit V1 via USB.
2. No PlatformIO, selecione a tarefa **PlatformIO: Build** para compilar (`env:esp32-devkit-v1`).
3. Pressione **PlatformIO: Upload** para gravar o firmware. O script já usa a taxa de 921600 bps.
4. Se necessário, mantenha o botão **BOOT** pressionado enquanto o terminal mostrar `Connecting...` e solte-o assim que a escrita iniciar.
5. Abra **PlatformIO: Monitor** para acompanhar a saída serial (115200 bps).

## Resolvendo falhas de gravação (``Connecting...`` infinito)
- **Confirme a placa correta**: este projeto agora utiliza `board = esp32doit-devkit-v1` no `platformio.ini`. Ajustes antigos para ESP32-C3 impedem o acesso ao chip.
- **Drivers e porta**: verifique se o driver USB-UART (CP210x/CH34x) está instalado e se a porta correta está selecionada em *Device* → *Port*.
- **Modo de boot**: durante o upload, mantenha o botão **BOOT (GPIO0)** pressionado ao aparecer `Connecting...`. Solte quando surgir `Writing at...`.
- **Alimentação e cabeamento**: utilize um cabo USB de dados confiável e evite alimentar sensores por GPIO que possam sobrecarregar a porta USB.
- **Pinos de bootstrap**: certifique-se de que GPIO0, GPIO2, GPIO12, GPIO15 e EN não estejam conectados a módulos que puxem os níveis lógicos para estados inválidos. Sensores flex devem usar apenas GPIO32–GPIO39.
- **Reset manual**: se ainda falhar, pressione rapidamente **EN/RST** assim que o upload for iniciado (logo após `Connecting...`).

## Diagnóstico de hardware
O firmware `src/main.cpp` executa:
- Varredura I2C (SDA=21, SCL=22)
- Configuração e leitura do MPU6050
- Leitura contínua das entradas analógicas configuradas para os sensores flex

Utilize o monitor serial para acompanhar leituras e validar a fiação da luva.

## Como usar este repositório gerado pelo assistente
- **Nenhum arquivo é enviado automaticamente para o seu GitHub**: as alterações ficam somente neste workspace até que você faça o download ou copie o conteúdo manualmente.
- **Integração com o VS Code**: para testar, extraia/baixe este projeto e abra a pasta no VS Code com a extensão PlatformIO instalada. A configuração `platformio.ini` já está ajustada para o ESP32 DevKit V1.
- **Sincronização com seu repositório**: caso deseje colocar estes arquivos no seu GitHub, faça o commit/push pelo seu ambiente local ou substitua os arquivos no seu repositório conforme necessário.
