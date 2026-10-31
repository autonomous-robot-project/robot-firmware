# robot-firmware

Código embarcado de atuação dos motores, leitura de sensores, comunicação com o sistema principal do robô e lógica de segurança.

## Análise estática

clang-tidy com o `.clang-tidy`, pela toolchain esp-clang (`idf_tools.py install esp-clang`, `pip install pyclang`), num diretório de build separado, para o build com GCC não ser alterado:

```
IDF_TOOLCHAIN=clang idf.py -B build-clang clang-check --run-clang-tidy-options "-config-file=$PWD/.clang-tidy"
```

O relatório vai para `warnings.txt`. O `IDF_TOOLCHAIN` vai no ambiente: o `clang-check` reconfigura o build sozinho e ignora uma opção `-D`.

## Ferramentas de bancada

`tools/motor_characterization`: caracterização dos motores em malha aberta, com as rodas suspensas. É um app ESP-IDF separado que reaproveita o componente `controller` (sem enlace, sem micro-ROS). Compile-o na própria pasta com `idf.py build`, grave o app em `0x10000` como o firmware do robô e rode no RPi `python3 characterize.py record <porta USB> <arquivo.csv>`: o script inicia o ensaio, grava o CSV e imprime a zona morta, a velocidade de regime e a constante de tempo de cada degrau, e a reta velocidade = ganho · (|duty| − zona morta). Depois, grave de volta o firmware do robô.

O freio depois de cada degrau é o freio total, que solicita o motor e a caixa de redução mecânica e termicamente. Os degraus vão do menor duty para o maior: observe os primeiros freios, em baixa velocidade, e corte a alimentação dos motores se a caixa mostrar problema (ADR 023).

`tools/front_range`: as medidas do sensor frontal (VL53L0X), para o T_S e a tabela de status da ADR 021. É um app ESP-IDF separado que roda a tarefa de sensor do componente `controller`, com a recuperação do I²C, e imprime uma linha CSV por medida; não aciona motor. Compile e grave como o `motor_characterization` e rode no RPi, um cenário por vez, `python3 front_range.py record <porta USB> <arquivo.csv> <segundos>` e `python3 front_range.py analyze <arquivo.csv> [status não classificáveis]`. Depois, grave de volta o firmware do robô.

## Testes de unidade no host

`test/host/run.sh` compila e roda, com `g++` puro e sem ESP-IDF, os testes de unidade das classes de controle que só usam o `control_config.h`. Os modelos do motor, do encoder e da malha fechada de uma roda estão em `test/host/wheel_model.h`. Hoje: `WheelPI` (integral limitada com a roda travada ou a referência fora de alcance, tempo de acomodação dentro de t_s, velocidade mantida em v_min pelo encoder, freio abaixo de v_min) e `WheelTrackingMonitor` (o ensaio da V-02: sem falha na partida, travamento reportado em até t* + T_f, liberado depois da soltura, nada julgado abaixo de v_min).
