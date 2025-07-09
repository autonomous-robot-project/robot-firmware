#include <Arduino.h>
#include <PID_v1.h>
#include <esp_task_wdt.h> // Para o Watchdog Timer
#include "driver/pcnt.h" // Biblioteca para o Pulse Counter
#include "uart_comm.h"
#include "motor.h"

// --- Pinos de Saída para os Motores ---
const int MOTOR_LEFT_PWM_PIN = 19;
const int MOTOR_LEFT_IN1_PIN = 5;
const int MOTOR_LEFT_IN2_PIN = 18;

const int MOTOR_RIGHT_PWM_PIN = 15;
const int MOTOR_RIGHT_IN1_PIN = 4;
const int MOTOR_RIGHT_IN2_PIN = 16;

// --- Configurações do PWM (LEDC) ---
const int PWM_FREQ = 25000;
const int PWM_RESOLUTION = 8;
const int PWM_CHANNEL_LEFT = 0;
const int PWM_CHANNEL_RIGHT = 1;

// --- Pinos e Constantes dos Encoders ---
const int ENCODER_LEFT_PIN = 14;
const int ENCODER_RIGHT_PIN = 13;
const int ENCODER_PULSES_PER_REV = 20;

DCMotor motorLeft;
DCMotor motorRight;

// --- Variáveis de Controle ---
double rpm_left = 0.0;     // (Input) RPM medido para a roda esquerda
double rpm_right = 0.0;    // (Input) RPM medido para a roda direita
double setpoint_left = 0.0;  // (Setpoint) RPM desejado para a roda esquerda
double setpoint_right = 0.0; // (Setpoint) RPM desejado para a roda direita
double output_left = 0.0;    // (Output) Saída do PID (PWM) para a roda esquerda
double output_right = 0.0;   // (Output) Saída do PID (PWM) para a roda direita

// --- Parâmetros de Tuning do PID ---
double Kp = 3.0, Ki = 0.1, Kd = 0.1;

// --- Objetos PID ---
PID pidLeft(&rpm_left, &output_left, &setpoint_left, Kp, Ki, Kd, DIRECT);
PID pidRight(&rpm_right, &output_right, &setpoint_right, Kp, Ki, Kd, DIRECT);

// --- Variáveis para o Failsafe de Comunicação ---
const int COMMAND_TIMEOUT_MS = 60; 
volatile TickType_t lastCommandTime; 

// --- Configurações do Loop de Controle ---
const int CONTROL_SAMPLE_TIME_MS = 10; // 10ms (100Hz)

// Handles das tarefas para o FreeRTOS
TaskHandle_t pidTaskHandle;
TaskHandle_t commTaskHandle;

// ================================================================================
// TAREFA DE CONTROLE (PID + Leitura de Encoder)
// ================================================================================
void pidControlTask(void *pvParameters) {
  Serial.println("Tarefa de Controle executando no Core 0.");
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(CONTROL_SAMPLE_TIME_MS);

  int16_t count_left = 0, last_count_left = 0;
  int16_t count_right = 0, last_count_right = 0;

  while (1) {
    // // --- Etapa 1: Ler os Encoders ---
    // pcnt_get_counter_value(PCNT_UNIT_0, &count_left);
    // pcnt_get_counter_value(PCNT_UNIT_1, &count_right);

    // int16_t delta_count_left = count_left - last_count_left;
    // int16_t delta_count_right = count_right - last_count_right;
    // last_count_left = count_left;
    // last_count_right = count_right;

    // // --- Etapa 2: Calcular a Velocidade (RPM) ---
    // rpm_left = (double)delta_count_left / ENCODER_PULSES_PER_REV * (60000.0 / CONTROL_SAMPLE_TIME_MS);
    // rpm_right = (double)delta_count_right / ENCODER_PULSES_PER_REV * (60000.0 / CONTROL_SAMPLE_TIME_MS);

    // // --- Etapa 3: Calcular o PID ---
    // pidLeft.Compute();
    // pidRight.Compute();
    
    // --- Etapa 4: Atuar nos Motores ---
    motorLeft.run(setpoint_left);
    motorRight.run(setpoint_right);

    // // Espera precisamente pelo próximo ciclo
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

//================================================================================
// TAREFA 2: Loop de Comunicação USB
//================================================================================
void communicationTask(void *pvParameters) {
  Serial.println("Tarefa de Comunicação executando no Core 1.");
  lastCommandTime = xTaskGetTickCount();

  while (1) {
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      command.trim();

      int tempL = 0, tempR = 0;
      int itemsParsed = sscanf(command.c_str(), "L%dR%d", &tempL, &tempR);

      if (itemsParsed == 2) {
        // Atualiza os SETPOINTS para o PID
        setpoint_left = tempL;
        setpoint_right = tempR;
        lastCommandTime = xTaskGetTickCount();
        Serial.printf("ACK: Setpoints recebidos. Esquerda: %.0f, Direita: %.0f\n", setpoint_left, setpoint_right);
      } else {
        Serial.printf("NACK: Formato de comando inválido. Recebido: '%s'\n", command.c_str());
      }
    }
  
    if ((xTaskGetTickCount() - lastCommandTime) > pdMS_TO_TICKS(COMMAND_TIMEOUT_MS)) {
      setpoint_left = 0;
      setpoint_right = 0;
    }


    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

//================================================================================
// FUNÇÃO DE SETUP PARA UM ENCODER (usando PCNT)
//================================================================================
void setup_encoder(int encoder_pin, pcnt_unit_t pcnt_unit) {
  pcnt_config_t pcnt_config = {
      .pulse_gpio_num = encoder_pin,
      .ctrl_gpio_num = -1, // Pino de controle (direção), não temos, então -1
      .lctrl_mode = PCNT_MODE_DISABLE, // Não usar controle de nível baixo
      .hctrl_mode = PCNT_MODE_DISABLE, // Não usar controle de nível alto
      .pos_mode = PCNT_COUNT_INC,      // Incrementar o contador na borda de subida
      .neg_mode = PCNT_COUNT_DIS,      // Não fazer nada na borda de descida
      .counter_h_lim = 32767,          // Limite máximo do contador
      .counter_l_lim = -32767,         // Limite mínimo do contador
      .unit = pcnt_unit,
      .channel = PCNT_CHANNEL_0,
  };
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(pcnt_unit);
  pcnt_counter_clear(pcnt_unit);
  pcnt_counter_resume(pcnt_unit);
}

//================================================================================
// FUNÇÃO DE SETUP para Motores e PID
//================================================================================
void setup_motors_and_pid(){

  // --- Configura os canais PWM (LEDC) ---
  ledcAttachChannel(MOTOR_LEFT_PWM_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_LEFT);
  ledcAttachChannel(MOTOR_RIGHT_PWM_PIN, PWM_FREQ, PWM_RESOLUTION, PWM_CHANNEL_RIGHT);

  motorLeft.attach(MOTOR_LEFT_IN1_PIN, MOTOR_LEFT_IN2_PIN, MOTOR_LEFT_PWM_PIN);
  motorRight.attach(MOTOR_RIGHT_IN1_PIN, MOTOR_RIGHT_IN2_PIN, MOTOR_RIGHT_PWM_PIN);

  // --- Configura os PIDs ---
  double max_pwm = pow(2, PWM_RESOLUTION) - 1;
  pidLeft.SetOutputLimits(-max_pwm, max_pwm);
  pidRight.SetOutputLimits(-max_pwm, max_pwm);

  pidLeft.SetSampleTime(CONTROL_SAMPLE_TIME_MS);
  pidRight.SetSampleTime(CONTROL_SAMPLE_TIME_MS);

  pidLeft.SetMode(AUTOMATIC);
  pidRight.SetMode(AUTOMATIC);
  
  Serial.println("Motores e PIDs configurados.");
}


//================================================================================
// SETUP: Configuração Inicial
//================================================================================
void setup()
{
  Serial.begin(115200);
  delay(1000);

  handshake();
  Serial.println("ESP32 ready.");

  // --- Configura os dois encoders em unidades de PCNT separadas ---
  Serial.println("Configurando encoder esquerdo na Unidade 0...");
  setup_encoder(ENCODER_LEFT_PIN, PCNT_UNIT_0);
  
  Serial.println("Configurando encoder direito na Unidade 1...");
  setup_encoder(ENCODER_RIGHT_PIN, PCNT_UNIT_1); // O ESP32 tem várias unidades de PCNT

  // --- Configura os Motores e os PIDs ---
  Serial.println("Configurando Motores e PIDs...");
  setup_motors_and_pid();

  // --- Cria as tarefas e as fixa nos núcleos ---
  xTaskCreatePinnedToCore(pidControlTask,"PIDControl",8192,NULL,2,&pidTaskHandle,0);
  xTaskCreatePinnedToCore(communicationTask,"USBComm",8192,NULL,1,&commTaskHandle,1);

  // Deleta a tarefa "loop" que o Arduino cria por padrão
  vTaskDelete(NULL);
}

void loop()
{
}
