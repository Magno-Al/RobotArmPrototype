#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <ESP32Servo.h>

// ================================================================================ PINOUT DEF
#define PINOUT_SG90X 22
#define PINOUT_SG90Y 19
#define PINOUT_SG90Z 18
#define PINOUT_SG90G 21

#define PINOUT_REDLED 5
#define PINOUT_YELLOWLED 13
#define PINOUT_GREENLED 12

#define PINOUT_EMERGENCYSTOPBUTTON 14
#define PINOUT_STARTBUTTON 27

struct MotorControlParams {
  Servo* motor;
  int currentPosition;
  int targetPosition;
};

enum MachineState {
  InProcess,
  Waiting,
  EmergencyStop
};

Servo sg90X;
Servo sg90Y;
Servo sg90Z;
Servo sg90G;

bool isReadyToProcess = false;
MachineState currentState;
xQueueHandle processStateQueue;

// PROCESS REGION ====================================================================================|
void motorControlTask(void *pvParameters) {
  MotorControlParams* params = (MotorControlParams*)pvParameters;
  Servo* motor = params->motor;

  // Verifica se o estado é EmergencyStop antes de iniciar a movimentação do motor
  if (xQueueReceive(processStateQueue, &currentState, 0) == pdPASS) {
    if(currentState == EmergencyStop) {
      vTaskDelete(NULL); // Se estiver em estado de emergência, encerra a tarefa
    }
  }

  // Mover o motor para a posição alvo
  if (params->currentPosition < params->targetPosition) {
    for (; params->currentPosition <= params->targetPosition; params->currentPosition++) {
      motor->write(params->currentPosition);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  } else {
    for (; params->currentPosition >= params->targetPosition; params->currentPosition--) {
      motor->write(params->currentPosition);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }
  vTaskDelete(NULL); // Tarefa concluída
}

void processTask(void *pvParameters) {
  Serial.println("Starting task processSetupTask");

  // Configurar parâmetros para cada motor
  MotorControlParams paramsX = { &sg90X, 0, 45 };
  MotorControlParams paramsY = { &sg90Y, 0, 45 };
  MotorControlParams paramsZ = { &sg90Z, 0, 60 };
  MotorControlParams paramsG = { &sg90G, 0, 45 };

  while(true) {
    if(isReadyToProcess) {
      paramsX.targetPosition = random(0, 181);
      paramsY.targetPosition = random(50, 181);
      paramsZ.targetPosition = random(0, 61);
      paramsG.targetPosition = random(0, 181);

      currentState = InProcess;
      if (xQueueSendToBack(processStateQueue, &currentState, portMAX_DELAY) != pdPASS) {
        Serial.println("Error sending state to queue");
      }

      xTaskCreate(motorControlTask, "motorXControlTask", 2048, &paramsX, 1, NULL);
      xTaskCreate(motorControlTask, "motorYControlTask", 2048, &paramsY, 1, NULL);
      xTaskCreate(motorControlTask, "motorZControlTask", 2048, &paramsZ, 1, NULL);
      xTaskCreate(motorControlTask, "motorGControlTask", 2048, &paramsG, 1, NULL);

      // Aguardar até que todos os motores concluam suas tarefas
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
    vTaskDelay(pdMS_TO_TICKS(2500));
  }
}

// ALARM REGION ======================================================================================|
void alarmTask(void *pvParameters) {

  while (true) {
    if (xQueueReceive(processStateQueue, &currentState, portMAX_DELAY) == pdPASS) {
      switch (currentState) {
        case InProcess: // BLINK LED GREEN
          while (currentState == InProcess) {
            digitalWrite(PINOUT_GREENLED, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(PINOUT_GREENLED, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));

            if (xQueueReceive(processStateQueue, &currentState, 0) == pdPASS) {
              digitalWrite(PINOUT_GREENLED, LOW);
              break;
            }
          }
          break;

        case Waiting: // BLINK LED YELLOW
          while (currentState == Waiting) {
            digitalWrite(PINOUT_YELLOWLED, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(PINOUT_YELLOWLED, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));

            if (xQueueReceive(processStateQueue, &currentState, 0) == pdPASS) {
              digitalWrite(PINOUT_YELLOWLED, LOW);
              break;
            }
          }
          break;

        case EmergencyStop: // BLINK LED RED
          while (currentState == EmergencyStop) {
            digitalWrite(PINOUT_REDLED, HIGH);
            vTaskDelay(pdMS_TO_TICKS(500));
            digitalWrite(PINOUT_REDLED, LOW);
            vTaskDelay(pdMS_TO_TICKS(500));

            if (xQueueReceive(processStateQueue, &currentState, 0) == pdPASS) {
              digitalWrite(PINOUT_REDLED, LOW);
              break;
            }
          }
          break;
      }
    }
  }
}

// SECURITY REGION ====================================================================================|
void securityTask(void *pvParameters) {
  while(true) {
    if(digitalRead(PINOUT_EMERGENCYSTOPBUTTON) == LOW) {
      Serial.println("Emergency Stop button pressed");
      currentState = EmergencyStop;
      if (xQueueSendToBack(processStateQueue, &currentState, portMAX_DELAY) != pdPASS) {
        Serial.println("Error sending state to queue");
      }
      isReadyToProcess = false; // Pausar o processamento
      vTaskDelay(pdMS_TO_TICKS(5000)); // Anti-rebounce delay
    }

    // Detectar botão Start para retomar o processo
    if(digitalRead(PINOUT_STARTBUTTON) == LOW) {
      Serial.println("Start button pressed");
      isReadyToProcess = true; // Retomar o processamento
      vTaskDelay(pdMS_TO_TICKS(5000)); // Anti-rebounce delay
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeno atraso para evitar polling excessivo
  }
}

// TASK PARA IMPRIMIR O ESTADO ATUAL ==================================================================|
void statusMonitorTask(void *pvParameters) {

  MachineState previousState = Waiting; // Estado inicial presumido

  while (true) {
    if (xQueueReceive(processStateQueue, &currentState, portMAX_DELAY) == pdPASS) {
      if (currentState != previousState) { // Só imprime se houver mudança de estado
        switch (currentState) {
          case InProcess:
            Serial.println("State: InProcess");
            break;
          case Waiting:
            Serial.println("State: Waiting");
            break;
          case EmergencyStop:
            Serial.println("State: EmergencyStop");
            break;
        }
        previousState = currentState; // Atualiza o estado anterior
      }
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Pequeno atraso para evitar polling excessivo
  }
}

// ================================================================================ SETUP
void setup() {
  Serial.begin(115200);
  Serial.println("Starting Setup");

  pinMode(PINOUT_EMERGENCYSTOPBUTTON, INPUT_PULLUP);
  pinMode(PINOUT_STARTBUTTON, INPUT_PULLUP);

  pinMode(PINOUT_REDLED, OUTPUT);
  pinMode(PINOUT_YELLOWLED, OUTPUT);
  pinMode(PINOUT_GREENLED, OUTPUT);

  sg90X.setPeriodHertz(50);
  sg90Y.setPeriodHertz(50);
  sg90Z.setPeriodHertz(50);
  sg90G.setPeriodHertz(50);

  if (!sg90X.attach(PINOUT_SG90X, 500, 2400)) {
    Serial.println("[Error] Failed to attach sg90X to pin 22");
  }

  if (!sg90Y.attach(PINOUT_SG90Y, 500, 2400)) {
    Serial.println("[Error] Failed to attach sg90Y to pin 19");
  }

  if (!sg90Z.attach(PINOUT_SG90Z, 500, 2400)) {
    Serial.println("[Error] Failed to attach sg90Z to pin 18");
  }

  if (!sg90G.attach(PINOUT_SG90G, 500, 2400)) {
    Serial.println("[Error] Failed to attach sg90G to pin 21");
  }

  randomSeed(analogRead(A0));

  processStateQueue = xQueueCreate(10, sizeof(MachineState));
  if (processStateQueue == NULL) {
    Serial.println("Error creating queue.");
    return;
  }
  Serial.println("Queue created.");

  isReadyToProcess = true;

  // Cria as tarefas FreeRTOS
  xTaskCreate(processTask, "processTask", 2048, NULL, 2, NULL);
  xTaskCreate(alarmTask, "alarmTask", 2048, NULL, 2, NULL);
  xTaskCreate(securityTask, "securityTask", 2048, NULL, 3, NULL);
  xTaskCreate(statusMonitorTask, "statusMonitorTask", 2048, NULL, 1, NULL);
}

void loop() {
  // O loop principal está vazio pois estamos usando FreeRTOS para gerenciar tarefas
}