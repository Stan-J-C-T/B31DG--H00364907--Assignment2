#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"


#include "semphr.h" 
// defining I/O pins
#define t1out 1  // T1 digital out pin
#define t2in 2   // T2 digital in pin
#define t3in 3   // T3 digital in pin
#define t4in 4   // T4 analogue in pin
#define t4out 10 // T4 LED pin out
#define t7in 7   // T7 digital in pin
#define t7out 8  // T7 digital out pin
QueueHandle_t buttonQueue;  // Queue for button press events
// variables for timing using micros()
unsigned long tm;
unsigned long newtm;

// freqency calculation on Task2 & 3 square waves
volatile unsigned long freqhigh;
volatile int freq1;
volatile int freq2;
int mapf1;
int mapf2;

// Task4 avg analogue value calculation
int a[10];
volatile int counter = 0;
int avg = 0;

char buffer[100];
int buffer_flag = 0;

void Task1(void *pvParameters)
{
    while (1)
    {
        // Task1 pulse sequence
        digitalWrite(t1out, HIGH);
        delayMicroseconds(180);
        digitalWrite(t1out, LOW);
        delayMicroseconds(40);
        digitalWrite(t1out, HIGH);
        delayMicroseconds(530);
        digitalWrite(t1out, LOW);
        delayMicroseconds(250);
        vTaskDelay(3 / portTICK_PERIOD_MS);
    }
}

int pulse_m(int pin, int state, int timeout)
{
    tm = micros();
    while (digitalRead(pin) != state)
    {
        if (micros() - tm > timeout)
        {
            return 0;
        }
    }
    newtm = micros();
    while (digitalRead(pin) == state)
    {
        if (micros() - newtm > timeout)
        {
            return 0;
        }
    }
    return micros() - tm;
}

void Task2(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
            freqhigh = pulseIn(t2in, HIGH, 3000);
            freq1 = (freqhigh == 0) ? 0 : 1000000.0 / (freqhigh * 2.0);
            xSemaphoreGive(frequencySemaphore);
        }

        // rate 50hz
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void Task3(void *pvParameters) {
    while (1) {
        if (xSemaphoreTake(frequencySemaphore, portMAX_DELAY) == pdTRUE) {
            freqhigh = pulseIn(t3in, HIGH, 3000);
            freq2 = (freqhigh == 0) ? 0 : 1000000.0 / (freqhigh * 2.0);
            xSemaphoreGive(frequencySemaphore);
        }

        // rate 125hz
        vTaskDelay(8 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


void Task4(void *pvParameters)
{
    while (1)
    {
        // counter 0-9
        if (counter > 9)
        {
            counter = 0;
        }
        // read analogue signal on pin
        a[counter] = analogRead(t4in);
        // average value
        avg = (a[0] + a[1] + a[2] + a[3] + a[4] + a[5] + a[6] + a[7] + a[8] + a[9]) / 10;

        counter++;

        // LED high if above 2047 (max 4096)
        if (avg > 2047)
        {
            digitalWrite(t4out, HIGH);
        }
        else
        {
            digitalWrite(t4out, LOW);
        }
        // rate 50hz
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}


int map_s(int x, int in_min, int in_max)
{
    if (x < in_min)
    {
        return out_min;
    }
    if (x > in_max)
    {
        return out_max;
    }
    return (x - in_min) * 99 / (in_max - in_min) ;
}

void Task5(void *pvParameters)
{
    // Task5 print T2 & 3 values
    // HardwareSerial *Serial = (HardwareSerial *)pvParameters;
    // 10hz
    while (1)
    {
        // setting outliers of freq range as 0 or 99
        mapf1 = map_s(freq1, 333, 1000);
        mapf2 = map_s(freq2, 500, 1000);

        // print T2 & 3 values
        sprintf(buffer, "T2: %d Hz, T3: %d Hz\n", mapf1, mapf2);
        buffer_flag = 1;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void ButtonTask(void *pvParameters) {
    int button_state = 0;
    int lastButtonState = LOW;

    while (1) {
        int currentButtonState = digitalRead(t7in);
        if (currentButtonState == HIGH && lastButtonState == LOW) {
            // Button pressed
            button_state ^= 1;  // Toggle state
            xQueueSend(buttonQueue, &button_state, portMAX_DELAY);  // Send state to queue
        }
        lastButtonState = currentButtonState;
        vTaskDelay(pdMS_TO_TICKS(10));  // Debounce delay
    }
}

void LedControlTask(void *pvParameters) {
    int receivedState;

    while (1) {
        if (xQueueReceive(buttonQueue, &receivedState, portMAX_DELAY) == pdPASS) {
            digitalWrite(t7out, receivedState ? HIGH : LOW);
        }
    }
}


void CPU_work(int time_ms) {
    volatile int dummy = 0;  
    long iterations = time_ms * 1000;  


    for (long i = 0; i < iterations; i++) {
        dummy += i;  
    }
}

void somePeriodicTask(void *pvParameters) {
    const TickType_t xDelay = 20 / portTICK_PERIOD_MS;  
    TickType_t xLastWakeTime = xTaskGetTickCount();  

    while (1) {
        CPU_work(2);  

        vTaskDelayUntil(&xLastWakeTime, xDelay);
    }
}

void setup()
{
    // // set baud rate 9600
    Serial.begin(115200);

    // // initialise output pins for Task1 & 4
    pinMode(t1out, OUTPUT);
    pinMode(t4out, OUTPUT);
    pinMode(t7out, OUTPUT);
    frequencySemaphore = xSemaphoreCreateMutex();
    if (frequencySemaphore == NULL) {
        Serial.println("Failed to create frequencySemaphore!");
    }
    // // initialise input pins for Task2, 3 & 4
    pinMode(t2in, INPUT);
    pinMode(t3in, INPUT);
    pinMode(t4in, INPUT);

    // initialise an extra pin 38 for output pwm
    // 700hz and duty = 50%
    ledcSetup(0, 700, 8);
    ledcAttachPin(38, 0);
    ledcWrite(0, 128);

    // // create tasks
    xTaskCreate(Task1, "Task1", 4096, NULL, 2, NULL);
    xTaskCreate(Task2, "Task2", 4096, NULL, 1, NULL);
    xTaskCreate(Task3, "Task3", 4096, NULL, 1, NULL);
    xTaskCreate(Task4, "Task4", 4096, NULL, 0, NULL);
    xTaskCreate(Task5, "Task5", 4096, NULL, 0, NULL);
     buttonQueue = xQueueCreate(10, sizeof(int));  // Create a queue capable of holding 10 integers
    if (buttonQueue == NULL) {
        Serial.println("Failed to create the queue");
        // Handle error
    }

    // Initialize other hardware settings
    pinMode(t7in, INPUT);
    pinMode(t7out, OUTPUT);

    // Start the scheduler with tasks
    xTaskCreate(ButtonTask, "ButtonTask", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
    xTaskCreate(LedControlTask, "LedControlTask", configMINIMAL_STACK_SIZE, NULL, 0, NULL);
    xTaskCreate(somePeriodicTask, "Periodic Task", 2048, NULL, 0, NULL);
    vTaskStartScheduler();
}

void loop()
{
    vTaskDelay(1 / portTICK_PERIOD_MS);
    if (buffer_flag == 1)
    {
        Serial.print(buffer);
        buffer_flag = 0;
    }
    // printf("loop task\n");
}
