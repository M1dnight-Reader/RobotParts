#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/mcpwm.h"
#include "driver/gpio.h"

#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include "freertos/queue.h"

#define IN1 GPIO_NUM_32
#define IN2 GPIO_NUM_33
#define IN3 GPIO_NUM_25
#define IN4 GPIO_NUM_26

#define ENA1 GPIO_NUM_5   // MCPWM0A
#define ENB1 GPIO_NUM_18  // MCPWM0B

#define IN5 GPIO_NUM_27
#define IN6 GPIO_NUM_14
#define IN7 GPIO_NUM_12
#define IN8 GPIO_NUM_13

#define ENA2 GPIO_NUM_19  // MCPWM1A
#define ENB2 GPIO_NUM_21  // MCPWM1B

#define UART_RX_PIN  16
#define UART_TX_PIN  17
#define UART_NUM     UART_NUM_1
#define BUF_SIZE     1024
#define RD_BUF_SIZE  (BUF_SIZE)
#define QUEUE_SIZE 64

// S //
typedef struct {
    uint32_t timestamp_ms; // millis()
    float rpmA, rpmB, rpmC, rpmD;
} rpm_msg_t;

typedef struct {
    float Kp, Ki, Kd;
    float integrator;
    float prev_error;
    float prev_measurement;
    float d_filtered;
    float out; // последнее значение управляющего (0..100)
    float out_min, out_max;
    float integrator_min, integrator_max;
    float deriv_tau; // фильтр времени для D (seconds)
} pid_t;

// V //
static const char *TAG = "uart_rx";
static QueueHandle_t uart_queue;

// F //
void setup_gpio();
void setup_mcpwm();
void set_motor(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, int duty, gpio_num_t dir1, gpio_num_t dir2, uint8_t direction);
void motor_task(void *arg);

void pid_init(pid_t *p, float Kp, float Ki, float Kd, float out_min, float out_max);
float pid_update(pid_t *p, float setpoint, float measurement, float Ts);
void pid_task(void *arg);

void uart_init_simple();
//void uart_rx_task(void *arg);
void uart_event_task(void *pvParameters);

void app_main(void) {
	setup_gpio();
	setup_mcpwm();

	xTaskCreate(motor_task, "motor_task", 2048, NULL, 5, NULL);
        
	static QueueHandle_t rpm_queue = NULL;
        uart_init_simple();
      	xTaskCreate(uart_event_task, "uart_rx_task", 2048, NULL, 5, NULL);
}

void setup_gpio() {
	gpio_config_t io_conf = {
		.mode = GPIO_MODE_OUTPUT,
                .pin_bit_mask = (1ULL << IN1) | (1ULL << IN2) | (1ULL << IN3) | (1ULL << IN4)
			      | (1ULL << IN5) | (1ULL << IN6) | (1ULL << IN7) | (1ULL << IN8)};
	gpio_config(&io_conf);
}

void setup_mcpwm() {
	mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, ENA1); // PWM для двигателя A
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, ENB1); // PWM для двигателя B
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, ENA2); // PWM для двигателя B
        mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, ENB2); // PWM для двигателя B

	mcpwm_config_t pwm_config = {
		.frequency = 1000,
		.cmpr_a = 0,
		.cmpr_b = 0,
		.counter_mode = MCPWM_UP_COUNTER,
		.duty_mode = MCPWM_DUTY_MODE_0,
	};

	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
	mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config);
}

void set_motor(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op, int duty, gpio_num_t dir1, gpio_num_t dir2, uint8_t direction) {
    // направление
    if (direction == 0) {
        gpio_set_level(dir1, 1);
        gpio_set_level(dir2, 0);
    } else {
        gpio_set_level(dir1, 0);
        gpio_set_level(dir2, 1);
    }
    // ШИМ
    mcpwm_set_duty(unit, timer, op, duty);
}

void motor_task(void *arg) {
    while (1) {
        // Setpoint test 1: A:60rpm, B:-60rpm, C:60rpm, D:-60rpm	
	// Первый драйвер
        set_motor(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A,  60, IN1, IN2, 1); // alpha
        set_motor(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B,  60, IN3, IN4, 1); // charl
        // Второй драйвер
        set_motor(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A,  80, IN5, IN6, 0); // delta
        set_motor(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_B,  80, IN7, IN8, 0); // bravo

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void uart_init_simple() {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // RX buffer size, no TX buffer required for reading
    uart_driver_install(UART_NUM, RD_BUF_SIZE * 2, 0, QUEUE_SIZE, &uart_queue, 0);
    //uart_driver_install(UART_NUM, RD_BUF_SIZE * 2, 0, 0         , NULL       , 0);
}

void uart_event_task(void *pvParameters) {
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*)malloc(BUF_SIZE);
    for (;;) {
        // ждём событие
        if (xQueueReceive(uart_queue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int rxlen = uart_read_bytes(UART_NUM, dtmp, event.size, pdMS_TO_TICKS(100));
                // dtmp содержит event.size байт — возможно часть строки
                // Собираем в ring buffer / line buffer до '\n'
                static size_t idx = 0;
                for (int i = 0; i < rxlen; ++i) {
                    uint8_t ch = dtmp[i];
                    if (ch == '\r') continue;
                    if (ch == '\n') {
                        dtmp[idx] = '\0';
                        // Обработка полной строки
                        ESP_LOGI(TAG, "Line: %s", (char*)dtmp);
                        
			// Парсинг
			//sscanf(dtmp, "A:%f,B:%f,C:%f,D:%f", &a,&b,&c,&d);
                        //printf("Scanned A:%f,B:%f,C:%f,D:%f", &a,&b,&c,&d);;

			idx = 0;
                    } else {
                        if (idx < BUF_SIZE - 1) {
                            dtmp[idx++] = ch;
                        } else {
                            // переполнение - сброс
                            idx = 0;
                        }
                    }
                }
            } else if (event.type == UART_FIFO_OVF) {
                ESP_LOGW(TAG, "UART FIFO overflow");
                uart_flush_input(UART_NUM);
                xQueueReset(uart_queue);
            } else if (event.type == UART_BUFFER_FULL) {
                ESP_LOGW(TAG, "UART Buffer full");
                uart_flush_input(UART_NUM);
                xQueueReset(uart_queue);
            } else {
                // другие события: UART_BREAK, PARITY_ERR и т.д.
            }
        }
    }
    free(dtmp);
    vTaskDelete(NULL);
}

void pid_init(pid_t *p, float Kp, float Ki, float Kd, float out_min, float out_max) {
    p->Kp = Kp; p->Ki = Ki; p->Kd = Kd;
    p->integrator = 0.0f;
    p->prev_error = 0.0f;
    p->prev_measurement = 0.0f;
    p->d_filtered = 0.0f;
    p->out = 0.0f;
    p->out_min = out_min; p->out_max = out_max;
    p->integrator_min = -1000.0f; p->integrator_max = 1000.0f;
    p->deriv_tau = 0.02f; // 20 ms default
}

// Ts in seconds
float pid_update(pid_t *p, float setpoint, float measurement, float Ts) {
    float error = setpoint - measurement;

    // P
    float P = p->Kp * error;

    // I
    p->integrator += p->Ki * error * Ts;
    // anti-windup by clamping integrator
    if (p->integrator > p->integrator_max) p->integrator = p->integrator_max;
    if (p->integrator < p->integrator_min) p->integrator = p->integrator_min;
    float I = p->integrator;

    // D (derivative on measurement to reduce kick)
    float d_meas = (measurement - p->prev_measurement) / Ts; // d(meas)/dt
    float D_raw = - p->Kd * d_meas;
    // low-pass filter for D (first-order)
    float alpha = p->deriv_tau / (p->deriv_tau + Ts);
    p->d_filtered = alpha * p->d_filtered + (1.0f - alpha) * D_raw;
    float D = p->d_filtered;

    // Compute unclamped output
    float u = P + I + D;

    // clamp output
    if (u > p->out_max) u = p->out_max;
    if (u < p->out_min) u = p->out_min;

    // Optionally limit slew rate:
    float max_delta = 10.0f * Ts; // e.g. 10 %/s -> scaled by Ts
    if ((u - p->out) > max_delta) u = p->out + max_delta;
    if ((u - p->out) < -max_delta) u = p->out - max_delta;

    // Save states
    p->prev_error = error;
    p->prev_measurement = measurement;
    p->out = u;

    return u;
}

void pid_task(void *arg) {
    const TickType_t Ts_ticks = pdMS_TO_TICKS(100); // Ts = 100 ms
    TickType_t last_wake = xTaskGetTickCount();
    rpm_msg_t msg;
    pid_t pidA, pidB, pidC, pidD;
    pid_init(&pidA, 0.6f, 0.1f, 0.01f, 0.0f, 100.0f);
    // init others...

    float setpointA = 60.0f; // target RPM example
    for (;;) {
        vTaskDelayUntil(&last_wake, Ts_ticks);

        // drain queue to get the freshest message
        bool have = false;
        while (xQueueReceive(rpm_queue, &msg, 0) == pdTRUE) {
            have = true;
            // keep looping to take the latest
        }

        if (!have) {
            // timeout — no new measurements in this window
            // either keep last measurement or set measurement to 0
            // Here: skip one update or use last known measurement stored in pid->prev_measurement
        } else {
            float rpmA_meas = msg.rpmA;
            float rpmB_meas = msg.rpmB;
            // compute PID
            float dutyA = pid_update(&pidA, setpointA, rpmA_meas, 0.1f);
            // apply to MCPWM
            mcpwm_set_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, dutyA);
            // similar for other motors...
        }
    }
}
