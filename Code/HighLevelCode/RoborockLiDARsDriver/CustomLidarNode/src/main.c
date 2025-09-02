#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "serial.h"
#include "gpio.h"
#include "parser.h"

volatile sig_atomic_t running = 1;

void sigint_handler(int sig) {
    running = 0;
}

int main() {
    signal(SIGINT, sigint_handler);
    
    // Открытие UART
    int uart_fd = open_serial("/dev/ttyAMA0", B115200);
    if (uart_fd < 0) {
        perror("Failed to open serial port");
        return 1;
    }
    
    // Настройка GPIO и PWM
    if (gpio_init() < 0) {
        fprintf(stderr, "GPIO initialization failed\n");
        close(uart_fd);
        return 1;
    }
    start_motor(1.0);  // Запуск мотора на 100% мощности
    
    // Основной цикл
    unsigned char buffer[256];
    while (running) {
        int bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            feed_data(buffer, bytes_read);  // Парсинг данных
        } else if (bytes_read < 0) {
            perror("Read error");
            break;
        }
        usleep(5000);  // Небольшая пауза
    }
    
    // Корректное завершение
    stop_motor();
    gpio_cleanup();
    close(uart_fd);
    printf("Program terminated\n");
    return 0;
}
