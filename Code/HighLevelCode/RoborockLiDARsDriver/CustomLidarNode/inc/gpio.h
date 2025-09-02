#ifndef GPIO_H
#define GPIO_H

int gpio_init(void);
void gpio_cleanup(void);
void start_motor(float duty_cycle);
void stop_motor(void);

#endif
