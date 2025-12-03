#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <stdio.h>
#include "exerciser_app.h"
 
/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000
 
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
 
/*
* A build error on this line means your board is unsupported.
*/
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
 
/* This thread will run in a loop */
void blinky_thread(void)
{
    int ret;
    bool led_state = true;
 
    if (!gpio_is_ready_dt(&led)) {
        printk("LED GPIO not ready\n");
        return;
    }
 
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        printk("LED GPIO configure failed\n");
        return;
    }
 
    while (1) {
        gpio_pin_toggle_dt(&led);
 
        led_state = !led_state;
        //printk("LED state: %s\n", led_state ? "ON" : "OFF");
 
        k_msleep(SLEEP_TIME_MS);
    }
}
