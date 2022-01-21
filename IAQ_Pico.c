#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "bsec_datatypes.h"
#include "bsec_integration.h"
#include "bsec_interface.h"
#include "bsec_serialized_configurations_iaq.h"

#define TIMEZONE "TZ=Europe/Berlin"
#define SDA 26
#define SCL 27
#define I2C_PORT i2c1
#define I2C_ADDR BME680_I2C_ADDR_PRIMARY

#define STARTMINICOMUSB "#!/bin/bash\nminicom -b 115200 -o -D /dev/ttyACM0"

#define STARTMINICOMUART "#!/bin/bash\nminicom -b 115200 -o -D /dev/serial0"

#define N_RVS 5

#define TEMP_OFFSET 2.0f
#define SAMPLE_RATE_MODE BSEC_SAMPLE_RATE_LP

#define RED 2
#define YLW 3
#define GRN 4
#define BUZ 15
#define FREQ 500

const char *name_config = "bsec_iaq.config";
const char *name_state = "bsec_iaq.state";
int buz_count = 0;
bool buzzing = false;


// F U N C T I O N S
void gpio_tone(int gpio_number, int frequency, int duration_ms) {
    if (frequency < 0) {
        gpio_put(gpio_number, 0);
    }
    else {
        int sleep_duration_ms = (int) (1000 / (2 * frequency));
        int period_duration_ms = 2 * sleep_duration_ms;
        for (int i = 0; i < duration_ms; i += period_duration_ms) {
            gpio_put(gpio_number, 1);
            sleep_ms(sleep_duration_ms);
            gpio_put(gpio_number, 0);
            sleep_ms(sleep_duration_ms);
        }
    }
}

void gpio_infinite_tone(int gpio_number, int frequency, bool on) {
    if (on) {
        gpio_tone(gpio_number, frequency, 2000);
    }
    else {
        gpio_put(gpio_number, 0);
    }
}

// Core 1 interrupt handler
void core1_interrupt_handler() {

    // Receive raw temparature, convert and print
    while(multicore_fifo_rvalid()) {
        bool buzzing = multicore_fifo_pop_blocking();
        gpio_infinite_tone(BUZ, FREQ, buzzing);
    }

    // Clear interrupt
    multicore_fifo_clear_irq();
}

// Core 1 main function
void core1_entry() {

    // Configure Core 1 interrupt
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC1, core1_interrupt_handler);
    irq_set_enabled(SIO_IRQ_PROC1, true);

    // Primary Core 1 loop
    while(1) {
        tight_loop_contents();
    }
}

void pi3g_gpio_init() {
    gpio_init(RED);
    gpio_init(YLW);
    gpio_init(GRN);
    gpio_init(BUZ);

    gpio_set_dir(RED, GPIO_OUT);
    gpio_set_dir(YLW, GPIO_OUT);
    gpio_set_dir(GRN, GPIO_OUT);
    gpio_set_dir(BUZ, GPIO_OUT);
}

void i2c_init_pi3g() {

    stdio_init_all();

#if !defined(I2C_PORT) || !defined(SDA) || !defined(SCL)
#warning i2c/bus_scan requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    i2c_init(I2C_PORT, 500 * 1000);
    gpio_set_function(SDA, GPIO_FUNC_I2C);
    gpio_set_function(SCL, GPIO_FUNC_I2C);
    gpio_pull_up(SDA);
    gpio_pull_up(SCL);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(SDA, SCL, GPIO_FUNC_I2C));
#endif
}

void i2c_deinit_pi3g() {
    i2c_deinit(I2C_PORT);
}

void i2c_check_sensor() {
    sleep_ms(1000);
    const uint8_t reg_chip_id = BME680_CHIP_ID_ADDR;
    uint8_t chipID[1];
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg_chip_id, 1, true);
    i2c_read_blocking(I2C_PORT, I2C_ADDR, chipID, 1, false);
    
    printf("EXPECTED CHIP ID = %d\n", BME680_CHIP_ID);
    if (chipID[0] != BME680_CHIP_ID) {
        printf("BME680 is not connected properly\n");
    }
    else {
        printf("ACTUAL   CHIP ID = %d\n", chipID[0]); 
        printf("BME680 is connected\n");
    }
}

int8_t pi3g_write(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t length) {
    int8_t rslt = BME680_OK;

    uint8_t buf[length+1];
    buf[0] = regAddr;
    for (int i = 1; i < length+1; i++) {
        buf[i] = regData[i-1];
    }

    int num_bytes;
    num_bytes = i2c_write_blocking(I2C_PORT, devAddr, buf, length+1, false);
    
    if (num_bytes != length+1) {
        printf("num_bytes != length\n");
        perror("pi3g_write");
        rslt = -1;
    }
    return rslt;
}

int8_t pi3g_read(uint8_t devAddr, uint8_t regAddr, uint8_t *regData, uint16_t length)  {
    int8_t rslt = BME680_OK;

    int num_bytes;
    i2c_write_blocking(I2C_PORT, devAddr, &regAddr, 1, true);
    num_bytes = i2c_read_blocking(I2C_PORT, devAddr, regData, length, false);
    if (num_bytes != length) {
        printf("num_bytes != length\n");
        perror("pi3g_read");
        rslt = -1;
    }
    return rslt;
}

void pi3g_sleep_ms(uint32_t tm_ms) {
    sleep_ms(tm_ms);
}

int64_t pi3g_timeStamp_us() {
    return time_us_64();
}

void output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
                float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status,
                float static_iaq, float co2_equivalent, float breath_voc_equivalent) {

    if (iaq < 100) {
        gpio_put(RED, 0);
        gpio_put(YLW, 0);
        gpio_put(GRN, 1);
        buz_count = 0;
    }
    else if (100 <= iaq && iaq < 200) {
        gpio_put(RED, 0);
        gpio_put(YLW, 1);
        gpio_put(GRN, 0);
        buz_count = 0;
    }
    else {
        gpio_put(RED, 1);
        gpio_put(YLW, 0);
        gpio_put(GRN, 0);
        buz_count++;
    }

    if (buz_count >= 20) {
        buzzing = true;
    }
    else {
        buzzing = false;
    }
    multicore_fifo_push_blocking(buzzing);

    printf("IAQ (%d): %.2f | ", iaq_accuracy, iaq);
    printf("Tmp: %.2f degC | Hum: %.2f %%rH | Prs: %.2f hPa | GsR: %.0f kOhm | ", temperature, humidity, pressure/100, gas/1000); // Ohm = \u03a9
    printf("BSEC Stat: %d | ", bsec_status);
    printf("eCO2: %.15f ppm | bVOCe: %.25f ppm | Static IAQ: %.2f", co2_equivalent, breath_voc_equivalent, static_iaq);
    printf("\r\n");
    fflush(stdout);
}

uint32_t pi3g_load_binary(uint8_t *b_buf, uint32_t n_buf, char *filename, uint32_t offset) {
    int32_t copied_bytes = 0;
    int8_t rslt = BME680_OK;

    // TODO

    return rslt;
}

uint32_t pi3g_load_state(const uint8_t *state_buf, uint32_t n_buf) {
    int32_t rslt = BME680_OK;
    // rslt = pi3g_load_binary(state_buf, n_buf, filename_state, 0)

    // TODO
    return rslt;
}

void pi3g_save_state(const uint8_t *state_buf, uint32_t length) {
    // TODO
}

uint32_t pi3g_load_config(uint8_t *conf_buf, uint32_t n_buf) {
    int32_t rslt = BME680_OK;
    // rlst = pi3g_load_binary(conf_buf, n_buf, filename_conf, 4);

    // TODO
    return rslt;
}

// |\/|  /_\   |  |\ |
// |  | /   \  |  | \|                                                 
int main() {
    
    // Allow user to start minicom
    // For USB execute      minicom -b 115200 -o -D /dev/ttyACM0
    // For UART execute     minicom -b 115200 -o -D /dev/serial0
    sleep_ms(5000);
    // Start UART Minicom
    // system(STARTMINICOMUART);

    // Initialize GPIO Pins
    pi3g_gpio_init();

    // Start Core 1 - Must be called before configuring interrupts
    multicore_launch_core1(core1_entry);

    // Initialize I2C
    i2c_init_pi3g();
    
    i2c_check_sensor();
    
    // Check BSEC version
    bsec_version_t version;
    bsec_get_version(&version);
    printf("BSEC version: %d.%d.%d.%d\n", version.major, version.minor, version.major_bugfix, version.minor_bugfix);

    return_values_init ret;
    ret = bsec_iot_init(SAMPLE_RATE_MODE, TEMP_OFFSET, pi3g_write, pi3g_read, pi3g_sleep_ms, pi3g_load_state, pi3g_load_config);

    if(ret.bme680_status) {
        printf("Failed to initialize BME680\nreturn %d\n", ret.bme680_status);
        return (int)ret.bme680_status;
    }
    else if (ret.bsec_status) {
        printf("Failed to initialize BSEC\nreturn %d\n", ret.bsec_status);
        return (int)ret.bsec_status;
    }

    printf("S T A R T I N G     M E A S U R E M E N T\n");
    bsec_iot_loop(pi3g_sleep_ms, pi3g_timeStamp_us, output_ready, pi3g_save_state, 100000);

    i2c_deinit_pi3g();

    return 0;
}