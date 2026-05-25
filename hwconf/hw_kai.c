/*
 * hw_kai.c
 * Implementasi hardware untuk KAI INVERTER FUEL PUMP
 * Project  : 2509-KAI-PIFP-A
 * MCU      : STM32F405RGT6
 * Driver   : IR2110 (3 buah, satu per fase)
 */

#include "hw.h"
#include "hw_kai.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "utils_math.h"
#include "terminal.h"
#include "mc_interface.h"
#include <math.h>

// =========================================================
// I2C state
// =========================================================
static volatile bool i2c_running = false;

static const I2CConfig i2cfg = {
    OPMODE_I2C,
    100000,
    STD_DUTY_CYCLE
};

// =========================================================
// hw_init_gpio()
// Setup semua pin GPIO saat boot
// =========================================================
void hw_init_gpio(void) {

    // Enable GPIO clock semua port yang dipakai
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // -------------------------------------------------
    // EN_GATE -> PB5
    // IR2110 SD pin: HIGH = enable gate driver
    // Set output dulu, lalu enable
    // -------------------------------------------------
    palSetPadMode(GPIOB, 5,
            PAL_MODE_OUTPUT_PUSHPULL |
            PAL_STM32_OSPEED_HIGHEST);
    ENABLE_GATE();

    // -------------------------------------------------
    // LED -> PB0 (GREEN), PB1 (RED)
    // -------------------------------------------------
    palSetPadMode(LED_GREEN_GPIO, LED_GREEN_PIN,
            PAL_MODE_OUTPUT_PUSHPULL |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(LED_RED_GPIO, LED_RED_PIN,
            PAL_MODE_OUTPUT_PUSHPULL |
            PAL_STM32_OSPEED_HIGHEST);
    LED_GREEN_OFF();
    LED_RED_OFF();

    // -------------------------------------------------
    // PWM output TIM1 (WAJIB - ini yang generate SVPWM)
    // High side: PA8=UH, PA9=VH, PA10=WH
    // Low side:  PB13=UL, PB14=VL, PB15=WL
    // -------------------------------------------------
    palSetPadMode(GPIOA, 8,  PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOA, 9,  PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOA, 10, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOB, 13, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOB, 14, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);
    palSetPadMode(GPIOB, 15, PAL_MODE_ALTERNATE(GPIO_AF_TIM1) |
            PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

    // -------------------------------------------------
    // Hall sensor -> PC6, PC7, PC8
    // -------------------------------------------------
    palSetPadMode(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1, PAL_MODE_INPUT_PULLUP);
    palSetPadMode(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2, PAL_MODE_INPUT_PULLUP);
    palSetPadMode(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3, PAL_MODE_INPUT_PULLUP);

    // -------------------------------------------------
    // ADC pins - set ke mode analog
    //
    // PC0=CH10 (I_U), PC1=CH11 (I_V), PC2=CH12 (I_W)
    // PA0=CH0  (SENS1/back-EMF U)
    // PA1=CH1  (SENS2/back-EMF V)
    // PA2=CH2  (SENS3/back-EMF W)
    // PA3=CH3  (NTC_1/TEMP_MOS)
    // PA5=CH5  (EXT)
    // PA6=CH6  (EXT2)
    // PC3=CH13 (spare)
    // PC4=CH14 (NTC_2/TEMP_MOTOR)
    // PC5=CH15 (VOLT_INPUT/VIN_SENS)
    // -------------------------------------------------
    palSetPadMode(GPIOC, 0, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 2, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 3, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 4, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOC, 5, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 0, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 1, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 3, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 5, PAL_MODE_INPUT_ANALOG);
    palSetPadMode(GPIOA, 6, PAL_MODE_INPUT_ANALOG);

    // -------------------------------------------------
    // UART TX/RX -> PC10, PC11 (USART3)
    // -------------------------------------------------
    palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN,
            PAL_MODE_ALTERNATE(GPIO_AF_USART3) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUDR_PULLUP);
    palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN,
            PAL_MODE_ALTERNATE(GPIO_AF_USART3) |
            PAL_STM32_OSPEED_HIGHEST |
            PAL_STM32_PUDR_PULLUP);

    // -------------------------------------------------
    // CAN -> PB8 (RX), PB9 (TX)
    // -------------------------------------------------
    palSetPadMode(HW_CANRX_PORT, HW_CANRX_PIN,
            PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
            PAL_STM32_OSPEED_HIGHEST);
    palSetPadMode(HW_CANTX_PORT, HW_CANTX_PIN,
            PAL_MODE_ALTERNATE(GPIO_AF_CAN1) |
            PAL_STM32_OSPEED_HIGHEST);
}

// =========================================================
// hw_setup_adc_channels()
// Mapping ADC channel ke ADC_Value[] array
//
// Pola VESC: ADC1/ADC2/ADC3 sampling simultan
// Setiap "rank" (slot) di ketiga ADC dibaca bersamaan
// Hasil disimpan ke ADC_Value[] secara berurutan:
//   slot1: [ADC1r1, ADC2r1, ADC3r1] -> index 0,1,2
//   slot2: [ADC1r2, ADC2r2, ADC3r2] -> index 3,4,5
//   slot3: [ADC1r3, ADC2r3, ADC3r3] -> index 6,7,8
//   slot4: [ADC1r4, ADC2r4, ADC3r4] -> index 9,10,11
//   slot5: [ADC1r5, ADC2r5, ADC3r5] -> index 12,13,14
//
// Injected channel (untuk current sampling saat switching):
//   ADC1/2/3 injected -> sama dengan rank1 (current U/V/W)
// =========================================================
void hw_setup_adc_channels(void) {
    uint8_t t_samp = ADC_SampleTime_15Cycles;

    // --- SLOT 1: Current sensing (injected juga disini) ---
    // ADC_IND_CURR1=0: PC0=CH10 -> I_U
    // ADC_IND_CURR2=1: PC1=CH11 -> I_V
    // ADC_IND_CURR3=2: PC2=CH12 -> I_W
    ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);

    // --- SLOT 2: Back-EMF / SENS ---
    // ADC_IND_SENS1=3: PA0=CH0  -> SENS1 (back-EMF U)
    // ADC_IND_SENS2=4: PA1=CH1  -> SENS2 (back-EMF V)
    // ADC_IND_SENS3=5: PA2=CH2  -> SENS3 (back-EMF W)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_0,  2, t_samp);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_1,  2, t_samp);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_2,  2, t_samp);

    // --- SLOT 3: EXT / TEMP_MOS ---
    // ADC_IND_EXT=6:      PA5=CH5  -> spare/external
    // ADC_IND_EXT2=7:     PA6=CH6  -> spare/external2
    // ADC_IND_TEMP_MOS=8: PA3=CH3  -> NTC_1 (suhu MOSFET)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_5,  3, t_samp);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_6,  3, t_samp);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_3,  3, t_samp);

    // --- SLOT 4: TEMP_MOTOR / VIN ---
    // ADC_IND_TEMP_MOTOR=9:  PC4=CH14 -> NTC_2 (suhu motor)
    // ADC_IND_VIN_SENS=10:   PC5=CH15 -> VOLT_INPUT (72V sense)
    // index 11:              PC3=CH13 -> spare
    ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 4, t_samp);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_15, 4, t_samp);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_13, 4, t_samp);

    // --- SLOT 5: Vrefint dan spare ---
    // ADC_IND_VREFINT=12: Vrefint
    // index 13, 14: spare
    ADC_RegularChannelConfig(ADC1, ADC_Channel_Vrefint, 5, t_samp);
    ADC_RegularChannelConfig(ADC2, ADC_Channel_0,       5, t_samp);
    ADC_RegularChannelConfig(ADC3, ADC_Channel_1,       5, t_samp);

    // --- INJECTED CHANNELS ---
    // Dipakai untuk current sampling presisi saat center PWM
    // Harus sama dengan slot 1 regular (current U/V/W)
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 1, t_samp);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 1, t_samp);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 1, t_samp);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 2, t_samp);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 2, t_samp);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 2, t_samp);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_10, 3, t_samp);
    ADC_InjectedChannelConfig(ADC2, ADC_Channel_11, 3, t_samp);
    ADC_InjectedChannelConfig(ADC3, ADC_Channel_12, 3, t_samp);
}

// =========================================================
// hw_start_i2c()
// =========================================================
void hw_start_i2c(void) {
    i2cAcquireBus(&HW_I2C_DEV);

    if (!i2c_running) {
        palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
                PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1 |
                PAL_STM32_PUDR_PULLUP);
        palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
                PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1 |
                PAL_STM32_PUDR_PULLUP);

        i2cStart(&HW_I2C_DEV, &i2cfg);
        i2c_running = true;
    }

    i2cReleaseBus(&HW_I2C_DEV);
}

// =========================================================
// hw_stop_i2c()
// =========================================================
void hw_stop_i2c(void) {
    i2cAcquireBus(&HW_I2C_DEV);

    if (i2c_running) {
        palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN, PAL_MODE_INPUT);
        palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN, PAL_MODE_INPUT);

        i2cStop(&HW_I2C_DEV);
        i2c_running = false;
    }

    i2cReleaseBus(&HW_I2C_DEV);
}

// =========================================================
// hw_try_restore_i2c()
// Dipanggil VESC saat I2C bus hang
// =========================================================
void hw_try_restore_i2c(void) {
    if (i2c_running) {
        i2cAcquireBus(&HW_I2C_DEV);

        palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1 |
                PAL_STM32_PUDR_PULLUP);
        palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1 |
                PAL_STM32_PUDR_PULLUP);

        palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
        palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
        chThdSleep(1);

        for (int i = 0; i < 16; i++) {
            palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
            chThdSleep(1);
            palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
            chThdSleep(1);
        }

        palClearPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);
        chThdSleep(1);
        palClearPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
        chThdSleep(1);
        palSetPad(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN);
        chThdSleep(1);
        palSetPad(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN);

        palSetPadMode(HW_I2C_SCL_PORT, HW_I2C_SCL_PIN,
                PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1 |
                PAL_STM32_PUDR_PULLUP);
        palSetPadMode(HW_I2C_SDA_PORT, HW_I2C_SDA_PIN,
                PAL_MODE_ALTERNATE(HW_I2C_GPIO_AF) |
                PAL_STM32_OTYPE_OPENDRAIN |
                PAL_STM32_OSPEED_MID1 |
                PAL_STM32_PUDR_PULLUP);

        HW_I2C_DEV.state = I2C_STOP;
        i2cStart(&HW_I2C_DEV, &i2cfg);

        i2cReleaseBus(&HW_I2C_DEV);
    }
}
