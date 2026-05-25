/*
 * hw_kai.h
 * Hardware configuration untuk KAI INVERTER FUEL PUMP
 * Project  : 2509-KAI-PIFP-A
 * MCU      : STM32F405RGT6
 * Driver   : IR2110 (bukan DRV8301)
 * Tegangan : 72V nominal
 * Shunt    : 0.018 Ohm x3 (low-side per phase, R15/R16/R17)
 */

#ifndef HW_KAI_H_
#define HW_KAI_H_

#define HW_NAME                 "KAI_72V"

// =========================================================
// HW PROPERTIES
// Tidak pakai DRV apapun - IR2110 tidak perlu SPI config
// =========================================================
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS

// =========================================================
// GATE ENABLE
// EN_GATE -> PB5 (sama dengan VESC 6, kebetulan cocok)
// IR2110 SD pin: HIGH = enable, LOW = shutdown
// =========================================================
#define ENABLE_GATE()           palSetPad(GPIOB, 5)
#define DISABLE_GATE()          palClearPad(GPIOB, 5)

// Tidak ada DRV fault pin hardware - return selalu 0 (no fault)
#define IS_DRV_FAULT()          0

// DCCAL tidak ada di IR2110 - kosongkan
#define DCCAL_ON()
#define DCCAL_OFF()

// =========================================================
// LED
// Dari skematik COM sheet: LED_RED dan LED_GREEN
// F405: LED_RED -> PB1, LED_GREEN -> PB0
// (sama dengan VESC 6 referensi, JANGAN pakai PA13/PA14 = SWD!)
// =========================================================
#define LED_GREEN_GPIO          GPIOB
#define LED_GREEN_PIN           0
#define LED_RED_GPIO            GPIOB
#define LED_RED_PIN             1
#define LED_GREEN_ON()          palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()         palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()            palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()           palClearPad(LED_RED_GPIO, LED_RED_PIN)

// =========================================================
// ADC VECTOR MAPPING
//
// Aturan utama VESC:
//   ADC_Value[n] = hasil sampling slot ke-n secara berurutan
//   ADC1/ADC2/ADC3 sampling SIMULTAN per slot
//   Slot 0 = hasil ADC1 rank1, ADC2 rank1, ADC3 rank1
//   dst.
//
// Layout (5 slot x 3 ADC = 15 channel):
//
//  Idx | ADC1 ch    | ADC2 ch    | ADC3 ch    | Signal
//  ----|------------|------------|------------|------------------
//   0  | CH10=PC0   | CH11=PC1   | CH12=PC2   | CURR1/CURR2/CURR3
//   1  | CH10=PC0   | CH11=PC1   | CH12=PC2   | (injected, same)
//   2  | CH10=PC0   | CH11=PC1   | CH12=PC2   | (injected, same)
//   3  | CH0 =PA0   | CH1 =PA1   | CH2 =PA2   | SENS1/SENS2/SENS3
//   4  | CH5 =PA5   | CH6 =PA6   | CH3 =PA3   | EXT/EXT2/TEMP_MOS
//   5  | CH14=PC4   | CH15=PC5   | CH13=PC3   | TEMP_MOTOR/x/AN_IN
//   6  | VREFINT    | CH0 =PA0   | CH1 =PA1   | VREFINT/-/-
//
// Mapping index ke ADC_Value[]:
//   ADC_IND_CURR1 = 0  -> ADC1 slot1 -> PC0 = I_U (output MCP6002 fase U)
//   ADC_IND_CURR2 = 1  -> ADC2 slot1 -> PC1 = I_V (output MCP6002 fase V)
//   ADC_IND_CURR3 = 2  -> ADC3 slot1 -> PC2 = I_W (output MCP6002 fase W)
//   ADC_IND_SENS1 = 3  -> ADC1 slot2 -> PA0 = SENS1 (back-EMF U)
//   ADC_IND_SENS2 = 4  -> ADC2 slot2 -> PA1 = SENS2 (back-EMF V)
//   ADC_IND_SENS3 = 5  -> ADC3 slot2 -> PA2 = SENS3 (back-EMF W)
//   ADC_IND_EXT   = 6  -> ADC1 slot3 -> PA5 = spare/EXT
//   ADC_IND_EXT2  = 7  -> ADC2 slot3 -> PA6 = spare/EXT2
//   ADC_IND_TEMP_MOS= 8-> ADC3 slot3 -> PA3 = NTC_1 (temp MOSFET)
//   ADC_IND_TEMP_MOTOR=9-> ADC1 slot4 -> PC4 = NTC_2 (temp motor)
//   ADC_IND_VIN_SENS=10 -> ADC2 slot4 -> PC5 = VOLT_INPUT (72V sense)
//   ADC_IND_VREFINT=12  -> ADC1 slot5 -> Vrefint
//
// VERIFIKASI: cek pin sambungan I_U/I_V/I_W dari skematik
// MCU F405 ke pin ADC. Sesuaikan jika berbeda.
// =========================================================

#define HW_ADC_CHANNELS         15
#define HW_ADC_INJ_CHANNELS     3
#define HW_ADC_NBR_CONV         5

// ADC indexes
#define ADC_IND_CURR1           0
#define ADC_IND_CURR2           1
#define ADC_IND_CURR3           2
#define ADC_IND_SENS1           3
#define ADC_IND_SENS2           4
#define ADC_IND_SENS3           5
#define ADC_IND_EXT             6
#define ADC_IND_EXT2            7
#define ADC_IND_TEMP_MOS        8
#define ADC_IND_TEMP_MOTOR      9
#define ADC_IND_VIN_SENS        10
#define ADC_IND_VREFINT         12

// =========================================================
// TEGANGAN INPUT
// Voltage divider: R84+R85 = 320k (atas), R86 = 10k (bawah)
// Rasio = (320k + 10k) / 10k = 33
// =========================================================
#define VIN_R1                  320000.0
#define VIN_R2                  10000.0
#define V_REG                   3.3
#define GET_INPUT_VOLTAGE()     ((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] \
                                * ((VIN_R1 + VIN_R2) / VIN_R2))

#define HW_LIM_VIN_MIN          10.0
#define HW_LIM_VIN_MAX          90.0
#define MCCONF_L_MIN_VOLTAGE    10.0
#define MCCONF_L_MAX_VOLTAGE    85.0

// =========================================================
// CURRENT SENSING
// Shunt: 0.018 Ohm (R15/R16/R17 dari skematik MOSFET)
// Op-amp: MCP6002
// Gain MCP6002 dari skematik: R52=1K, R54=1K -> gain = 1 + R54/R52 = 2
// Tapi ada juga R-feedback lain, UKUR gain aktual dengan multimeter!
// Sementara set 10.0 dulu, sesuaikan setelah pengujian.
// =========================================================
#define CURRENT_SHUNT_RES       0.018
#define CURRENT_AMP_GAIN        10.0

#define MCCONF_L_MAX_ABS_CURRENT    150.0
#define MCCONF_L_CURRENT_MAX        50.0
#define MCCONF_L_CURRENT_MIN        -50.0
#define MCCONF_L_IN_CURRENT_MAX     50.0
#define MCCONF_L_IN_CURRENT_MIN     -10.0

// =========================================================
// ADC MACROS
// =========================================================
#define ADC_VOLTS(ch)           ((float)ADC_Value[ch] / 4096.0 * V_REG)

// NTC Thermistor - pull-up 10k ke 3.3V, Beta=3380
// Dari skematik SENSOR: R89=10k dan R90=10k pull-up ke 3V3
#define NTC_RES(adc_val)        ((4095.0 * 10000.0) / adc_val - 10000.0)
#define NTC_TEMP(adc_ind)       (1.0 / ((logf(NTC_RES(ADC_Value[adc_ind]) / 10000.0) \
                                / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_RES_MOTOR(adc_val)  (10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP_MOTOR(beta)    (1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) \
                                / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

#define HW_LIM_TEMP_FET_START       80.0
#define HW_LIM_TEMP_FET_END         100.0
#define HW_LIM_TEMP_MOTOR_START     80.0
#define HW_LIM_TEMP_MOTOR_END       100.0
#define MCCONF_L_TEMP_FET_START     80.0
#define MCCONF_L_TEMP_FET_END       100.0

// =========================================================
// PWM & FOC
// =========================================================
#define HW_DEAD_TIME_NSEC       2500.0
#define MCCONF_FOC_F_ZV         20000.0
#define MCCONF_DEFAULT_MOTOR_TYPE   MOTOR_TYPE_FOC

// =========================================================
// HALL SENSOR
// HAL_U->PC6, HAL_V->PC7, HAL_W->PC8
// Sesuai skematik SENSOR -> Filter HAL -> F405 MCU
// =========================================================
#define HW_HALL_ENC_GPIO1       GPIOC
#define HW_HALL_ENC_PIN1        6
#define HW_HALL_ENC_GPIO2       GPIOC
#define HW_HALL_ENC_PIN2        7
#define HW_HALL_ENC_GPIO3       GPIOC
#define HW_HALL_ENC_PIN3        8
#define READ_HALL1()            palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()            palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()            palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// =========================================================
// ENCODER / EXTI (wajib ada meski tidak pakai encoder)
// Hall sensor pakai PC8 -> EXTI line 8
// =========================================================
#define HW_ENC_TIM              TIM4
#define HW_ENC_TIM_AF           GPIO_AF_TIM4
#define HW_ENC_TIM_CLK_EN()     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ENC_TIM_ISR_VEC      TIM4_IRQHandler
#define HW_ENC_TIM_ISR_CH       TIM4_IRQn
#define HW_ENC_EXTI_LINE        EXTI_Line8
#define HW_ENC_EXTI_CH          EXTI9_5_IRQn
#define HW_ENC_EXTI_ISR_VEC     EXTI9_5_IRQHandler
#define HW_ENC_EXTI_PORTSRC     EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC      EXTI_PinSource8

// =========================================================
// ICU - servo input (wajib ada, tidak dipakai)
// =========================================================
#define HW_ICU_TIMER            TIM9
#define HW_ICU_TIM_CLK_EN()     RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE)
#define HW_ICU_DEV              ICUD9
#define HW_ICU_CHANNEL          ICU_CHANNEL_1
#define HW_ICU_GPIO_AF          GPIO_AF_TIM9
#define HW_ICU_GPIO             GPIOA
#define HW_ICU_PIN              2

// =========================================================
// I2C - SCL=PB10, SDA=PB11
// =========================================================
#define HW_I2C_DEV              I2CD2
#define HW_I2C_GPIO_AF          GPIO_AF_I2C2
#define HW_I2C_SCL_PORT         GPIOB
#define HW_I2C_SCL_PIN          10
#define HW_I2C_SDA_PORT         GPIOB
#define HW_I2C_SDA_PIN          11

// =========================================================
// UART - USART3: TX=PC10, RX=PC11
// =========================================================
#define HW_UART_DEV             SD3
#define HW_UART_GPIO_AF         GPIO_AF_USART3
#define HW_UART_TX_PORT         GPIOC
#define HW_UART_TX_PIN          10
#define HW_UART_RX_PORT         GPIOC
#define HW_UART_RX_PIN          11
#define HW_UART_P_BAUD          115200

// =========================================================
// CAN - CAN1: TX=PB9, RX=PB8
// =========================================================
#define HW_CAN_DEV              CAND1
#define HW_CAN_AF               GPIO_AF_CAN1
#define HW_CAN_GPIO_AF          GPIO_AF_CAN1
#define HW_CANTX_PORT           GPIOB
#define HW_CANTX_PIN            9
#define HW_CANRX_PORT           GPIOB
#define HW_CANRX_PIN            8

// =========================================================
// SPI (dummy - tidak dipakai, tapi wajib ada agar compile)
// =========================================================
#define HW_SPI_DEV              SPID1
#define HW_SPI_GPIO_AF          GPIO_AF_SPI1
#define HW_SPI_PORT_NSS         GPIOA
#define HW_SPI_PIN_NSS          4
#define HW_SPI_PORT_SCK         GPIOA
#define HW_SPI_PIN_SCK          5
#define HW_SPI_PORT_MOSI        GPIOA
#define HW_SPI_PIN_MOSI         7
#define HW_SPI_PORT_MISO        GPIOA
#define HW_SPI_PIN_MISO         6

// Function declarations
void hw_init_gpio(void);
void hw_setup_adc_channels(void);
void hw_start_i2c(void);
void hw_stop_i2c(void);
void hw_try_restore_i2c(void);

#endif /* HW_KAI_H_ */
