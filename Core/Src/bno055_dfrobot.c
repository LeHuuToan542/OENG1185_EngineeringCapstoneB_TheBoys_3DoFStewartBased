/*
 * bno055_dfrobot.c
 *
 * STM32 HAL / pure-C port of the BNO055 behavior used by the supplied
 * DFRobot_BNO055 Arduino library.
 *
 * CubeMX owns:
 *   - system clock
 *   - GPIO alternate functions
 *   - I2C initialization
 *   - UART initialization
 *
 * This file only communicates with the BNO055 through HAL.
 */

#include "bno055_dfrobot.h"

#include <stddef.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* BNO055 register map - page 0                                                */
/* -------------------------------------------------------------------------- */

#define BNO055_REG_CHIP_ID            0x00U
#define BNO055_CHIP_ID_VALUE          0xA0U

#define BNO055_REG_PAGE_ID            0x07U

#define BNO055_REG_EUL_HEADING_LSB    0x1AU
#define BNO055_REG_QUA_W_LSB          0x20U

#define BNO055_REG_CALIB_STAT         0x35U
#define BNO055_REG_SYS_STATUS         0x39U
#define BNO055_REG_UNIT_SEL           0x3BU
#define BNO055_REG_OPR_MODE           0x3DU
#define BNO055_REG_PWR_MODE           0x3EU
#define BNO055_REG_SYS_TRIGGER        0x3FU
#define BNO055_REG_AXIS_MAP_CONFIG    0x41U

/* -------------------------------------------------------------------------- */
/* BNO055 register map - page 1                                                */
/* -------------------------------------------------------------------------- */

#define BNO055_REG_ACC_CONFIG         0x08U
#define BNO055_REG_GYR_CONFIG_0       0x0AU

/* -------------------------------------------------------------------------- */
/* Values matching the supplied DFRobot library                                */
/* -------------------------------------------------------------------------- */

#define BNO055_MODE_CONFIG            0x00U
#define BNO055_MODE_NDOF              0x0CU

#define BNO055_POWER_NORMAL           0x00U

/* DFRobot eMapConfig_P1 */
#define BNO055_AXIS_MAP_P1            0x24U

/*
 * DFRobot setUnit():
 *   ACC = mg             bit 0 = 1
 *   GYR = dps            bit 1 = 0
 *   EUL = degrees        bit 2 = 0
 *   TEMP = Celsius       bit 4 = 0
 *   Android orientation bit 7 = 1
 */
#define BNO055_UNIT_DFROBOT           0x81U

#define BNO055_ACC_RANGE_MASK         0x03U
#define BNO055_ACC_RANGE_4G           0x01U

#define BNO055_GYR_RANGE_MASK         0x07U
#define BNO055_GYR_RANGE_2000DPS      0x00U

#define BNO055_SYS_TRIGGER_RST_SYS    0x20U

#define BNO055_I2C_TIMEOUT_MS         100U

/* Startup-zero filtering */
#define BNO055_ZERO_SAMPLE_COUNT      32U
#define BNO055_ZERO_SAMPLE_DELAY_MS   20U
#define BNO055_GYRO_CAL_POLL_MS       200U

/* -------------------------------------------------------------------------- */
/* Internal UART debug helpers                                                 */
/* -------------------------------------------------------------------------- */

static void debug_putc(BNO055_t *dev, char c)
{
    if ((dev == NULL) || (dev->huart == NULL))
    {
        return;
    }

    (void)HAL_UART_Transmit(dev->huart,
                           (uint8_t *)&c,
                           1U,
                           HAL_MAX_DELAY);
}

static void debug_puts(BNO055_t *dev, const char *text)
{
    if ((dev == NULL) || (dev->huart == NULL) || (text == NULL))
    {
        return;
    }

    (void)HAL_UART_Transmit(dev->huart,
                           (uint8_t *)text,
                           (uint16_t)strlen(text),
                           HAL_MAX_DELAY);
}

static void debug_newline(BNO055_t *dev)
{
    debug_puts(dev, "\r\n");
}

static void debug_print_uint8(BNO055_t *dev, uint8_t value)
{
    char text[4];
    uint8_t n = 0U;

    if (value >= 100U)
    {
        text[n++] = (char)('0' + (value / 100U));
        value %= 100U;
        text[n++] = (char)('0' + (value / 10U));
        text[n++] = (char)('0' + (value % 10U));
    }
    else if (value >= 10U)
    {
        text[n++] = (char)('0' + (value / 10U));
        text[n++] = (char)('0' + (value % 10U));
    }
    else
    {
        text[n++] = (char)('0' + value);
    }

    text[n] = '\0';
    debug_puts(dev, text);
}

/*
 * Print a floating-point number as exactly 8 ASCII characters with 2 decimal
 * places.  Leading spaces are inserted when required.
 *
 * Examples:
 *      0.00  -> "    0.00"
 *     -0.56  -> "   -0.56"
 *     12.34  -> "   12.34"
 *   -180.00  -> " -180.00"
 *
 * This is used by BNO055_PrintEuler() so the Simulink payload is always:
 *
 *     8 + 1 TAB + 8 + 1 TAB + 8 = 26 bytes
 */
static void debug_print_float_fixed_8_2(BNO055_t *dev, float value)
{
    char digits[8];
    uint8_t digit_count = 0U;
    uint8_t text_length;
    uint8_t pad;
    uint32_t magnitude;
    uint32_t whole;
    uint32_t frac;
    int32_t scaled;

    if (value >= 0.0f)
    {
        scaled = (int32_t)(value * 100.0f + 0.5f);
    }
    else
    {
        scaled = (int32_t)(value * 100.0f - 0.5f);
    }

    magnitude = (scaled < 0)
              ? (uint32_t)(-(int64_t)scaled)
              : (uint32_t)scaled;

    whole = magnitude / 100U;
    frac = magnitude % 100U;

    /*
     * Build the integer part backwards.
     */
    if (whole == 0U)
    {
        digits[digit_count++] = '0';
    }
    else
    {
        while ((whole > 0U) && (digit_count < sizeof(digits)))
        {
            digits[digit_count++] = (char)('0' + (whole % 10U));
            whole /= 10U;
        }
    }

    /*
     * Number of visible characters before left padding:
     * optional '-' + integer digits + '.' + 2 decimals.
     */
    text_length = digit_count + 3U;

    if (scaled < 0)
    {
        text_length++;
    }

    /*
     * Every field must be exactly 8 bytes.
     * The BNO055 relative Euler ranges fit comfortably inside this width.
     */
    pad = (text_length < 8U) ? (uint8_t)(8U - text_length) : 0U;

    while (pad-- > 0U)
    {
        debug_putc(dev, ' ');
    }

    if (scaled < 0)
    {
        debug_putc(dev, '-');
    }

    while (digit_count > 0U)
    {
        debug_putc(dev, digits[--digit_count]);
    }

    debug_putc(dev, '.');
    debug_putc(dev, (char)('0' + ((frac / 10U) % 10U)));
    debug_putc(dev, (char)('0' + (frac % 10U)));
}

/* -------------------------------------------------------------------------- */
/* HAL I2C helpers                                                             */
/* -------------------------------------------------------------------------- */

static BNO055_Status_t read_regs(BNO055_t *dev,
                                 uint8_t reg,
                                 uint8_t *data,
                                 uint16_t len)
{
    HAL_StatusTypeDef result;

    if ((dev == NULL) ||
        (dev->hi2c == NULL) ||
        (data == NULL) ||
        (len == 0U))
    {
        if (dev != NULL)
        {
            dev->last_status = BNO055_STATUS_PARAMETER_ERROR;
        }

        return BNO055_STATUS_PARAMETER_ERROR;
    }

    result = HAL_I2C_Mem_Read(dev->hi2c,
                              dev->address_hal,
                              reg,
                              I2C_MEMADD_SIZE_8BIT,
                              data,
                              len,
                              BNO055_I2C_TIMEOUT_MS);

    if (result != HAL_OK)
    {
        dev->last_status = BNO055_STATUS_DEVICE_NOT_DETECTED;
        return dev->last_status;
    }

    dev->last_status = BNO055_STATUS_OK;
    return dev->last_status;
}

static BNO055_Status_t write_regs(BNO055_t *dev,
                                  uint8_t reg,
                                  const uint8_t *data,
                                  uint16_t len)
{
    HAL_StatusTypeDef result;

    if ((dev == NULL) ||
        (dev->hi2c == NULL) ||
        (data == NULL) ||
        (len == 0U))
    {
        if (dev != NULL)
        {
            dev->last_status = BNO055_STATUS_PARAMETER_ERROR;
        }

        return BNO055_STATUS_PARAMETER_ERROR;
    }

    result = HAL_I2C_Mem_Write(dev->hi2c,
                               dev->address_hal,
                               reg,
                               I2C_MEMADD_SIZE_8BIT,
                               (uint8_t *)data,
                               len,
                               BNO055_I2C_TIMEOUT_MS);

    if (result != HAL_OK)
    {
        dev->last_status = BNO055_STATUS_DEVICE_NOT_DETECTED;
        return dev->last_status;
    }

    dev->last_status = BNO055_STATUS_OK;
    return dev->last_status;
}

static BNO055_Status_t write_reg(BNO055_t *dev,
                                 uint8_t reg,
                                 uint8_t value)
{
    return write_regs(dev, reg, &value, 1U);
}

static BNO055_Status_t read_reg(BNO055_t *dev,
                                uint8_t reg,
                                uint8_t *value)
{
    return read_regs(dev, reg, value, 1U);
}

/* -------------------------------------------------------------------------- */
/* Page handling                                                               */
/* -------------------------------------------------------------------------- */

static BNO055_Status_t set_page(BNO055_t *dev, uint8_t page)
{
    BNO055_Status_t status;

    if ((dev == NULL) || (page > 1U))
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    if (dev->current_page == page)
    {
        return BNO055_STATUS_OK;
    }

    status = write_reg(dev, BNO055_REG_PAGE_ID, page);

    if (status == BNO055_STATUS_OK)
    {
        dev->current_page = page;
    }

    return status;
}

/* -------------------------------------------------------------------------- */
/* Register helpers                                                            */
/* -------------------------------------------------------------------------- */

static BNO055_Status_t update_bits(BNO055_t *dev,
                                   uint8_t reg,
                                   uint8_t mask,
                                   uint8_t value)
{
    uint8_t current = 0U;
    BNO055_Status_t status;

    status = read_reg(dev, reg, &current);

    if (status != BNO055_STATUS_OK)
    {
        return status;
    }

    current &= (uint8_t)~mask;
    current |= (uint8_t)(value & mask);

    return write_reg(dev, reg, current);
}

static int16_t little_endian_int16(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[1] << 8U) |
                     (uint16_t)data[0]);
}

/*
 * Return the shortest signed angular difference.
 * Example: current=1 deg, zero=359 deg -> +2 deg, not -358 deg.
 */
static float wrap_angle_180(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }

    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}


/* -------------------------------------------------------------------------- */
/* Calibration wait                                                           */
/* -------------------------------------------------------------------------- */

static BNO055_Status_t wait_for_gyro_calibration(BNO055_t *dev)
{
    BNO055_Calibration_t cal;
    BNO055_Status_t status;
    uint8_t last_gyro = 0xFFU;

    if (dev == NULL)
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    debug_puts(dev, "Keep the IMU stationary while gyro calibrates...");
    debug_newline(dev);

    for (;;)
    {
        status = BNO055_ReadCalibration(dev, &cal);

        if (status != BNO055_STATUS_OK)
        {
            return status;
        }

        /*
         * Print only when the gyro calibration level changes so PuTTY
         * remains readable.
         */
        if (cal.gyro != last_gyro)
        {
            debug_puts(dev, "Gyro calibration: ");
            debug_print_uint8(dev, cal.gyro);
            debug_puts(dev, "/3");
            debug_newline(dev);

            last_gyro = cal.gyro;
        }

        if (cal.gyro >= 3U)
        {
            debug_puts(dev, "Gyro calibration complete");
            debug_newline(dev);
            return BNO055_STATUS_OK;
        }

        HAL_Delay(BNO055_GYRO_CAL_POLL_MS);
    }
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                  */
/* -------------------------------------------------------------------------- */

void BNO055_Attach(BNO055_t *dev,
                   I2C_HandleTypeDef *hi2c,
                   UART_HandleTypeDef *huart,
                   uint8_t address_7bit)
{
    if (dev == NULL)
    {
        return;
    }

    memset(dev, 0, sizeof(*dev));

    dev->hi2c = hi2c;
    dev->huart = huart;
    dev->address_hal = (uint16_t)((uint16_t)address_7bit << 1U);
    dev->current_page = 0xFFU;

    dev->zero_heading = 0.0f;
    dev->zero_roll = 0.0f;
    dev->zero_pitch = 0.0f;
    dev->zero_valid = false;

    dev->last_status = BNO055_STATUS_OK;
}

bool BNO055_TestConnection(BNO055_t *dev)
{
    uint8_t chip_id = 0U;

    if (dev == NULL)
    {
        return false;
    }

    dev->current_page = 0xFFU;

    if (set_page(dev, 0U) != BNO055_STATUS_OK)
    {
        return false;
    }

    if (read_reg(dev,
                 BNO055_REG_CHIP_ID,
                 &chip_id) != BNO055_STATUS_OK)
    {
        return false;
    }

    if (chip_id != BNO055_CHIP_ID_VALUE)
    {
        dev->last_status = BNO055_STATUS_DEVICE_NOT_DETECTED;
        return false;
    }

    dev->last_status = BNO055_STATUS_OK;
    return true;
}

BNO055_Status_t BNO055_Reset(BNO055_t *dev)
{
    BNO055_Status_t status;

    if (dev == NULL)
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    dev->current_page = 0xFFU;

    status = set_page(dev, 0U);

    if (status != BNO055_STATUS_OK)
    {
        return status;
    }

    status = write_reg(dev,
                       BNO055_REG_SYS_TRIGGER,
                       BNO055_SYS_TRIGGER_RST_SYS);

    if (status != BNO055_STATUS_OK)
    {
        return status;
    }

    /*
     * Same delay used by DFRobot reset().
     * The device may temporarily stop responding during reset.
     */
    HAL_Delay(700U);

    dev->current_page = 0xFFU;
    dev->last_status = BNO055_STATUS_OK;

    return dev->last_status;
}

BNO055_Status_t BNO055_Begin(BNO055_t *dev)
{
    uint8_t system_status = 0xFFU;
    uint8_t timeout = 0U;
    BNO055_Status_t status;

    if ((dev == NULL) || (dev->hi2c == NULL))
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    debug_puts(dev, "Testing BNO055 connection...");
    debug_newline(dev);

    if (!BNO055_TestConnection(dev))
    {
        debug_puts(dev, "BNO055 device not detected");
        debug_newline(dev);
        return dev->last_status;
    }

    debug_puts(dev, "BNO055 CHIP_ID OK");
    debug_newline(dev);

    /*
     * The supplied DFRobot begin() resets the device after checking CHIP_ID.
     */
    status = BNO055_Reset(dev);

    if (status != BNO055_STATUS_OK)
    {
        return status;
    }

    /*
     * Match the supplied library:
     * poll SYS_STATUS every 10 ms until it becomes 0,
     * with a 100-iteration timeout.
     */
    do
    {
        if (set_page(dev, 0U) != BNO055_STATUS_OK)
        {
            return dev->last_status;
        }

        if (read_reg(dev,
                     BNO055_REG_SYS_STATUS,
                     &system_status) != BNO055_STATUS_OK)
        {
            return dev->last_status;
        }

        HAL_Delay(10U);
        timeout++;
    }
    while ((system_status != 0U) && (timeout < 100U));

    if (timeout == 100U)
    {
        dev->last_status = BNO055_STATUS_READY_TIMEOUT;
        return dev->last_status;
    }

    HAL_Delay(100U);

    /* CONFIG mode */
    if (write_reg(dev,
                  BNO055_REG_OPR_MODE,
                  BNO055_MODE_CONFIG) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    HAL_Delay(50U);

    /* DFRobot axis map P1 */
    if (write_reg(dev,
                  BNO055_REG_AXIS_MAP_CONFIG,
                  BNO055_AXIS_MAP_P1) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    HAL_Delay(10U);

    /* DFRobot units: mg, degrees, dps, Celsius, Android orientation */
    if (write_reg(dev,
                  BNO055_REG_UNIT_SEL,
                  BNO055_UNIT_DFROBOT) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    /* Page 1: accelerometer +/-4 g */
    if (set_page(dev, 1U) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    if (update_bits(dev,
                    BNO055_REG_ACC_CONFIG,
                    BNO055_ACC_RANGE_MASK,
                    BNO055_ACC_RANGE_4G) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    /* Page 1: gyroscope +/-2000 dps */
    if (update_bits(dev,
                    BNO055_REG_GYR_CONFIG_0,
                    BNO055_GYR_RANGE_MASK,
                    BNO055_GYR_RANGE_2000DPS) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    /* Return to page 0 */
    if (set_page(dev, 0U) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    /* Normal power mode */
    if (write_reg(dev,
                  BNO055_REG_PWR_MODE,
                  BNO055_POWER_NORMAL) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    HAL_Delay(10U);

    /* NDOF sensor-fusion mode */
    if (write_reg(dev,
                  BNO055_REG_OPR_MODE,
                  BNO055_MODE_NDOF) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    HAL_Delay(50U);

    /*
     * The BNO055 performs its own internal sensor calibration.  Do not
     * choose the relative startup zero until the gyroscope reports level 3.
     */
    status = wait_for_gyro_calibration(dev);

    if (status != BNO055_STATUS_OK)
    {
        dev->last_status = status;
        return status;
    }

    debug_puts(dev, "Averaging 32 startup orientation samples...");
    debug_newline(dev);

    status = BNO055_SetCurrentOrientationAsZero(dev);

    if (status != BNO055_STATUS_OK)
    {
        return status;
    }

    dev->last_status = BNO055_STATUS_OK;

    debug_puts(dev, "BNO055 begin success");
    debug_newline(dev);
    debug_puts(dev, "Averaged startup orientation set to yaw=0 pitch=0 roll=0");
    debug_newline(dev);

    return dev->last_status;
}

BNO055_Status_t BNO055_ReadEulerAbsolute(BNO055_t *dev,
                                         BNO055_Euler_t *euler)
{
    uint8_t raw[6];
    int16_t heading_raw;
    int16_t roll_raw;
    int16_t pitch_raw;

    if ((dev == NULL) || (euler == NULL))
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    if (set_page(dev, 0U) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    if (read_regs(dev,
                  BNO055_REG_EUL_HEADING_LSB,
                  raw,
                  sizeof(raw)) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    heading_raw = little_endian_int16(&raw[0]);
    roll_raw = little_endian_int16(&raw[2]);
    pitch_raw = little_endian_int16(&raw[4]);

    /*
     * Same conversion as DFRobot getEul():
     * 16 LSB = 1 degree.
     */
    euler->heading = (float)heading_raw / 16.0f;
    euler->roll = (float)roll_raw / 16.0f;
    euler->pitch = (float)pitch_raw / 16.0f;

    dev->last_status = BNO055_STATUS_OK;
    return dev->last_status;
}

BNO055_Status_t BNO055_SetCurrentOrientationAsZero(BNO055_t *dev)
{
    BNO055_Euler_t sample;
    BNO055_Status_t status;

    float heading_reference = 0.0f;
    float roll_reference = 0.0f;

    float heading_sum = 0.0f;
    float roll_sum = 0.0f;
    float pitch_sum = 0.0f;

    uint32_t valid_samples = 0U;
    uint32_t i;

    if (dev == NULL)
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    /*
     * Average 32 fused Euler readings instead of using one instantaneous
     * reading as zero.
     *
     * Heading and roll are unwrapped relative to the first sample before
     * averaging. This prevents values around 359/0 degrees from averaging
     * incorrectly to about 180 degrees.
     */
    for (i = 0U; i < BNO055_ZERO_SAMPLE_COUNT; i++)
    {
        status = BNO055_ReadEulerAbsolute(dev, &sample);

        if (status != BNO055_STATUS_OK)
        {
            dev->zero_valid = false;
            return status;
        }

        if (valid_samples == 0U)
        {
            heading_reference = sample.heading;
            roll_reference = sample.roll;
        }

        heading_sum +=
            heading_reference +
            wrap_angle_180(sample.heading - heading_reference);

        roll_sum +=
            roll_reference +
            wrap_angle_180(sample.roll - roll_reference);

        pitch_sum += sample.pitch;
        valid_samples++;

        if ((i + 1U) < BNO055_ZERO_SAMPLE_COUNT)
        {
            HAL_Delay(BNO055_ZERO_SAMPLE_DELAY_MS);
        }
    }

    if (valid_samples == 0U)
    {
        dev->zero_valid = false;
        dev->last_status = BNO055_STATUS_ERROR;
        return dev->last_status;
    }

    dev->zero_heading =
        wrap_angle_180(heading_sum / (float)valid_samples);

    dev->zero_roll =
        wrap_angle_180(roll_sum / (float)valid_samples);

    dev->zero_pitch =
        pitch_sum / (float)valid_samples;

    dev->zero_valid = true;
    dev->last_status = BNO055_STATUS_OK;

    return dev->last_status;
}

BNO055_Status_t BNO055_ReadEuler(BNO055_t *dev,
                                 BNO055_Euler_t *euler)
{
    BNO055_Euler_t absolute;
    BNO055_Status_t status;

    if ((dev == NULL) || (euler == NULL))
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    status = BNO055_ReadEulerAbsolute(dev, &absolute);

    if (status != BNO055_STATUS_OK)
    {
        return status;
    }

    if (!dev->zero_valid)
    {
        /* No zero has been captured yet: return absolute orientation. */
        *euler = absolute;
        return BNO055_STATUS_OK;
    }

    /*
     * Euler-offset zeroing.
     * Heading and roll can cross +/-180 or 0/360 boundaries, so wrap them.
     * Pitch is subtracted directly.
     */
    euler->heading = wrap_angle_180(absolute.heading - dev->zero_heading);
    euler->roll = wrap_angle_180(absolute.roll - dev->zero_roll);
    euler->pitch = absolute.pitch - dev->zero_pitch;

    dev->last_status = BNO055_STATUS_OK;
    return dev->last_status;
}

BNO055_Status_t BNO055_ReadQuaternion(BNO055_t *dev,
                                      BNO055_Quaternion_t *q)
{
    uint8_t raw[8];

    if ((dev == NULL) || (q == NULL))
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    if (set_page(dev, 0U) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    if (read_regs(dev,
                  BNO055_REG_QUA_W_LSB,
                  raw,
                  sizeof(raw)) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    q->w = (float)little_endian_int16(&raw[0]) / 16384.0f;
    q->x = (float)little_endian_int16(&raw[2]) / 16384.0f;
    q->y = (float)little_endian_int16(&raw[4]) / 16384.0f;
    q->z = (float)little_endian_int16(&raw[6]) / 16384.0f;

    dev->last_status = BNO055_STATUS_OK;
    return dev->last_status;
}

BNO055_Status_t BNO055_ReadCalibration(BNO055_t *dev,
                                       BNO055_Calibration_t *cal)
{
    uint8_t value = 0U;

    if ((dev == NULL) || (cal == NULL))
    {
        return BNO055_STATUS_PARAMETER_ERROR;
    }

    if (set_page(dev, 0U) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    if (read_reg(dev,
                 BNO055_REG_CALIB_STAT,
                 &value) != BNO055_STATUS_OK)
    {
        return dev->last_status;
    }

    cal->mag = value & 0x03U;
    cal->accel = (value >> 2U) & 0x03U;
    cal->gyro = (value >> 4U) & 0x03U;
    cal->system = (value >> 6U) & 0x03U;

    dev->last_status = BNO055_STATUS_OK;
    return dev->last_status;
}

const char *BNO055_StatusString(BNO055_Status_t status)
{
    switch (status)
    {
        case BNO055_STATUS_OK:
            return "everything ok";

        case BNO055_STATUS_ERROR:
            return "unknown error";

        case BNO055_STATUS_DEVICE_NOT_DETECTED:
            return "device not detected";

        case BNO055_STATUS_READY_TIMEOUT:
            return "device ready time out";

        case BNO055_STATUS_DEVICE_STATUS_ERROR:
            return "device internal status error";

        case BNO055_STATUS_PARAMETER_ERROR:
            return "parameter error";

        default:
            return "unknown status";
    }
}

void BNO055_PrintStatus(BNO055_t *dev,
                        BNO055_Status_t status)
{
    if ((dev == NULL) || (dev->huart == NULL))
    {
        return;
    }

    debug_puts(dev, BNO055_StatusString(status));
    debug_newline(dev);
}

void BNO055_PrintEuler(BNO055_t *dev, const BNO055_Euler_t *euler)
                        {
  if ((dev == NULL) || (dev->huart == NULL) || (euler == NULL)) {
    return;
  }

  /*
   * Fixed serial protocol for the existing Simulink model:
   *
   *   Header:
   *       "ypr\t"
   *
   *   Payload:
   *       yaw[8] + TAB + pitch[8] + TAB + roll[8]
   *
   *   Payload length:
   *       8 + 1 + 8 + 1 + 8 = 26 bytes
   *
   *   End token:
   *       CR/LF
   *
   * Example complete line:
   *
   *   ypr\t    0.00\t   -0.56\t    0.31\r\n
   *
   * Simulink Serial Receive:
   *   Header    = [121 112 114 9]
   *   Data size = [1 26]
   *   End token = CR/LF
   *
   * Scan String:
   *   %f\t%f\t%f
   *
   * Output order:
   *   1 = yaw
   *   2 = pitch
   *   3 = roll
   */

  debug_puts(dev, "ypr\t");

  debug_print_float_fixed_8_2(dev, euler->heading);
  debug_putc(dev, '\t');

  debug_print_float_fixed_8_2(dev, euler->pitch);
  debug_putc(dev, '\t');
  
  debug_print_float_fixed_8_2(dev, euler->roll);
  

  debug_newline(dev);
}
