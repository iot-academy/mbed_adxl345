/*
 * mbed library program
 *  ADXL345: 3-axis accelerometer, made by Analog Devices
 *  http://www.analog.com/static/imported-files/data_sheets/ADXL345.pdf
 *
 * Copyright (c) 2017 Kenji Arai / JH1PJL
 *  http://www.page.sannet.ne.jp/kenjia/index.html
 *  http://mbed.org/users/kenjiArai/
 *      Modify:     August    13th, 2017
 *      Revised:    September 23rd, 2017
 *
 */

#ifndef ADXL345_H
#define ADXL345_H

#include "mbed.h"

//  ADXL345 Address
//  7bit address = 0x1D or 0x53 (depends on ALT ADDRESS pin)
#define ADXL345_G_CHIP_ADDR  (0x53 << 1)    //  ALT ADDRESS = Ground
#define ADXL345_V_CHIP_ADDR  (0x1D << 1)    //  ALT ADDRESS = Vdd

//   ADXL345 ID
#define ADXL345_DEVICE_ID       0xE5

//  Register's definition
#define ADXL345_DEVID           0x00
#define ADXL345_THRESH_TAP      0x1D
#define ADXL345_OFSX            0x1E
#define ADXL345_OFSY            0x1F
#define ADXL345_OFSZ            0x20
#define ADXL345_DUR             0x21
#define ADXL345_LATENT          0x22
#define ADXL345_WINDOW          0x23
#define ADXL345_THRESH_ACT      0x24
#define ADXL345_THRESH_INACT    0x25
#define ADXL345_TIME_INACT      0x26
#define ADXL345_ACT_INACT_CTL   0x27
#define ADXL345_THRESH_FF       0x28
#define ADXL345_TIME_FF         0x29
#define ADXL345_TAP_AXES        0x2A
#define ADXL345_ACT_TAP_STATUS  0x2B
#define ADXL345_BW_RATE         0x2C
#define ADXL345_POWER_CTL       0x2D
#define ADXL345_INT_ENABLE      0x2E
#define ADXL345_INT_MAP         0x2F
#define ADXL345_INT_SOURCE      0x30
#define ADXL345_DATA_FORMAT     0x31
#define ADXL345_DATAX0          0x32
#define ADXL345_DATAX1          0x33
#define ADXL345_DATAY0          0x34
#define ADXL345_DATAY1          0x35
#define ADXL345_DATAZ0          0x36
#define ADXL345_DATAZ1          0x37
#define ADXL345_FIFO_CTL        0x38
#define ADXL345_FIFO_STATUS     0x39

// Data Rate
#define ADXL345_LOW_PWR         0x10
#define ADXL345_NOT_LOW_PWR     0x00
#define ADXL345_DR_R10HZ        0x00
#define ADXL345_DR_R20HZ        0x01
#define ADXL345_DR_R39HZ        0x02
#define ADXL345_DR_R78HZ        0x03
#define ADXL345_DR_1R56HZ       0x04
#define ADXL345_DR_3R13HZ       0x05
#define ADXL345_DR_6R25HZ       0x06
#define ADXL345_DR_12R5HZ       0x07
#define ADXL345_DR_25HZ         0x08
#define ADXL345_DR_50HZ         0x09
#define ADXL345_DR_100HZ        0x0A
#define ADXL345_DR_200HZ        0x0B
#define ADXL345_DR_400HZ        0x0C
#define ADXL345_DR_800HZ        0x0D
#define ADXL345_DR_1R6KHZ       0x0E
#define ADXL345_DR_3R2KHZ       0x0F

// FIFO Mode
#define ADXL345_FIFO_BYPASS     0x00    // Not use FIFO
#define ADXL345_FIFO_FIFO       0x40    // FIFO collects 32 then stop
#define ADXL345_FIFO_STREAM     0x80    // last 32 & continue sampling
#define ADXL345_FIFO_TRIGER     0xC0    // Start by trigger

// FIFO Trigger Source
#define ADXL345_FIFO_TRG_INT1   0x00
#define ADXL345_FIFO_TRG_INT2   0x20

// FIFO Trigger Source
#define ADXL345_FIFO_SAMPLES    0x31    // default value

// Full Scale
#define ADXL345_FS_2G           0x00
#define ADXL345_FS_4G           0x01
#define ADXL345_FS_8G           0x02
#define ADXL345_FS_16G          0x03
#define ADXL345_FULL_RES_16G    0x0B

/** Interface for Analog Devices : 3-axis accelerometer
 *      Chip: ADXL345
 *
 * @code
 * #include "mbed.h"
 *
 * // I2C Communication
 * I2C i2c(D14,D15);  // SDA, SCL
 * ADXL345 acc(i2c);
 *
 * int main() {
 *  float f[3];
 *   while(1){
 *      acc.read_data(f);
 *   }
 * }
 * @endcode
 */

class ADXL345
{
public:
    /** Configure data pin (with other devices on I2C line)
      * @param I2C PinName SDA &SDL
      * @param device address
      * @param output data rate selection, power down mode, 0.1Hz to 3.2KHz
      * @param full scale selection, +/-2g to +/-16g
      */
    ADXL345(PinName p_sda, PinName p_scl,
           uint8_t addr, uint8_t data_rate, uint8_t fullscale);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param device address
      */
    ADXL345(PinName p_sda, PinName p_scl, uint8_t addr);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      */
    ADXL345(PinName p_sda, PinName p_scl);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @param device address
      * @param output data rate selection, power down mode, 0.1Hz to 3.2KHz
      * @param full scale selection, +/-2g to +/-16g
      */
    ADXL345(I2C& p_i2c,
           uint8_t addr, uint8_t data_rate, uint8_t fullscale);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @default output data rate selection = 100Hz
      * @default full scale selection = +/-2g
      */
    ADXL345(I2C& p_i2c, uint8_t addr);

    /** Configure data pin (with other devices on I2C line)
      * @param I2C previous definition
      * @default address check both G & V
      * @default output data rate selection = 100Hz
      * @default full scale selection = +/-2g
      */
    ADXL345(I2C& p_i2c);

    /** Read a float type data from acc
      * @param float type of three arry's address, e.g. float dt_usr[3];
      * @return acc motion data unit: m/s/s(m/s2)
      * @return dt_usr[0]->x, dt_usr[1]->y, dt_usr[2]->z
      */
    void read_data(float *dt_usr);

    /** Read a float type data from acc
      * @param float type of three arry's address, e.g. float dt_usr[3];
      * @return acc motion data unit: mg
      * @return dt_usr[0]->x, dt_usr[1]->y, dt_usr[2]->z
      */
    void read_mg_data(float *dt_usr);

    /** Read a float type data from acc
      * @param float type of three arry's address, e.g. float dt_usr[3];
      * @return acc motion data unit: g
      * @return dt_usr[0]->x, dt_usr[1]->y, dt_usr[2]->z
      */
    void read_g_data(float *dt_usr);

    /** Read a acc ID number
      * @param none
      * @return ID is okay (I_AM_ ADXL345(0x33)) or not
      */
    uint8_t read_id();

    /** Read Data Ready flag
      * @param none
      * @return true = Ready
      */
    bool data_ready();

    /** Set I2C clock frequency
      * @param freq.
      * @return none
      */
    void frequency(int hz);

    /** Read register (general purpose)
      * @param register's address
      * @return register data
      */
    uint8_t read_reg(uint8_t addr);

    /** Write register (general purpose)
      * @param register's address
      * @param data
      * @return none
      */
    void write_reg(uint8_t addr, uint8_t data);

    /** data print for debug
      * @param none
      * @return none
      */
    void debug_print(void);

    /** Self-Test Feature
      * @param none
      * @return none
      */
    void self_test(void);

protected:
    void initialize(uint8_t, uint8_t, uint8_t);
    void read_reg_data(char *data);
    void read_mg_g_data(float *dt_usr, uint8_t n);

    I2C *_i2c_p;
    I2C &_i2c;

private:
    float   fs_factor;  // full scale factor
    char    dt[2];      // working buffer
    uint8_t acc_addr;   // acc sensor address
    uint8_t acc_id;     // acc ID
    bool    acc_ready;  // acc is on I2C line = 1, not = 0
    uint8_t setting_data[4];    // Reg. recovery data

};

#endif      // ADXL345_H
