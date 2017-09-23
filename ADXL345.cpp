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

#include "ADXL345.h"

// definition for Nomalization
#define ADXL345_SENSITIVITY_2G  4.0f
#define ADXL345_SENSITIVITY_4G  8.0f
#define ADXL345_SENSITIVITY_8G  16.0f
#define ADXL345_SENSITIVITY_16G 32.0f
#define ADXL345_SEN_FULL_RES    4.0f

//Gravity at Earth's surface in m/s/s
#define GRAVITY                (9.80665f / 1000)

#if MBED_MAJOR_VERSION == 2
#define WAIT_MS(x)       wait_ms(x)
#elif  MBED_MAJOR_VERSION == 5
#define WAIT_MS(x)       Thread::wait(x)
#else
#error "Running on Unknown OS"
#endif

ADXL345::ADXL345 (PinName p_sda, PinName p_scl,
    uint8_t addr, uint8_t data_rate, uint8_t fullscale) :
  _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    initialize (addr, data_rate, fullscale);
}

ADXL345::ADXL345 (PinName p_sda, PinName p_scl, uint8_t addr) :
  _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    initialize (addr, ADXL345_DR_200HZ, ADXL345_FULL_RES_16G);
}

ADXL345::ADXL345 (PinName p_sda, PinName p_scl) :
  _i2c_p(new I2C(p_sda, p_scl)), _i2c(*_i2c_p)
{
    _i2c.frequency(400000);
    initialize(ADXL345_V_CHIP_ADDR, ADXL345_DR_200HZ, ADXL345_FULL_RES_16G);
    if (acc_ready == false){
        initialize(ADXL345_G_CHIP_ADDR, ADXL345_DR_200HZ, ADXL345_FULL_RES_16G);
    }
}

ADXL345::ADXL345 (I2C& p_i2c,
    uint8_t addr, uint8_t data_rate, uint8_t fullscale) : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    initialize (addr, data_rate, fullscale);
}

ADXL345::ADXL345 (I2C& p_i2c, uint8_t addr) : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    initialize (addr, ADXL345_DR_200HZ, ADXL345_FULL_RES_16G);
}

ADXL345::ADXL345 (I2C& p_i2c) : _i2c(p_i2c)
{
    _i2c.frequency(400000);
    initialize(ADXL345_V_CHIP_ADDR, ADXL345_DR_200HZ, ADXL345_FULL_RES_16G);
    if (acc_ready == false){
        initialize(ADXL345_G_CHIP_ADDR, ADXL345_DR_200HZ, ADXL345_FULL_RES_16G);
    }
}

void ADXL345::initialize (uint8_t addr, uint8_t data_rate, uint8_t fullscale)
{
    // Check acc is available or not
    acc_addr = addr;
    dt[0] = ADXL345_DEVID;
    _i2c.write(acc_addr, dt, 1, true);
    _i2c.read(acc_addr, dt, 1, false);
    if (dt[0] == ADXL345_DEVICE_ID){
        acc_ready = true;
    } else {
        acc_ready = false;
        return;     // acc chip is NOT on I2C line then terminate
    }
    //  BW Rate
    dt[0] = ADXL345_BW_RATE;
    dt[1] = data_rate | ADXL345_NOT_LOW_PWR;    // normal(not low power mode)
    setting_data[0] = dt[1];
    _i2c.write(acc_addr, dt, 2, false);
    //  Data format (measurement range)
    dt[0] = ADXL345_DATA_FORMAT;
    dt[1] = fullscale;
    setting_data[1] = dt[1];
    _i2c.write(acc_addr, dt, 2, false);
    switch (fullscale){
        case ADXL345_FS_2G:
            fs_factor = ADXL345_SENSITIVITY_2G;
            break;
        case ADXL345_FS_4G:
            fs_factor = ADXL345_SENSITIVITY_4G;
            break;
        case ADXL345_FS_8G:
            fs_factor = ADXL345_SENSITIVITY_8G;
            break;
        case ADXL345_FS_16G:
            fs_factor = ADXL345_SENSITIVITY_16G;
            break;
        case ADXL345_FULL_RES_16G:
            fs_factor = ADXL345_SEN_FULL_RES;
            break;
        default:
            fs_factor = 1.0f;
            break;
    }
    //  Data ready flag
    dt[0] = ADXL345_INT_ENABLE;
    dt[1] = 0x80;
    setting_data[2] = dt[1];
    _i2c.write(acc_addr, dt, 2, false);
    //  Start measurement mode
    dt[0] = ADXL345_POWER_CTL;
    dt[1] = 0x08;
    setting_data[3] = dt[1];
    _i2c.write(acc_addr, dt, 2, false);
    // offset compensation
    dt[0] = ADXL345_OFSX;
    dt[1] = 0x01; 
    _i2c.write(acc_addr, dt, 2, false);
    dt[0] = ADXL345_OFSY;
    dt[1] = 0x00;
    _i2c.write(acc_addr, dt, 2, false);
    dt[0] = ADXL345_OFSZ;
    dt[1] = 0x00;
    _i2c.write(acc_addr, dt, 2, false);
}

void ADXL345::read_reg_data(char *data)
{
    // read all of X,Y & Z
    dt[0] = ADXL345_DATAX0;
    _i2c.write(acc_addr, dt, 1, true);
    _i2c.read(acc_addr, data, 6, false);
}

void ADXL345::read_mg_data(float *dt_usr)
{
    return read_mg_g_data(dt_usr, 0);
}

void ADXL345::read_g_data(float *dt_usr)
{
    return read_mg_g_data(dt_usr, 1);
}

void ADXL345::read_data(float *dt_usr)
{
    return read_mg_g_data(dt_usr, 2);
}

void ADXL345::read_mg_g_data(float *dt_usr, uint8_t n)
{
    char    data[6];
    float   fct;

    if (acc_ready == false){
        dt_usr[0] = 0;
        dt_usr[1] = 0;
        dt_usr[2] = 0;
        return;
    }
    read_reg_data(data);
    if (n == 0){
        fct = fs_factor;
    } else if (n == 1){
        fct = fs_factor / 1000.0f;
    } else {
        fct = fs_factor * GRAVITY;
    }  
    // change data type
    dt_usr[0] = float(int16_t((data[1] << 8) | data[0])) * fct;
    dt_usr[1] = float(int16_t((data[3] << 8) | data[2])) * fct;
    dt_usr[2] = float(int16_t((data[5] << 8) | data[4])) * fct;
}

uint8_t ADXL345::read_id()
{
    dt[0] = ADXL345_DEVID;
    _i2c.write(acc_addr, dt, 1, true);
    _i2c.read(acc_addr, dt, 1, false);
    return (uint8_t)dt[0];
}

bool ADXL345::data_ready()
{
    if (acc_ready == true){
        dt[0] = ADXL345_INT_SOURCE;
        _i2c.write(acc_addr, dt, 1, true);
        _i2c.read(acc_addr, dt, 1, false);
        if (dt[0] & 0x80){  // Check ready bit
            return true;
        } else {
            return false;
        }
    }
    return false;
}

void ADXL345::frequency(int hz)
{
    _i2c.frequency(hz);
}

uint8_t ADXL345::read_reg(uint8_t addr)
{
    if (acc_ready == true){
        dt[0] = addr;
        _i2c.write(acc_addr, dt, 1, true);
        _i2c.read(acc_addr, dt, 1, false);
    } else {
        dt[0] = 0xff;
    }
    return (uint8_t)dt[0];
}

void ADXL345::write_reg(uint8_t addr, uint8_t data)
{
    if (acc_ready == true){
        dt[0] = addr;
        dt[1] = data;
        _i2c.write(acc_addr, dt, 2, false);
    }
}

void ADXL345::debug_print(void)
{
    printf("ADXL345 3-axes accelerometer\r\n");
    printf(" DEVID=0x%02x\r\n", read_reg(ADXL345_DEVID));
    printf(" THRESH_TAP=0x%02x\r\n", read_reg(ADXL345_THRESH_TAP));
    printf(" OFSX=0x%02x,", read_reg(ADXL345_OFSX));
    printf(" OFSY=0x%02x,", read_reg(ADXL345_OFSY));
    printf(" OFSZ=0x%02x\r\n", read_reg(ADXL345_OFSZ));
    printf(" DUR=0x%02x,", read_reg(ADXL345_DUR));
    printf(" LATENT=0x%02x,", read_reg(ADXL345_LATENT));
    printf(" WINDOW=0x%02x\r\n", read_reg(ADXL345_WINDOW));
    printf(" THRESH_ACT=0x%02x,", read_reg(ADXL345_THRESH_ACT));
    printf(" THRESH_INACT=0x%02x\r\n", read_reg(ADXL345_THRESH_INACT));
    printf(" TIME_INACT=0x%02x,", read_reg(ADXL345_TIME_INACT));
    printf(" ACT_INACT_CTL=0x%02x\r\n",
                    read_reg(ADXL345_ACT_INACT_CTL));
    printf(" THRESH_FF=0x%02x,", read_reg(ADXL345_THRESH_FF));
    printf(" TIME_FF=0x%02x,", read_reg(ADXL345_TIME_FF));
    printf(" TAP_AXES=0x%02x,", read_reg(ADXL345_TAP_AXES));
    printf(" ACT_TAP_STATUS=0x%02x\r\n",
                    read_reg(ADXL345_ACT_TAP_STATUS));
    printf(" BW_RATE=0x%02x\r\n", read_reg(ADXL345_BW_RATE));
    printf(" POWER_CTL=0x%02x\r\n", read_reg(ADXL345_POWER_CTL));
    printf(" INT_ENABLE=0x%02x,", read_reg(ADXL345_INT_ENABLE));
    printf(" INT_MAP=0x%02x,", read_reg(ADXL345_INT_MAP));
    printf(" INT_SOURCE=0x%02x\r\n", read_reg(ADXL345_INT_SOURCE));
    printf(" DATA_FORMAT=0x%02x\r\n", read_reg(ADXL345_DATA_FORMAT));
    printf(" DATAX0=0x%02x,", read_reg(ADXL345_DATAX0));
    printf(" 1=0x%02x,", read_reg(ADXL345_DATAX1));
    printf(" DATAY0=0x%02x,", read_reg(ADXL345_DATAY0));
    printf(" 1=0x%02x,", read_reg(ADXL345_DATAY1));
    printf(" DATAZ0=0x%02x,", read_reg(ADXL345_DATAZ0));
    printf(" 1=0x%02x\r\n", read_reg(ADXL345_DATAZ1));
    printf(" FIFO_CTL=0x%02x,", read_reg(ADXL345_FIFO_CTL));
    printf(" FIFO_STATUS=0x%02x\r\n", read_reg(ADXL345_FIFO_STATUS));
    // internal data
    printf(" ---- fs_factor=%f, acc_addr=0x%02x\r\n", fs_factor, acc_addr);
}

void ADXL345::self_test(void)
{
    float dt0[3] ={0};
    float dt1[3] ={0};
    float dt2[3];

    dt[0] = ADXL345_DATA_FORMAT;
    dt[1] = 0x0B;
    _i2c.write(acc_addr, dt, 2, false);
    dt[0] = ADXL345_POWER_CTL;
    dt[1] = 0x08;
    _i2c.write(acc_addr, dt, 2, false);
    dt[0] = ADXL345_INT_ENABLE;
    dt[1] = 0x80;
    _i2c.write(acc_addr, dt, 2, false);
    WAIT_MS(40);
    for(uint8_t n = 0; n < 100; n++){
        read_data(dt2);
        dt0[0] += dt2[0];
        dt0[1] += dt2[1];
        dt0[2] += dt2[2];
    }
    dt0[0] /= 100;
    dt0[1] /= 100;
    dt0[2] /= 100;
    //
    dt[0] = ADXL345_DATA_FORMAT;
    dt[1] = 0x8B;
    _i2c.write(acc_addr, dt, 2, false);
    WAIT_MS(40);
    for(uint8_t n = 0; n < 100; n++){
        read_data(dt2);
        dt1[0] += dt2[0];
        dt1[1] += dt2[1];
        dt1[2] += dt2[2];
    }
    dt1[0] /= 100;
    dt1[1] /= 100;
    dt1[2] /= 100;
    printf("X, 1st, %+8.4f, 2nd, %+8.4f, diff, %+8.4f\r\n",
            dt0[0], dt1[0], dt0[0] - dt1[0]);
    printf("Y, 1st, %+8.4f, 2nd, %+8.4f, diff, %+8.4f\r\n",
            dt0[1], dt1[1], dt0[1] - dt1[1]);
    printf("Z, 1st, %+8.4f, 2nd, %+8.4f, diff, %+8.4f\r\n",
            dt0[2], dt1[2], dt0[2] - dt1[2]);
    //Recover original setting
    //  BW Rate
    dt[0] = ADXL345_BW_RATE;
    dt[1] = setting_data[0];
    _i2c.write(acc_addr, dt, 2, false);
    //  Data format (measurement range)
    dt[0] = ADXL345_DATA_FORMAT;
    dt[1] = setting_data[1];
    _i2c.write(acc_addr, dt, 2, false);
    //  Data ready flag
    dt[0] = ADXL345_INT_ENABLE;
    dt[1] = setting_data[2];
    _i2c.write(acc_addr, dt, 2, false);
    //  Start measurement mode
    dt[0] = ADXL345_POWER_CTL;
    dt[1] = setting_data[3];
    _i2c.write(acc_addr, dt, 2, false);
    WAIT_MS(40);
    read_data(dt2); // dummy read
}
