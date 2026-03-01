#include "ti_msp_dl_config.h"
#include "core.h"
#include <stdint.h>
#include <stdbool.h>

#define MAX30102_ADDR       0x57
#define REG_INTR_STATUS_1   0x00
#define REG_FIFO_WR_PTR     0x04
#define REG_OVF_COUNTER     0x05
#define REG_FIFO_RD_PTR     0x06
#define REG_FIFO_DATA       0x07
#define REG_FIFO_CONFIG     0x08
#define REG_MODE_CONFIG     0x09
#define REG_SPO2_CONFIG     0x0A
#define REG_LED1_PA         0x0C
#define REG_LED2_PA         0x0D
#define REG_PART_ID         0xFF

#define SAMPLE_DELAY_MS     10      
#define AC_ALPHA_DEN        16      
#define PEAK_HYSTERESIS     200    
#define REFRACTORY_MS       280    
#define BPM_AVG_SIZE        6      
#define MIN_FINGER_IR       5000
#define SPO2_AVG_SIZE       8
#define SPO2_VALID_MIN      80


static void i2c_wait_idle(void)
{
    while (!(DL_I2C_getControllerStatus(I2C_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE));
}

static void i2c_wait_not_busy(void)
{
    while (DL_I2C_getControllerStatus(I2C_INST) &
           DL_I2C_CONTROLLER_STATUS_BUSY);
}

static void write_reg(uint8_t reg, uint8_t val)
{
    uint8_t d[2] = {reg, val};

    i2c_wait_idle();
    DL_I2C_fillControllerTXFIFO(I2C_INST, d, 2);
    DL_I2C_startControllerTransfer(I2C_INST,
                                   MAX30102_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   2);
    i2c_wait_not_busy();
    i2c_wait_idle();
}

static void read_bytes(uint8_t reg, uint8_t *buf, uint8_t n)
{
    i2c_wait_idle();
    DL_I2C_fillControllerTXFIFO(I2C_INST, &reg, 1);
    DL_I2C_startControllerTransfer(I2C_INST,
                                   MAX30102_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_TX,
                                   1);
    i2c_wait_not_busy();
    i2c_wait_idle();

    DL_I2C_startControllerTransfer(I2C_INST,
                                   MAX30102_ADDR,
                                   DL_I2C_CONTROLLER_DIRECTION_RX,
                                   n);

    for (uint8_t i = 0; i < n; i++)
    {
        while (DL_I2C_isControllerRXFIFOEmpty(I2C_INST));
        buf[i] = DL_I2C_receiveControllerData(I2C_INST);
    }

    i2c_wait_not_busy();
    i2c_wait_idle();
}

static uint8_t read_u8(uint8_t reg)
{
    uint8_t val = 0;
    read_bytes(reg, &val, 1);
    return val;
}

static bool fifo_has_data(void)
{
    uint8_t wr = read_u8(REG_FIFO_WR_PTR);
    uint8_t rd = read_u8(REG_FIFO_RD_PTR);
    return wr != rd;
}

static void max30102_init(void)
{
    write_reg(REG_MODE_CONFIG, 0x40);  
    delay_ms(100);

    write_reg(REG_FIFO_CONFIG, 0x1F);
    write_reg(REG_SPO2_CONFIG, 0x27);

    write_reg(REG_LED1_PA, 0x24);      
    write_reg(REG_LED2_PA, 0x24);      

    write_reg(REG_FIFO_WR_PTR,  0x00);
    write_reg(REG_FIFO_RD_PTR,  0x00);
    write_reg(REG_OVF_COUNTER,  0x00);

    write_reg(REG_MODE_CONFIG, 0x03);  
}

static uint32_t read_both_samples(uint32_t *red_out)
{
    uint8_t buf[6];
    read_bytes(REG_FIFO_DATA, buf, 6);

    *red_out = ((uint32_t)buf[0] << 16 |
                (uint32_t)buf[1] <<  8 |
                (uint32_t)buf[2])  & 0x03FFFF;

    uint32_t ir = ((uint32_t)buf[3] << 16 |
                   (uint32_t)buf[4] <<  8 |
                   (uint32_t)buf[5])  & 0x03FFFF;
    return ir;
}

static uint32_t update_bpm_average(uint32_t new_period_ms)
{
    static uint32_t periods[BPM_AVG_SIZE] = {0};
    static uint8_t  idx                   = 0;
    static uint8_t  count                 = 0;

    if (new_period_ms == 0)
    {
        for (uint8_t i = 0; i < BPM_AVG_SIZE; i++) periods[i] = 0;
        idx   = 0;
        count = 0;
        return 0;
    }

    if (count == BPM_AVG_SIZE)
    {
        uint32_t sum = 0;
        for (uint8_t i = 0; i < BPM_AVG_SIZE; i++)
            sum += periods[i];
        uint32_t current_avg = sum / BPM_AVG_SIZE;

        uint32_t diff = (new_period_ms > current_avg)
                        ? (new_period_ms - current_avg)
                        : (current_avg - new_period_ms);

        if (diff * 100 > current_avg * 50)
            return 60000 / current_avg;
    }

    periods[idx] = new_period_ms;
    idx          = (idx + 1) % BPM_AVG_SIZE;
    if (count < BPM_AVG_SIZE) count++;

    if (count < BPM_AVG_SIZE) return 0;

    uint32_t sum = 0;
    for (uint8_t i = 0; i < BPM_AVG_SIZE; i++)
        sum += periods[i];

    return 60000 / (sum / BPM_AVG_SIZE);
}

static const uint8_t spo2_table[185] = {
    95, 95, 95, 96, 96, 96, 97, 97, 97, 97,  /* 0   – 9   */
    98, 98, 98, 98, 99, 99, 99, 99, 99, 99,  /* 10  – 19  */
    99, 99, 99, 99, 99, 98, 98, 98, 98, 98,  /* 20  – 29  */
    98, 97, 97, 97, 97, 96, 96, 96, 96, 95,  /* 30  – 39  */
    95, 95, 94, 94, 94, 93, 93, 93, 92, 92,  /* 40  – 49  */
    92, 91, 91, 91, 90, 90, 89, 89, 89, 88,  /* 50  – 59  */
    88, 87, 87, 87, 86, 86, 85, 85, 84, 84,  /* 60  – 69  */
    83, 83, 82, 82, 81, 81, 80, 80, 79, 79,  /* 70  – 79  */
    78, 78, 77, 77, 76, 76, 75, 74, 74, 73,  /* 80  – 89  */
    73, 72, 72, 71, 71, 70, 69, 69, 68, 68,  /* 90  – 99  */
    67, 66, 66, 65, 65, 64, 63, 63, 62, 62,  /* 100 – 109 */
    61, 60, 60, 59, 58, 58, 57, 57, 56, 55,  /* 110 – 119 */
    55, 54, 53, 53, 52, 51, 51, 50, 49, 49,  /* 120 – 129 */
    48, 47, 47, 46, 45, 45, 44, 43, 43, 42,  /* 130 – 139 */
    41, 40, 40, 39, 38, 38, 37, 36, 35, 35,  /* 140 – 149 */
    34, 33, 32, 32, 31, 30, 29, 29, 28, 27,  /* 150 – 159 */
    26, 25, 25, 24, 23, 22, 21, 21, 20, 19,  /* 160 – 169 */
    18, 17, 16, 15, 15, 14, 13, 12, 11, 10,  /* 170 – 179 */
     9,  8,  7,  6,  5                        /* 180 – 184 */
};

static uint32_t update_spo2_average(uint32_t new_spo2)
{
    static uint32_t samples[SPO2_AVG_SIZE] = {0};
    static uint8_t  idx                    = 0;
    static uint8_t  count                  = 0;

    if (new_spo2 == 0)
    {
        for (uint8_t i = 0; i < SPO2_AVG_SIZE; i++) samples[i] = 0;
        idx   = 0;
        count = 0;
        return 0;
    }

    samples[idx] = new_spo2;
    idx          = (idx + 1) % SPO2_AVG_SIZE;
    if (count < SPO2_AVG_SIZE) count++;

    if (count < SPO2_AVG_SIZE) return 0;    

    uint32_t sum = 0;
    for (uint8_t i = 0; i < SPO2_AVG_SIZE; i++)
        sum += samples[i];

    return sum / SPO2_AVG_SIZE;
}

int main(void)
{
    SYSCFG_DL_init();
    delay_ms(10);

    put((unsigned char *)"Heart Rate Monitor Starting...\r\n");

    uint8_t id = read_u8(REG_PART_ID);
    if (id != 0x15)
    {
        put((unsigned char *)"ERROR: MAX30102 not found! Check wiring.\r\n");
        while (1);
    }
    put((unsigned char *)"MAX30102 detected.\r\n");

    max30102_init();

    put((unsigned char *)"Stabilising... please wait.\r\n");
    delay_ms(500);
    put((unsigned char *)"Place finger on sensor.\r\n");

    uint32_t dc_estimate        = 0;    
    uint32_t dc_red             = 0;    
    uint32_t last_peak_time     = 0;
    uint32_t current_time       = 0;
    uint32_t last_beat_time     = 0;
    bool     pulse_detected     = false;
    bool     dc_initialized     = false;
    bool     finger_present     = false;
    int32_t  ac_ir_at_peak      = 0;
    int32_t  ac_red_at_peak     = 0;

    while (1)
    {
        if (!fifo_has_data())
        {
            delay_ms(SAMPLE_DELAY_MS);
            current_time += SAMPLE_DELAY_MS;
            continue;
        }

        uint32_t red = 0;
        uint32_t ir  = read_both_samples(&red);
        current_time += SAMPLE_DELAY_MS;

        if (ir < MIN_FINGER_IR)
        {
            if (finger_present)
            {
                put((unsigned char *)"No finger detected - waiting...\r\n");

                dc_initialized  = false;
                pulse_detected  = false;
                last_peak_time  = 0;
                last_beat_time  = 0;
                ac_ir_at_peak   = 0;
                ac_red_at_peak  = 0;
                update_bpm_average(0);
                update_spo2_average(0);

                finger_present  = false;
            }

            delay_ms(SAMPLE_DELAY_MS);
            continue;
        }

        if (!finger_present)
        {
            put((unsigned char *)"Finger detected - measuring...\r\n");
            finger_present = true;
        }

        if (!dc_initialized)
        {
            dc_estimate    = ir;
            dc_red         = red;
            dc_initialized = true;
        }

        int32_t signed_ir  = (int32_t)ir;
        int32_t signed_dc  = (int32_t)dc_estimate;
        int32_t correction = (signed_ir - signed_dc) / (int32_t)AC_ALPHA_DEN;
        dc_estimate        = (uint32_t)(signed_dc + correction);
        int32_t ac_ir      = signed_ir - (int32_t)dc_estimate;

        int32_t signed_red    = (int32_t)red;
        int32_t signed_dc_red = (int32_t)dc_red;
        int32_t corr_red      = (signed_red - signed_dc_red) / (int32_t)AC_ALPHA_DEN;
        dc_red                = (uint32_t)(signed_dc_red + corr_red);
        int32_t ac_red        = signed_red - (int32_t)dc_red;

        if (ac_ir > (int32_t)PEAK_HYSTERESIS && !pulse_detected)
        {
            ac_ir_at_peak  = ac_ir;
            ac_red_at_peak = ac_red;
        }

        if (ac_ir > (int32_t)PEAK_HYSTERESIS && !pulse_detected)
        {
            pulse_detected = true;

            bool in_refractory = (last_beat_time != 0) &&
                                 ((current_time - last_beat_time) < REFRACTORY_MS);

            if (!in_refractory && last_peak_time != 0)
            {
                uint32_t period = current_time - last_peak_time;

                if (period > 300 && period < 1500)
                {
                    uint32_t bpm = update_bpm_average(period);

                    if (bpm > 0)
                    {
                        uint32_t spo2 = 0;

                        if (ac_ir_at_peak > 0 && ac_red_at_peak > 0 &&
                            dc_red > 0 && dc_estimate > 0)
                        {
                            uint32_t r_scaled =
                                ((uint32_t)ac_red_at_peak * dc_estimate * 100)
                                / ((uint32_t)ac_ir_at_peak * dc_red);

                            if (r_scaled < 185)
                                spo2 = update_spo2_average(spo2_table[r_scaled]);
                        }

                        put((unsigned char *)"BPM:  ");
                        put_num(bpm);

                        if (spo2 >= SPO2_VALID_MIN)
                        {
                            put((unsigned char *)"  |  SpO2: ");
                            put_num(spo2);
                            put((unsigned char *)"%\r\n");
                        }
                        else
                        {
                            put_nl();
                        }
                    }

                    last_beat_time = current_time;
                }
            }

            last_peak_time = current_time;
        }

        if (ac_ir < -(int32_t)PEAK_HYSTERESIS)
        {
            pulse_detected = false;
        }

        delay_ms(SAMPLE_DELAY_MS);
    }
}
