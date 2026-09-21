#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


#define QMC5883_LSB_PER_G 12000.0f
#define QMC5883_Gauss_to_uT 100.0f

static int addr1 = 0x0D;
int16_t mag[3];
float mx,my,mz;




void I2C_Init(){
    i2c_init(i2c0, 400000);
    gpio_set_function(0, GPIO_FUNC_I2C);
    gpio_set_function(1, GPIO_FUNC_I2C);
    gpio_pull_up(0);
    gpio_pull_up(1);
}


void qmc5883_init(){
    uint8_t qmc_reg1[] = {0x0B, 0x01};
    uint8_t qmc_reg2[] = {0x09, 0x0D};
    i2c_write_blocking(i2c0, addr1, qmc_reg1, 2, false);
    i2c_write_blocking(i2c0, addr1, qmc_reg2, 2, false);
}

void qmc5883_read_raw(){
   uint8_t buffer[6];
    uint8_t val = 0x00;
    i2c_write_blocking(i2c0, addr1, &val, 1, true);
    i2c_read_blocking(i2c0, addr1, buffer, 6, false);
    for (int i = 0; i < 3; i++) { mag[i] = (buffer[(i * 2) + 1] << 8 | buffer[i * 2]); }
}



int main() {

    sleep_ms(1000);

    I2C_Init();
    qmc5883_init();

    stdio_usb_init();



    while(1){
        qmc5883_read_raw();


        mx = ( (float)mag[0] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
        my = ( (float)mag[1] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;
        mz = ( (float)mag[2] / QMC5883_LSB_PER_G ) * QMC5883_Gauss_to_uT;

        
        printf("%.2f\t%.2f\t%.2f\n",  mx, my, mz);


        sleep_ms(50);


    }
}