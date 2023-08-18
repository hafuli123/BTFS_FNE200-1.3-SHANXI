#ifndef HEADER_CONFIG_H
#define HEADER_CONFIG_H

/*****************CONFIG BEGIN**********************/
#define SPI_MODE                    //use SPI driver
//#define INPUT_VOLT_1_8        // 1.8V input voltage
#define APDU_PRINT              //print command interaction
/*****************CONFIG END************************/




#ifdef SPI_MODE
#define ENABLE_SPI
#else
#define ENABLE_I2C
#endif

#endif        /*!HEADER_CONFIG_H*/
