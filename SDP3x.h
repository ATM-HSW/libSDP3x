#include "mbed.h"

#ifndef SDP3X_H
#define SDP3X_H

// Enumeration to declare Error_Codes
typedef enum
{
  SDP3X_WRONG_MODE  = -4,
  SDP3X_INVALID_CMD = -3,

  SDP3X_I2C_FAIL    = -2,
  SDP3X_CRC_FAIL    = -1,

  SDP3X_OK          =  0,

  SDP3X_NOT_READY   =  1,
} SDP3X_RET_VAL;

/*---------*/
/* Defines */
/*---------*/

#define SDP3X_ADDR                                        (0x21 << 1)

#define SDP3X_MEASUREMENT_TIME_MS                         50  // 45ms + 5ms reserve
#define SDP3X_MEASUREMENT_CONT_START_MS                   22  // 20ms + 2ms reserve

// Standard lengths of packages in Bytes
#define SDP3X_CRC_PACK_LEN 3    // Datalength of a data package + CRC in an I2C Sensormessage
#define SDP3X_CRC_LEN      1    // Length of a CRC information
#define SDP3X_DATA_LEN     2    // Length of pure data (e.g. dp/mass or Temp)

// Command Codes
#define SDP3X_CMD_LEN      2    // Length of a SPD3x Command Code

// Datalengths to be read from I2C bus
#define SDP3X_DP_LEN          3    // Datalength when reading dP
#define SDP3X_DP_T_LEN        6    // Datalength when reading dP & Temp
#define SDP3X_DP_T_SF_LEN     9    // Datalength when reading dP & Temp & Scale Factor
#define SDP3X_PRODREFNUM_LEN  4    // Datalength when reading product and reference numbers
#define SDP3X_IDENT_LEN      18    // Datalength of Ident-Information including CRC Bytes

// convert two 8 bit values to one word
#define BIU16(data, start) (((uint16_t)(data)[start]) << 8 | ((data)[start + 1]))

// SDP3x sensor I2C address
// The adress can be overwritten by defining SDP3x_I2C_ADDRESS in your sketch directly

#ifndef SDP3x_I2C_ADDRESS
#define SDP3x_I2C_ADDRESS 0x21
#endif

#define SDP_MEASURE_CMD_CONT_AVG_PDIF_0 0x36
#define SDP_MEASURE_CMD_CONT_AVG_PDIF_1 0x15

#define SDP_MEASURE_CMD_CONT_PDIF_0 0x36
#define SDP_MEASURE_CMD_CONT_PDIF_1 0x1E

#define SDP_MEASURE_CMD_CONT_STOP_0 0x3F
#define SDP_MEASURE_CMD_CONT_STOP_1 0xF9

#define SDP_MEASURE_CMD_TRIG_PDIFF_0 0x36
#define SDP_MEASURE_CMD_TRIG_PDIFF_1 0x2F

#define SDP_IDENT_0 0x36
#define SDP_IDENT_1 0x7c
#define SDP_IDENT_2 0xE1
#define SDP_IDENT_3 0x02

#define SDP_SOFTRESET 0x06

class SDP3xClass {
  public:
    SDP3xClass(I2C *i2c, uint8_t addr = SDP3X_ADDR) : _i2c(i2c), _address(addr) {_i2c=i2c;};

    bool isSDP32();
    bool isSDP31();

    uint8_t softReset();

    uint8_t readIdent(uint32_t* productnumber, uint64_t* serialnumber = NULL);

    uint8_t startContinuousAverageMeasurement();
    uint8_t startContinuousMeasurement();
    uint8_t stopContinuousMeasurement();

    uint8_t startTriggeredMeasurement();

    uint8_t readValues(uint16_t *diffpressure, uint16_t *temperature = NULL, uint16_t *scalefactor = NULL);

    uint8_t readTriggeredMeasurement(uint16_t *diffpressure, uint16_t *temperature = NULL, uint16_t *scalefactor = NULL);
  private:
    I2C* _i2c;
    uint8_t  _address;                                // Individual SDP3x IC Address
};

#endif
