#include "mbed.h"
#include "SDP3x.h"

/******************************************************************************
 * Global Functions
 ******************************************************************************/

/* ###########################################################################
 *
 * General functions
 *
 * ###########################################################################
 */
/******************************************************************************
 * readIdent
 *  Gets full ident data
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::readIdent(uint32_t* productnumber, uint64_t* serialnumber)
{
  uint8_t buf[SDP3X_IDENT_LEN];
  uint8_t _cmd[2];

  _cmd[0] = SDP_IDENT_0;
  _cmd[1] = SDP_IDENT_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  _cmd[0] = SDP_IDENT_2;
  _cmd[1] = SDP_IDENT_3;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  /* Read product and serial number from I2C Sensor
   * command 1: 0x367C
   * command 2: 0xE102
   * consecutive read: 18 bytes
   * Byte1: Product number [31:24]
   * Byte2: Product number [23:16]
   * Byte3: CRC
   * Byte4: Product number [15:8]
   * Byte5: Product number [7:0]
   * Byte6: CRC
   * Byte7: Serial number [63:56]
   * Byte8: Serial number [55:48]
   * Byte9: CRC
   * Byte10: Serial number [47:40]
   * Byte11: Serial number [39:32]
   * Byte12: CRC
   * Byte13: Serial number [31:24]
   * Byte14: Serial number [23:16]
   * Byte15: CRC
   * Byte16: Serial number [15:8]
   * Byte17: Serial number [7:0]
   * Byte18: CRC
  */
  if(this->_i2c->read(this->_address, (char*)buf, SDP3X_IDENT_LEN, false))
    return (uint8_t)SDP3X_I2C_FAIL;

  if(productnumber!=NULL)
    *productnumber = (((uint32_t)BIU16(buf, 0)) << 16 | (BIU16(buf, 0+3)));
  if(serialnumber!=NULL)
    *serialnumber = ( ((uint64_t)BIU16(buf, 6)) << 48 | ((uint64_t)BIU16(buf, 0+9)) << 32 | ((uint64_t)BIU16(buf, 0+12)) << 16 | ((uint64_t)BIU16(buf, 0+15)));

  return (uint8_t)SDP3X_OK;
}

/******************************************************************************
 * isSDP32
 *  Gets full ident data and test 0xyyyy02yy
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
bool SDP3xClass::isSDP32()
{
  uint8_t buf[SDP3X_IDENT_LEN];
  uint8_t _cmd[2];

  _cmd[0] = SDP_IDENT_0;
  _cmd[1] = SDP_IDENT_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  _cmd[0] = SDP_IDENT_2;
  _cmd[1] = SDP_IDENT_3;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  /* Read product and serial number from I2C Sensor
   * command 1: 0x367C
   * command 2: 0xE102
   * consecutive read: 18 bytes
   * Byte1: Product number [31:24]
   * Byte2: Product number [23:16]
   * Byte3: CRC
   * Byte4: Product number [15:8]
   * Byte5: Product number [7:0]
   * Byte6: CRC
   * Byte7: Serial number [63:56]
   * Byte8: Serial number [55:48]
   * Byte9: CRC
   * Byte10: Serial number [47:40]
   * Byte11: Serial number [39:32]
   * Byte12: CRC
   * Byte13: Serial number [31:24]
   * Byte14: Serial number [23:16]
   * Byte15: CRC
   * Byte16: Serial number [15:8]
   * Byte17: Serial number [7:0]
   * Byte18: CRC
  */
  if(this->_i2c->read(this->_address, (char*)buf, SDP3X_IDENT_LEN, false))
    return (uint8_t)SDP3X_I2C_FAIL;

  uint32_t ret = (((uint32_t)BIU16(buf, 0)) << 16 | (BIU16(buf, 0+3)));
  ret = ret & 0x00000200;

  return ret?true:false;
}


/******************************************************************************
 * isSDP31
 *  Gets full ident data and test 0xyyyy01yy
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
bool SDP3xClass::isSDP31()
{
  uint8_t buf[SDP3X_IDENT_LEN];
  uint8_t _cmd[2];

  _cmd[0] = SDP_IDENT_0;
  _cmd[1] = SDP_IDENT_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  _cmd[0] = SDP_IDENT_2;
  _cmd[1] = SDP_IDENT_3;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  /* Read product and serial number from I2C Sensor
   * command 1: 0x367C
   * command 2: 0xE102
   * consecutive read: 18 bytes
   * Byte1: Product number [31:24]
   * Byte2: Product number [23:16]
   * Byte3: CRC
   * Byte4: Product number [15:8]
   * Byte5: Product number [7:0]
   * Byte6: CRC
   * Byte7: Serial number [63:56]
   * Byte8: Serial number [55:48]
   * Byte9: CRC
   * Byte10: Serial number [47:40]
   * Byte11: Serial number [39:32]
   * Byte12: CRC
   * Byte13: Serial number [31:24]
   * Byte14: Serial number [23:16]
   * Byte15: CRC
   * Byte16: Serial number [15:8]
   * Byte17: Serial number [7:0]
   * Byte18: CRC
  */
  if(this->_i2c->read(this->_address, (char*)buf, SDP3X_IDENT_LEN, false))
    return (uint8_t)SDP3X_I2C_FAIL;

  uint32_t ret = (((uint32_t)BIU16(buf, 0)) << 16 | (BIU16(buf, 0+3)));
  ret = ret & 0x00000100;

  return ret?true:false;
}

/******************************************************************************
 * softReset
 *
 *  After the reset command the sensor will take maximum 20ms to reset. During
 *  this time the sensor will not acknowledge its address nor accept commands.
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::softReset()
{
  uint8_t _cmd[1];

  _cmd[0] = SDP_SOFTRESET;

  if(this->_i2c->write(0, (char*)_cmd, 1, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  return (uint8_t)SDP3X_OK;
}

/* ###########################################################################
 *
 * Continuous Measurement
 *
 * The sensor measures both the differential pressure and temperature. Both
 * measurement results can be read out through one single I 2 C read header.
 *
 * Continuous measurements can be started up in different configurations:
 * - averaging
 * - update rate 1ms
 * ###########################################################################
 */
/******************************************************************************
 * startContinuousAverageMeasurement
 *  Gets all sensor data
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::startContinuousAverageMeasurement()
{
  uint8_t _cmd[2];

  // triggered mode with 50ms conversion time
  _cmd[0] = SDP_MEASURE_CMD_CONT_AVG_PDIF_0;
  _cmd[1] = SDP_MEASURE_CMD_CONT_AVG_PDIF_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  return (uint8_t)SDP3X_OK;
}

/******************************************************************************
 * startContinuousAverageMeasurement
 *  Gets all sensor data
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::startContinuousMeasurement()
{
  uint8_t _cmd[2];

  // triggered mode with 50ms conversion time
  _cmd[0] = SDP_MEASURE_CMD_CONT_PDIF_0;
  _cmd[1] = SDP_MEASURE_CMD_CONT_PDIF_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  return (uint8_t)SDP3X_OK;
}

/******************************************************************************
 * stopContinuousMeasurement
 *  Gets all sensor data
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::stopContinuousMeasurement()
{
  uint8_t _cmd[2];

  // triggered mode with 50ms conversion time
  _cmd[0] = SDP_MEASURE_CMD_CONT_STOP_0;
  _cmd[1] = SDP_MEASURE_CMD_CONT_STOP_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  return (uint8_t)SDP3X_OK;
}

/* ###########################################################################
 *
 * Triggered Measurement
 *
 * During a triggered measurement the sensor measures both differential pressure
 * and temperature. The measurement starts directly after the command has been
 * sent. The command needs to be repeated with every measurement.
 * ###########################################################################
 */

/******************************************************************************
 * startTriggeredMeasurement
 *  Gets all sensor data
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::startTriggeredMeasurement()
{
  uint8_t _cmd[2];

  // triggered mode with 50ms conversion time
  _cmd[0] = SDP_MEASURE_CMD_TRIG_PDIFF_0;
  _cmd[1] = SDP_MEASURE_CMD_TRIG_PDIFF_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;

  return (uint8_t)SDP3X_OK;
}


/******************************************************************************
 * readPDiffTempScal
 *  Gets all sensor data
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::readValues(uint16_t *diffpressure, uint16_t *temperature, uint16_t *scalefactor)
{
  uint8_t buf[SDP3X_DP_T_SF_LEN];
  uint8_t buflen = SDP3X_DP_T_SF_LEN;

  if(temperature==NULL && scalefactor==NULL)
    buflen = SDP3X_DP_LEN;
  else if(temperature!=NULL && scalefactor==NULL)
    buflen = SDP3X_DP_T_LEN;
  else if(scalefactor!=NULL)
    buflen = SDP3X_DP_T_SF_LEN;

  /* Read full sensor data from I2C Sensor
   * command 1: 0x362f: Differential pressure  w/o clock streching
   * consecutive read: 9 bytes
   * Byte1: Differential Pressure 8msb
   * Byte2: Differential Pressure 8lsb
   * Byte3: CRC
   * Byte4: Temperature 8msb
   * Byte5: Temperature 8lsb
   * Byte6: CRC
   * Byte7: Scale Factor differential pressure 8msb
   * Byte8: Scale Factor differential pressure 8lsb
   * Byte9: CRC
  */
  if(this->_i2c->read(this->_address, (char*)buf, buflen, false))
    return (uint8_t)SDP3X_I2C_FAIL;

  if(diffpressure!=NULL)
    *diffpressure = ((uint16_t)BIU16(buf, 0));
  if(temperature!=NULL)
    *temperature  = ((uint16_t)BIU16(buf, 3));
  if(scalefactor!=NULL)
    *scalefactor  = ((uint16_t)BIU16(buf, 6));

  return (uint8_t)SDP3X_OK;
}



/******************************************************************************
 * readPDiffTempScalTriggered
 *  Gets all sensor data
 *
 * @return uint8_t - i2c error
 ******************************************************************************/
uint8_t SDP3xClass::readTriggeredMeasurement(uint16_t *diffpressure, uint16_t *temperature, uint16_t *scalefactor)
{
  uint8_t buf[SDP3X_DP_T_SF_LEN];
  uint8_t buflen = SDP3X_DP_T_SF_LEN;
  uint8_t _cmd[2];

  // triggered mode with 50ms conversion time
  _cmd[0] = SDP_MEASURE_CMD_TRIG_PDIFF_0;
  _cmd[1] = SDP_MEASURE_CMD_TRIG_PDIFF_1;

  if(this->_i2c->write(this->_address, (char*)_cmd, SDP3X_CMD_LEN, true))
    return (uint8_t)SDP3X_I2C_FAIL;
  
  if(temperature==NULL && scalefactor==NULL)
    buflen = SDP3X_DP_LEN;
  else if(temperature!=NULL && scalefactor==NULL)
    buflen = SDP3X_DP_T_LEN;
  else if(scalefactor!=NULL)
    buflen = SDP3X_DP_T_SF_LEN;

  // wait for data conversion in sensor
  wait_ms(SDP3X_MEASUREMENT_TIME_MS);
  
  /* Read full sensor data from I2C Sensor
   * command 1: 0x362f: Differential pressure  w/o clock streching
   * consecutive read: 9 bytes
   * Byte1: Differential Pressure 8msb
   * Byte2: Differential Pressure 8lsb
   * Byte3: CRC
   * Byte4: Temperature 8msb
   * Byte5: Temperature 8lsb
   * Byte6: CRC
   * Byte7: Scale Factor differential pressure 8msb
   * Byte8: Scale Factor differential pressure 8lsb
   * Byte9: CRC
  */
  if(this->_i2c->read(this->_address, (char*)buf, buflen, false))
    return (uint8_t)SDP3X_I2C_FAIL;

  if(diffpressure!=NULL)
    *diffpressure = ((uint16_t)BIU16(buf, 0));
  if(temperature!=NULL)
    *temperature  = ((uint16_t)BIU16(buf, 3));
  if(scalefactor!=NULL)
    *scalefactor  = ((uint16_t)BIU16(buf, 6));

  return (uint8_t)SDP3X_OK;
}
