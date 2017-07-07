
// read StateOfCharge of battery
static void I2CBatterySOC()
{
	I2CAddress = 0xAA;	
	
	I2CTxBufSize = 1;
	I2CRxBufSize = 1;	
	
	I2CTxBuffer[0] = 0x02;

	I2CRxBuffer[0] = 0x00;
	I2CRxBuffer[1] = 0x00;
	
	HAL_I2C_Master_Transmit(&hi2c1, I2CAddress, (uint8_t*)I2CTxBuffer, I2CTxBufSize, 2);	 
  HAL_Delay (2);
	HAL_I2C_Master_Receive(&hi2c1,I2CAddress, (uint8_t *)I2CRxBuffer, I2CRxBufSize, 3);	
	
	BatterySOC[1] = I2CRxBuffer[0]; //LSB
	
	//////////////////////
	
	I2CTxBufSize = 1;
	I2CRxBufSize = 1;	
	
	I2CTxBuffer[0] = 0x03;

	I2CRxBuffer[0] = 0x00;
	I2CRxBuffer[1] = 0x00;
	
	HAL_I2C_Master_Transmit(&hi2c1, I2CAddress, (uint8_t*)I2CTxBuffer, I2CTxBufSize, 2);	 
  HAL_Delay (2);
	HAL_I2C_Master_Receive(&hi2c1,I2CAddress, (uint8_t *)I2CRxBuffer, I2CRxBufSize, 3);	
	
	BatterySOC[0] = I2CRxBuffer[0]; //MSB
}
