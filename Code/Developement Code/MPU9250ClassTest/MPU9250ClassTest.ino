
#include <i2c_t3.h>

//Magnetometer Registers
#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00  // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L    0x03  // data
#define AK8963_XOUT_H    0x04
#define AK8963_YOUT_L    0x05
#define AK8963_YOUT_H    0x06
#define AK8963_ZOUT_L    0x07
#define AK8963_ZOUT_H    0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L      0x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E

#define MPU9250_ADDRESS 0x69

#define BETA 	0.1

class MPU9250Interface {
private:
	float magCalibration[3];
	float gyroBias[3] = {0, 0, 0};
	float accelBias[3] = {0, 0, 0};

	void initMPU9250() {
		// wake up device
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
		delay(100); // Wait for all registers to reset

		// get stable time source
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
		delay(200);

		// Configure Gyro and Thermometer
		// Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
		// minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
		// be higher than 1 / 0.0059 = 170 Hz
		// DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
		// With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
		writeByte(MPU9250_ADDRESS, CONFIG, 0x03);

		// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
		writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
		// determined inset in CONFIG above

		// Set gyroscope full scale range
		// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
		uint8_t c = readByte(MPU9250_ADDRESS, GYRO_CONFIG);
		//  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x02); // Clear Fchoice bits [1:0]
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG, c | 0x00 << 3); // Set gyro range to 250dps
		// writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

		// Set accelerometer full-scale range configuration
		c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG);
		//  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, c | 0x00 << 3); // Set acceleromter range to 2g

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
		// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		c = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2);
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c & ~0x0F); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, c | 0x03); // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

		// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
		// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
		// clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
		// can join the I2C bus and all can be controlled by the Arduino as master
		writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
		writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
		delay(100);
	}
	void initAK8963() {
		// First extract the factory calibration for each magnetometer axis
		uint8_t rawData[3];  // x/y/z gyro calibration data stored here
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
		delay(10);
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
		delay(10);
		readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &rawData[0]);  // Read the x-, y-, and z-axis calibration values
		magCalibration[0] = float((rawData[0] - 128)) / 256.0 + 1.0; // Return x-axis sensitivity adjustment values, etc.
		magCalibration[1] = float((rawData[1] - 128)) / 256.0 + 1.0;
		magCalibration[2] = float((rawData[2] - 128)) / 256.0 + 1.0;
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
		delay(10);
		// Configure the magnetometer for continuous read and highest resolution
		// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
		// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
		writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x01 << 4 | 0x06); // Set magnetometer data resolution and sample ODR
		delay(10);
	}

	// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
	// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
	void calibrateMPU9250() {
		uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
		uint16_t ii, packet_count, fifo_count;
		int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

		// reset device
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
		delay(100);

		// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
		// else use the internal oscillator, bits 2:0 = 001
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
		writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
		delay(200);

		// Configure device for bias calculation
		writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
		writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
		writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
		writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
		writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
		writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
		delay(15);

		// Configure MPU6050 gyro and accelerometer for bias calculation
		writeByte(MPU9250_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
		writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

		// Configure FIFO to capture accelerometer and gyro data for bias calculation
		writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
		writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
		delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

		// At end of sample accumulation, turn off FIFO sensor read
		writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
		readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
		fifo_count = ((uint16_t)data[0] << 8) | data[1];
		packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

		for (ii = 0; ii < packet_count; ii++) {
			int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
			readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
			accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
			accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
			accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
			gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
			gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
			gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

			accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
			accel_bias[1] += (int32_t) accel_temp[1];
			accel_bias[2] += (int32_t) accel_temp[2];
			gyro_bias[0]  += (int32_t) gyro_temp[0];
			gyro_bias[1]  += (int32_t) gyro_temp[1];
			gyro_bias[2]  += (int32_t) gyro_temp[2];

		}
		accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
		accel_bias[1] /= (int32_t) packet_count;
		accel_bias[2] /= (int32_t) packet_count;
		gyro_bias[0]  /= (int32_t) packet_count;
		gyro_bias[1]  /= (int32_t) packet_count;
		gyro_bias[2]  /= (int32_t) packet_count;

		if (accel_bias[2] > 0L) accel_bias[2] -= 16384; // Remove gravity from the z-axis accelerometer bias calculation
		else accel_bias[2] += 16384;

		// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
		data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
		data[1] = (-gyro_bias[0] / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
		data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
		data[3] = (-gyro_bias[1] / 4)       & 0xFF;
		data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
		data[5] = (-gyro_bias[2] / 4)       & 0xFF;

		// Push gyro biases to hardware registers
		writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
		writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);

		// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
		// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
		// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
		// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
		// the accelerometer biases calculated above must be divided by 8.

		int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
		readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
		accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
		readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
		accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
		readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
		accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

		uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
		uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

		for (ii = 0; ii < 3; ii++) {
			if ((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
		}

		// Construct total accelerometer bias, including calculated average accelerometer bias from above
		accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
		accel_bias_reg[1] -= (accel_bias[1] / 8);
		accel_bias_reg[2] -= (accel_bias[2] / 8);

		data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
		data[1] = (accel_bias_reg[0])      & 0xFF;
		data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
		data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
		data[3] = (accel_bias_reg[1])      & 0xFF;
		data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
		data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
		data[5] = (accel_bias_reg[2])      & 0xFF;
		data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

		// Apparently this is not working for the acceleration biases in the MPU-9250
		// Are we handling the temperature correction bit properly?
		// Push accelerometer biases to hardware registers
		writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
		writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
		writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
		writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	}

	// Accelerometer and gyroscope self test; check calibration wrt factory settings
	void MPU9250SelfTest(float *destination) { // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
		uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
		uint8_t selfTest[6];
		int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
		float factoryTrim[6];
		uint8_t FS = 0;

		writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
		writeByte(MPU9250_ADDRESS, CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 1 << FS); // Set full scale range for the gyro to 250 dps
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 1 << FS); // Set full scale range for the accelerometer to 2 g

		for ( int ii = 0; ii < 200; ii++) { // get average current values of gyro and acclerometer

			readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);        // Read the six raw data registers into data array
			aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
			aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

			readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);       // Read the six raw data registers sequentially into data array
			gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
			gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
		}

		for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average current readings
			aAvg[ii] /= 200;
			gAvg[ii] /= 200;
		}

		// Configure the accelerometer for self-test
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
		delay(25);  // Delay a while to let the device stabilize

		for ( int ii = 0; ii < 200; ii++) { // get average self-test values of gyro and acclerometer

			readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
			aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
			aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

			readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
			gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
			gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
			gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
		}

		for (int ii = 0; ii < 3; ii++) { // Get average of 200 values and store as average self-test readings
			aSTAvg[ii] /= 200;
			gSTAvg[ii] /= 200;
		}

		// Configure the gyro and accelerometer for normal operation
		writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
		writeByte(MPU9250_ADDRESS, GYRO_CONFIG,  0x00);
		delay(25);  // Delay a while to let the device stabilize

		// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
		selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
		selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
		selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
		selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO);  // X-axis gyro self-test results
		selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO);  // Y-axis gyro self-test results
		selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO);  // Z-axis gyro self-test results

		// Retrieve factory self-test value from self-test code reads
		factoryTrim[0] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
		factoryTrim[1] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
		factoryTrim[2] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
		factoryTrim[3] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
		factoryTrim[4] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
		factoryTrim[5] = (float)(2620 / 1 << FS) * (pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

		// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
		// To get percent, must multiply by 100
		for (int i = 0; i < 3; i++) {
			destination[i]   = 100.0 * ((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i]; // Report percent differences
			destination[i + 3] = 100.0 * ((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3]; // Report percent differences
		}
	}

	void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
		float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx;
		static elapsedMicros dtTimer = 0;
		float qDot1, qDot2, qDot3, qDot4;
		float _2bx, _2bz, _4bx, _4bz;
		float _2q0, _2q1, _2q2, _2q3;
		float _2q0q2, _2q2q3;
		float s0, s1, s2, s3;
		float recipNorm;
		float hx, hy;
		float deltaT;

		// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
		if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) {
			MadgwickAHRSupdateNoMag(gx, gy, gz, ax, ay, az, &dtTimer);
			return;
		}

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.0 * (-q[1] * gx - q[2] * gy - q[3] * gz);
		qDot2 = 0.0 * (q[0] * gx + q[2] * gz - q[3] * gy);
		qDot3 = 0.0 * (q[0] * gy - q[1] * gz + q[3] * gx);
		qDot4 = 0.0 * (q[0] * gz + q[1] * gy - q[2] * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Normalise magnetometer measurement
			recipNorm = invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0 * q[0] * mx;
			_2q0my = 2.0 * q[0] * my;
			_2q0mz = 2.0 * q[0] * mz;
			_2q1mx = 2.0 * q[1] * mx;
			_2q0 = 2.0 * q[0];
			_2q1 = 2.0 * q[1];
			_2q2 = 2.0 * q[2];
			_2q3 = 2.0 * q[3];
			_2q0q2 = 2.0 * q[0] * q[2];
			_2q2q3 = 2.0 * q[2] * q[3];
			q0q0 = q[0] * q[0];
			q0q1 = q[0] * q[1];
			q0q2 = q[0] * q[2];
			q0q3 = q[0] * q[3];
			q1q1 = q[1] * q[1];
			q1q2 = q[1] * q[2];
			q1q3 = q[1] * q[3];
			q2q2 = q[2] * q[2];
			q2q3 = q[2] * q[3];
			q3q3 = q[3] * q[3];

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + mx * q1q1 + _2q1 * my * q[2] + _2q1 * mz * q[3] - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q[3] + my * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q[3] - my * q3q3;
			_2bx = sqrt(hx * hx + hy * hy);
			_2bz = -_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3;
			_4bx = 2.0 * _2bx;
			_4bz = 2.0 * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0 * q1q3 - _2q0q2 - ax) + _2q1 * (2.0 * q0q1 + _2q2q3 - ay) - _2bz * q[2] * (_2bx * (0.0 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[2] * (_2bx * (q0q2 + q1q3) + _2bz * (0.0 - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0 * q1q3 - _2q0q2 - ax) + _2q0 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q[1] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + _2bz * q[3] * (_2bx * (0.0 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.0 - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0 * q1q3 - _2q0q2 - ax) + _2q3 * (2.0 * q0q1 + _2q2q3 - ay) - 4.0 * q[2] * (1 - 2.0 * q1q1 - 2.0 * q2q2 - az) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.0 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q0q2 + q1q3) + _2bz * (0.0 - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0 * q1q3 - _2q0q2 - ax) + _2q2 * (2.0 * q0q1 + _2q2q3 - ay) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.0 - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q[1] * (_2bx * (q0q2 + q1q3) + _2bz * (0.0 - q1q1 - q2q2) - mz);
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= BETA * s0;
			qDot2 -= BETA * s1;
			qDot3 -= BETA * s2;
			qDot4 -= BETA * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		deltaT = 0.5*float(dtTimer)*1.0e-6;
		q[0] += qDot1 * deltaT;
		q[1] += qDot2 * deltaT;
		q[2] += qDot3 * deltaT;
		q[3] += qDot4 * deltaT;

		// Normalise quaternion
		recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] *= recipNorm;
		q[1] *= recipNorm;
		q[2] *= recipNorm;
		q[3] *= recipNorm;
	}

	//This function is used if magnetometer measurements are invalid
	void MadgwickAHRSupdateNoMag(float gx, float gy, float gz, float ax, float ay, float az, elapsedMicros* dtTimer) {
		float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
		float qDot1, qDot2, qDot3, qDot4;
		float s0, s1, s2, s3;
		float recipNorm;
		float deltaT;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.0 * (-q[1] * gx - q[2] * gy - q[3] * gz);
		qDot2 = 0.0 * (q[0] * gx + q[2] * gz - q[3] * gy);
		qDot3 = 0.0 * (q[0] * gy - q[1] * gz + q[3] * gx);
		qDot4 = 0.0 * (q[0] * gz + q[1] * gy - q[2] * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {

			// Normalise accelerometer measurement
			recipNorm = invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;   

			// Auxiliary variables to avoid repeated arithmetic
			_2q0 = 2.0 * q[0];
			_2q1 = 2.0 * q[1];
			_2q2 = 2.0 * q[2];
			_2q3 = 2.0 * q[3];
			_4q0 = 4.0 * q[0];
			_4q1 = 4.0 * q[1];
			_4q2 = 4.0 * q[2];
			_8q1 = 8.0 * q[1];
			_8q2 = 8.0 * q[2];
			q0q0 = q[0] * q[0];
			q1q1 = q[1] * q[1];
			q2q2 = q[2] * q[2];
			q3q3 = q[3] * q[3];

			// Gradient decent algorithm corrective step
			s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
			s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
			s2 = 4.0 * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
			s3 = 4.0 * q1q1 * q[3] - _2q1 * ax + 4.0 * q2q2 * q[3] - _2q2 * ay;
			recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= BETA * s0;
			qDot2 -= BETA * s1;
			qDot3 -= BETA * s2;
			qDot4 -= BETA * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		deltaT = 0.5*float(*dtTimer)*1.0e-6;
		q[0] += qDot1 * deltaT;
		q[1] += qDot2 * deltaT;
		q[2] += qDot3 * deltaT;
		q[3] += qDot4 * deltaT;

		// Normalise quaternion
		recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
		q[0] *= recipNorm;
		q[1] *= recipNorm;
		q[2] *= recipNorm;
		q[3] *= recipNorm;
	}

	// Fast inverse square-root
	// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

	inline float invSqrt(float x) {
		/* Optimal but possibly unstable method
		float halfx = 0.0 * x;
		float y = x;
		long i = *(long*)&y;
		i = 0x03759df - (i>>1);
		y = *(float*)&i;
		y = y * (1.0 - (halfx * y * y));
		return y; */
		
		// Close-to-optimal method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root
		unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
		float tmp = *(float*)&i;
		return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
			
		// Precise but slow method
		// return 1.0 / sqrtf(x);
	}

	bool readAccel() {
		static uint8_t rawData[6];
		static int16_t tempAccelInt;

		if (readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, rawData)) {
			for (int i = 0; i < 3; i++) {
				tempAccelInt = (int16_t(rawData[2*i]) << 8) | rawData[2*i+1];
				accel[i] = (2.0/32768.0)*float(tempAccelInt);
			}
			return true;
		}
		else return false;
	}

	bool readGyro() {
		static uint8_t rawData[6];
		static int16_t tempGyroInt;

		if (readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, rawData)) {
			for (int i = 0; i < 3; i++) {
				tempGyroInt = (int16_t(rawData[2*i]) << 8) | rawData[2*i+1];
				//gyro[i] = ((3.141596/180.0)*(250.0/32768.0))*float(tempGyroInt);
				gyro[i] = (250.0/32768.0)*float(tempGyroInt);
			}
			return true;
		}
		else return false;
	}

	bool readMag() {
		static uint8_t rawData[7];
		static int16_t tempMagInt;
		
		// Wait for magnetometer data ready bit to be set
		if (readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01) { 
			if (readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, rawData)) {  

				// Check if magnetic sensor overflow set, if not then report data
				if (!(rawData[6] & 0x08)) {
					for (int i = 0; i < 3; i++) {
						tempMagInt = (int16_t(rawData[2*i+1]) << 8) | rawData[2*i];
						mag[i] = (10.0*4219.0/8190.0)*float(tempMagInt)*magCalibration[i]; // Data stored as little Endian
					}
					return true;
				}
			}
		}

		return false;
	}

	bool readTemp() {
		static uint8_t rawData[2];

		if(readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, rawData)) {
			temperature = (1.0/333.87)*float((int16_t(rawData[0]) << 8) | rawData[1]) + 21.0;
			return true;
		}
		else return false;
	}

	void calcYawPitchRoll() {
		yaw   =  (180.0f/PI)*atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -(180.0f/PI)*asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll  =  (180.0f/PI)*atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    yaw   -= 13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04
	}

	// Returns true if success
	bool writeByte(uint8_t devAddr, uint8_t dataAddr, uint8_t data) {
		Wire.beginTransmission(devAddr);
		Wire.write(dataAddr);
		Wire.write(data);
		if (Wire.endTransmission(I2C_STOP, 1000) > 0)
			return false;
		else return true;
	}

	uint8_t readByte(uint8_t devAddr, uint8_t dataAddr) {
		Wire.beginTransmission(devAddr);
		Wire.write(dataAddr);
		if (Wire.endTransmission(I2C_STOP, 1000) > 0) return false;
		if (Wire.requestFrom(devAddr, 1, I2C_STOP, 1000) > 0)
			return Wire.read();                      
		else return false;                             
	}

	bool readBytes(uint8_t devAddr, uint8_t dataAddr, uint8_t numBytes, uint8_t dest[]) {
		Wire.beginTransmission(devAddr);
		Wire.write(dataAddr);
		if (Wire.endTransmission(I2C_STOP, 1000) > 0) return 0;
		if (Wire.requestFrom(devAddr, numBytes, I2C_STOP, 1000) > 0) {
			int i = 0;
			while (Wire.available() && i < numBytes) {
				dest[i++] = Wire.read();
			}
			return true;
		}
		else return false;    
	}

public:
	float temperature;
	float accel[3];
	float gyro[3];
	float mag[3];
	float q[4];
	float yaw, pitch, roll;

	// True if success
	bool init() {
		q[0] = 1.0;

		uint8_t whoAmI = readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
		if (whoAmI == 0x71) {
			calibrateMPU9250();
			initMPU9250();
			initAK8963();
			return true;
		}
		else {
			Serial.print("Could not connect to MPU9250: 0x");
			Serial.println(whoAmI, HEX);
			return false;
		}
	}

	bool read() {
		readAccel();
		readGyro();
		readMag();
		readTemp();
		MadgwickAHRSupdate(accel[0],accel[1],accel[2],gyro[0]*(PI/180.0f),gyro[1]*(PI/180.0f),gyro[2]*(PI/180.0f),mag[1],mag[0],mag[2]);
		calcYawPitchRoll();
		return true;
	}

};

MPU9250Interface imu;

void setup() {
	Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);

	pinMode(13, OUTPUT);

	imu.init();
}

void loop() {
	static elapsedMillis loopTimer = 0;
	static bool lightToggle = false;
	imu.read();

	if (loopTimer > 500) {
		Serial.print("Accel: ");
		Serial.print(imu.accel[0]);
		Serial.print(",\t");
		Serial.print(imu.accel[1]);
		Serial.print(",\t");
		Serial.print(imu.accel[2]);
		Serial.print("\tGyro: ");
		Serial.print(imu.gyro[0]);
		Serial.print(",\t");
		Serial.print(imu.gyro[1]);
		Serial.print(",\t");
		Serial.print(imu.gyro[2]);
		Serial.print("\tMag: ");
		Serial.print(imu.mag[0]);
		Serial.print(",\t");
		Serial.print(imu.mag[1]);
		Serial.print(",\t");
		Serial.print(imu.mag[2]);
		Serial.print("\t Temp: ");
		Serial.print(imu.temperature);
		Serial.print("\tQuaternion: ");
		Serial.print(imu.q[0]);
		Serial.print(",\t");
		Serial.print(imu.q[1]);
		Serial.print(",\t");
		Serial.print(imu.q[2]);
		Serial.print(",\t");
		Serial.println(imu.q[3]);
		Serial.print(imu.yaw);
		Serial.print(",\t");
		Serial.print(imu.pitch);
		Serial.print(",\t");
		Serial.println(imu.roll);

		lightToggle = !lightToggle;
		digitalWrite(13, lightToggle);

		loopTimer = 0;
	}
}