#include "opt3101.h"
#include "i2c.h"
#include <string.h>

int8_t i2cSensorsRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr) {
	HAL_StatusTypeDef status;
	uint16_t DevAddress = *(uint8_t*)intfPtr << 1;
	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, &regAddr, 1, 1000);
	status = HAL_I2C_Master_Receive(&hi2c1, DevAddress, regData, len, 1000);
	// osSemaphoreAcquire(sensorI2C.i2cRxDmaSemaphore, osWaitForever);
	/**
	 * HAL_StatusTypeDef: 0, 1, 2, 3
	 * BMI08X_INTF_RET_TYPE: 0, -1, -2, ..., -9
	 */
	return -status;
}

/*! @brief Sensor I2C write function */
int8_t i2cSensorsWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr) {
	static uint8_t sBuffer[32];
	HAL_StatusTypeDef status;
	uint16_t DevAddress = *(uint8_t*)intfPtr << 1;
	memset(sBuffer, 0, 32);
	sBuffer[0] = regAddr;
	memcpy(sBuffer + 1, regData, len);
	status = HAL_I2C_Master_Transmit(&hi2c1, DevAddress, sBuffer, len + 1, 1000);
	return -status;
}

int8_t opt3101ResetAndWait(opt3101_dev* dev) {
	int8_t rslt = 0;
	dev->timingGeneratorEnabled = false;

	// Set SOFTWARE_RESET to 1, but don't use writeReg, because the OPT3101 will
	// stop acknowledging after it receives the first register value byte.
	rslt = dev->write(0, NULL, 0, dev->intf_ptr);
	rslt |= dev->write(1, NULL, 0, dev->intf_ptr);

	if (rslt) { return rslt; }


	// Wait for INIT_LOAD_DONE to be set, indicating that the OPT3101 is done
	// loading settings from its EEPROM.
	uint32_t data;
	do {
		HAL_Delay(5);
		rslt |= opt3101ReadReg(dev, 3, &data);
		if (rslt) return rslt;
	} while (!(data & (1 << 8)));

	return rslt;
}

int8_t opt3101ReadReg(opt3101_dev* dev, uint8_t reg, uint32_t* value) {
	int8_t rslt = 0;
	uint8_t data[3] = { 0 };

	if ((rslt = dev->read(reg, data, 3, dev->intf_ptr)) != 0) {
		return rslt;
	}

	*value = data[0];
	*value |= (uint16_t)data[1] << 8;
	*value |= (uint32_t)data[2] << 16;

	return rslt;
}

int8_t opt3101WriteReg(opt3101_dev* dev, uint8_t reg, uint32_t value) {
	uint8_t data[3] = { value & 0xff, (value >> 8) & 0xff, (value >> 16) & 0xff };
	return dev->write(reg, data, 3, dev->intf_ptr);
}

int8_t opt3101Init(opt3101_dev* dev) {
	int8_t rslt = 0;
	rslt |= opt3101ResetAndWait(dev);
	rslt |= opt3101ConfigureDefault(dev);

	return rslt;
}

int8_t opt3101ConfigureDefault(opt3101_dev* dev) {
	int8_t rslt = 0;
	rslt |= opt3101WriteReg(dev, 0x89, 7000); // TG_OVL_WINDOW_START = 7000
	rslt |= opt3101WriteReg(dev, 0x6e, 0x0a0000); // EN_TEMP_CONV = 1
	// CLIP_MODE_FC = 1
	// CLIP_MODE_TEMP = 0
	// CLIP_MODE_OFFSET = 0
	rslt |= opt3101WriteReg(dev, 0x50, 0x200101);
	

	// IQ_READ_DATA_SEL = 2: This lets us read "raw" IQ values later.
	uint32_t reg2e;
	rslt |= opt3101ReadReg(dev, 0x2e, &reg2e);

	reg2e = (reg2e & ~((uint32_t)7 << 9)) | (2 << 9);
	rslt |= opt3101WriteReg(dev, 0x2e, reg2e);
	
	rslt |= opt3101SetMonoshotMode(dev);
	rslt |= opt3101SetFrameTiming(dev, 512);
	return rslt;
}

int8_t opt3101SetMonoshotMode(opt3101_dev* dev) {
	int8_t rslt = 0;
	// MONOSHOT_FZ_CLKCNT = default
	// MONOSHOT_NUMFRAME = 1
	// MONOSHOT_MODE = 3
	rslt |= opt3101WriteReg(dev, 0x27, 0x26ac07);

	// DIS_GLB_PD_OSC = 1
	// DIS_GLB_PD_AMB_DAC = 1
	// DIS_GLB_PD_REFSYS = 1
	// (other fields default)
	rslt |= opt3101WriteReg(dev, 0x76, 0x000121);

	// POWERUP_DELAY = 95
	rslt |= opt3101WriteReg(dev, 0x26, (uint32_t)95 << 10 | 0xF);
	return rslt;
}

int8_t opt3101SetContinuousMode(opt3101_dev* dev) {
	// MONOSHOT_FZ_CLKCNT = default
	// MONOSHOT_NUMFRAME = 1
	// MONOSHOT_MODE = 0
	return opt3101WriteReg(dev, 0x27, 0x26ac04);
}

int8_t opt3101SetFrameTiming(opt3101_dev* dev, uint16_t subFrameCount) {
	int8_t rslt = 0;
	// Make sure subFrameCount is a power of 2 between 1 and 4096.
	if (subFrameCount < 1 || subFrameCount > 4096 ||
	subFrameCount & (subFrameCount - 1)) {
	subFrameCount = 4096;
	}

	// Implement equation 6 from sbau310.pdf to calculate
	// XTALK_FILT_TIME CONST, but without floating-point operations.
	uint8_t timeConst = 0;
	while ((subFrameCount << timeConst) < 1024) { timeConst++; }

	uint32_t reg2e;
	rslt |= opt3101ReadReg(dev, 0x2e, &reg2e);
	reg2e = (reg2e & ~(uint32_t)0xF00000) | (uint32_t)timeConst << 20;
	rslt |= opt3101WriteReg(dev, 0x2e, reg2e);

	// Set NUM_SUB_FRAMES and NUM_AVG_SUB_FRAMES.
	rslt |= opt3101WriteReg(dev, 0x9f, (subFrameCount - 1) | (uint32_t)(subFrameCount - 1) << 12);

	// Set TG_SEQ_INT_MASK_START and TG_SEQ_INT_MASK_END according to what
	// the OPT3101 datasheet says, but it's probably not needed.
	rslt |= opt3101WriteReg(dev, 0x97, (subFrameCount - 1) | (uint32_t)(subFrameCount - 1) << 12);

	// Assuming that SUB_VD_CLK_CNT has not been changed, each sub-frame is
	// 0.25 ms.  The +3 is to make sure we round up.
	uint16_t frameTimeMs = (subFrameCount + 3) / 4;

	// Add a ~6% margin in case the OPT3101 clock is running faster.
	dev->frameDelayTimeMs = frameTimeMs + (frameTimeMs + 15) / 16;

	return rslt;
}

int8_t opt3101StartSample(opt3101_dev* dev) {
	int8_t rslt = 0;
	if (!dev->timingGeneratorEnabled)
		opt3101EnableTimingGenerator(dev);
	// Set MONOSHOT_BIT to 0 before setting it to 1, as recommended here:
	// https://e2e.ti.com/support/sensors/f/1023/p/756598/2825649#2825649
	rslt |= opt3101WriteReg(dev, 0x00, 0x000000);
	rslt |= opt3101WriteReg(dev, 0x00, 0x800000);
	dev->startSampleTimeMs = dev->millis();
	return rslt;
}

void opt3101EnableTimingGenerator(opt3101_dev* dev) {
	opt3101WriteReg(dev, 0x80, REG80_DEFAULT | 1); // TG_EN = 1
	dev->timingGeneratorEnabled = true;
}

void opt3101DisableTimingGenerator(opt3101_dev* dev) {
  opt3101WriteReg(dev, 0x80, REG80_DEFAULT);  // TG_EN = 0
  dev->timingGeneratorEnabled = false;
}

bool opt3101IsSampleDone(opt3101_dev* dev) {
  return (uint16_t)(dev->millis() - dev->startSampleTimeMs) > dev->frameDelayTimeMs;
}

int8_t opt3101ReadOutputRegs(opt3101_dev* dev) {
	int8_t rslt = 0;
	uint32_t reg08, reg09, reg0a;
	rslt |= opt3101ReadReg(dev, 0x08, &reg08);
	rslt |= opt3101ReadReg(dev, 0x09, &reg09);
	rslt |= opt3101ReadReg(dev, 0x0a, &reg0a);

	dev->channelUsed = reg08 >> 18 & 3;
	if (dev->channelUsed > 2) dev->channelUsed = 2;
	dev->brightnessUsed = reg08 >> 17 & 1;

	// i = readReg(0x3b);
	// if (getLastError()) { return; }
	// if (i > 0x7fffff) { i -= 0x1000000; }
	// q = readReg(0x3c);
	// if (getLastError()) { return; }
	// if (q > 0x7fffff) { q -= 0x1000000; }
	dev->amplitude = reg09 & 0xFFFF; // AMP_OUT
	dev->phase = reg08 & 0xFFFF; // PHASE_OUT
	// c / (2 * 10 MHz * 0x10000) = 0.22872349395 mm ~= 14990/0x10000
	dev->distanceMillimeters = (int32_t)dev->phase * 14990 >> 16;
	dev->ambient = reg0a >> 2 & 0x3FF; // AMB_DATA
	dev->temperature = reg0a >> 12 & 0xFFF; // TMAIN

	return rslt;
}

int8_t opt3101Sample(opt3101_dev* dev) {
	int8_t rslt = 0;
	rslt |= opt3101StartSample(dev);
	dev->delay(dev->frameDelayTimeMs);
	rslt |= opt3101ReadOutputRegs(dev);

	return rslt;
}

int8_t opt3101SetChannel(opt3101_dev* dev, uint8_t channel) {
	int8_t rslt = 0;
	if (channel > 2) channel = OPT3101_CHANNEL_AUTO;

	uint32_t reg2a;
	rslt |= opt3101ReadReg(dev, 0x2a, &reg2a);

	if (channel == OPT3101_CHANNEL_AUTO)
		reg2a |= (1 << 0); // EN_TX_SWITCH = 1
	else {
		reg2a &= ~((uint32_t)1 << 0); // EN_TX_SWITCH = 0
		reg2a = (reg2a & ~((uint32_t)3 << 1)) | (channel & 3) << 1;
	}

	rslt |= opt3101WriteReg(dev, 0x2a, reg2a);
	return rslt;
}

int8_t opt3101NextChannel(opt3101_dev* dev) {
	return opt3101SetChannel(dev, (dev->channelUsed + 1) % 3);
}

int8_t opt3101SetBrightness(opt3101_dev* dev, OPT3101Brightness br) {
	int8_t rslt = 0;
	uint32_t reg2a;
	rslt |= opt3101ReadReg(dev, 0x2a, &reg2a);

	if (br == BR_ADAPTIVE)
	    reg2a |= (uint16_t)1 << 15;  // EN_ADAPTIVE_HDR = 1
	else {
		// EN_ADAPTIVE_HDR = 0
		// SEL_HDR_MODE = hdr
		reg2a = (reg2a & ~(uint32_t)0x18000) | ((uint32_t)br & 1) << 16;
	}

	rslt |= opt3101WriteReg(dev, 0x2a, reg2a);
	return rslt;
}

int8_t opt3101EnableDataReadyOutput(opt3101_dev* dev, uint8_t gpPin) {
	int8_t rslt = 0;
	if (gpPin < 1) gpPin = 1;
	if (gpPin > 2) gpPin = 2;

	// DIG_GPO_SEL0 = 9 (DATA_RDY)
	uint32_t reg0b;
	rslt |= opt3101ReadReg(dev, 0x0b, &reg0b);
	reg0b = (reg0b & ~(uint32_t)0xF) | 9;
  
	rslt |= opt3101WriteReg(dev, 0x0b, reg0b);

	uint32_t reg78;
	rslt |= opt3101ReadReg(dev, 0x78, &reg78);
	switch (gpPin) {
		case 1:
		// GPO1_MUX_SEL = 2 (DIG_GPO_0)
		// GPIO1_OBUF_EN = 1
		reg78 = (reg78 & ~((uint32_t)7 << 6)) | (2 << 6) | (1 << 12);
		case 2:
		// GPO2_MUX_SEL = 2 (DIG_GPO_0)
		// GPIO2_OBUF_EN = 1
		reg78 = (reg78 & ~((uint32_t)7 << 9)) | (2 << 9) | ((uint16_t)1 << 15);
	}
	rslt |= opt3101WriteReg(dev, 0x78, reg78);
	return rslt;
}