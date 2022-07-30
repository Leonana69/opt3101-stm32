#ifndef __OPT3101_H__
#define __OPT3101_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum {
  BR_LOW = 0,
  BR_HIGH = 1,
  BR_ADAPTIVE = 255
} OPT3101Brightness;

#define OPT3101_CHANNEL_AUTO 255
#define REG80_DEFAULT 0x4e1e
#define OPT3101_CHIP_ADDR 0x58

typedef int8_t (*opt3101_read_fptr_t)(uint8_t reg_addr, uint8_t *read_data, uint32_t len, void *intf_ptr);
typedef int8_t (*opt3101_write_fptr_t)(uint8_t reg_addr, const uint8_t *read_data, uint32_t len, void *intf_ptr);
typedef uint16_t (*opt3101_timestamp_fptr_t) ();
typedef void (*opt3101_delay_ms) (uint32_t);

typedef struct _opt3101_dev {
  uint8_t channelUsed;
  uint8_t brightnessUsed;
  uint16_t ambient;
  uint16_t temperature;
  // int32_t i, q;
  uint16_t amplitude;
  int16_t phase;
  int16_t distanceMillimeters;

  bool timingGeneratorEnabled;
  uint16_t startSampleTimeMs;
  uint16_t frameDelayTimeMs;

  void *intf_ptr;
  opt3101_read_fptr_t read;
  opt3101_write_fptr_t write;
  opt3101_timestamp_fptr_t millis;
  opt3101_delay_ms delay;
} opt3101_dev;


int8_t opt3101WriteReg(opt3101_dev* dev, uint8_t reg, uint32_t value);
int8_t opt3101ReadReg(opt3101_dev* dev, uint8_t reg, uint32_t* value);

int8_t opt3101Init(opt3101_dev* dev);
int8_t opt3101ResetAndWait(opt3101_dev* dev);
int8_t opt3101ConfigureDefault(opt3101_dev* dev);
int8_t opt3101SetChannel(opt3101_dev* dev, uint8_t channel);
int8_t opt3101NextChannel(opt3101_dev* dev);
int8_t opt3101SetBrightness(opt3101_dev* dev, OPT3101Brightness br);
int8_t opt3101SetMonoshotMode(opt3101_dev* dev);
int8_t opt3101SetContinuousMode(opt3101_dev* dev);
int8_t opt3101SetFrameTiming(opt3101_dev* dev, uint16_t subFrameCount);
void opt3101EnableTimingGenerator(opt3101_dev* dev);
void opt3101DisableTimingGenerator(opt3101_dev* dev);
int8_t opt3101EnableDataReadyOutput(opt3101_dev* dev, uint8_t gpPin);
int8_t opt3101StartSample(opt3101_dev* dev);
bool opt3101IsSampleDone(opt3101_dev* dev);
int8_t opt3101ReadOutputRegs(opt3101_dev* dev);
int8_t opt3101Sample(opt3101_dev* dev);


int8_t i2cSensorsRead(uint8_t regAddr, uint8_t *regData, uint32_t len, void *intfPtr);
int8_t i2cSensorsWrite(uint8_t regAddr, const uint8_t *regData, uint32_t len, void *intfPtr);

#ifdef __cplusplus
}
#endif

#endif /* __OPT3101_H__ */