
#ifndef ARDUINO_USB_MODE
#error This ESP32 SoC has no Native USB interface
#elif ARDUINO_USB_MODE == 1
#warning This sketch should be used when USB is in OTG mode
void setup() {}
void loop() {}
#else
#include "USB.h"
#include "USBHID.h"
#include "USBVendor.h"

#include "DAP_config.h"
#include "DAP.h"

static uint8_t TxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];
static uint8_t RxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];

// https://github.com/raspberrypi/debugprobe/blob/master/src/usb_descriptors.c#L91
static const uint8_t report_descriptor[] = {
  TUD_HID_REPORT_DESC_GENERIC_INOUT(CFG_TUD_HID_EP_BUFSIZE)
};

USBCDC USBSerial; // Provides USB reflash
USBVendor Vendor; // CMSIS-DAPv2
USBHID HID;

// Plain CMSIS-DAP
class CustomHIDDevice : public USBHIDDevice {
public:
  //uint8_t TxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];
  //uint8_t RxDataBuffer[CFG_TUD_HID_EP_BUFSIZE];

  CustomHIDDevice(void) {
    static bool initialized = false;
    if (!initialized) {
      initialized = true;
      HID.addDevice(this, sizeof(report_descriptor));
    }
  }

  void begin(void) {
    HID.begin();
  }

  uint16_t _onGetDescriptor(uint8_t *buffer) {
    memcpy(buffer, report_descriptor, sizeof(report_descriptor));
    return sizeof(report_descriptor);
  }

  bool send(uint8_t *value) {
    return HID.SendReport(0, value, 8);
  }

  void _onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
    USBSerial.printf("REP\n");
  }

  void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
    uint32_t response_size = TU_MIN(CFG_TUD_HID_EP_BUFSIZE, len);

    // https://github.com/raspberrypi/debugprobe/blob/master/src/main.c#L145
    // https://github.com/myelin/arduino-cmsis-dap/blob/master/arduino-cmsis-dap.ino#L122
    uint32_t sz = DAP_ProcessCommand( (uint8_t*)buffer, TxDataBuffer) & 0xFFFF;

    // Note timeout set to 0
    HID.SendReport(0, TxDataBuffer, sz, 0);
  }
};

CustomHIDDevice Device;

void init_usb() {
  // Change product name so OpenOCD recognizes us
  // https://github.com/myelin/arduino-cmsis-dap/blob/master/arduino-cmsis-dap.ino#L43
  // https://github.com/espressif/arduino-esp32/blob/master/cores/esp32/USB.cpp#L312
  // Warning: USB CDC on boot must be disabled
  USB.productName("ESP32 CMSIS-DAP"); 

  Device.begin();
  Vendor.begin();
  USBSerial.begin();
  USB.begin();
}

void handle_vendor() {
  if (Vendor.available()) {
    Vendor.read(RxDataBuffer, sizeof(RxDataBuffer));

    uint32_t sz = DAP_ProcessCommand(RxDataBuffer, TxDataBuffer) & 0xFFFF;

    Vendor.write(TxDataBuffer, sz);
    Vendor.flush();
  }
}

//
// 3.3V --> ? ohms --> MEASURE --> 2.5 ohms --> GND
//
// ADC range: 0 mV - 750 mV
//

uint8_t adc_pins[] = {1};
uint8_t adc_pins_count = sizeof(adc_pins) / sizeof(uint8_t);

// Calibration's broken
#define ADC_TO_MV (750.0/4096)

double mv_to_ohms(double mv) {
  double y = (3300.0 - mv) / 3300.0;
  double r = -2.5 - 2.5/(y - 1);

  return r;
}

int adc_results_pending = 0;

void IRAM_ATTR adc_callback() {
  adc_results_pending = 1;
}

void handle_adc() {
  adc_continuous_data_t *result = NULL;

  if (!adc_results_pending)
    return;

  if (analogContinuousRead(&result, 0)) {
    for (int i = 0; i < adc_pins_count; i++) {
      double ohms = mv_to_ohms(result[i].avg_read_raw * ADC_TO_MV);
      int milliamps = int(3300.0 / ohms + 0.5);

      USBSerial.printf("\nADC PIN %d data:"
                       "\n   Avg raw value = %d"
                       "\n   Avg millivolts (BROKEN) value = %d"
                       "\n   Avg millivolts (FIX) value = %d"
                       "\n   ohms = %lf"
                       "\n     mA = %d", result[i].pin, result[i].avg_read_raw, result[i].avg_read_mvolts, int(result[i].avg_read_raw * ADC_TO_MV + 0.5), ohms, milliamps);

      USBSerial.flush();
    }

    adc_results_pending = 0;
  }
}

void init_adc() {
  analogContinuousSetAtten(ADC_0db);

  uint32_t conversions_per_pin = 2000;
  uint32_t samplerate = conversions_per_pin * 10;

  // https://github.com/espressif/arduino-esp32/blob/master/docs/en/api/adc.rst
  analogContinuous( adc_pins, adc_pins_count, conversions_per_pin, samplerate, &adc_callback );
  analogContinuousStart();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);

  init_usb();
  init_adc();
}

void loop() {
  handle_vendor();
  handle_adc();

  // A small powersaving measure
  // Tested to be effective
  //
  // ESP-IDF does this when all FreeRTOS tasks are blocked... which is never?
  // Not unless we do a delay(1) in the main loop
  //
  // We can escape the latency of delay(1) and still have some power savings if we do a waiti manually
  //
  cpu_ll_waiti();
}
#endif /* ARDUINO_USB_MODE */
