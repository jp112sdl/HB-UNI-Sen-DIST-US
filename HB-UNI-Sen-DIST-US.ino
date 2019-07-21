//- -----------------------------------------------------------------------------------------------------------------------
// AskSin++
// 2016-10-31 papa Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
// 2018-04-16 jp112sdl Creative Commons - http://creativecommons.org/licenses/by-nc-sa/3.0/de/
//- -----------------------------------------------------------------------------------------------------------------------

// define this to read the device id, serial and device type from bootloader section
// #define USE_OTA_BOOTLOADER

#define EI_NOTEXTERNAL
#include <EnableInterrupt.h>
#include <AskSinPP.h>
#include <LowPower.h>

#include <Register.h>
#include <MultiChannelDevice.h>

// Arduino Pro mini 8 Mhz
// Arduino pin for the config button
#define CONFIG_BUTTON_PIN  8
#define LED_PIN            4

// 1 SENSOR
//                  SENSOR  1
byte SENSOR_EN_PINS[]   =  {5}; //VCC Pin des Sensors
byte SENSOR_ECHO_PINS[] =  {6};
byte SENSOR_TRIG_PINS[] =  {14};

//// 2 SENSOREN
////                  SENSOR  1   2
//byte SENSOR_EN_PINS[]   =  {5,  7}; //VCC Pin des Sensors
//byte SENSOR_ECHO_PINS[] =  {6,  3};
//byte SENSOR_TRIG_PINS[] =  {14, 9};

#define BATT_EN_PIN        15 //A1
#define BATT_SENS_PIN      17 //A3

// number of available peers per channel
#define PEERS_PER_CHANNEL  6

// all library classes are placed in the namespace 'as'
using namespace as;

//Korrekturfaktor der Clock-Ungenauigkeit, wenn keine RTC verwendet wird
#define SYSCLOCK_FACTOR    0.88

enum UltrasonicSensorTypes {
  JSN_SR04T_US100,
  MAXSONAR,
};

// define all device properties
const struct DeviceInfo PROGMEM devinfo = {
  {0xF9, 0xD6, 0x01},          // Device ID
  "JPDIST0001",                // Device Serial
  {0xF9, 0xD6},                // Device Model
  0x10,                        // Firmware Version
  0x53,                        // Device Type
  {0x01, 0x01}                 // Info Bytes
};

/**
   Configure the used hardware
*/
typedef AskSin<StatusLed<LED_PIN>, BatterySensorUni<BATT_SENS_PIN, BATT_EN_PIN, 0>, Radio<AvrSPI<10, 11, 12, 13>, 2>> BaseHal;
class Hal : public BaseHal {
  public:
    void init (const HMID& id) {
      BaseHal::init(id);
      battery.init(seconds2ticks(60UL * 60) * SYSCLOCK_FACTOR, sysclock); //battery measure once an hour
      battery.low(22);
      battery.critical(19);
    }

    bool runready () {
      return sysclock.runready() || BaseHal::runready();
    }
} hal;


DEFREGISTER(UReg0, MASTERID_REGS, DREG_LOWBATLIMIT, 0x20, 0x21)
class UList0 : public RegList0<UReg0> {
  public:
    UList0 (uint16_t addr) : RegList0<UReg0>(addr) {}

    bool Sendeintervall (uint16_t value) const {
      return this->writeRegister(0x20, (value >> 8) & 0xff) && this->writeRegister(0x21, value & 0xff);
    }
    uint16_t Sendeintervall () const {
      return (this->readRegister(0x20, 0) << 8) + this->readRegister(0x21, 0);
    }

    void defaults () {
      clear();
      lowBatLimit(22);
      Sendeintervall(180);
    }
};

DEFREGISTER(UReg1, 0x01, 0x02, 0x03)
class UList1 : public RegList1<UReg1> {
  public:
    UList1 (uint16_t addr) : RegList1<UReg1>(addr) {}
    bool distanceOffset (uint16_t value) const {
      return this->writeRegister(0x01, (value >> 8) & 0xff) && this->writeRegister(0x02, value & 0xff);
    }
    uint16_t distanceOffset () const {
      return (this->readRegister(0x01, 0) << 8) + this->readRegister(0x02, 0);
    }

    bool sensorType (uint16_t value) const {
      return this->writeRegister(0x03, value & 0xff);
    }
    uint16_t sensorType () const {
      return this->readRegister(0x03, 0);
    }

    void defaults () {
      clear();
      distanceOffset(0);
      sensorType(0);
    }
};

class MeasureEventMsg : public Message {
  public:
    void init(uint8_t msgcnt, uint8_t channel, uint16_t dist, uint8_t volt) {
      Message::init(0x0e, msgcnt, 0x53, BIDI | WKMEUP, channel & 0xff, (dist >> 8) & 0xff);
      pload[0] = dist & 0xff;
      pload[1] = volt & 0xff;
    }
};

class MeasureChannel : public Channel<Hal, UList1, EmptyList, List4, PEERS_PER_CHANNEL, UList0>, public Alarm {
    MeasureEventMsg msg;
    uint16_t        distance;
    uint8_t         last_flags = 0xff;

  public:
    MeasureChannel () : Channel(), Alarm(0), distance(0) {}
    virtual ~MeasureChannel () {}

    void measure() {
      uint32_t m_value = 0;
      if (last_flags != flags()) {
        this->changed(true);
        last_flags = flags();
      }

      digitalWrite(SENSOR_EN_PINS[number() - 1], HIGH);
      _delay_ms(300);

      switch (this->getList1().sensorType()) {
        case JSN_SR04T_US100:
          digitalWrite(SENSOR_TRIG_PINS[number() - 1], LOW);
          delayMicroseconds(2);
          digitalWrite(SENSOR_TRIG_PINS[number() - 1], HIGH);
          delayMicroseconds(10);
          digitalWrite(SENSOR_TRIG_PINS[number() - 1], LOW);
          break;
        case MAXSONAR:
          break;
        default:
          DPRINTLN(F("Invalid Sensor Type selected"));
          break;

      }
      
      m_value = pulseIn(SENSOR_ECHO_PINS[number() - 1], HIGH);
      m_value = (m_value * 1000UL / 57874UL);
      digitalWrite(SENSOR_EN_PINS[number() - 1], LOW);

      distance = (m_value > this->getList1().distanceOffset()) ? m_value - this->getList1().distanceOffset() : 0;

      DPRINT(F("MEASURE (")); DDEC(number()); DPRINT(F("): ")); DDEC(m_value); DPRINTLN(F(" cm"));
      DPRINT(F("OFFSET  (")); DDEC(number()); DPRINT(F("): ")); DDEC(this->getList1().distanceOffset()); DPRINTLN(F(" cm"));
      DPRINT(F("DISTANCE(")); DDEC(number()); DPRINT(F("): ")); DDEC(distance); DPRINTLN(F(" cm"));
    }

    virtual void trigger (__attribute__ ((unused)) AlarmClock& clock) {
      measure();
      uint8_t msgcnt = device().nextcount();
      tick = delay();
      msg.init(msgcnt, number(), distance,  device().battery().current());
      if (msgcnt % 20 == 1) device().sendPeerEvent(msg, *this); else device().broadcastEvent(msg, *this);
      sysclock.add(*this);
    }

    uint32_t delay () {
      return seconds2ticks(max(10, device().getList0().Sendeintervall()) * SYSCLOCK_FACTOR);
    }

    void configChanged() {
      DPRINT(F("*DISTANCE_OFFSET (")); DDEC(number()); DPRINT(F("): ")); DDECLN(this->getList1().distanceOffset());
      DPRINT(F("*SENSOR_TYPE     (")); DDEC(number()); DPRINT(F("): ")); DDECLN(this->getList1().sensorType());
    }

    void setup(Device<Hal, UList0>* dev, uint8_t number, uint16_t addr) {
      Channel::setup(dev, number, addr);
      for (byte i = 0; i < sizeof(SENSOR_EN_PINS); i++) {
        pinMode(SENSOR_ECHO_PINS[i], INPUT_PULLUP);
        pinMode(SENSOR_TRIG_PINS[i], OUTPUT);
        pinMode(SENSOR_EN_PINS[i], OUTPUT);
      }
      sysclock.add(*this);
    }

    uint8_t status () const {
      return 0;
    }

    uint8_t flags () const {
      uint8_t flags = this->device().battery().low() ? 0x80 : 0x00;
      return flags;
    }
};

class UType : public MultiChannelDevice<Hal, MeasureChannel, sizeof(SENSOR_EN_PINS), UList0> {
  public:
    typedef MultiChannelDevice<Hal, MeasureChannel, sizeof(SENSOR_EN_PINS), UList0> TSDevice;
    UType(const DeviceInfo& info, uint16_t addr) : TSDevice(info, addr) {}
    virtual ~UType () {}

    virtual void configChanged () {
      TSDevice::configChanged();
      DPRINT(F("*LOW BAT Limit: "));
      DDECLN(this->getList0().lowBatLimit());
      this->battery().low(this->getList0().lowBatLimit());
      DPRINT(F("*Sendeintervall: ")); DDECLN(this->getList0().Sendeintervall());
    }
};

UType sdev(devinfo, 0x20);
ConfigButton<UType> cfgBtn(sdev);

void setup () {
  DINIT(57600, ASKSIN_PLUS_PLUS_IDENTIFIER);
  if (sizeof(SENSOR_EN_PINS) != sizeof(SENSOR_EN_PINS) || sizeof(SENSOR_ECHO_PINS) != sizeof(SENSOR_ECHO_PINS) || sizeof(SENSOR_TRIG_PINS) != sizeof(SENSOR_TRIG_PINS)) {
    DPRINTLN(F("!!! ERROR: SENSOR PIN ARRAYS PRÜFEN"));
  } else {
    sdev.init(hal);
    DDEVINFO(sdev);
    buttonISR(cfgBtn, CONFIG_BUTTON_PIN);
    sdev.initDone();
  }
}

void loop() {
  bool worked = hal.runready();
  bool poll = sdev.pollRadio();
  if ( worked == false && poll == false ) {
    hal.activity.savePower<Sleep<>>(hal);
  }
}


