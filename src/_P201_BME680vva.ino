//#######################################################################################################
//#################### Plugin 201 BME680 I2C Temp/Hum/Barometric/Pressure/Gas Resistence Sensor  ########
//#######################################################################################################

//https://www.hackster.io/xxlukas84/bme680-gy-mcu680v1-indoor-air-quality-meter-901d02
//https://github.com/op0xA5/GYMCU680
//https://github.com/shimosaurus/mcu680

/*******************************************************************************
 * Copyright 2017
 * Written by Rossen Tchobanski (rosko@rosko.net)
 * BSD license, all text above must be included in any redistribution
 *
 * Release notes:
   Adafruit_BME680 Library v1.0.5 required (https://github.com/adafruit/Adafruit_BME680/tree/1.0.5)
/******************************************************************************/


//#ifdef PLUGIN_BUILD_DEV
//#ifdef PLUGIN_BUILD_TESTING

#include <ESPeasySerial.h>
#include "_Plugin_Helper.h"


#define PLUGIN_201
#define PLUGIN_ID_201         201
#define PLUGIN_NAME_201       "Environment - BME680vva"
#define PLUGIN_VALUENAME1_201 "Temperature"
#define PLUGIN_VALUENAME2_201 "Humidity"
#define PLUGIN_VALUENAME3_201 "Pressure"
#define PLUGIN_VALUENAME4_201 "IAQ"
#define PLUGIN_VALUENAME5_201 "Gas"
#define PLUGIN_VALUENAME6_201 "Altitude"
#define PLUGIN_VALUENAME7_201 "Pressure_kPa"


// Optional - you can define air pressure correction for your altitude
#define ALT_CORRECTION 3144

#define flagDataTemperature  0x01
#define flagDataHumidity     0x02
#define flagDataPressure     0x04
#define flagDataIAQ          0x08
#define flagDataGas          0x10
#define flagDataAltitude     0x20


struct P201_data_struct : public PluginTaskData_base {
  ESPeasySerial *easySerial = nullptr;
  bool portInit = false;
  int16_t _serial_rx; 
  int16_t _serial_tx;

  P201_data_struct() {
    reset();
  }

  ~P201_data_struct() { reset(); }

  void reset() {
    portInit = false;
    if (easySerial != nullptr) {
      delete easySerial;
      easySerial = nullptr;
    }
  }

  bool init(const int16_t serial_rx, const int16_t serial_tx) {
    _serial_rx = serial_rx;
    _serial_tx = serial_tx;

    if (serial_rx < 0 || serial_tx < 0)
      return false;
    reset();
    easySerial = new ESPeasySerial(serial_rx, serial_tx);
    easySerial->begin(9600);

    if (!easySerial->listen()){
      addLog(LOG_LEVEL_INFO, String("BME680vva: listen fail: "));
      reset();
      return false;
    }
    delay(5000);    
  
    if (4!=(easySerial->write(0XA5) + easySerial->write(0X55) + easySerial->write(0X3F) + easySerial->write(0X39))){
      addLog(LOG_LEVEL_INFO, String("BME680vva: send1 fail: "));
      reset();
      return false;
    }
    delay(300); 

    if (4!=(easySerial->write(0XA5) + easySerial->write(0X56) + easySerial->write(0X02) + easySerial->write(0XFD))){
      addLog(LOG_LEVEL_INFO, String("BME680vva: send2 fail: "));
      reset();
      return false;
    }
    delay(300); 

    addLog(LOG_LEVEL_INFO, F("BME680vva: finish initialize "));
    portInit = true;
    return true;
  }

  bool readData(byte &mask, float &Temperature, float &Humidity, float &Pressure_kPa, float &altPressure, uint32_t &Gas, uint16_t &IAQ, int16_t &Altitude){
    byte mhzResp[20];    // 9 byte response buffer

    memset(mhzResp, 0, sizeof(mhzResp));

    long timer = millis() + 300;
    uint counter = 0;
    while (!timeOutReached(timer) && (counter < sizeof(mhzResp))) {
      if (easySerial->available() > 0) {
        byte value = easySerial->read();
        // start 0x5A
        if ((counter==0 || counter==1) && value!=0x5A){
          counter = 0;
          continue;
        }
        mhzResp[counter++] = value;
      } else {
        delay(10);
      }
    }

    String log = String("readData ") + counter + String(" bytes, rx:")+_serial_rx+String(",tx:")+_serial_tx;
    addLog(LOG_LEVEL_INFO, log);

    if (counter==20){
      // check 2 first bytes
      if(mhzResp[0]==0x5A&&mhzResp[1]==0x5A){

        // check checksum
        unsigned char i=0,sum=0;
        for(i=0;i<19;i++){
          sum+=mhzResp[i]; 
        }

        //checksum rights
        if(sum==mhzResp[19] ) {
          uint16_t temp1=0;
          int16_t temp2=0;

          mask = mhzResp[3];

          temp2=(mhzResp[4]<<8|mhzResp[5]);   
          Temperature=(float)temp2/100.0;
          temp1=(mhzResp[6]<<8|mhzResp[7]);
          Humidity=(float)temp1/100.0; 
          uint32_t tmpPressure=((uint32_t)mhzResp[8]<<16)|((uint32_t)mhzResp[9]<<8)|(uint32_t)mhzResp[10];

          // uint8_t IAQ_accuracy= (mhzResp[11]&0xf0)>>4;
          IAQ=((mhzResp[11]&0x0F)<<8)|mhzResp[12];

          Gas=((uint32_t)mhzResp[13]<<24)|((uint32_t)mhzResp[14]<<16)|((uint16_t)mhzResp[15]<<8)|mhzResp[16];
          Altitude=(mhzResp[17]<<8)|mhzResp[18]; 

//          Pressure_kPa += ((float)tmpPressure + (float)Altitude*10.0)/100.0;
          Pressure_kPa += ((float)tmpPressure)/100.0;

          altPressure =((float)tmpPressure)/133.3223684;

          return true;
        }
      }else{
        addLog(LOG_LEVEL_INFO, "FAIL");
      }
    }

    return false;
  }
};

boolean Plugin_201(byte function, struct EventStruct *event, String& string)
{
  boolean success = false;

  switch (function)
  {
    case PLUGIN_DEVICE_ADD:
      {
        Device[++deviceCount].Number = PLUGIN_ID_201;
        Device[deviceCount].Type = DEVICE_TYPE_SERIAL;
        Device[deviceCount].VType = SENSOR_TYPE_QUAD;
        Device[deviceCount].Ports = 0;
        Device[deviceCount].PullUpOption = false;
        Device[deviceCount].InverseLogicOption = false;
        Device[deviceCount].FormulaOption = true;
        Device[deviceCount].ValueCount = 7;
        Device[deviceCount].SendDataOption = true;
        Device[deviceCount].TimerOption = true;
        Device[deviceCount].GlobalSyncOption = true;
        break;
      }

    case PLUGIN_GET_DEVICENAME:
      {
        string = F(PLUGIN_NAME_201);
        break;
      }

    case PLUGIN_GET_DEVICEVALUENAMES:
      {
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[0], PSTR(PLUGIN_VALUENAME1_201));
        //ExtraTaskSettings.TaskDeviceValueDecimals[0] = 2;

        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[1], PSTR(PLUGIN_VALUENAME2_201));
        //ExtraTaskSettings.TaskDeviceValueDecimals[1] = 2;

        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[2], PSTR(PLUGIN_VALUENAME3_201));
        //ExtraTaskSettings.TaskDeviceValueDecimals[2] = 2;

        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[3], PSTR(PLUGIN_VALUENAME4_201));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[4], PSTR(PLUGIN_VALUENAME5_201));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[5], PSTR(PLUGIN_VALUENAME6_201));
        strcpy_P(ExtraTaskSettings.TaskDeviceValueNames[6], PSTR(PLUGIN_VALUENAME7_201));
        //ExtraTaskSettings.TaskDeviceValueDecimals[6] = 2;
        break;
      }

    case PLUGIN_GET_DEVICEGPIONAMES:
      {
        serialHelper_getGpioNames(event);
        break;
      }

    case PLUGIN_WEBFORM_SHOW_CONFIG:
      {
        string += serialHelper_getSerialTypeLabel(event);
        success = true;
        break;
      }

    case PLUGIN_WEBFORM_LOAD:
      {
        serialHelper_webformLoad(event);

        success = true;
        break;
      }

    case PLUGIN_WEBFORM_SAVE:
      {
        P201_data_struct *P201_data =
            static_cast<P201_data_struct *>(getPluginTaskData(event->TaskIndex));
        if (nullptr == P201_data) {
          return success;
        }
        serialHelper_webformSave(event);
        success = true;
        break;
      }

    case PLUGIN_INIT:
      {
        addLog(LOG_LEVEL_INFO, F("BME680vva:  init "));

        initPluginTaskData(event->TaskIndex, new P201_data_struct());
        success = P201_performInit(event);
        break;
      }

    case PLUGIN_EXIT: {
      addLog(LOG_LEVEL_INFO, F("BME680vva: exit "));

      clearPluginTaskData(event->TaskIndex);
      success = true;
      break;
    }

    case PLUGIN_READ:
      {
        P201_data_struct *P201_data =
            static_cast<P201_data_struct *>(getPluginTaskData(event->TaskIndex));
        if (nullptr == P201_data) {
          return success;
        }
        if (P201_data->portInit){
          float Temperature; 
          float Humidity; 
          float Pressure; 
          uint32_t Gas; 
          float Pressure_kPa; 
          uint16_t IAQ;
          int16_t Altitude;
          byte mask;

          if (P201_data->readData(mask, Temperature, Humidity, Pressure_kPa, Pressure, Gas, IAQ, Altitude)){
            if ((mask & flagDataTemperature)!=0){
              UserVar[event->BaseVarIndex + 0] = Temperature;
            }
            if ((mask & flagDataHumidity)!=0){
              UserVar[event->BaseVarIndex + 1] = Humidity;
            }
            if ((mask & flagDataPressure)!=0){
              UserVar[event->BaseVarIndex + 2] = Pressure;
            }
            if ((mask & flagDataIAQ)!=0){
              UserVar[event->BaseVarIndex + 3] = IAQ;
            }
            //if ((mask & flagDataGas)!=0){
              UserVar[event->BaseVarIndex + 4] = Gas;
            //}
            //if ((mask & flagDataAltitude)!=0){
              UserVar[event->BaseVarIndex + 5] = Altitude;
            //}
            UserVar[event->BaseVarIndex + 6] = Pressure_kPa;
            addLog(LOG_LEVEL_INFO, F("BME680vva: read. set new values."));
          }else{
            addLog(LOG_LEVEL_INFO, F("BME680vva: read. there are no new values."));
          }

        }else{
          addLog(LOG_LEVEL_INFO, F("BME680vva: read. no port init... "));
          P201_performInit(event);
        }


        success = true;
        break;
      }
    case PLUGIN_WRITE:
      {
    addLog(LOG_LEVEL_INFO, F("BME680vva: write "));
        P201_data_struct *P201_data =
            static_cast<P201_data_struct *>(getPluginTaskData(event->TaskIndex));
        if (nullptr == P201_data) {
          return success;
        }
        success = true;
        break;
      }

  }
  return success;
}

bool P201_performInit(struct EventStruct *event) {
  bool success = false;
  const int16_t serial_rx = CONFIG_PIN1;
  const int16_t serial_tx = CONFIG_PIN2;
    addLog(LOG_LEVEL_INFO, F("BME680vva: start "));

//  String log_str = String("BME680vva: starting perform init rx:")+serial_rx+String(", tx:")+serial_tx;
//  addLog(LOG_LEVEL_INFO, log_str);
  P201_data_struct *P201_data =
      static_cast<P201_data_struct *>(getPluginTaskData(event->TaskIndex));
  if (nullptr == P201_data) {
    return success;
  }
  if (P201_data->init(serial_rx, serial_tx)) {
    success = true;
    addLog(LOG_LEVEL_INFO, F("BME680vva: Init OK "));

    //delay first read, because hardware needs to initialize on cold boot
    //otherwise we get a weird value or read error
    Scheduler.schedule_task_device_timer(event->TaskIndex, millis() + 15000);
  }
  return success;
}

//#endif
