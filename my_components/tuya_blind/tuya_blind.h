#pragma once

#ifndef TUYA_BLIND_H
#define TUYA_BLIND_H

//#include <Arduino.h>
#include <stdarg.h>
#include <vector>
#include <string>
#include <ctime>
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/version.h"
#include "esphome/core/helpers.h"
#include "esphome/core/util.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"

#if __has_include("esphome/core/macros.h")
   #include "esphome/core/macros.h" // VERSION_CODE
#else
   #define VERSION_CODE(major, minor, patch) ((major) << 16 | (minor) << 8 | (patch))
#endif

//#define TBLIND_PRINT_RAW_PROTO 0  // отладочный вывод протокола в лог
//#define TBLIND_MOTOR_REVERS 1     // настройка реверса мотора, 1 - значение
//#define TBLIND_RESTORE 10000      // восстановление настроек после перезагрузки
//#define TBLIND_VIRTUAL_POS 500    // режим рассчета позиции мотора во время работы, 500 - период обновления в ms
//#define TBLIND_MOTOR_SPEED 0      // скорость мотора
//#define TBLIND_CHILDREN_LOCK      // скорость мотора
//#define TBLIND_ATOM               // режим атом

#define MY_LOGD ESP_LOGD
#define MY_LOGE ESP_LOGE
#define MY_LOGV ESP_LOGV
#define MY_LOGW ESP_LOGW
#define MY_LOGI ESP_LOGI

// типы пакетов
enum tCommand:uint8_t {HEARTBEAT=0,            //- периодический HEARTBEAT 
                       PRODUCT_QUERY=1,        //- идентификатор устройства
                       CONF_QUERY=2,           //- статус наличия ног управления диодом WIFI и ноги с кнопкой 
                       WIFI_STATE=3,           //- статус связи
                       WIFI_RESET=4,           //- ресет
                       WIFI_SELECT=5,          //- pairing mode
                       DATAPOINT_DELIVER = 6,  //- установка данных в MCU
                       DATAPOINT_REPORT = 7,   //- ответ на запрос данных
                       DATAPOINT_QUERY=8,      //- запрос настроек
                       LOCAL_TIME_QUERY=0x1C,  //- синхронизация времени
                       GET_NETWORK_STATUS=0x2B //- подтверждение получения ответа на запрос о статусе сети
};

// типы инфы о состоянии связи
enum tMainState:uint8_t {wsPair=0,      // SmartConfig pairing status
                         wsNoConf=1,    // AP pairing status - устройство не имеет настроек WIFI
                         wsWifiConf=2,  // Wi-Fi has been configured, but not connected to the router - устройство имеет настройки wifi, но не подключено
                         wsWifi=3,      // Wi-Fi has been configured, and connected to the router - успешно подключено и нужен первый набор данных
                         wsWifiCloud=4, // Wi-Fi has been connected to the router and the cloud - нужен второй набор данных
                         wsLowPow=5,    // Wi-Fi device is in the low power consumption mode
                         wsAPSmart=6    // Both SmartConfig and AP pairing status are available
};

// булевы переменные протокола
enum tStat:uint8_t {OFF=0, ON=1, UNDEF=0xFF};

// типы переменных 
enum tDataType:uint8_t {dtBool=1, dtVal=2, dtEnum=4, dtBits=5};

// значение данных
enum tDpid:uint8_t {idControl=1,
                    idCurr_Percent=8,
                    idIssue_Percent=9,
                    idDir=0x0B,
                    idSet_Limits=0x10,
                    idSet_Best_Pos=0x13,
                    idChild_Lock=0x65,
                    idGo_Best_Pos=0x66,
                    idAtom_Mode=0x67,
                    idSpeed=0x68,
};

// состояния режима Control
enum tEnControl:uint8_t {tenOpen=0, tenStop=1, tenClose=2, tenUndef=0xFF};

// состояния скорости мотора
enum tMoSpeed:uint8_t {tmoLow=0, tmoMid=1, tmoHigh=2, tmoUndef=0xFF};

// состояния настройки лимитов
enum tLim:uint8_t {tliOpen=0, tliClose=1, tliReset=2, tliUndef=0xFF};

// структура хранения данных во флеше
struct sBlindSave{
   float speed[3]={0};
};

#ifndef TBLIND_MOTOR_SPEED

 #if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
    #define myStr const char*
 #else
    #define myStr const std::string
 #endif
    myStr _low="Low";
    myStr _mid="Mid";
    myStr _high="High";
 #if ESPHOME_VERSION_CODE >= VERSION_CODE(2025, 11, 0)
    const esphome::FixedVector<const char *> str_speed =
 #else
    std::vector<std::string> str_speed =
 #endif
                                      {_low,   _mid,  _high};
#endif

// Tested device version {"p":"ane3cx9ffd45xdpl","v":"1.0.0","m":2,"mt":3,"n":0} 

namespace esphome {
namespace tuya_blind {

using namespace esphome;
using namespace esphome::text_sensor;
using namespace esphome::cover;
using namespace esphome::switch_;
using namespace esphome::select;

static const char *const TAG = "TuyaBlind";

using cover::CoverCall;
using cover::CoverOperation;
using text_sensor::TextSensor;    
using uart::UARTDevice;  
using uart::UARTComponent;
using select::Select;  

// настройки таймаутов
constexpr uint32_t HEARTBEAT_INTERVAL = 15; // переодичность передачи сигнала активности процессору (sec)
constexpr uint32_t UART_TIMEOUT = 300; //время ожидания uart (mS), пока это время не истечет после приема или отправки - ждем
constexpr uint32_t SEND_TIMEOUT = 300; //время ожидания между сеансами отправки данных (mS)
constexpr uint32_t CALIBRATE_READ_TIME = 4000; // период определения установки лимитов 4 сек
constexpr uint32_t RE_READ_DATAPONT_TIME = 40000; // задержка повторного чтения параметров 40 сек

// настройка корректировки расчета скорости
constexpr uint8_t FACTOR_SPEED_MISS = 10; // минимальный процент не совпадения старой и новой рассчетной скорости, при котором новая скорость замещает старую
constexpr float SPEED_FILTER_COEF = 0.1; // коефицитент ползущего фильтра при усреднении измеренной скорости

class TuyaBlind;

//////// контролы /////////////
class TuyaBlind_Button : public button::Button , public Component, public esphome::Parented<TuyaBlind> {
 protected:
    void press_action(){
        this->press_callback_.call();
    }
 friend class TuyaBlind;   
};

class TuyaBlind_Switch : public switch_::Switch, public Component, public esphome::Parented<TuyaBlind> {
  protected:
    void write_state(bool state) override { 
       if(this->state!=state){
          this->state_callback_.call(state);
          this->publish_state(state); 
       }
     }
  friend class TuyaBlind;   
 };

class TuyaBlind_Select : public select::Select, public Component, public esphome::Parented<TuyaBlind> {
   protected:
      void control(const std::string &value) override {
         if(
  #if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 1, 0)
          this->current_option()!=value
  #else
          this->state!=value
  #endif
         ){
            this->publish_state(value); 
            //this->state_callback_.call(value);
          }
      }
   friend class TuyaBlind;   
};


///////////// основной объект //////////////////////
class TuyaBlind : public cover::Cover, public Component {

 private:
    const uint8_t COMMAND_START[2] = {0x55, 0xAA}; // хидер протокола обмена
    esphome::text_sensor::TextSensor *sensor_mcu_id_{nullptr}; // MCU продукт ID
    // указатель на UART, по которому общаемся со шторамии
    esphome::uart::UARTComponent *_tuya_serial;
    uint8_t receivedCommand[254]; // буфер чтения
    uint8_t commandLength=0; // длинна команды определяется во время получения пакета
    int16_t receiveIndex=0; // количество принятых байт
    uint8_t sendCounter=0; //маршрутизатор отправки
    uint8_t waitReply=0xFF; //буфер крайней отправленной команды, для контроля ответа
    uint32_t lastSend=0; // время отправки последнего байта
    uint32_t lastRead=0; // время получения последнего байта
    bool sendRight=false; //флаг разрешения внеочередной отправки данных, когда идентификатор полученного ответа от терстата совпал с запросом
    uint32_t heatBearTimer=0; // таймер контроля получения ответа на пинг
    uint8_t proto_vers=0; // версия MCU , от этого зависит протокол
    tStat child_lock=UNDEF; // состояние внешней блокировки
    tStat new_child_lock=UNDEF; // новое состояние внешней блокировки
    #ifdef TBLIND_CHILDREN_LOCK
       TuyaBlind_Switch* lock_switch{nullptr}; // свитч режима чилдрен лок
    #endif
    tStat atom_mode=UNDEF; // состояние режима атом
    tStat new_atom_mode=UNDEF; // новое состояние состояние режима атом
    #ifdef TBLIND_ATOM
       TuyaBlind_Switch* atom_switch{nullptr}; // свитч режима atom
    #endif
    // при наличии калибровки при включении приходит сначала позиция 100, а потом реальная в случае 100 приходит 99,
    // если мы получаем текущее положение <100 в первые секунды после включения, то устройство имеет установленные пределы
    bool _no_calibrate=true; // флаг отсутствия калибровки
    bool _cal_up=false;
    bool _cal_down=false;
    uint32_t timerGetCalibrate=0; // таймер таймслота определения калибровки
    bool needReReadParams=true;  // флаг необходимости перезапросить данные устройства
    // для передачи статуса связи
    uint8_t oldNetState=0xFF;
    uint8_t netState=0xFF;
    // нога статуса WIFI
    GPIOPin* status_pin_{nullptr};
    int16_t status_pin_reported_=-1;
    // нога контроля сигнала ресет протокол от MCU
    GPIOPin* reset_pin_{nullptr};
    int16_t reset_pin_reported_=-1;
    // флаги контроля выполнения  
    bool _set_stop=false; // флаг команды останнова
    bool _set_open=false;
    bool _set_close=false;
    #ifdef TBLIND_MOTOR_SPEED
       tMoSpeed new_motor_speed=(tMoSpeed)TBLIND_MOTOR_SPEED; // новая скорость мотора
       tMoSpeed motor_speed=(tMoSpeed)TBLIND_MOTOR_SPEED; //скорость вращения мотора
    #else 
       tMoSpeed new_motor_speed=tmoUndef; // новая скорость мотора
       tMoSpeed motor_speed=tmoUndef; //скорость вращения мотора
       TuyaBlind_Select* speed_select{nullptr}; // select для установки скорости
    #endif
    tLim limit_comm=tliUndef; // команда установки параметров крайних точек
    TuyaBlind_Button* lim_up_button{nullptr}; // кнопка установки крайнего положения открытия
    TuyaBlind_Button* lim_down_button{nullptr}; // кнопка установки крайнего положения закрытия
    TuyaBlind_Button* lim_reset_button{nullptr}; // кнопка сброса крайних положений открытия-закрытия
    uint8_t _set_pos=0xFF; // запрошенная позиция в процентах !!!
    
    #ifdef TBLIND_VIRTUAL_POS
       uint8_t _old_pos=0xFF; // буфер старого положения штор в процентах !!!
       uint8_t _dest_pos=0xFF; // цель позционирования в процентах !!!
       float _speed[3]={0,0,0}; // скорость позиционирования, рассчитывается
       uint32_t _timer_pos=0; // таймер старта позиционирования штор
       uint32_t _timer_update=0; // таймер визуализации позиционирования
       float get_speed(){
          if(motor_speed<3){
             return _speed[motor_speed];
          }
          return 0;
       }
  
     #ifdef TBLIND_RESTORE // режим восстановления скоростей после отключения питания
       
       sBlindSave saveData; 
       uint32_t saveTimer=0; 
       #if ESPHOME_VERSION_CODE > VERSION_CODE(2026, 1, 5)
           ESPPreferenceObject storage = this->make_entity_preference<sBlindSave>();
       #else
           ESPPreferenceObject storage = global_preferences->make_preference<sBlindSave>(this->get_object_id_hash());
       #endif
       
       // проверка, при необходимости сохранение данных
       void saveDataFlash(){
             bool needSave=false;
             for(uint8_t i=0;i<3;i++){
                 if(saveData.speed[i]!=_speed[i] && _speed[i]!=0){
                    saveData.speed[i]=_speed[i];
                    needSave=true;
                 }
             }
             if(needSave){
                if (storage.save(&saveData) && global_preferences->sync()){ // данные успешно сохранены
                   MY_LOGV(TAG, "Data store to flash, speed_0:%f, speed_1:%f, speed_2:%f,", saveData.speed[0],saveData.speed[1],saveData.speed[2]); 
                } else {
                   MY_LOGE(TAG, "Data store to flash - ERROR !");
                }
             }
       }

       // получение данных восстановления из флеша
       inline bool loadDataFlash(){
             if(storage.load(&saveData)){ // читаем сохраненные данные
                for(uint8_t i=0; i<3; i++){
                   if(saveData.speed[i]!=0){
                      _speed[i]=saveData.speed[i];  
                   }
                }
                MY_LOGV(TAG, "Stored data loaded, speed_0:%f, speed_1:%f, speed_2:%f,", _speed[0], _speed[1], _speed[2]); 
                return true;
             } else {
                MY_LOGE(TAG, "Stored data load - ERROR !"); 
             }
          return false;
       }
    
     #endif //TBLIND_RESTORE
       void set_speed(float speed){
          if(motor_speed<3){
             if(_speed[motor_speed]!=speed && speed!=0){
                _speed[motor_speed]=speed;
                #ifdef TBLIND_RESTORE
                   saveDataFlash();
                #endif
             }  
          }
       }
    #endif //TBLIND_VIRTUAL_POS
    
    // считать значение из двух байт INDIAN
    inline uint16_t get16(uint8_t* data){
       uint8_t buff[2]={data[1],data[0]};
       return *((uint16_t*)(&buff));
    }
    
    // считать значение из 4 байт INDIAN
    uint32_t get32(uint8_t* data){
       uint8_t buff[4]={data[3],data[2],data[1],data[0]};
       return *((uint32_t*) (&buff));
    }

    // загрузить значение в буфер в формате indian
    void put32(uint8_t* buff, uint32_t val){
       buff[3]=(uint8_t)val;
       val/=0x100;
       buff[2]=(uint8_t)val;
       val/=0x100;
       buff[1]=(uint8_t)val;
       val/=0x100;
       buff[0]=(uint8_t)val;
    }

    // получить текст из буфера данных
    std::string getStringFromBuff(uint8_t* buff, uint8_t count){
       std::string text="";
       for(uint8_t i=0; i<count; i++){
          text+=(char)buff[i];
       }
       return text;
    }

    void _debugMsg(const std::string &msg, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0, ...) {
        va_list vl;
        va_start(vl, line);
        esp_log_vprintf_(dbgLevel, TAG, line, msg.c_str(), vl);
        va_end(vl);
    }

    // для тестовой печати протокола обмена
    void _debugPrintPacket(uint8_t *packet, uint8_t length, bool dirrect, uint8_t dbgLevel = ESPHOME_LOG_LEVEL_DEBUG, unsigned int line = 0) {
        std::string st = "";
        char textBuf[11];
        // заполняем время получения пакета
        memset(textBuf, 0, 11);
        sprintf(textBuf, "%010u", esphome::millis());
        st = st + textBuf + ": ";
        // формируем преамбулы
        if (dirrect) {
            st += "Out: ";  // преамбула входящего пакета
        } else  {
            st += " In: ";  // преамбула исходящего пакета
        }
        // формируем данные
        for (size_t i = 0; i < length; i++) {
            // для нормальных пакетов надо заключить заголовок в []
            if (i == 0) st += "[";
            // для нормальных пакетов надо заключить CRC в []
            if (i == length-1) st += "[";

            memset(textBuf, 0, 11);
            sprintf(textBuf, "%02X", packet[i]);
            st += textBuf;

            // для нормальных пакетов надо заключить заголовок в []
            if (i==5) st += "]";
            // для нормальных пакетов надо заключить CRC в []
            if (i == length-1) st += "]";

            st += " ";
        }
        if (line == 0) line = __LINE__;
        _debugMsg(st, dbgLevel, line);
    }

    // очистка буфер приема
    void resetAll() {
       receiveIndex = -1;
       commandLength = -1;
    }
/////////////////////// ОТПРАВКА ИСХОДЯЩИХ ПАКЕТОВ ///////////////////////

// компоновка и отправка пакета MCU
    void sendCommand(uint8_t comm, uint8_t length=0, uint8_t* data=nullptr){
       if (_tuya_serial!=nullptr){
          sendRight=false; // снять флаг ускорения передачи данных
          uint8_t controll_buffer[length+1+6]={COMMAND_START[0],COMMAND_START[1],0,0,0,0};           
          uint8_t chkSum=COMMAND_START[0]+COMMAND_START[1]; //контрольная сумма
          controll_buffer[3]=comm; // команда
          controll_buffer[5]=length; //размер буфера данных
          _tuya_serial->write_array(controll_buffer,6); // отправляем хидер
          chkSum+=comm;
          chkSum+=length;
          _tuya_serial->write_array(data,length); // отправляем тело
          for(uint8_t i=0; i<length; i++){ // считаем КС
             chkSum+=data[i];
          }
          memcpy(controll_buffer+6, data, length); // сохраняем данные для лога
          _tuya_serial->write_array(&chkSum,1); // отправляем КС
          controll_buffer[length+6]=chkSum;
          #ifdef TBLIND_PRINT_RAW_PROTO
             _debugPrintPacket(controll_buffer, length+6+1, true, TBLIND_PRINT_RAW_PROTO);          
          #endif
          lastSend=esphome::millis(); //время отправки последнего байта
          waitReply=comm; // запоминаем команду которую отправили
       } else {
          uint32_t timer=-10000;
          if(esphome::millis()-timer>10000){
             timer=esphome::millis();
             MY_LOGE(TAG,"There is no UART port, work is impossible");
          }
       }
    }

//отправка тестовых запросов
    void sendTest(uint8_t comm){
       MY_LOGE("!!!!!!!!!","Send TEST command: %u", comm);
       sendCommand((uint8_t)comm);  
    }
    
//отправка статусных запросов
    void sendComm(tCommand comm){
       MY_LOGV(TAG,"Send short command: %u", comm);
       sendCommand((uint8_t)comm);  
    }

//отправка  статусных команд
    void sendComm(tDpid comm, tDataType dataType, uint8_t data){
       MY_LOGV(TAG,"Send command Id:%02X, valType:%d, data:%d", comm, dataType, data);
       uint8_t buff[8]={comm, dataType,0,0,0,0,0,0};
       if(dataType==dtVal){ // отправка значения 4 байта
          buff[3]=4;  
          put32(buff+4, data);   
       } else { // отправка значения 1 байт
          buff[3]=1;
          buff[4]=data;
       }
       sendCommand(DATAPOINT_DELIVER, 4+buff[3], buff);   
    }

//получение текущего статуса связи
    uint8_t getNetState() {
       uint8_t status = wsWifiConf;
       if(network::is_connected()) {
          // если контролировать подключение к серверу ХА, то это повлечет перегрузку, а это не нужно
          if(proto_vers>=3/* && (api_is_connected() || mqtt_is_connected())*/){
             status=wsWifiCloud;
          } else {
             status=wsWifi;
          }
       } else {
          #ifdef USE_CAPTIVE_PORTAL
             if (captive_portal::global_captive_portal != nullptr && captive_portal::global_captive_portal->is_active()) {
                status = wsNoConf;
             }
          #endif
       }
       return status;
    }

//отправка инфо о статусе модуля связи, это влияет на отдачу данных в модуль связи
    void sendNetState(uint8_t status=wsWifiConf) {
       MY_LOGD(TAG,"Send main net status: %u", status);
       sendCommand(WIFI_STATE, 1, &status);
    }

// отправка статуса связи
    void sendNetworkStatus(){
       uint8_t payload=getNetState();
       MY_LOGD(TAG,"Send Network Status: %d", payload);
       sendCommand(GET_NETWORK_STATUS,1,&payload);   
    }

// Разбор служебных пакетов
    bool processCommand(uint8_t commandByte/*3*/, uint16_t length/*5*/) {
       bool knownCommand = false;
       switch (commandByte) {
         case HEARTBEAT: {
           if(receivedCommand[6]==OFF){
              MY_LOGD(TAG,"Get first reply HEARTBEAT");
              heatBearTimer = esphome::millis();
              knownCommand = true;
              sendCounter=1;
           } else if(receivedCommand[6]==ON){
              MY_LOGD(TAG,"Get every reply HEARTBEAT after %d sec",esphome::millis()/1000);
              heatBearTimer = esphome::millis();
              knownCommand = true;
           } else {
              MY_LOGE(TAG,"Get reply HEARTBEAT: %d - ERROR !!!", receivedCommand[6]);
              break;
           } 
           if(sendCounter==1){
               sendCounter=2;
           }
           break;
         }
         case PRODUCT_QUERY: { // идентификатор изделия Query product information
           std::string id=getStringFromBuff((uint8_t*)receivedCommand+6,length);
           MY_LOGD(TAG,"Get Product info: %s", id.c_str());
           if(sensor_mcu_id_!=nullptr){
              sensor_mcu_id_->publish_state(id.c_str());   
           }
           knownCommand = true;
           if(sendCounter==2){ // пришел ответ на команду, переходим дальше
              sendCounter=3;
           }
           break;
         }
         case CONF_QUERY: { //0x55 0xAA  0x01(03)  0x02  0x00  0x00  CS  Query working mode of the module set by MCU
           MY_LOGD(TAG,"Get Stat Confirm reply");
           if(length==0){ // нет дополнительных ног
              knownCommand = true;
           } else if(length==4){ // есть дополнительные ноги
              status_pin_reported_=get16(receivedCommand+6);
              reset_pin_reported_=get16(receivedCommand+8);
              char one[]={(char)('A'+receivedCommand[6]),0};
              char two[]={(char)('A'+receivedCommand[8]),0};
              MY_LOGI(TAG,"Get pins setting: WiFi pin is P%s_%u, Reset pin is P%s_%u.",one,receivedCommand[7],two,receivedCommand[9]);
              if(this->status_pin_== nullptr){
                 MY_LOGE(TAG,"It is necessary to configure status_pin, perhaps it is P%s_%u.",one,receivedCommand[7]);
              } else {
                 if (!((this->status_pin_)->is_internal()) || ((InternalGPIOPin *)(this->status_pin_))->get_pin() != receivedCommand[7]){
                    MY_LOGW(TAG,"The configured status_pin does not meet TuyaMcu requirements, must be P%s_%u. Errors are possible.",one,receivedCommand[7]);
                 }
              }
              if(this->reset_pin_== nullptr){
                 MY_LOGE(TAG,"It is necessary to configure reset_pin, perhaps it is P%s_%u.",two,receivedCommand[9]);
              } else {
                 if (!((this->reset_pin_)->is_internal()) || ((InternalGPIOPin *)(this->reset_pin_))->get_pin() != receivedCommand[9]){
                    MY_LOGW(TAG,"The configured reset_pin does not meet TuyaMcu requirements, must be P%s_%u. Errors are possible.",two,receivedCommand[9]);
                 }      
              }
           }
           if(sendCounter==3){
             sendCounter=4; // переход к передаче статуса wifi
           }
           break;
         }
         case WIFI_STATE: { //55 aa 01(03) 03 00 00 CS,  Report the network status of the device
           MY_LOGD(TAG,"Get Confirm Network reply");
           knownCommand = true; // пришло подтверждение получения сетевого статуса
           oldNetState=netState; // снимаем признак отправки статуса
           if(sendCounter==4){
              sendCounter=5;
           }
           break;
         }
         case WIFI_RESET: { //55 aa 01(03) 04 00 00 CS Reset Wi-Fi
           MY_LOGD(TAG,"Get Reset from MCU");
           sendComm(WIFI_RESET); // отвечаем что приняли ресет
           sendCounter=1; // запускаем карусель
           knownCommand = true;
           break;
         }
         case WIFI_SELECT: {  //55 aa 01(03) 05 00 00 Reset Wi-Fi and select the pairing mode  //ресет - мигает диод
           MY_LOGD(TAG,"Get Pairing mode");
           knownCommand = true;
           break;
         }
         case LOCAL_TIME_QUERY: {  //55 aa 01(03) 1c 00 00 1c запрос времени (в протоколе ТУИ это другая команда (WTF) ???)
           MY_LOGE(TAG,"Get Date/time request - unsupported");
           knownCommand = false;
           break;
         }
         case GET_NETWORK_STATUS: {  //0x2B 55;AA;03;2B;00;00;2D Get the MAC address of the module
           MY_LOGD(TAG,"Get Network status request");
           knownCommand = true;
           sendNetworkStatus();
           break;
         }
         default:
           MY_LOGE(TAG,"Get Unexpect request 0x%X",commandByte);
           break;
       }
       return knownCommand;
    }

// Разбор данных устройства
    bool processStatusCommand(uint8_t dpid, uint32_t data) {
       bool ret=true;
       auto temp_operation=this->current_operation;
       auto temp_position=this->position;
       MY_LOGD(TAG,"GET DP: 0x%02X",dpid);           
       switch (dpid) {
         case idControl: { //open, stop, close
            if(data==0){
               temp_operation = cover::COVER_OPERATION_OPENING;
               _set_open=false;
            } else if(data==1){
               temp_operation = cover::COVER_OPERATION_IDLE;
               _set_pos=0xFF; // сбросить запрос позиционирования
               _set_stop=false;
            } else if(data==2){
               temp_operation = cover::COVER_OPERATION_CLOSING;
               _set_close=false;
            } else {
               break;
            } 
            if(_no_calibrate){
              MY_LOGI(TAG,"PLEASE, SETTING LIMITS !");
            }
            break;
         }
         case idIssue_Percent: { // ответ на наш запрос позиционирования
            MY_LOGD(TAG,"Confirm GOTO POSITION:%d, current: %d", data, _old_pos);
            if(data<=100){
              #ifdef TBLIND_VIRTUAL_POS
               if(get_speed()!=0 && _old_pos!=0xFF && _no_calibrate==false){ // скорость рассчитанна, старое положение есть
                  _dest_pos=data; // активация рассчета положения привода
                  _timer_pos=esphome::millis(); //засекаем время начала позиионирования
               }
              #endif
               _set_pos=0xFF;
               float quePos=(float)data/100.0;
               if(quePos > this->position){
                  temp_operation = cover::COVER_OPERATION_OPENING;
               } else {
                  temp_operation = cover::COVER_OPERATION_CLOSING;
               }
            } else {
               break;
            }
            break;
         }
         case idCurr_Percent: { // остановка устройства в положении в %
            uint32_t _now=esphome::millis();
            // наличие калибровки можно определить в момент получения данных с устройства в ответ на DataPoint request
            // а именно положение жалюзи, если устройство не калибровано, то процессор присылает
            // позицию 100%, если калибровано, то от 0 до 99 (даже если она реально 100)
            // определяем наличие калибровки, если калибровка отсутствует, пришел ответ о позиции мменьше 100%,
            // активен таймер чтения признака калибровки (запускается при отправке запроса на чтение настроек устройства DataPoint request)
            // время ожидания ответа на DataPoint request не истекло
            if(_no_calibrate && data<100 &&  _now-timerGetCalibrate<CALIBRATE_READ_TIME ){
               _no_calibrate=false;
               needReReadParams=false;
               MY_LOGD(TAG,"All limits are set");
            }
            _set_open=false;
            _set_stop=false;
            _set_close=false;
            _set_pos=0xFF;
            #ifdef TBLIND_VIRTUAL_POS 
               _dest_pos=0xFF; // признак достижения позиции
            #endif
            MY_LOGD(TAG,"Get POSITION:%d",data);
            temp_position = (float)data/100.0;
            #ifdef TBLIND_VIRTUAL_POS 
            // тут рассчет скорости перемещения
            if(_no_calibrate==false){
               if(_old_pos!=0xFF && _timer_pos!=0){
                  uint32_t deltatime=esphome::millis()-_timer_pos;
                  _timer_pos=0;
                  if(deltatime>0){
                     float deltapos=abs((float)_old_pos-data);
                     if(deltapos>2){
                        float spd=get_speed(); // старая скорость
                        float new_spd=deltapos/deltatime; // новая скорость
                        if(abs(new_spd-spd)*(100/FACTOR_SPEED_MISS)>spd){ // если разница больше 5 части
                            MY_LOGV(TAG,"Set new speed: %f, old: %f", new_spd, spd);
                        } else if(spd!=new_spd){
                            new_spd = spd*(1.0-SPEED_FILTER_COEF)+(new_spd*SPEED_FILTER_COEF); // ползущий фильтр
                            MY_LOGV(TAG,"Core speed %f => %f", spd, new_spd);
                        }
                        set_speed(new_spd);
                     }
                  }
               }
            } else if(_now>=CALIBRATE_READ_TIME){
               set_speed(0);
            }                
            #endif
            temp_operation = cover::COVER_OPERATION_IDLE;
            _old_pos=data;
            break;
         }
         case idDir: { // настройка направления вращения мотора
            MY_LOGD(TAG,"%s MOTOR DIRECTION:%d",(data!=TBLIND_MOTOR_REVERS)? "Core":"Confirm" ,data);
            if(data!=TBLIND_MOTOR_REVERS){
               sendComm(idDir, dtEnum, (uint8_t) TBLIND_MOTOR_REVERS); // отправляем настройку реверса
            }                
            break;
         }
         case idSpeed: { // настройка скорости вращения мотора
            MY_LOGD(TAG,"%s MOTOR SPEED:%d",(data!=motor_speed)? "Get":"Confirm" ,data);
            #ifndef TBLIND_MOTOR_SPEED
               if(motor_speed!=data){ // обновить контрол
                  if(speed_select!=nullptr){ // 
                     speed_select->publish_state(str_speed[data]);   
                  } else {
                     motor_speed=(tMoSpeed)data;
                  }
               }
               new_motor_speed=tmoUndef;
            #else
               if(motor_speed!=data){
                  new_motor_speed=motor_speed;   
               } else {
                  new_motor_speed=tmoUndef;
               }
            #endif   
            break;
         }
         case idChild_Lock: { // блокировка внешнего управления
            #ifdef TBLIND_CHILDREN_LOCK
               MY_LOGD(TAG,"%s CHILD LOCK state:%d",(data!=child_lock)? "Get":"Confirm" ,data);
               if(child_lock!=data){ // обновить состояние
                  if(lock_switch!=nullptr){ // 
                     lock_switch->publish_state(data);   
                  } else {
                     child_lock=(tStat)data;
                  }
               }
               new_child_lock=UNDEF;
            #else 
               MY_LOGD(TAG,"Get CHILD LOCK state:%d, functional not setting",data);
            #endif
            break;
         }
         case idAtom_Mode: { // если режим не поддерживается, нужно отключить
            #ifdef TBLIND_ATOM
               MY_LOGD(TAG,"%s ATOM state:%d",(data!=atom_mode)? "Get":"Confirm" ,data);
               if(atom_mode!=data){ // обновить состояние
                  if(atom_switch!=nullptr){ // 
                     atom_switch->publish_state(data);   
                  } else {
                     atom_mode=(tStat)data;
                  }
               }
               new_atom_mode=UNDEF;
            #else
               MY_LOGD(TAG,"%s ATOM state:%d",(data!=OFF)? "Core":"Confirm" ,data);
               if(data!=OFF){
                  sendComm(idAtom_Mode, dtBool, OFF); // отключаем режим ATOM
               }
            #endif
            break;
         }
         case idGo_Best_Pos:
         case idSet_Best_Pos: { // этот режим не поддерживается
            MY_LOGD(TAG,"Best position mode not supported");
            break;
         }
         case  idSet_Limits: { // репорт установка лимитов
            MY_LOGD(TAG,"Get report %s limit(s) settings.", (data==0)? "OPEN":((data==1)? "CLOSE":"RESET"));
            if(data==tliReset){
               _no_calibrate=true;
               _cal_up=false;
               _cal_down=false;
            } else if(data==tliClose){
               _cal_down=true;
            } else if(data==tliOpen){
               _cal_up=true;
            }
            if(_no_calibrate && _cal_down && _cal_up){
               _no_calibrate=false;
               MY_LOGD(TAG,"Calibrate - YES");               
            }
            limit_comm=tliUndef;
            break;
         }
         default: {
           MY_LOGE(TAG,"Get Unexpect DP: 0x%02X",dpid);
           ret = false;
           break;
         }
       }       
       if(ret && (temp_operation!=this->current_operation || temp_position!=this->position)){
          this->current_operation = temp_operation;
          this->position = temp_position;
          this->publish_state(); // если что то изменилось - публикуем
      }
      return ret;
    }

// РАЗБОР полученного пакета 
    void processSerialCommand() {
       #ifdef TBLIND_PRINT_RAW_PROTO
          _debugPrintPacket(receivedCommand, receiveIndex+1, false, TBLIND_PRINT_RAW_PROTO);
       #endif
       if (commandLength > -1) {
          bool knownCommand = false;
          proto_vers=receivedCommand[2]; // запоминаем версию протокола MCU
          if(proto_vers==0x03){
             if (receivedCommand[3] == DATAPOINT_REPORT) { // пакет данных функционального назначения
               if(sendCounter==5){
                  sendCounter=6; // после запроса данных скопом
               } 
               uint16_t dataSize=get16(receivedCommand+4); //размер посылки без хидера
               uint16_t valSize=get16(receivedCommand+8); // размер буфера значения
               if(dataSize-valSize==4){ // размеры соответствуют друг другу
                  if (valSize==1 && (receivedCommand[7]==dtBool || receivedCommand[7]==dtEnum || receivedCommand[7]==dtBits)){ // проверка соответствия типу даннных
                     knownCommand = processStatusCommand(receivedCommand[6], receivedCommand[10]); // однобайтовое значение
                  } else if(valSize==4 && receivedCommand[7]==dtVal){
                     knownCommand = processStatusCommand(receivedCommand[6], get32(receivedCommand+10)); // 32битное значение
                  }
               }
             } else { // общий пакет протокола туя
               knownCommand = processCommand(receivedCommand[3], receivedCommand[5]);
             }
          } else {
             MY_LOGE(TAG,"Unsupported proto version");           
          }
          if (!knownCommand) {
            MY_LOGE(TAG,"Unknown command");
            _debugPrintPacket(receivedCommand, receiveIndex+1, false, ESPHOME_LOG_LEVEL_ERROR);
          }
       }
    }

// сюда пихаем байт присланый MCU
    void inData(uint8_t inChar){
      if((int16_t)(sizeof(receivedCommand))>receiveIndex){ // контроль переполнения буфера входных данных
         receiveIndex++;
      }
      receivedCommand[receiveIndex] = inChar;
      if(receiveIndex==0 && COMMAND_START[0] != inChar){ //проверка хидера 55
         resetAll(); //  не совпало - ресетимся
      } else if (receiveIndex==1) { // проверка хидера 55 AA
        if (COMMAND_START[1]!=inChar) {
           if(COMMAND_START[0]==inChar){ // если опять 55
              receiveIndex=0; // считаем, что это нулевой байт пакета
          } else {
              resetAll(); //  не совпало - ресетимся
           }
        }
      } else if (receiveIndex == 5) { // считаем размер пакета
        commandLength = receivedCommand[4] * 0x100 + inChar;
      } else if ((commandLength > -1) && (receiveIndex == (6 + commandLength))) { // получили полный пакет
        uint8_t expChecksum = 0;         //проверяем КС
        for (uint8_t i = 0; i < receiveIndex; i++) {
          expChecksum += receivedCommand[i];
        }
        if (expChecksum == receivedCommand[receiveIndex]) {
          processSerialCommand();
          if(waitReply==receivedCommand[3]){ // пришел ответ на запрос
             sendRight=true; // можно отправлять новые пакеты     
          }
        }
        resetAll();
      }
    }
    
 protected:

    void control(const cover::CoverCall &call) override {
       if (call.get_position().has_value()) { // получили запрос нового положения
          float target = *call.get_position(); 
          _set_pos=(uint8_t)(100*target);
       }
       if (call.get_stop()) {
          _set_stop=true;   
       }
       this->publish_state();
    }

 public:
   
   cover::CoverTraits get_traits() override {
     auto traits = cover::CoverTraits();
     traits.set_supports_position(true);
     traits.set_supports_stop(true);
     traits.set_is_assumed_state(true);
     return traits;
   }

    void dump_config() override{
        LOG_COVER("", "Tuya Blind (proto3)", this);
        LOG_TEXT_SENSOR("  ", "MCU product ID", this->sensor_mcu_id_);
        #ifdef TBLIND_VIRTUAL_POS
           ESP_LOGCONFIG(TAG, "Public move position in move, every: %d uS",TBLIND_VIRTUAL_POS);
           #ifdef TBLIND_RESTORE
              ESP_LOGCONFIG(TAG, "Restore calculated data enabled, save period: %d uS",TBLIND_RESTORE);
           #endif
        #endif
        if(status_pin_reported_!=-1){
           char buff[]={(char)('A'+status_pin_reported_/0x100),0}; 
           ESP_LOGCONFIG(TAG, "The MCU requires the use of a P%s_%u output to indicate WiFi status",buff,(uint8_t)status_pin_reported_);    
           if(status_pin_==nullptr){ 
              MY_LOGE(TAG, "You need to configure WiFi status output pin");    
           }
        }
        if(reset_pin_reported_!=-1){
           char buff[]={(char)('A'+reset_pin_reported_/0x100),0}; 
           ESP_LOGCONFIG(TAG, "The MCU requires the use of a P%s_%u as reset protocol input",buff,(uint8_t)reset_pin_reported_);    
           if(status_pin_==nullptr){ 
              MY_LOGE(TAG, "You need to configure reset input pin");    
           }
        }
        #ifdef TBLIND_MOTOR_SPEED
           ESP_LOGCONFIG(TAG, "Fixed speed of movementFixed speed of movement %s",(TBLIND_MOTOR_SPEED==0)?"slowly":((TBLIND_MOTOR_SPEED==3)?"quickly":"normal"));
        #else
           if(this->speed_select!=nullptr) LOG_SELECT("", "Speed selector ", this->speed_select);
        #endif
        #ifdef TBLIND_MOTOR_REVERS
           ESP_LOGCONFIG(TAG, "Motor rotation is %s",(TBLIND_MOTOR_REVERS==1)?"reversed":"normal");
        #endif
        #ifdef TBLIND_CHILDREN_LOCK
           if(this->lock_switch!=nullptr) LOG_SWITCH("", "Children lock ", this->lock_switch);
        #endif
        #ifdef TBLIND_ATOM
           if(this->atom_switch!=nullptr) {
              LOG_SWITCH("", "Step by step (atom) mode control ", this->atom_switch);
           } else {
              ESP_LOGE(TAG, "Attention, there is no atom mode control, and it is recommended to fix it.");
           }
        #else
           ESP_LOGCONFIG(TAG, "Automatic off atom mode");
        #endif
    }

    void initBlind(esphome::uart::UARTComponent *parent = nullptr) { // инициализация объекта
        _tuya_serial = parent;
    }
    float get_setup_priority() const override { return esphome::setup_priority::DATA;}
    void set_product_id_text(text_sensor::TextSensor *sensor){sensor_mcu_id_=sensor;}

    void setup() override {
       sendCounter=1; // нужно запустить карусель обмена данными
       #ifdef TBLIND_VIRTUAL_POS
         #ifdef TBLIND_RESTORE
            loadDataFlash();
         #endif  
       #endif      

    }

#ifdef TBLIND_CHILDREN_LOCK
    // подключение чилдрен лок
    void set_children_lock_switch(TuyaBlind_Switch* switch_){ // подключение свитча children lock
       lock_switch=switch_;
       switch_->add_on_state_callback([this](bool st){ 
            // обработка переключения свитча
            if(!std::isnan(st)){
               if(lock_switch->state!=st){ // новое состояние свича
                   if(st){
                       new_child_lock=ON;
                   } else {
                       new_child_lock=OFF;
                   }
                   child_lock=new_child_lock;
               }
            }
       });
    }
#endif

#ifdef TBLIND_ATOM
    // подключение свитча атом режима
    void set_atom_switch(TuyaBlind_Switch* switch_){ // подключение свитча 
       atom_switch=switch_;
       switch_->add_on_state_callback([this](bool st){ 
            // обработка переключения свитча
            if(!std::isnan(st)){
               if(atom_switch->state!=st){ // новое состояние свича
                   if(st){
                       new_atom_mode=ON;
                   } else {
                       new_atom_mode=OFF;
                   }
                   atom_mode=new_atom_mode;
               }
            }
       });
    }
#endif

#ifndef TBLIND_MOTOR_SPEED
    // подключение селектора скорости мотора
    void set_speed_select(TuyaBlind_Select *select_){
       speed_select=select_;
       select_->traits.set_options(str_speed);
 #if ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 1, 0)
       select_->add_on_state_callback([this](size_t pos){ 
 #else
       select_->add_on_state_callback([this](std::string new_value,  size_t pos){ 
 #endif
          if(motor_speed != pos){
             new_motor_speed = (tMoSpeed)pos; 
             motor_speed = (tMoSpeed)pos;
          }
       });
    }
#endif

    void set_limits_buttons(TuyaBlind_Button* _up, TuyaBlind_Button* _down, TuyaBlind_Button* _reset){
       lim_up_button=_up;
       lim_up_button->add_on_press_callback([this](){limit_comm=tliOpen;});
       lim_down_button=_down;
       lim_down_button->add_on_press_callback([this](){limit_comm=tliClose;});
       lim_reset_button=_reset;
       lim_reset_button->add_on_press_callback([this](){limit_comm=tliReset;});
    }

    void loop() override {
        // читаем UART порт
        if (_tuya_serial!=nullptr){
           while(_tuya_serial->available()){ //читаем
              uint8_t data;
              _tuya_serial->read_byte(&data);
              lastRead=esphome::millis();//время чтения последнего байта
              inData(data); // обрабатываем
           }               
        }

      #ifdef TBLIND_VIRTUAL_POS
         #ifdef TBLIND_RESTORE
            uint32_t now=esphome::millis();
            if(_no_calibrate==false && now-saveTimer>=TBLIND_RESTORE){ //мотор калиброван и настало время сохрвнить данные
               saveTimer=now; // пытаемся сохранить данные по таймеру и только в останове
               saveDataFlash();
            }
        #endif
        // рассчет текущего положения во время позиционирования
        if(_no_calibrate==false && _dest_pos!=0xFF){
           uint32_t _now=esphome::millis();
           if(_now-_timer_update>=TBLIND_VIRTUAL_POS){
              _timer_update=_now;
              float calc_pos=(float)get_speed()*(_now-_timer_pos); // пройденный путь
              if(_old_pos<_dest_pos){ // закрывается
                 calc_pos=calc_pos+_old_pos;   
              } else { // открывается
                 calc_pos=-calc_pos+_old_pos;
              }
              if(calc_pos<=100.0 && calc_pos>=0.0){ // фильтрация бреда
                 this->position = calc_pos/100.0;
                 MY_LOGD(TAG,"New calculated position:%f",this->position);
                 this->publish_state();
              }
           }
        }
      #endif  
      
        // карусель обмена данными
        if(esphome::millis()-lastSend>UART_TIMEOUT && (esphome::millis()-lastRead>UART_TIMEOUT || sendRight)){ //можно отправлять
          if(sendCounter==1){ // отправим первый heatbear
             MY_LOGD(TAG,"Send first HEARTBEAT");          
             sendComm(HEARTBEAT); 
          } else if(sendCounter==2){
             MY_LOGD(TAG,"Send Prod_ID request");          
             sendComm(PRODUCT_QUERY);
          } else if(sendCounter==3){
             MY_LOGD(TAG,"Send State request");          
             sendComm(CONF_QUERY);
          } else if(sendCounter==4){ // отправка стартового сетевого состояния
             MY_LOGD(TAG,"Send WiFi_OK state");          
             sendNetState(getNetState());
          } else if(sendCounter==5){ // отправка стартового сетевого состояния
             MY_LOGD(TAG,"Send DataPoint request");
             sendComm(DATAPOINT_QUERY);
             timerGetCalibrate=esphome::millis(); // таймер таймслота определения калибровки
          } else if(sendCounter==6){ // остальной процессинг
             if(needReReadParams &&_no_calibrate && esphome::millis()>40000){
                sendCounter=5; // запустим запрос снова
                needReReadParams=false;   
                MY_LOGD(TAG,"Repeat send DataPoint request, for calibrate read");                
             } else if(esphome::millis()-lastSend>SEND_TIMEOUT){
                static uint32_t lastSendPing=esphome::millis();
                if(_set_stop ){
                   MY_LOGD(TAG,"Send STOP(2)");
                   sendComm(idControl, dtEnum, tenStop); // остановить   
                } else if (_set_pos!=0xFF) {
                   MY_LOGD(TAG,"Send GOTO POSITION(2): %d",_set_pos);
                   sendComm(idIssue_Percent, dtVal, (uint8_t)(_set_pos)); // запрос установки в позиию
                } else if (_set_open) {
                   MY_LOGD(TAG,"Send OPEN");
                   sendComm(idControl, dtEnum, tenOpen); // открыть   
                } else if (_set_close) {
                   MY_LOGD(TAG,"Send CLOSE");
                   sendComm(idControl, dtEnum, tenClose); // закрыть  
                } else if (new_motor_speed!=tmoUndef) { // скорость мотора
                   MY_LOGD(TAG,"Send MOTOR SPEED: %d", (uint8_t)new_motor_speed); 
                   sendComm(idSpeed, dtEnum, new_motor_speed); 
              #ifdef TBLIND_CHILDREN_LOCK
                } else if (new_child_lock!=UNDEF) { // состояние внешней блокировки
                      MY_LOGD(TAG,"Send CHILD LOCK: %d", new_child_lock); 
                      sendComm(idChild_Lock, dtBool, (uint8_t)new_child_lock); 
              #endif
              #ifdef TBLIND_ATOM
                } else if (new_atom_mode!=UNDEF) { // состояние внешней блокировки
                   MY_LOGD(TAG,"Send ATOM MODE STATE: %d", new_atom_mode); 
                   sendComm(idAtom_Mode, dtBool, (uint8_t)new_atom_mode); 
              #endif
                } else if (limit_comm!=tliUndef) { //  команда установки лимитов
                   MY_LOGD(TAG,"Send LIMIT COMMAND: %d", limit_comm); 
                   sendComm(idSet_Limits, dtEnum, (uint8_t)limit_comm); 
                } else if(esphome::millis()-lastSendPing>HEARTBEAT_INTERVAL*1000){
                   MY_LOGV(TAG,"Send regular HEARTBEAT");          
                   sendComm(HEARTBEAT);
                   lastSendPing+=HEARTBEAT_INTERVAL*1000;
                }
             }
          }
       }
    }

    friend class TuyaBlind_Switch;
    friend class TuyaBlind_Button;
    friend class TuyaBlind_Select;

};

}  // namespace tuya_blind
}  // namespace esphome



#endif //TUYA_TERMO_H
