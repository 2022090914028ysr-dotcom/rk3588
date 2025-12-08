/*
 * @Version      : V1.0
 * @Date         : 2024-10-14 10:50:22
 * @Description  : serial api windows
 */
#include "interface/serial/interface_serial.hpp"
#include <windows.h>
#include <thread>
#include <chrono>

namespace nvistar{

class InterfaceSerialImpl{
public:
  //var 
  HANDLE  _fd = INVALID_HANDLE_VALUE;   //File descriptor for the port
  bool open_flag = false;               //serial open flag
  //serial var 
  std::string port_name = "/dev/ttyUSB";
  int baud_rate = 230400;
  InterfaceSerial::serial_parity_t   parity = InterfaceSerial::ParityNone;
  InterfaceSerial::serial_databits_t databits = InterfaceSerial::DataBits8;
  InterfaceSerial::serial_stopbits_t stopbits = InterfaceSerial::StopOne;
  InterfaceSerial::serial_flowcontrol_t flowcontrol = InterfaceSerial::FlowNone;
  
  //function 

  /**
  * @Function: serial_set_databits
  * @Description: set databits 
  * @Return: bool
  * @param {DCB} *cfg
  * @param {serial_databits_t} databits
  */
  bool serial_set_databits(DCB *cfg,InterfaceSerial::serial_databits_t databits){
    //set databits 
    switch (databits) {
      case InterfaceSerial::DataBits5:{
        cfg->ByteSize = 5;
        break;
      }
      case InterfaceSerial::DataBits6:{
        cfg->ByteSize = 6;
        break;
      }
      case InterfaceSerial::DataBits7:{
        cfg->ByteSize = 7;
        break;
      }
      case InterfaceSerial::DataBits8:{
        cfg->ByteSize = 8;
        break;
      }
      default:{
        cfg->ByteSize = 8;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_parity
  * @Description: set parity 
  * @Return: bool 
  * @param {DCB} *cfg
  * @param {serial_parity_t} parity
  */
  bool serial_set_parity(DCB *cfg,InterfaceSerial::serial_parity_t parity){
    switch (parity) {
      case InterfaceSerial::ParityNone:{
        cfg->Parity = NOPARITY;
        break;
      }
      case InterfaceSerial::ParityEven:{
        cfg->Parity = EVENPARITY;
        break;
      }
      case InterfaceSerial::ParityOdd:{
        cfg->Parity = ODDPARITY;
        break;
      }
      case InterfaceSerial::ParityMark:{
        cfg->Parity = MARKPARITY;
        break;
      }
      case InterfaceSerial::ParitySpace:{
        cfg->Parity = SPACEPARITY;
        break;
      }
      default:{
        cfg->Parity = NOPARITY;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_stopbits
  * @Description: set stopbit 
  * @Return: bool 
  * @param {DCB} *cfg
  * @param {serial_stopbits_t} stop_bits
  */
  bool serial_set_stopbits(DCB *cfg, InterfaceSerial::serial_stopbits_t stopbits){
    switch (stopbits) {
      case InterfaceSerial::StopOne:{
        cfg->StopBits = ONESTOPBIT;
        break;
      }
      case InterfaceSerial::StopOneAndHalf:{
        cfg->StopBits = ONE5STOPBITS;
        break;
      }
      case InterfaceSerial::StopTwo:{
        cfg->StopBits = TWOSTOPBITS;
        break;
      }
      default:{
        cfg->StopBits = ONESTOPBIT;
        break;
      }
    }
    return true;
  }
  /**
  * @Function: serial_set_flowcontrol
  * @Description: set flowcontrol 
  * @Return: bool 
  * @param {DCB} *cfg
  * @param {serial_flowcontrol_t} flowcontrol
  */
  bool serial_set_flowcontrol(DCB *cfg, InterfaceSerial::serial_flowcontrol_t flowcontrol){
    switch (flowcontrol) {
      case InterfaceSerial::FlowNone:{
        cfg->fOutxCtsFlow = false;
        cfg->fRtsControl = RTS_CONTROL_DISABLE;
        cfg->fOutX = false;
        cfg->fInX = false;
        break;
      }
      case InterfaceSerial::FlowHardware:{
        cfg->fOutxCtsFlow = true;
        cfg->fRtsControl = RTS_CONTROL_HANDSHAKE;
        cfg->fOutX = false;
        cfg->fInX = false;
        break;
      }
      case InterfaceSerial::FlowSoftware:{
        cfg->fOutxCtsFlow = false;
        cfg->fRtsControl = RTS_CONTROL_DISABLE;
        cfg->fOutX = true;
        cfg->fInX = true;
        break;
      }
      default:{

        break;
      }
    } 
    return true;
  }
  
  /**
  * @Function: serial_set_baudrate
  * @Description: set serial baudrate 
  * @Return: bool 
  * @param {int} baudrate
  */
  bool serial_set_baudrate(DCB *cfg, int baudrate){
    cfg->BaudRate = baudrate;
    return true;
  }
};

/**
 * @Function: InterfaceSerial
 * @Description: 
 * @Return: None
 */
InterfaceSerial::InterfaceSerial() : _impl(new InterfaceSerialImpl){

}

/**
 * @Function: ~InterfaceSerial
 * @Description: 
 * @Return: None
 */
InterfaceSerial::~InterfaceSerial(){
  serial_close();
  delete _impl;
}

/**
 * @Function: serial_open
 * @Description: open serial
 * @Return: bool --- success or fail
 * @param {string} port_name 
 * @param {int} baud_rate
 * @param {int} party
 * @param {int} data_bits
 * @param {int} stop_bits
 * @param {int} flow_control
 */
bool InterfaceSerial::serial_open(std::string port_name, int baud_rate,
                  serial_parity_t parity,
                  serial_databits_t databits,
                  serial_stopbits_t stopbits,
                  serial_flowcontrol_t flow_control){
  std::wstring port_name_wstring(port_name.begin(), port_name.end());
  std::wstring port_name_open = L"\\\\.\\" + port_name_wstring;
  LPCWSTR port_name_lp = port_name_open.c_str();
  //save parameters
  _impl->port_name = port_name;
  _impl->baud_rate = baud_rate;
  _impl->parity = parity;
  _impl->databits = databits;
  _impl->stopbits = stopbits;
  _impl->flowcontrol = flow_control;
  //open the serial 
  _impl->_fd = CreateFileW(port_name_lp,                   //communication port string (COMX)
                        GENERIC_READ | GENERIC_WRITE,   //read/write types
                        0,                                  //comm devices must be opened with exclusive access
                        NULL,                      //no security attributes
                        OPEN_EXISTING,            //comm devices must use OPEN_EXISTING
                        FILE_ATTRIBUTE_NORMAL,     //Async I/O or sync I/O
                        0);
  //open failed
  if(_impl->_fd == INVALID_HANDLE_VALUE){
    return false;
  }
  DCB cfg = {0};
  cfg.DCBlength = sizeof(cfg);
  cfg.fBinary = true;
  cfg.fAbortOnError = false;
  cfg.fNull = false;
  //set parameter 
  _impl->serial_set_parity(&cfg, parity);            //set parity 
  _impl->serial_set_databits(&cfg, databits);        //set databits 
  _impl->serial_set_stopbits(&cfg, stopbits);        //set stopbits 
  _impl->serial_set_flowcontrol(&cfg, flow_control); //set flowcontrol
  _impl->serial_set_baudrate(&cfg, baud_rate);          //set baudrate
  // activate settings
  if (!SetCommState(_impl->_fd, &cfg)){
    CloseHandle(_impl->_fd);
    _impl->_fd = INVALID_HANDLE_VALUE;
    return false;
  }
  //discards all characters from the output or input buffer of a specified communications resource. It
  //can also terminate pending read or write operations on the resource.
  PurgeComm(_impl->_fd, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_RXABORT);
  //set timeouts
  COMMTIMEOUTS cfg_timeout = {0};
  cfg_timeout.ReadIntervalTimeout = MAXDWORD;
  cfg_timeout.ReadTotalTimeoutConstant = 0;
  cfg_timeout.ReadTotalTimeoutMultiplier = MAXDWORD;
  cfg_timeout.WriteTotalTimeoutConstant = 0;
  cfg_timeout.WriteTotalTimeoutMultiplier = 0;
  if(!SetCommTimeouts(_impl->_fd, &cfg_timeout)){
    CloseHandle(_impl->_fd);
    _impl->_fd = INVALID_HANDLE_VALUE;
    return false;
  }

  return true;
}

/**
 * @Function: serial_close
 * @Description: close serialport
 * @Return: void
 */
void InterfaceSerial::serial_close(){
  if(!serial_isopen()){
    return;
  }
  //stop all event
  SetCommMask(_impl->_fd, 0);
  //clear buffer
  PurgeComm(_impl->_fd, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXCLEAR | PURGE_RXABORT);
  //close 
  CloseHandle(_impl->_fd);
  _impl->_fd = INVALID_HANDLE_VALUE;
}

/**
 * @Function: serial_reopen
 * @Description: serial reopen 
 * @Return: void 
 */
void InterfaceSerial::serial_reopen(){
  //check the value
  if(_impl->port_name.empty()) {
    return;
  }
  //first close the serial 
  serial_close();
  //delay some times 
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  //reopen the serial 
  serial_open(_impl->port_name, _impl->baud_rate,
              _impl->parity, _impl->databits,_impl->stopbits, _impl->flowcontrol);
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
}

/**
 * @Function: serial_isopen
 * @Description: judge serial is open?
 * @Return: bool 
 */
bool InterfaceSerial::serial_isopen(){
  if(_impl->_fd != INVALID_HANDLE_VALUE){
    return true;
  }
  return false;
}

/**
 * @Function: serial_read
 * @Description: serial read data 
 * @Return: int --- read true length
 * @param {uint8_t} *data
 * @param {int} max_length
 */
int InterfaceSerial::serial_read(uint8_t *data,int max_length){
  if(!serial_isopen()){
    return 0;
  }
  DWORD bytes_read;
  if(!ReadFile(_impl->_fd, data, static_cast<DWORD>(max_length), &bytes_read, NULL)){
    return 0;
  }
  return static_cast<int>(bytes_read);
}

/**
 * @Function: serial_write
 * @Description: serial write data 
 * @Return: int --- write true length 
 * @param {uint8_t*} data
 * @param {int} length
 */
int InterfaceSerial::serial_write(const uint8_t* data,int length){
  if(!serial_isopen()){
    return 0;
  }
  DWORD bytes_write;
  if(!WriteFile(_impl->_fd, data, static_cast<DWORD>(length), &bytes_write, NULL)){
    return 0;
  }
  return static_cast<int>(bytes_write);
}

/**
 * @Function: serial_flush
 * @Description: flush serial input and output
 * @Return: void
 */
void InterfaceSerial::serial_flush(){
  if(!serial_isopen()){
    return;
  }
  PurgeComm(_impl->_fd, PURGE_RXCLEAR);
  PurgeComm(_impl->_fd, PURGE_TXCLEAR);
}

}