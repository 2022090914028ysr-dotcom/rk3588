/*
 * @Version      : V1.0
 * @Date         : 2024-10-12 17:35:27
 * @Description  : serial api unix
 */
#ifndef __SERIAL_H__
#define __SERIAL_H__

#include <string>

namespace nvistar{

#ifndef DLL_EXPORT
  #ifdef _MSC_VER
    #define DLL_EXPORT __declspec(dllexport)
  #else
    #define DLL_EXPORT
  #endif 
#endif 

class InterfaceSerialImpl;

class DLL_EXPORT InterfaceSerial{
  public:
    //serial parity define 
    typedef enum{
      ParityNone = 0,
      ParityOdd,
      ParityEven,
      ParityMark,
      ParitySpace,
    }serial_parity_t;
    //serial data bits define 
    typedef enum{
      DataBits5 = 5,
      DataBits6 = 6,
      DataBits7 = 7,
      DataBits8 = 8,
    }serial_databits_t;
    //serial stop bits deifne 
    typedef enum{
      StopOne = 0,
      StopOneAndHalf = 1,
      StopTwo = 2,
    }serial_stopbits_t;
    //serial flowcontrol 
    typedef enum{
      FlowNone = 0,
      FlowHardware = 1,
      FlowSoftware = 2,
    }serial_flowcontrol_t;

    InterfaceSerial();
    ~InterfaceSerial();
    bool serial_open(std::string port_name = "/dev/ttyS1",
                int baud_rate = 230400,
                serial_parity_t   parity = ParityNone,
                serial_databits_t databits = DataBits8,
                serial_stopbits_t stopbits = StopOne,
                serial_flowcontrol_t flowcontrol = FlowNone);          //open serial
    void serial_close();                                //close serial
    void serial_reopen();                               //serial reopen 
    bool serial_isopen();                               //serial is open?
    int  serial_read(uint8_t *data,int max_length);     //read serial data 
    int  serial_write(const uint8_t* data,int length);  //write serial data
    void serial_flush();                                //flush serial data 
  private:
    InterfaceSerialImpl *_impl;
};
}



#endif
