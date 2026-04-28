/*
 * @Version      : V1.0
 * @Date         : 2024-10-17 11:33:45
 * @Description  : console 
 */
#ifndef __INTERFACE_CONSOLE_H__
#define __INTERFACE_CONSOLE_H__

namespace nvistar{

#ifndef DLL_EXPORT
  #ifdef _MSC_VER
    #define DLL_EXPORT __declspec(dllexport)
  #else
    #define DLL_EXPORT
  #endif 
#endif 

class DLL_EXPORT InterfaceConsole{
public:
  InterfaceConsole();
  ~InterfaceConsole();

  void print_normal(const char* noraml_, ...);
  void print_noerr(const char* noraml_, ...);
  void print_error(const char* error_, ...);
  void print_warn(const char* error_, ...);
private:
  char out[1024] = { 0 };
  //linux color type info 
  #if !defined (_WIN32)
      #define COLOR_NONE "\033[m"
      #define COLOR_RED "\033[1;31m"
      #define COLOR_GREEN "\033[1;32m"
      #define COLOR_YELLOW "\033[1;33m"
      #define COLOR_CYAN "\033[1;36m"
  #endif 
};
}

#endif