/*
 * @Version      : V1.0
 * @Date         : 2024-10-17 11:38:45
 * @Description  : 
 */
#include "interface/console/interface_console.hpp"
#include <stdio.h>
#include <stdarg.h>
#if defined(_WIN32)
	#include <WinSock2.h>
  #include <windows.h>
#else 
  #include <unistd.h>
#endif

namespace nvistar{

InterfaceConsole::InterfaceConsole(){
}

InterfaceConsole::~InterfaceConsole(){
}

void InterfaceConsole::print_normal(const char* noraml_, ...){
  va_list args;
  va_start(args, noraml_);
  vsnprintf(out, sizeof(out), noraml_, args);
  va_end(args);
#if defined (_WIN32)
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
  printf("%s\r\n", out);
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
#else
  printf(COLOR_NONE);
  printf("%s\n", out);
  printf(COLOR_NONE);
#endif
}

void InterfaceConsole::print_noerr(const char* noerr_, ...){
  va_list args;
  va_start(args, noerr_);
  vsnprintf(out, sizeof(out), noerr_, args);
  va_end(args);
#if defined (_WIN32)
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x02);
  printf("%s\r\n", out);
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
#else
  printf(COLOR_GREEN);
  printf("%s\n", out);
  printf(COLOR_NONE);
#endif
}

void InterfaceConsole::print_error(const char* error_, ...){
  va_list args;
  va_start(args, error_);
  vsnprintf (out, sizeof(out), error_, args);
  va_end(args);
#if defined (_WIN32)
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x04);
  printf ("[error]: ");
  printf ("%s", out);
  printf ("\r\n");
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
#else
  printf(COLOR_RED);
  printf("[error]: ");
  printf("%s\n", out);
  printf(COLOR_NONE);
#endif
}

void InterfaceConsole::print_warn(const char* warning_, ...){
  va_list args;
  va_start(args, warning_);
  vsnprintf(out, sizeof(out), warning_, args);
  va_end(args);
#if defined (_WIN32)
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x06);
  printf ("[warn]: ");;
  printf ("%s\r\n", out);
  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), 0x07);
#else
  printf(COLOR_YELLOW);
  printf("[warn]: ");
  printf("%s\n", out);
  printf(COLOR_NONE);
#endif
}

}