#include "PID.h"
#include <iostream>
using namespace std;
PID::PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PID::compute(float target, float current) {
    // ¼ÆËãÎó²î
    float error = target - current;
    // Îó²îµÄÀÛ¼Æ
    intergral += error;
    // ±¾´ÎÎó²îºÍÉÏÒ»´ÎÎó²îµÄ²îÒì
    derivative = error - prevError;
    // Ì×ÓÃpidµÄ¹«Ê½
    targetpoint = kp * error + ki * intergral + kd * derivative;
    // ¼ÇÂ¼ÉÏÒ»´ÎµÄÎó²î
    prevError = error;
    return targetpoint;
}

void PID::reset() {
    targetpoint = 0;
    intergral = 0;
    derivative = 0;
    prevError = 0;
}

void PID::Set_PID(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}
