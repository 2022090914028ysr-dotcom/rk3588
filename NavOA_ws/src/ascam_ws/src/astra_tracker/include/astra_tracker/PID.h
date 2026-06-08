
class PID {
public:
    float kp;
    float ki;
    float kd;
    float targetpoint;
    // ÉÏÒ»´ÎµÄÎó²î
    float prevError;
    // »ý·Ö
    float intergral;
    // Î¢·Ö
    float derivative;

    PID(float kp, float ki, float kd);

    void Set_PID(float kp, float ki, float kd);

    /**
     * pidµÄ¼ÆËãº¯Êý
     * @param target  Ä¿±êÖµ
     * @param current µ±Ç°Öµ
     * @return  pwm
     */
    float compute(float target, float current);

    /**
     *  ÖØÖÃËùÓÐµÄÎó²î: µ±ÉèÖÃµÄËÙ¶È ºÍ ÉÏÒ»´Î²»Ò»Ñù
     */
    void reset();
};
