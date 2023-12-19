struct PID
{
    float target_val;
    float actual_val;
    float err;
    float err_last;
    float Kp, Ki, Kd;
    float integral;

    PID(float target_val, float Kp, float Ki, float Kd) : target_val(target_val), Kp(Kp), Ki(Ki), Kd(Kd)
    {
        actual_val = err = err_last = integral = 0.0;
    }
    float calc_pid(float temp_val)
    {
        err = target_val - temp_val;
        integral += err;
        actual_val = Kp * err + Ki * integral + Kd * (err - err_last);
        err_last = err;
        return actual_val;
    }
    void set_pid(float p, float i, float d) { Kp = p, Ki = i, Kd = d; }
};
