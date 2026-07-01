class PID
{
    double _dt;
    double _min;
    double _max;
    double _Kp;
    double _Ki;
    double _Kd;
    double _pre_error;
    double _integral;

    public:
    // Kp -  proportional gain
    // Ki -  Integral gain
    // Kd -  derivative gain
    // dt -  loop interval time
    // max - maximum value of manipulated variable
    // min - minimum value of manipulated variable
    PID( double dt, double min, double max, double Kp, double Ki, double Kd )
    {
        _dt=dt;
        _min=min;
        _max=max;
        _Kp=Kp;
        _Ki=Ki;
        _Kd=Kd;
        _pre_error=0;
        _integral=0;
    }
    ~PID()
    {

    }

    // Returns the manipulated variable given a setpoint and current process value
    double Calculate(double setpoint,double pv)
    {
        // Calculate error
        double error = setpoint - pv;

        // Proportional term
        double Pout = _Kp * error;

        // Integral term
        _integral += error * _dt;
        double Iout = _Ki * _integral;

        // Derivative term
        double derivative = (error - _pre_error) / _dt;
        double Dout = _Kd * derivative;

        // Calculate total output
        double output = Pout + Iout + Dout;

        // Restrict to max/min
        if( output > _max )
        {
            output = _max;
        }
        else if( output < _min )
        {
            output = _min;
        }

        // Save error to previous error
        _pre_error = error;

        return output;
    }

    void Reset()
    {
        _integral=0.0;
        _pre_error=0.0;
    }
};
