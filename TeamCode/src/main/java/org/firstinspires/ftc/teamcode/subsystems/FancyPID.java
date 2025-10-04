package org.firstinspires.ftc.teamcode.subsystems;

public class FancyPID {
        public double PID_P = 1;
        public double PID_I = 0;
        public double PID_D = 0;
        public double velo = 0;
        public double pidInt = 0;
        public double pidDev = 0;
        public double errorCur = 0;
        public double errorPrev = 0;
        public double iMax;
        public double iRange;
        public double Kp;
        public double Ki;
        public double Kd;
        public double target;
        public boolean arrived;
        public double errorTol;
        public double dTol;

        public void PID(){

        }

        public void setCoefficients(double p, double i, double d) {
            Kp = p;
            Ki = i;
            Kd = d;
        }
        public void init() {
            pidInt = 0;
            pidDev = 0;
            errorPrev = target;
        }

        public void update(double current) {
            errorCur = target - current;
            PID_P = errorCur * Kp;

            pidDev = errorCur - errorPrev;
            errorPrev = errorCur;
            PID_D = pidDev * Kd;

            if (Math.abs(PID_P) >= iRange) {
                pidInt = 0;
            } else {
                pidInt += errorCur;
            }

            if (Math.abs(pidInt * Ki) >= iMax) {
                pidInt = Math.signum(pidInt) * iMax / Ki;
            }
            if ((Math.signum(errorCur) != Math.signum(pidInt)) || (Math.abs(errorCur) < errorTol)) {
                pidInt = 0;
            }
            if (Math.abs(errorCur) <= errorTol && Math.abs(pidDev) <= dTol) {
                arrived = true;
            }
            else{
                arrived = false;
            }
            PID_I = pidInt * Ki;

            velo = PID_P + PID_I + PID_D;
        }

}
