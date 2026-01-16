package org.firstinspires.ftc.teamcode.subsystems;

public class TurretPID {
        public double PID_P = 1;
        public double PID_I = 0;
        public double PID_D = 0;
        public double PID_FRot = 0;
        public double PID_FVel = 0;
        public double out = 0;
        public double pidInt = 0;
        public double pidDev = 0;
        public double errorCur = 0;
        public double errorPrev = 0;
        public double iMax;
        public double iRange;
        public double Kp;
        public double Ki;
        public double Kd;
        public double KFR;
        public double KFV;
        public double target = 0;
        double maxLimit = 18000;
        double minLimit = -12000;
        public void setCoefficients(double p, double i, double d, double fr, double fv) {
            Kp = p;
            Ki = i;
            Kd = d;
            KFR = fr;
            KFV = fv;
        }
        public void init() {
            pidInt = 0;
            pidDev = 0;
            errorPrev = target;
        }
        public void update(double current, double robotRotVel, double robotOrthogVel) {
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
            if ((Math.signum(errorCur) != Math.signum(pidInt))) {
                pidInt = 0;
            }
            PID_I = pidInt * Ki;

            PID_FRot = robotRotVel * KFR; // rotational velocity will be in rad/sec
            PID_FVel = robotOrthogVel * KFV; // orthog velocity will be in dist/sec


            if (current > minLimit && current < maxLimit) {
                out = PID_P + PID_I + PID_D + PID_FRot + PID_FVel;
            } else if (current <= minLimit && Math.signum(out) == 1){
                out = PID_P + PID_I + PID_D + PID_FRot + PID_FVel;
            } else if (current >= maxLimit && Math.signum(out) == -1){
                out = PID_P + PID_I + PID_D + PID_FRot + PID_FVel;
            } else {
                out = 0;
            }
        }

}
