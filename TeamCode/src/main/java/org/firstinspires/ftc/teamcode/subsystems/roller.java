package org.firstinspires.ftc.teamcode.subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;

public class roller {
    DcMotor roller;

    double intakeSpeed = 1;
    int lastDir = 0;
    public enum RM {
        INTAKE,
        OUTTAKE,
        HOLD,
        IDLE
    }
    public RM rollerMode = RM.IDLE;

    public void init() {
        roller = hardwareMap.get(DcMotor.class, "roller");
    }

    public void update() {
        if (rollerMode == RM.INTAKE){
            roller.setPower(intakeSpeed);
            lastDir = 1;
        }
        else if(rollerMode == RM.OUTTAKE){
            roller.setPower(-intakeSpeed);
            lastDir = 0;
        }
        else if(rollerMode == RM.HOLD){
            roller.setPower(0.2);
        }
        else if(rollerMode == RM.IDLE){
            roller.setPower(lastDir * 0.1);
        }
    }



}
