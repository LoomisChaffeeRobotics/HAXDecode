package org.firstinspires.ftc.teamcode.practiceArchive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class DrumIndexer extends OpMode{
        DcMotor encoder;
        int groundIntakeSlot;
        double CurrentTick;
        double CurrentAngle;

        @Override
        public void init() {
            encoder = hardwareMap.get(DcMotorEx.class, "encoder");
            encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            groundIntakeSlot=0;
        }

        @Override
        public void loop() {
            CurrentTick = (encoder.getCurrentPosition()) % 8192;
            while(CurrentTick<0) CurrentTick+=8192;
            CurrentAngle = (CurrentTick * 360 / 8192) % 360;

            if (340 <= CurrentAngle && CurrentAngle <360) groundIntakeSlot = 1;
            else if (0 <= CurrentAngle && CurrentAngle <=20) groundIntakeSlot = 1;
            else if (100 <= CurrentAngle && CurrentAngle <= 140) groundIntakeSlot = 2;
            else if (220 <= CurrentAngle && CurrentAngle <= 260) groundIntakeSlot = 3;
            else groundIntakeSlot = 0;

            telemetry.addData("Slot #", groundIntakeSlot);
            telemetry.addData("Current Tick", CurrentTick);
            telemetry.addData("Current Angle", CurrentAngle);
            telemetry.update();
        }

}
