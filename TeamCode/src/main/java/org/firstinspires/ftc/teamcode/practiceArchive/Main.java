//package org.firstinspires.ftc.teamcode.practiceArchive;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.PoseVelocity2d;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.MecanumDrive;
//import org.firstinspires.ftc.teamcode.subsystems.revolver.DrumIntakeTurretManager;
//
//@TeleOp
//@Disabled
//public class Main extends OpMode {
//    //public static int pointer = 0;
//    public static float gain = 1;
//    FtcDashboard dash = FtcDashboard.getInstance();
//    Telemetry t2 = dash.getTelemetry();
//    DrumIntakeTurretManager pController = new DrumIntakeTurretManager();
//    roller rollerControl = new roller();
//    MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
//    @Override
//    public void init() {
//        pController.init(hardwareMap);
//        pController.curMode = DrumIntakeTurretManager.revMode.INTAKING;
//        rollerControl.rollerMode = roller.RM.IDLE;
//        rollerControl.init();
//    }
//    @Override
//    public void loop() {
//        //-------------------gamepad actions--------------------------
//        //changing the gain
//        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_x, gamepad1.left_stick_y), gamepad1.right_stick_x));
//        if (gamepad1.dpad_up && !gamepad1.dpadUpWasPressed()){
//            gain += 0.05;
//            pController.setGain(gain);
//        }
//        else if (gamepad1.dpad_down && !gamepad1.dpadDownWasPressed()){
//            gain -= 0.05;
//            pController.setGain(gain);
//        }
//        //intake
//        if(gamepad1.left_trigger > 0){
//            rollerControl.rollerMode = roller.RM.INTAKE;
//        }
//        else if(gamepad1.left_bumper){
//            rollerControl.rollerMode = roller.RM.OUTTAKE;
//        }
//        else{
//            rollerControl.rollerMode = roller.RM.IDLE;
//        }
//        //shooting&revolver
//        if (gamepad1.cross){
//            pController.curMode = DrumIntakeTurretManager.revMode.CONTFIRE;
//        }
//        else if (gamepad1.a){
//            pController.tarColor = "green";
//            pController.curMode = DrumIntakeTurretManager.revMode.FIRECOLOR;
//        }
//        else if (gamepad1.right_bumper && !gamepad1.rightBumperWasPressed()){
//            pController.tarColor = "purple";
//            pController.curMode = DrumIntakeTurretManager.revMode.FIRECOLOR;
//        }
//        else if(pController.curMode == DrumIntakeTurretManager.revMode.CONTFIRE ){
//            pController.curMode = DrumIntakeTurretManager.revMode.INTAKING;
//        }
//        //--------------------------loopActions------------------------
//        pController.update();
//        pController.updateTelemetry(t2);
//        pController.updateTelemetry(telemetry);
//        rollerControl.update();
//    }
//}
