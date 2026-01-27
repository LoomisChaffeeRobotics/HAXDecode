package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.practiceArchive.limelighttesting;

@TeleOp
@Config
public class TurretSpeedEvalutor extends OpMode {
    DcMotorEx innerTurret;
    DcMotorEx outerTurret;
    limelighttesting limelight;
    FtcDashboard Dash=FtcDashboard.getInstance();
    Telemetry t2=Dash.getTelemetry();
    MecanumDrive drive;

    double currentlocationx = 0;
    double rawdistancex = 0;
    double currentlocationy = 0;
    double rawdistancey = 0;
    double rawDistance_inch = 0;
    double rawDistance_meters = 0;
    double currentvelocity_x = 0;
    double currentvelocity_y = 0;
    double velocitymagnitude = 0;
    double costheta = 0;
    double abs_distance;
    double tof = 0;
    double distanceUpdatedAfterVector_inch = 0;
    double distanceUpdatedAfterVector_meter = 0;
    double projectionMagnitude_inch = 0;
    double upperTurretSpeed = 0;
    double lowerTurretSpeed = 0;

    @Override
    public void init() {
        limelight = new limelighttesting();
        limelight.init(); //i forgot how to do it emam can you help  it's gonna give you an error probably
        innerTurret=hardwareMap.get(DcMotorEx.class, "innerTurret");
        outerTurret=hardwareMap.get(DcMotorEx.class, "outerTurret");

        innerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        innerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outerTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        innerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outerTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
    }
    public double timeOfFlight (double distance){
        abs_distance = Math.abs(distance);
        if(abs_distance<=1.00) return 0.2*abs_distance+0.5;
        else if(1.00<abs_distance && abs_distance<=1.50) return 0.16*abs_distance+0.54;
        else if(1.50<abs_distance && abs_distance<=2.00) return 0.14*abs_distance+0.57;
        else if(2.00<abs_distance && abs_distance<=2.50) return 0.16*abs_distance+0.53;
        else if(2.50<abs_distance && abs_distance<=3.00) return 0.18*abs_distance+0.48;
        else if(3.00<abs_distance && abs_distance<=3.50) return 0.14*abs_distance+0.6;
        else return 0.2*abs_distance+0.39;
    }
    public double findUpperSpeed (double distance){
        abs_distance = Math.abs(distance);
        if(abs_distance<=1.00) return 36*abs_distance+2353;
        else if(1.00<abs_distance && abs_distance<=1.50) return 374*abs_distance+2015;
        else if(1.50<abs_distance && abs_distance<=2.00) return 596*abs_distance+1682;
        else if(2.00<abs_distance && abs_distance<=2.50) return 738*abs_distance+1398;
        else if(2.50<abs_distance && abs_distance<=3.00) return 912*abs_distance+963;
        else if(3.00<abs_distance && abs_distance<=3.50) return 1134*abs_distance+209;
        else return 0.2*abs_distance+0.39;
    }
    public double findLowerSpeed (double distance){
        abs_distance = Math.abs(distance);
        if(abs_distance<=1.00) return 1080*abs_distance+1575;
        else if(1.00<abs_distance && abs_distance<=1.50) return 1006*abs_distance+1649;
        else if(1.50<abs_distance && abs_distance<=2.00) return 1014*abs_distance+1637;
        else if(2.00<abs_distance && abs_distance<=2.50) return 938*abs_distance+1789;
        else if(2.50<abs_distance && abs_distance<=3.00) return 932*abs_distance+1804;
        else if(3.00<abs_distance && abs_distance<=3.50) return 1044*abs_distance+1468;
        else return 1074*abs_distance+1363;
    }
    @Override
    public void loop() {
        currentlocationx = limelight.botpose.getPosition().x;
        currentlocationy = limelight.botpose.getPosition().y;
        rawdistancex = (-58.346457) - currentlocationx;
        rawdistancey = (-55.629921) - currentlocationy;

        //get distance to goal
        rawDistance_inch=Math.sqrt(rawdistancex*rawdistancex + rawdistancey*rawdistancey);
        rawDistance_meters=rawDistance_inch*2.54/100;

        currentvelocity_x = drive.localizer.update().linearVel.x;
        currentvelocity_y = drive.localizer.update().linearVel.y;
        velocitymagnitude = Math.sqrt(currentvelocity_x*currentvelocity_x + currentvelocity_y*currentvelocity_y);

        //get cos theta of angle between two vectors;
        costheta = ((currentlocationx * currentvelocity_x) + (currentlocationy * currentvelocity_y)) / (rawDistance_inch * velocitymagnitude);

        //evaluate projection
        projectionMagnitude_inch = velocitymagnitude * costheta;

        //find time of flight
        tof = timeOfFlight(rawDistance_meters);

        distanceUpdatedAfterVector_inch = rawDistance_inch - projectionMagnitude_inch*tof;
        distanceUpdatedAfterVector_meter = distanceUpdatedAfterVector_inch*2.54/100;

        //evaluate turret speed
        upperTurretSpeed = findUpperSpeed(distanceUpdatedAfterVector_meter);
        lowerTurretSpeed = findLowerSpeed(distanceUpdatedAfterVector_meter);

        outerTurret.setVelocity(upperTurretSpeed);
        innerTurret.setVelocity(lowerTurretSpeed);
    }
}
