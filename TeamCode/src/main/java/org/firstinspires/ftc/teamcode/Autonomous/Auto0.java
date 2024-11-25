package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.Vector;

@Autonomous(name = "Hope0", group = "Hope")
public class Auto0 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

    }
    //Speed is the power of the motors.
    //Vector "location" will be a standard x,y vector.
    private void driving(double speed, Vector<Integer> location){
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

    }
}
