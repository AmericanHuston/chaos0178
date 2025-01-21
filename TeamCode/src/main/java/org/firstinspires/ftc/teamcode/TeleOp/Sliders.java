package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@Disabled
@TeleOp(name="TestSliders", group="TeleOp")
public class Sliders extends LinearOpMode {

    public DcMotor sliderLeft = hardwareMap.get(DcMotor.class, "SliderLeft");
    public DcMotor sliderRight = hardwareMap.get(DcMotor.class, "SliderRight");

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        double power = 0.35;
        while (opModeIsActive()) {
            while (this.gamepad1.dpad_up) {
                sliderLeft.setPower(power);
                sliderRight.setPower(power);
            }
            sliderRight.setPower(0.0);
            sliderLeft.setPower(0.0);
            while (this.gamepad1.dpad_down) {
                sliderLeft.setPower(-power);
                sliderRight.setPower(-power);
            }
            sliderRight.setPower(0.0);
            sliderLeft.setPower(0.0);
        }
    }
}