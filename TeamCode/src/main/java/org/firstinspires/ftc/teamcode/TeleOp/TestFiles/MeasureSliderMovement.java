package org.firstinspires.ftc.teamcode.TeleOp.TestFiles;

import static org.firstinspires.ftc.teamcode.TeleOp.TestFiles.Methods.driving;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MeasureSliderMovement", group = "TeleOp")
@Config
public class MeasureSliderMovement extends LinearOpMode {

    public DcMotor SliderLeft;
    public DcMotor SliderRight;
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        SliderLeft = hardwareMap.get(DcMotor.class, "SliderLeft");
        SliderRight = hardwareMap.get(DcMotor.class, "SliderRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        //Don't edit code below this point
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        int leftPos = SliderLeft.getCurrentPosition();
        int rightPos = SliderRight.getCurrentPosition();
        double power = 0.35;
        //Don't edit code above this point
        while (opModeIsActive()) {
            while (this.gamepad2.dpad_up) {
                SliderLeft.setPower(power);
                SliderRight.setPower(-power);
                driving();
                leftPos = SliderLeft.getCurrentPosition();
                rightPos = SliderRight.getCurrentPosition();
                telemetry.addData("leftPos", -leftPos);
                telemetry.addData("rightPos", rightPos);
                telemetry.update();
            }
            driving();
            SliderRight.setPower(0.0);
            SliderLeft.setPower(0.0);
            while (this.gamepad2.dpad_down) {
                SliderLeft.setPower(-power);
                SliderRight.setPower(power);
                driving();
                leftPos = SliderLeft.getCurrentPosition();
                rightPos = SliderRight.getCurrentPosition();
                telemetry.addData("leftPos", -leftPos);
                telemetry.addData("rightPos", rightPos);
                telemetry.update();
            }
            driving();
            SliderRight.setPower(0.0);
            SliderLeft.setPower(0.0);
        }
    }
}