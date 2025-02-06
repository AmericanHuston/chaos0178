package org.firstinspires.ftc.teamcode.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MeasureSliderMovement", group = "TeleOp")
@Config
@Disabled
public class MeasureSliderMovement extends LinearOpMode {

    public DcMotor SliderLeft;
    public DcMotor SliderRight;
    private Servo claw;

    @Override
    public void runOpMode() {
        claw = hardwareMap.get(Servo.class, "claw");
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
        SliderLeft.getCurrentPosition();
        int leftPos;
        int rightPos;
        double power = 0.35;
        //Don't edit code above this point
        while (opModeIsActive()) {
            while (this.gamepad2.dpad_up) {
                SliderLeft.setPower(power);
                SliderRight.setPower(-power);
                driving();
                servo();
                leftPos = SliderLeft.getCurrentPosition();
                rightPos = SliderRight.getCurrentPosition();
                telemetry.addData("leftPos", -leftPos);
                telemetry.addData("rightPos", rightPos);
                telemetry.update();
            }
            driving();
            servo();
            SliderRight.setPower(0.0);
            SliderLeft.setPower(0.0);
            while (this.gamepad2.dpad_down) {
                SliderLeft.setPower(-power);
                SliderRight.setPower(power);
                servo();
                driving();
                leftPos = SliderLeft.getCurrentPosition();
                rightPos = SliderRight.getCurrentPosition();
                telemetry.addData("leftPos", leftPos);
                telemetry.addData("rightPos", -rightPos);
                telemetry.update();
            }
            servo();
            driving();
            SliderRight.setPower(0.0);
            SliderLeft.setPower(0.0);
        }
    }
    public void servo(){
        double INCREMENT = 0.01;
        double MAX_POS = 1.0;
        double MIN_POS = 0.0;
        int CYCLE_MS = 50;
        double position = (MIN_POS - MAX_POS);
        while (this.gamepad2.b) {
            position = position + INCREMENT;
        }
        while (this.gamepad2.a){
            position = position - INCREMENT;
        }
        claw.setPosition(position); //Tell the servo to go to the correct pos
        sleep(CYCLE_MS);
        idle();
    }
    public void driving() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        double y = -gamepad1.left_stick_y / 2; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x / 2;
        double rx = gamepad1.right_stick_x / 2;
        if (gamepad1.right_trigger >= 0.01) {
            y = y * 2;
            x = x * 2;
            rx = rx * 2;
        }
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        if (gamepad1.left_stick_button) {
            imu.resetYaw();
        }
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}