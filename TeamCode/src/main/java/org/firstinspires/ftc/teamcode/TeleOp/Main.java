package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Main", group = "TeleOp")
public class Main extends LinearOpMode {
    public DcMotor SliderLeft;
    public DcMotor SliderRight;
    private Servo claw;
    private Servo arm;
    double backRightPower;
    double frontRightPower;
    double frontLeftPower;
    double backLeftPower;
    @Override
    public void runOpMode() throws InterruptedException {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        SliderLeft = hardwareMap.get(DcMotor.class, "SliderLeft");
        SliderRight = hardwareMap.get(DcMotor.class, "SliderRight");
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        SliderLeft.getCurrentPosition();
        final double sliderSpeed = 0.35;
        while(opModeIsActive()){
            if (gamepad1.left_stick_button) {
                imu.resetYaw();
            }
            driving();
            action();
            if (this.gamepad2.dpad_up){
                slidersGo(sliderSpeed);
                driving();
                action();
            }
            slidersStop();
            if(this.gamepad2.dpad_down){
                slidersGo(-sliderSpeed);
                driving();
                action();
            }
            slidersStop();
            if(this.gamepad2.left_trigger > 0.1){
                servo();
                driving();
                action();
            }
            if(this.gamepad2.right_trigger > 0.1){
                servo();
                driving();
                action();
            }
        }
    }
    //Fixes slider stopping issue
    public void slidersStop(){
        int rightTarget = SliderRight.getCurrentPosition();
        int leftTarget = SliderLeft.getCurrentPosition();
        SliderRight.setPower(0.1);
        SliderLeft.setPower(0.1);
        SliderLeft.setTargetPosition(leftTarget);
        SliderRight.setTargetPosition(rightTarget);
    }
    //Sliders don't stop
    public void slidersGo(double power){
        SliderLeft.setPower(power);
        SliderRight.setPower(-power);
        double leftPos = SliderLeft.getCurrentPosition();
        double rightPos = SliderRight.getCurrentPosition();
        telemetry.addData("leftPos", -leftPos);
        telemetry.addData("rightPos", rightPos);
        telemetry.update();
    }
    //yet to be tested, kind of works
    public void servo(){
        double INCREMENT = 0.5;
        int CYCLE_MS = 10;
        double position = claw.getPosition();
        while(this.gamepad2.right_trigger > 0.1) {
            position = position + INCREMENT;
            claw.setPosition(position); //Tell the servo to go to the correct pos
            sleep(CYCLE_MS);
            idle();
        }
        while(this.gamepad2.left_trigger > 0.1){
            position = position - INCREMENT;
            claw.setPosition(position); //Tell the servo to go to the correct pos
            sleep(CYCLE_MS);
            idle();
        }
        claw.setPosition(position); //Tell the servo to go to the correct pos
        sleep(CYCLE_MS);
        idle();
    }
    //driving is working, field centric
    public void driving() {
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        if (gamepad1.left_stick_button) {
            imu.resetYaw();
        }
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        imu.initialize(parameters);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    public void action() {
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class,"frontLeft");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class,"backLeft");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class,"frontRight");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class,"backRight");
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
}
