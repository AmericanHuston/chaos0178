package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "FirstAuto", group = "TeleOp")
public class FirstAuto extends LinearOpMode {
    public DcMotor SliderLeft;
    public DcMotor SliderRight;
    private Servo claw;
    private Servo arm;

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
            while(this.gamepad2.dpad_up){
                sliders(sliderSpeed);
                driving();
            }
            while(this.gamepad2.dpad_down){
                sliders(-sliderSpeed);
                driving();
            }
            while(this.gamepad2.left_trigger > 0.0){
                double increment = this.gamepad2.left_trigger;
                servo(increment);
                driving();
            }
            while(this.gamepad2.right_trigger > 0.0){
                double increment = this.gamepad2.right_trigger;
                servo(increment);
                driving();
            }
        }
    }
    public void sliders(double power){
        SliderLeft.setPower(power);
        SliderRight.setPower(-power);
        double leftPos = SliderLeft.getCurrentPosition();
        double rightPos = SliderRight.getCurrentPosition();
        telemetry.addData("leftPos", -leftPos);
        telemetry.addData("rightPos", rightPos);
        telemetry.update();
    }
    public void servo(double INCREMENT){
        double MAX_POS = 1.0;
        double MIN_POS = 0.0;
        int CYCLE_MS = 50;
        double position = claw.getPosition();
        position = position + INCREMENT;
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
        double y = gamepad1.left_stick_y / 2; // Remember, Y stick value is reversed
        double x = -gamepad1.left_stick_x / 2;
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
