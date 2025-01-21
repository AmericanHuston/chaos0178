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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Locale;

@Config
@TeleOp(name="CoachTesting", group="Linear OpMode")
@Disabled
public class CoachTesting extends LinearOpMode {
    boolean rampUp = true;
    public static double position = 0.0;
    public static double MAX_POS =  1.0;
    public static double MIN_POS = 0.0;
    public static double INCREMENT = 0.02;
    @Override
    public void runOpMode() throws InterruptedException {
        //Don't edit code below this point
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        //Don't edit code above this point
        GoBildaPinpointDriver pinpoint; // Declare OpMode member for the Odometry Computer


        double SLIDER_POWER = 0.40;
        double arm_max = 0.52;
        double arm_min = 0.2;
        double claw_min = 0.1;
        double claw_max = 0.53;
        int slider_max = 4000;
        int slider_min = 30;
        int CYCLE_MS = 50;
        DcMotor sliderLeft = hardwareMap.get(DcMotor.class, "SliderLeft");
        DcMotor sliderRight = hardwareMap.get(DcMotor.class, "SliderRight");
        // The sliders are mirrored from each other, so we need to reverse the direction of one of them.
        sliderRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // Hold the sliders in position
        sliderLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");

        // Reverse the right side motors. This may be wrong for your setup.
        // If your robot moves backwards when commanded to go forwards,
        // reverse the left side instead.
        // See the note about this earlier on this page.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class,"pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        double DesiredArmPosition = 0.0;
        double DesiredClawPosition = 0.0;
        double DesiredServo0Position = 0.0;
        waitForStart();

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            //Simultaneous motion causes problems so wait to move until the other controller stops moving.
            double DesiredSliderPower;
            if (gamepad2.dpad_up){
                DesiredSliderPower = SLIDER_POWER;
            } else if (gamepad2.dpad_down){
                DesiredSliderPower = -SLIDER_POWER;
            } else {
                DesiredSliderPower = 0.0;
            }

            if (gamepad2.a) {
                DesiredArmPosition = arm_max;
            } else if (gamepad2.b){
                DesiredArmPosition = arm_min;
            }

            if (gamepad2.x) {
                DesiredClawPosition = claw_min;
            } else if (gamepad2.y) {
                DesiredClawPosition = claw_max;
            }

            if (gamepad2.right_bumper) {
                // Tweak Max/min values when the right bumper is held down and another button is pushed.
                if (gamepad2.a) {
                    arm_max += INCREMENT;
                }
                if (gamepad2.b) {
                    arm_min += INCREMENT;
                }
                if (gamepad2.x) {
                    claw_min += INCREMENT;
                }
                if (gamepad2.y) {
                    claw_max += INCREMENT;
                }
                if (gamepad2.dpad_up) {
                    slider_max += 10;
                }
                if (gamepad2.dpad_down) {
                    slider_min += 10;
                }
            }

            if (gamepad2.left_bumper) {
                if (gamepad2.a) {
                    arm_max -= INCREMENT;
                }
                if (gamepad2.b) {
                    arm_min -= INCREMENT;
                }
                if (gamepad2.x) {
                    claw_min -= INCREMENT;
                }
                if (gamepad2.y) {
                    claw_max -= INCREMENT;
                }
                if (gamepad2.dpad_up) {
                    slider_max -= 10;
                }
                if (gamepad2.dpad_down) {
                    slider_min -= 10;
                }
            }


            pinpoint.update();
            Pose2D pos = pinpoint.getPosition();
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            double y = gamepad1.left_stick_y/2; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x/2;
            double rx = gamepad1.right_stick_x/2;
            if (gamepad1.right_trigger >= 0.01){
                y = y*2;
                x = x*2;
                rx = rx*2;
            }
            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            //rev hub facing away if you want this to work.
            if (gamepad1.back) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // Sliders via motor power
            // sliderLeft.setPower(DesiredSliderPower);
            //sliderRight.setPower(DesiredSliderPower);

            // Sliders via position
            if (gamepad2.dpad_up) {
                int target_position = slider_max;
                sliderLeft.setTargetPosition(target_position);
                sliderRight.setTargetPosition(target_position);
                sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderLeft.setPower(0.5);
                sliderRight.setPower(0.5);
            }

            if (gamepad2.dpad_down) {
                int target_position = slider_min;
                sliderLeft.setTargetPosition(target_position);
                sliderRight.setTargetPosition(target_position);
                sliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sliderLeft.setPower(0.5);
                sliderRight.setPower(0.5);
            }


            telemetry.addData("DesiredSliderPower", DesiredSliderPower);
            telemetry.addData("SliderLeftPos", sliderLeft.getCurrentPosition());
            telemetry.addData("SliderRightPos", sliderRight.getCurrentPosition());
            telemetry.addData("DesiredArmPosition", DesiredArmPosition);
            telemetry.addData("DesiredClawPosition", DesiredClawPosition);
            telemetry.addData("claw_min", claw_min);
            telemetry.addData("claw_max", claw_max);
            telemetry.addData("arm_min", arm_min);
            telemetry.addData("arm_max", arm_max);
            telemetry.addData("slider_min", slider_min);
            telemetry.addData("slider_max", slider_max);


            telemetry.addData("ROBOT", "Status:" + "Front Right =" + Math.round(frontRightPower * 100.0)/100.0);
            telemetry.addData("ROBOT", "Status:" + "Front Left =" + Math.round(frontLeftPower * 100.0)/100.0);//Echos information using classification.
            telemetry.addData("ROBOT", "Status:" + "Back Left =" + Math.round(backLeftPower * 100.0)/100.0);
            telemetry.addData("ROBOT", "Status:" + "Back Right =" + Math.round(backRightPower * 100.0)/100.0);

            telemetry.update();
        }
    }
}
