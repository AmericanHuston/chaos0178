package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;


@Config
@TeleOp(name = "arm", group = "TeleOp")
public class arm extends LinearOpMode {
    public DcMotorEx SliderLeft;
    public DcMotorEx SliderRight;
    public DcMotorEx Shoulder;
    double backRightPower;
    double frontRightPower;
    double frontLeftPower;
    double backLeftPower;
    enum armState{
        RESTING,
        BASKET,
        SPECIMEN,
        COLLECTION,
        above_bar,
        below_bar,
        prehang,
        posthang

    }
    armState state;

    public static double rest;

    public static double RESTING_VELOCITY = 200;
    public static double BASKET_VELOCITY = 200;
    public static double SPECIMEN_VELOCITY = 210;
    public static double COLLECTION_VELOCITY = 210;
    public static int hangHeight = 2500;
    public static double Slidervelocityup = 2500;
    public static double Slidervelocitydown = 1200;
    public static int resting_position = 50;
    public static int basket_position = 170;
    public static int specimen_position = 370;
    public static int collection_position = 500;
    //public static double wristpos_resting = 0.15;
    //public static double wristpos_basket = 0.6;
    //public static double wristpos_specimen = 0.8;
    //public static double wristpos_collection = 0.85;
    public static int slidersdown = 40;
    public static int slidersup = 3500;
    public static double MAX_POS     =  1.0;     // Maximum rotational position
    public static double MIN_POS     =  0.0;// Minimum rotational position
    public static int slider_above_bar_position = 1050;
    public static int slider_below_bar_position = 400;
    public static int shoulder_bar_position = 170;
    //public static double wrist_bar_position = 0.39;
    public static int shoulder_bar_velotity = 200;
    public static double kp = 0.2;
    public static double desired_claw_position;
    public static int desired_shoulder_position;
    public static double desired_shoulder_velocity;
    public static double output;
    public static int desired_slider_position;
    public static double desired_slider_velocity;
    public static double desired_wrist_position;


    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo wrist;
    Servo claw;
    TouchSensor sliderButton;
    //public static double  wrist_position = (MAX_POS - MIN_POS) / 2;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        imu = hardwareMap.get(IMU.class, "imu");
        sliderButton = hardwareMap.touchSensor.get("sliderButton");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        SliderLeft = hardwareMap.get(DcMotorEx.class, "SliderLeft");
        SliderRight = hardwareMap.get(DcMotorEx.class, "SliderRight");
        Shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
        Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SliderRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        SliderRight.setDirection(DcMotorSimple.Direction.REVERSE); //It needs to be reversed because...
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        final double sliderSpeed = 0.35;
        state = armState.RESTING;
        claw.setPosition(0.5);
        wrist.setPosition(0.1);
        boolean openClaw = true;
        boolean changedClaw = false;
        boolean openWrist = true;
        boolean changedWrist = false;


        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.back) {
                imu.resetYaw();
            }
            if(gamepad2.right_trigger > 0.01) {
                desired_claw_position = Range.scale(gamepad2.right_trigger, 0.0, 1.0, 0.5, 0.99);
            }
            if(gamepad2.right_bumper && !changedClaw){
                if(openClaw) {
                    desired_claw_position = 0.99;
                }else{
                    desired_claw_position = 0.5;
                }
                changedClaw = true;
                openClaw = !openClaw;
            } else if (!gamepad2.right_bumper) {
                changedClaw = false;
            }

            if(gamepad2.left_trigger > 0.01){
                desired_wrist_position = 0.25;
            }
            if(gamepad2.left_bumper && !changedWrist){
                if(openWrist) {
                    desired_wrist_position = 0.5;
                }else{
                    desired_wrist_position = 0.1;
                }
                changedWrist = true;
                openWrist = !openWrist;
            } else if (!gamepad2.left_bumper) {
                changedWrist = false;
            }
            if (gamepad2.right_stick_x> 0.01|| gamepad2.right_stick_x < -0.01)  {
                int shoulder_position = Shoulder.getTargetPosition();
                int shoulder_change = (int)gamepad2.right_stick_x * 10;
                desired_shoulder_position = shoulder_position + shoulder_change;;

            }

            // wrist_position = gamepad2.left_stick_y;
            if (gamepad2.y) { state = armState.RESTING; }
            if (gamepad2.x) { state = armState.BASKET; }
            if (gamepad2.a) { state = armState.SPECIMEN; }
            if (gamepad2.b) { state = armState.COLLECTION; }
            if (gamepad2.dpad_left) { state = armState.above_bar; }
            if (gamepad2.dpad_right) { state = armState.below_bar; }
            if (gamepad2.dpad_up) { state = armState.prehang; }
            if (gamepad2.dpad_down) { state = armState.posthang; }

            driving();
            if (gamepad1.a) { pointAtBasket(); }
            arm();
            if (gamepad1.dpad_up) { sliderMove(50);}
            if (gamepad1.dpad_down) {sliderMove(-50);}
            action();
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("arm ticks", rest);
            telemetry.addData("Current ", Shoulder.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ClawPos", output);
            telemetry.update();
        }
    }

    public void sliderMove(int moveTickAmount){
        if (state == armState.prehang) {
            hangHeight += moveTickAmount;
        }
    }
    //sets shoulder motor position need the right presets
    public void arm(){
        telemetry.addData("state", String.valueOf(state));
        switch (state){
            case RESTING:
                desired_shoulder_position = resting_position;
                desired_shoulder_velocity = RESTING_VELOCITY;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case BASKET:
                desired_shoulder_position = basket_position;
                desired_shoulder_velocity = BASKET_VELOCITY;
                //desired_wrist_position = wristpos_basket;
                desired_slider_position = slidersup;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case SPECIMEN:
                desired_shoulder_position = specimen_position;
                desired_shoulder_velocity = SPECIMEN_VELOCITY;
                //desired_wrist_position = wristpos_specimen;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case COLLECTION:
                desired_shoulder_position = collection_position;
                desired_shoulder_velocity = COLLECTION_VELOCITY;
                //desired_wrist_position = wristpos_collection;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case above_bar:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                //desired_wrist_position = wrist_bar_position;
                desired_slider_position = slider_above_bar_position;
                desired_slider_velocity = Slidervelocityup;
                break;
            case below_bar:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                //desired_wrist_position = wrist_bar_position;
                desired_slider_position = slider_below_bar_position;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case prehang:
                desired_shoulder_position = resting_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                desired_slider_position = hangHeight;
                desired_slider_velocity = Slidervelocityup;
                break;
            case posthang:
                desired_slider_position = resting_position;
                desired_slider_velocity = Slidervelocityup;
                break;
        }
    }

    //driving is working, field centric
    private void pointAtBasket() {
        double pointedAtBasket = -45.0;
        pointAtAngle(pointedAtBasket);
    }

    private void pointAtAngle(double pointAt){
        double MAXPOWER = 0.5;
        double  Kp = kp;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
        double power = Kp *(pointAt - currentYaw);
        power = Range.clip(power, -MAXPOWER, MAXPOWER);
        backLeftPower = power;
        frontLeftPower = power;
        backRightPower  = -power;
        frontRightPower = -power;
    }
    public void driving() {
        double y = -gamepad1.left_stick_y / 2; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x / 2;
        double rx = gamepad1.right_stick_x / 2;
        if (gamepad1.right_trigger >= 0.01) {
            y = y * 2;
            x = x * 2;
            rx = rx * 2;
        }
        //speed up/slow down
        if (gamepad1.left_trigger >= 0.01) {
            y = y / 2;
            x = x / 2;
            rx = rx / 2;
        }
        //When dpad down is pressed it will point at basket

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        rotX = rotX * 1.1;
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }

    public void action() {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
        wrist.setPosition(desired_wrist_position);
        claw.setPosition(desired_claw_position);
        Shoulder.setTargetPosition(desired_shoulder_position);
        Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shoulder.setVelocity(desired_shoulder_velocity);
        SliderLeft.setTargetPosition(desired_slider_position);
        SliderRight.setTargetPosition(desired_slider_position);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderLeft.setVelocity(desired_slider_velocity);
        SliderRight.setVelocity(desired_slider_velocity);
        if (sliderButton.isPressed()){
            SliderLeft.setMotorDisable();
            SliderRight.setMotorDisable();
        }


    }

}
