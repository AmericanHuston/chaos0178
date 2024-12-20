package org.firstinspires.ftc.teamcode.TeleOp;


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
    double backRightPower;
    double frontRightPower;
    double frontLeftPower;
    double backLeftPower;
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo arm;
    Servo claw;
    DcMotor elbow;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        elbow = hardwareMap.get(DcMotor.class, "Shoulder");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        SliderLeft = hardwareMap.get(DcMotor.class, "SliderLeft");
        SliderRight = hardwareMap.get(DcMotor.class, "SliderRight");
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


        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.back) {
                imu.resetYaw();
            }
            if(this.gamepad2.dpad_up){
                sliderPreset1();
//                slidersGo(sliderSpeed);
            }
            if(this.gamepad2.dpad_down){
                slidersGo(-sliderSpeed); //Go down, so negative
            }
            if(this.gamepad2.left_trigger> 0.1) {
                servo(claw, 0.5);
            }
            if(this.gamepad2.right_trigger > 0.1){
                servo(claw, -0.5);
            }

            driving();
            if (gamepad1.b){
                pointAtAngle(-45.0);
            }
            if (gamepad1.x){
                pointAtAngle(0.0);
            }
            if (gamepad1.a) {
                pointAtBasket();
            }

            elbowJoint();
            action();
            slidersStop();
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.update();
        }
    }
    //Fixes slider stopping issue
    public void slidersStop(){
        int rightTarget = SliderRight.getCurrentPosition();
        int leftTarget = SliderLeft.getCurrentPosition();
        SliderLeft.setTargetPosition(leftTarget);
        SliderRight.setTargetPosition(rightTarget);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setPower(0.2);
        SliderLeft.setPower(0.2);

    }
    public void slidersGo(double power){
        int leftPos = SliderLeft.getCurrentPosition();
        int rightPos = SliderRight.getCurrentPosition();
        SliderRight.setTargetPosition(100);
        SliderLeft.setTargetPosition(100);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("leftPos", leftPos);
        telemetry.addData("rightPos", rightPos);
        telemetry.update();
        SliderLeft.setPower(power);
        SliderRight.setPower(power);
    }

    public void servo(Servo servo, double increment){
        double position = servo.getPosition();
        position = position + increment;
        servo.setPosition(position); //Tell the servo to go to the correct pos
    }
    public void elbowJoint(){
        //This works but there are no software stops and gravity is causing it to smash both ends.
        double ENX = -gamepad2.left_stick_x;
        double EX = gamepad2.left_stick_x;
        double elbowPower = (ENX - EX) * 0.5;
        elbow.setPower(elbowPower);
    }
    //driving is working, field centric
/*    private void pointAtBasket() { //still need to work on this
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
        double pointedAtBasket = -25.0; //The angle works okay, but it still only drives in one direction.
        double power = .50 * (.01 * (pointedAtBasket - currentYaw));
        if(pointedAtBasket < currentYaw) {
            backLeftPower = -power;
            frontLeftPower = -power;
            backRightPower = power;
            frontRightPower = power;

        }else{

            backLeftPower = power;
            frontLeftPower = power;
            backRightPower = -power;
            frontRightPower = -power;
        }
*/
    private void pointAtBasket(){
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
        double pointedAtBasket = -60.0; //numbers need to be tested
        double power = 0.50 * (0.01 *(pointedAtBasket - currentYaw));
        if(pointedAtBasket < currentYaw) {
            backLeftPower = power;
            frontLeftPower = power;
            backRightPower  = -power;
            frontRightPower = -power;
        } else {
            backLeftPower = -power;
            frontLeftPower = -power;
            backRightPower = power;
            frontRightPower = power;
        }
        telemetry.addData("Power: ", power);
        telemetry.update();
    }
    private void pointAtAngle(double pointAt){
        double MAXPOWER = 0.5;
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
        double power = 0.05 *(pointAt - currentYaw);
        power = clamp(power, -MAXPOWER, MAXPOWER);
        backLeftPower = power;
        frontLeftPower = power;
        backRightPower  = -power;
        frontRightPower = -power;
    }

    public double clamp(double input, double min, double max) {
        double result = input;
        if (input > max) {
            result = max;
        } else if (input < min) {
            result = min;
        }
        return result;
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
    }
    //High basket, up_dpad
    public void sliderPreset1(){
        int rightTargetPos = 3800;
        int leftTargetPos = 3800;
        if(SliderRight.getCurrentPosition() < rightTargetPos || SliderLeft.getCurrentPosition() < leftTargetPos) {
            SliderLeft.setPower(0.35);
            SliderRight.setPower(0.35);
            SliderLeft.setTargetPosition(leftTargetPos);
            SliderRight.setTargetPosition(rightTargetPos);
            telemetry.addData("sliderpreset1", "Active");
        }
        telemetry.addData("sliderpreset1", "inactive");

    }
}