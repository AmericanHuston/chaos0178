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
    public DcMotor SliderLeft;//This stuff will exist at some point
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
    Servo wrist;
    //DcMotor shoulder;
    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");//guess what the stuff actually exists now
        //shoulder = hardwareMap.get(DcMotor.class, "Shoulder");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        arm = hardwareMap.get(Servo.class, "arm");
        wrist = hardwareMap.get(Servo.class, "wrist");
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
        final double sliderSpeed = 0.60;


        waitForStart();

        while(opModeIsActive()){
            if (gamepad1.back) {//resets the IMU
                imu.resetYaw();
            }
            if(this.gamepad2.dpad_up){//Slider top preset
                sliderPreset1();
            }
            if(this.gamepad2.dpad_down){//goes down, eventually needs to be run to position zero
                slidersGo(-sliderSpeed); //Go down, so negative
            }
            if(this.gamepad2.left_trigger> 0.1) {//Opens claw
                servo(claw, 0.5);
            }
            if(this.gamepad2.right_trigger > 0.1){//Closes the claw
                servo(claw, -0.5);
            }
            if(this.gamepad2.left_bumper){//wrist at 90
                servo(wrist, -0.5);
            }
            if(this.gamepad2.right_bumper){//wrist at zero
                servo(wrist, 0.5);
            }
            if(this.gamepad2.a){
                servo(arm, -0.4);
            }
            if(this.gamepad2.b){
                servo(arm, 0.4);
            }
            driving();//driving function
            if (gamepad1.a) {//point at basket right turn
                pointAtBasketRight();
            }
            if(gamepad1.b) { // point at basket left turn
                pointAtBasketLeft();
            }

            //shoulderJoint();//shoulder function
            action();//thinking function
            slidersStop();//hold to slider position
            telemetry.addData("Yaw", imu.getRobotYawPitchRollAngles().getYaw());//telemetry
            telemetry.update();//telemetry to screen
        }
    }
    //Fixes slider stopping issue
    public void slidersStop() {
        int rightTarget = SliderRight.getCurrentPosition();
        int leftTarget = SliderLeft.getCurrentPosition();
        SliderLeft.setTargetPosition(leftTarget);
        SliderRight.setTargetPosition(rightTarget);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setPower(0.2);
        SliderLeft.setPower(0.2);

    }
    public void slidersGo(double power){ //This will eventually be replaced by presets
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

    public void servo(Servo servo, double increment){ //servo open, close
        double position = servo.getPosition();
        position = position + increment;
        servo.setPosition(position); //Tell the servo to go to the correct pos
    }
    /*public void shoulderJoint(){
        if (gamepad2.left_stick_y != 0.0){
            double ENX = -gamepad2.left_stick_x;
            double EX = gamepad2.left_stick_x;
            double shoulderPower = (ENX - EX) * 0.4;
            shoulder.setPower(shoulderPower);
        }else {
            //This works but there are no software stops and gravity is causing it to smash both ends.
            int target = shoulder.getCurrentPosition();
            shoulder.setTargetPosition(target);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            shoulder.setPower(0.4);
        }
        shoulder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    */

    //driving is working, field centric
    public void pointAtBasketRight() { //still need to work on this
        double currentYaw = imu.getRobotYawPitchRollAngles().getYaw();
        double pointedAtBasket = -25.0; //The angle works okay, but it still only drives in one direction.
        double power = .50 * (.01 * (pointedAtBasket - currentYaw));
        if (pointedAtBasket < currentYaw) {
            backLeftPower = -power;
            frontLeftPower = -power;
            backRightPower = power;
            frontRightPower = power;
        } else {
            backLeftPower = power;
            frontLeftPower = power;
            backRightPower = -power;
            frontRightPower = -power;
        }
    }
    public void pointAtBasketLeft(){
        double currentYawL = imu.getRobotYawPitchRollAngles().getYaw();
        double pointedAtBasketL = -45.0;//this needs to be worked on
        double powerL = 0.50 * (0.01 *(pointedAtBasketL - currentYawL));
        if(pointedAtBasketL < currentYawL) {
            //I tried the arrow in the other direction (> instead of <) and it resulted the same as the above code
            backLeftPower = powerL;
            frontLeftPower = powerL;
            backRightPower  = -powerL;
            frontRightPower = -powerL;
        } else {
            backLeftPower = -powerL;
            frontLeftPower = -powerL;
            backRightPower = powerL;
            frontRightPower = powerL;
        }
    }
    public void driving() { //Field centric diving code

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
    public void action() { //setting motor powers
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }
    public void sliderPreset1(){ //top preset, the high basket, up_dpad
        int rightTargetPos = 3800;
        int leftTargetPos = 3800;
        if(SliderRight.getCurrentPosition() < rightTargetPos || SliderLeft.getCurrentPosition() < leftTargetPos) {
            SliderLeft.setPower(0.35);
            SliderRight.setPower(0.35);
            SliderLeft.setTargetPosition(leftTargetPos);
            SliderRight.setTargetPosition(rightTargetPos);
            telemetry.addData("SliderPreset1", "Active");
        }
        telemetry.addData("SliderPreset1", "Inactive");
    }
}