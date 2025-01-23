package org.firstinspires.ftc.teamcode.EnvironmentLocations;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.EnvironmentLocations.LocalUtils.*;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Board0 {

    //Initialization:
    DcMotorEx SliderLeft;
    DcMotorEx SliderRight;
    DcMotorEx Shoulder;
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo wrist;
    Servo claw;
    TouchSensor sliderButton;
    public enum armStates {
        RESTING,
        BASKET,
        SPECIMEN,
        COLLECTION,
        ABOVE_BAR,
        BELOW_BAR

    }
    public enum drivingDirection{
        FORWARD,
        BACKWARD,
        LEFT,
        RIGHT
    }
    public enum clawPositions{
        CLAW_OPEN,
        CLAW_CLOSED
    }
    public enum stateMachineAct{
        ARM,
        CLAW,
        DRIVE
    }
    armStates armState;
    clawPositions clawState;
    drivingDirection driveState;

    final static double CLAW_OPEN = 0.5;
    final static double CLAW_CLOSED = 0.99;

    double backRightPower;
    double frontRightPower;
    double frontLeftPower;
    double backLeftPower;
    int driveTimeInMs;
    double driveSpeed;
    double RESTING_VELOCITY = 400;
    double BASKET_VELOCITY = 400;
    double SPECIMEN_VELOCITY = 400;
    double COLLECTION_VELOCITY = 400;
    double slider_velocity_up = 2500;
    double slider_velocity_down = 1200;
    int resting_position = 50;
    int basket_position = 170;
    int specimen_position = 370;
    int collection_position = 500;
    int sliders_down = 40;
    int sliders_up = 3500;
    int slider_above_bar_position = 1100;
    int slider_below_bar_position = 400;
    int shoulder_bar_position = 200;
    int shoulder_bar_velocity = 400;
    double desired_claw_position;
    int desired_shoulder_position;
    double desired_shoulder_velocity;
    int desired_slider_position;
    double desired_slider_velocity;
    double desired_wrist_position = 0.5;
    public Board0(){}
    public void init(HardwareMap hw){
        imu = hw.get(IMU.class, "imu");
        sliderButton = hw.touchSensor.get("sliderButton");
        frontLeftMotor = hw.dcMotor.get("frontLeft");
        backLeftMotor = hw.dcMotor.get("backLeft");
        frontRightMotor = hw.dcMotor.get("frontRight");
        backRightMotor = hw.dcMotor.get("backRight");
        claw = hw.get(Servo.class, "claw");
        wrist = hw.get(Servo.class, "wrist");
        SliderLeft = hw.get(DcMotorEx.class, "SliderLeft");
        SliderRight = hw.get(DcMotorEx.class, "SliderRight");
        Shoulder = hw.get(DcMotorEx.class, "Shoulder");
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
    }

    public void setArmState(armStates armState){
        this.armState = armState;
    }

    public armStates getArmState() {
        return this.armState;
    }

    public void setClawState(clawPositions clawState){
        this.clawState = clawState;
    }

    public clawPositions getClawState() {
        return this.clawState;
    }

    public void setDriveState(drivingDirection driveState) {
        this.driveState = driveState;
    }

    public void setDriveTime(int timeInMs){
        this.driveTimeInMs = timeInMs;
    }

    public void setDriveSpeed(double driveSpeed){
        this.driveSpeed = driveSpeed;
    }

    public double getDriveSpeed(){
        return this.driveSpeed;
    }

    public void stateMachineClaw() {
        if (clawState == clawPositions.CLAW_OPEN){
            desired_claw_position = CLAW_OPEN;
        }else if (clawState == clawPositions.CLAW_CLOSED){
            desired_claw_position = CLAW_CLOSED;
        }
    }

    public void stateMachineArm(){
        switch (armState) {
            case RESTING:
                desired_shoulder_position = resting_position;
                desired_shoulder_velocity = RESTING_VELOCITY;
                desired_slider_position = sliders_down;
                desired_slider_velocity = slider_velocity_down;
                break;
            case BASKET:
                desired_shoulder_position = basket_position;
                desired_shoulder_velocity = BASKET_VELOCITY;
                desired_slider_position = sliders_up;
                desired_slider_velocity = slider_velocity_up;
                break;
            case SPECIMEN:
                desired_shoulder_position = specimen_position;
                desired_shoulder_velocity = SPECIMEN_VELOCITY;
                desired_slider_position = sliders_down;
                desired_slider_velocity = slider_velocity_down;
                break;
            case COLLECTION:
                desired_shoulder_position = collection_position;
                desired_shoulder_velocity = COLLECTION_VELOCITY;
                desired_slider_position = sliders_down;
                desired_slider_velocity = slider_velocity_down;
                break;
            case ABOVE_BAR:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velocity;
                desired_slider_position = slider_above_bar_position;
                desired_slider_velocity = slider_velocity_up;
                break;
            case BELOW_BAR:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velocity;
                //desired_wrist_position = wrist_bar_position;
                desired_slider_position = slider_below_bar_position;
                desired_slider_velocity = slider_velocity_down;
                break;
        }
    }
    public void stateMachinesAct(stateMachineAct stateMachine) {
        //Tell all the motors to do what they are supposed to do.
        switch (stateMachine){
            case ARM:
                wrist.setPosition(desired_wrist_position);
                Shoulder.setTargetPosition(desired_shoulder_position);
                Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Shoulder.setVelocity(desired_shoulder_velocity);
                SliderLeft.setTargetPosition(desired_slider_position);
                SliderRight.setTargetPosition(desired_slider_position);
                SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                SliderLeft.setVelocity(desired_slider_velocity);
                SliderRight.setVelocity(desired_slider_velocity);
                if(desired_shoulder_velocity < 0) {
                    if (sliderButton.isPressed()) {
                        SliderLeft.setMotorDisable();
                        SliderRight.setMotorDisable();
                    }
                }
            case CLAW:
                claw.setPosition(desired_claw_position);
            case DRIVE:
                driveAction(driveTimeInMs);
                break;
        }
    }

    public void stateMachinesThink(stateMachineAct stateMachine){
        switch (stateMachine){
            case CLAW:
                stateMachineClaw();
                break;
            case DRIVE:
                stateMachineDrive();
                break;
            case ARM:
                stateMachineArm();
                break;
        }
    }

    //Not intended for manual here... used internally
    public void drive(double x, double y, double rx){
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        frontLeftPower = (rotY + rotX + rx) / denominator;
        backLeftPower = (rotY - rotX + rx) / denominator;
        frontRightPower = (rotY - rotX - rx) / denominator;
        backRightPower = (rotY + rotX - rx) / denominator;
    }

    private void stateMachineDrive(){
        double speed = clamp(0.0, 1.0, driveSpeed);
        switch (driveState){
            case FORWARD:
                drive(0.0, -speed, 0.0);
                break;
            case BACKWARD:
                drive(0.0, speed, 0.0);
                break;
            case LEFT:
                drive(-speed, 0.0, 0.0);
                break;
            case RIGHT:
                drive(speed, 0.0, 0.0);
                break;
        }
    }

    public void driveAction(int timeInMs) {
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);

        sleep(timeInMs);

        frontLeftMotor.setPower(0.0);
        backLeftMotor.setPower(0.0);
        frontRightMotor.setPower(0.0);
        backRightMotor.setPower(0.0);
    }
}