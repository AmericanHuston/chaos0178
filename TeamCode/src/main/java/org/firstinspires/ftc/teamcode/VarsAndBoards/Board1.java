package org.firstinspires.ftc.teamcode.VarsAndBoards;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.robot.Robot;

public class Board1 {

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
    public enum devices {
        ARM,
        SLIDERS,
        CLAW
    }
    public enum armStates {
        RESTING,
        BASKET,
        SPECIMEN,
        COLLECTION,
        ABOVE_BAR,
        BELOW_BAR,
        PRE_HANG,
        POST_HANG,
        WALL_GRAB

    }
    public enum clawPositions{
        CLAW_OPEN,
        CLAW_CLOSED
    }

    armStates armState;
    clawPositions clawState;

    final static double CLAW_OPEN = RobotPosVars.CLAW_OPEN;
    final static double CLAW_CLOSED = RobotPosVars.CLAW_CLOSED;

    public static int slidersWall = RobotPosVars.slidersWall;
    public static int armWall = RobotPosVars.armWall;
    public static double RESTING_VELOCITY = RobotPosVars.RESTING_VELOCITY;
    public static double BASKET_VELOCITY = RobotPosVars.BASKET_VELOCITY;
    public static double SPECIMEN_VELOCITY = RobotPosVars.SPECIMEN_VELOCITY;
    public static double COLLECTION_VELOCITY = RobotPosVars.COLLECTION_VELOCITY;
    public static int hangHeight = RobotPosVars.hangHeight;
    public static double slider_velocity_up = RobotPosVars.slider_velocity_up;
    public static double slider_velocity_down = RobotPosVars.slider_velocity_down;
    public static int resting_position = RobotPosVars.resting_position;
    public static int basket_position = RobotPosVars.basket_position;
    public static int specimen_position = RobotPosVars.specimen_position;
    public static int collection_position = RobotPosVars.collection_position;
    public static int sliders_down = RobotPosVars.sliders_down;
    public static int sliders_up = RobotPosVars.sliders_up;
    public static int slider_above_bar_position = RobotPosVars.slider_above_bar_position;
    public static int slider_below_bar_position = RobotPosVars.slider_below_bar_position;
    public static int shoulder_bar_position = RobotPosVars.shoulder_bar_position;
    public static int shoulder_bar_velocity = RobotPosVars.shoulder_bar_velocity;

    /**Desired positions don't belong in RobotPosVars.**/
    public static double desired_claw_position;
    public static int desired_shoulder_position;
    public static double desired_shoulder_velocity;
    public static int desired_slider_position;
    public static double desired_slider_velocity;
    public static double desired_wrist_position;

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

    private void stateMachineClaw() {
        if (clawState == clawPositions.CLAW_OPEN){
            desired_claw_position = CLAW_OPEN;
        }else if (clawState == clawPositions.CLAW_CLOSED){
            desired_claw_position = CLAW_CLOSED;
        }
    }

    private void stateMachineArm(){
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
                desired_slider_velocity = slider_velocity_down;
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
                desired_slider_position = slider_below_bar_position;
                desired_slider_velocity = slider_velocity_down;
                break;
            case PRE_HANG:
                desired_shoulder_position = resting_position;
                desired_shoulder_velocity = shoulder_bar_velocity;
                desired_slider_position = hangHeight;
                desired_slider_velocity = slider_velocity_up;
                break;
            case POST_HANG:
                desired_slider_position = 1000;//used to be resting_position
                desired_slider_velocity = slider_velocity_up;
                break;
            case WALL_GRAB:
                desired_shoulder_position = armWall;
                desired_shoulder_velocity = COLLECTION_VELOCITY;
                desired_slider_position = slidersWall;
                desired_slider_velocity = slider_velocity_up;
        }
    }
    private void stateMachineSliders(){
        //TODO
    }
    private void stateMachinesAct(devices stateMachine) {
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
            case SLIDERS:
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
                break;
        }
    }

    private void stateMachinesThink(devices stateMachine){
        switch (stateMachine){
            case CLAW:
                stateMachineClaw();
                break;
                //Don't need this one anymore:
            case ARM:
                stateMachineArm();
                break;
            case SLIDERS:
                //TODO
                break;
        }
    }

    public void DO_ALL(){
        stateMachinesThink(devices.ARM);
        stateMachinesAct(devices.ARM);
        stateMachinesThink(devices.CLAW);
        stateMachinesAct(devices.CLAW);
        stateMachinesThink(devices.SLIDERS);
        stateMachinesAct(devices.SLIDERS);
    }
}