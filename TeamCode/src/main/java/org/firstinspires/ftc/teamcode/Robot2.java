package org.firstinspires.ftc.teamcode;


import static java.lang.Double.valueOf;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.VarsAndBoards.Utils.DataLogger;

import java.util.ArrayList;

@Config
public class Robot2 {
    public DcMotorEx SliderLeft;
    public DcMotorEx SliderRight;
    public DcMotorEx Shoulder;
    public LED rightLEDRed;
    public LED rightLEDGreen;
    public LED leftLEDRed;
    public LED leftLEDGreen;
    public enum armState{
        RESTING,
        BASKET,
        SPECIMEN,
        COLLECTION,
        ABOVE_BAR,
        BELOW_BAR,
        PREHANG,
        POSTHANG
    }
    armState state;
    final static double CLAW_OPEN = 0.5;
    final static double CLAW_CLOSED = 0.99;
    public static int slidersWall = 1450;
    public static int armWall = 500;
    public static double RESTING_VELOCITY = 250;
    public static double BASKET_VELOCITY = 250;
    public static double SPECIMEN_VELOCITY = 260;
    public static double COLLECTION_VELOCITY = 260;
    public static int hangHeight = 2500;
    public static double Slidervelocityup = 2500;
    public static double Slidervelocitydown = 1200;
    public static int resting_position = 50;
    public static int basket_position = 170;
    public static int specimen_position = 400;
    public static int collection_position = 500;
    public static int slidersdown = 40;
    public static int slidersup = 3450;
    public static double MAX_POS     =  1.0;     // Maximum rotational position
    public static double MIN_POS     =  0.0;// Minimum rotational position
    public static int slider_above_bar_position = 1900;
    public static int slider_below_bar_position = 1275;
    public static int shoulder_bar_position = 100;
    //public static double wrist_bar_position = 0.39;
    public static int shoulder_bar_velotity = 200;
    public static double kp = 0.2;
    public static double desired_claw_position = 0.99;
    public static double desired_miniClaw_position = 0.99;
    public static int desired_shoulder_position;
    public static double desired_shoulder_velocity;
    public static double output;
    public static int desired_slider_position;
    public static double desired_slider_velocity;
    public static double desired_wrist_position = 0.5;
    private static Pose lastPose;
    public double botHeading;
    public boolean changedClaw = false;
    public boolean changedWrist = false;

    DataLogger Logger = new DataLogger();
    IMU imu;
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;
    Servo wrist;
    Servo claw;
    Servo miniClaw;
    TouchSensor sliderButton;
    GoBildaPinpointDriver pinpoint;

    public void init(HardwareMap hardwareMap) {

        imu = hardwareMap.get(IMU.class, "imu");
        sliderButton = hardwareMap.touchSensor.get("sliderButton");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        backRightMotor = hardwareMap.dcMotor.get("backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        miniClaw = hardwareMap.get(Servo.class, "miniClaw");
        SliderLeft = hardwareMap.get(DcMotorEx.class, "SliderLeft");
        SliderRight = hardwareMap.get(DcMotorEx.class, "SliderRight");
        Shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
        Shoulder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        Shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Shoulder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        SliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        SliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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
        rightLEDRed = hardwareMap.get(LED.class, "rightLEDRed");
        rightLEDGreen = hardwareMap.get(LED.class, "rightLEDGreen");
        leftLEDRed = hardwareMap.get(LED.class, "leftLEDRed");
        leftLEDGreen = hardwareMap.get(LED.class, "leftLEDGreen");
        rightLEDGreen.off();
        leftLEDGreen.off();
        rightLEDRed.on();
        leftLEDRed.on();
    }

    public static void setLastPose(Pose savePose){
        lastPose = savePose;
    }
    public static Pose getLastPose(){
        return lastPose;
    }

    public double getClawPosition(){
        return claw.getPosition();
    }

    public int getSlidersPosition(){
        return (int)(SliderLeft.getCurrentPosition() + SliderRight.getCurrentPosition())/2;
    }

    public double getMiniClawPosition() {
        return miniClaw.getPosition();
    }

    public void setArmState(armState armState){
        this.state = armState;
        updateDesiredValues();
    }
    public void GreenOnLED () {
        rightLEDRed.off();
        leftLEDRed.off();
        rightLEDGreen.on();
        leftLEDGreen.on();
    }
    public void RedOnLED() {
        rightLEDGreen.off();
        leftLEDGreen.off();
        rightLEDRed.on();
        leftLEDRed.on();
    }

    public armState getArmState() {
        return this.state;
    }
    public boolean isClawOpen(){
        return(claw.getPosition() < 0.7);
    }
    public boolean isMiniClawOpen(){
        return(miniClaw.getPosition() < 0.85);
    }
    public double getWristPosition(){
        return wrist.getPosition();
    }
    public int getShoulderPosition(){
        return Shoulder.getCurrentPosition();
    }
    public void setShoulderPosition(int x){
        desired_shoulder_position = x;
    }
    public void resetIMU() {
        imu.resetYaw();
        pinpoint.resetPosAndIMU();
    }
    public void setClawPosition(double x){
        desired_claw_position = x;
    }
    public void setMiniClawPosition(double x){
        desired_miniClaw_position = x;
    }
    public void setWristPosition(double x){
        desired_wrist_position = x;
    }
    public void openClaw(){
        desired_claw_position = 0.5;
        claw.setPosition(desired_claw_position);
    }
    public void closeClaw(){
        desired_claw_position = 0.99;
        claw.setPosition(desired_claw_position);
    }
    public void openMiniClaw(){
        desired_miniClaw_position = 0.5;
        miniClaw.setPosition(desired_miniClaw_position);
    }
    public void closeMiniClaw(){
        desired_miniClaw_position = 0.99;
        miniClaw.setPosition(desired_miniClaw_position);
    }
    public void wristVertical(){
        desired_wrist_position = 0.1;
    }
    public void wristHorizontal(){
        desired_wrist_position = 0.5;
    }
    public void wrist45(){
        desired_wrist_position = 0.25;
    }
    public void sliderMove(int moveTickAmount){
        if (state == armState.PREHANG) {
            hangHeight += moveTickAmount;
        }
    }
    //sets shoulder motor position need the right presets
    private void updateDesiredValues(){
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
                desired_slider_position = slidersup;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case SPECIMEN:
                desired_shoulder_position = specimen_position;
                desired_shoulder_velocity = SPECIMEN_VELOCITY;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case COLLECTION:
                desired_shoulder_position = collection_position;
                desired_shoulder_velocity = COLLECTION_VELOCITY;
                desired_slider_position = slidersdown;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case ABOVE_BAR:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                desired_slider_position = slider_above_bar_position;
                desired_slider_velocity = Slidervelocityup;
                break;
            case BELOW_BAR:
                desired_shoulder_position = shoulder_bar_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                desired_slider_position = slider_below_bar_position;
                desired_slider_velocity = Slidervelocitydown;
                break;
            case PREHANG:
                desired_shoulder_position = resting_position;
                desired_shoulder_velocity = shoulder_bar_velotity;
                desired_slider_position = hangHeight;
                desired_slider_velocity = Slidervelocityup;
                break;
            case POSTHANG:
                desired_slider_position = 1000;//used to be resting_position
                desired_slider_velocity = Slidervelocityup;
                break;
        }
    }
    public void toggleClaw() {
        if (!isClawOpen()){
            desired_claw_position = CLAW_OPEN;
            desired_miniClaw_position = CLAW_OPEN;
        } else {
            desired_claw_position = CLAW_CLOSED;
            desired_miniClaw_position = CLAW_CLOSED;
        }
    }

    public void shoulderAct() {
        Shoulder.setTargetPosition(desired_shoulder_position);
        Shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Shoulder.setVelocity(desired_shoulder_velocity);
    }
    public void wristAct() {
        wrist.setPosition(desired_wrist_position);
    }
    public void clawAct() {
        claw.setPosition(desired_claw_position);
    }
    public void miniClawAct() {
        miniClaw.setPosition(desired_miniClaw_position);
    }
    public void slidersAct() {
        SliderLeft.setTargetPosition(desired_slider_position);
        SliderRight.setTargetPosition(desired_slider_position);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderLeft.setVelocity(desired_slider_velocity);
        SliderRight.setVelocity(desired_slider_velocity);
        if (sliderButton.isPressed()){
            SliderLeft.setMotorDisable();
            SliderRight.setMotorDisable();
            SliderLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SliderRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
    public void sliderNoTouchAct() {
        SliderLeft.setTargetPosition(desired_slider_position);
        SliderRight.setTargetPosition(desired_slider_position);
        SliderLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderLeft.setVelocity(desired_slider_velocity);
        SliderRight.setVelocity(desired_slider_velocity);
    }
    public void allAct(){
        slidersAct();
        shoulderAct();
        wristAct();
        clawAct();
        miniClawAct();
    }

    public void LogRobotState(){
        //Shoulder
        Logger.addData(getShoulderPosition());
        //State
        Logger.addData(getArmState().toString());
        //Wrist (unnecessary)
        Logger.addData(getWristPosition());
        //Claw (unnecessary)
        Logger.addData(getClawPosition());
        //MiniClaw (unnecessary)
        Logger.addData(getMiniClawPosition());
        //Sliders
        Logger.addData(getSlidersPosition());
        Logger.update();
    }

    public void LogRobotPosition(double x, double y, double heading){
        Logger.addData(x);
        Logger.addData(y);
        Logger.addData(heading);
        Logger.update();
    }

    public int[] getLastLoggedRobotPosition(){
        int[] Result = new int[3];
        Result[0] = Integer.parseInt(Logger.read(6,0));
        Result[1] = Integer.parseInt(Logger.read(7,0));
        Result[2] = Integer.parseInt(Logger.read(8,0));
        return Result;
    }

    public void getLastLoggedRobotState(){
        //current shoulder position = Logger.read(0,0);
        this.state = armState.valueOf(Logger.read(1,0));
        //current slider position == Logger.read(5,0);
    }
}
