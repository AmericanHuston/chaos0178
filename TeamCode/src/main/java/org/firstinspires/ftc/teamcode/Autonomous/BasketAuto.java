package org.firstinspires.ftc.teamcode.Autonomous;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.VarsAndBoards.Board0;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "BasketAuto", group = "PedroAutos")
public class BasketAuto extends OpMode {
    Robot2 robot = new Robot2();
    private Follower follower;
    private Timer state_timer;
    private Timer Op_mode_timer;
    private int autoState = 0;
    private final int MIN_WALL_POS = 8;
    private final int t1 = 24;
    private final int t2 = 48;
    private final int t3 = 72;
    private final int t4 = 96;
    private final int t5 = 120;
    private final int t6 = 144; //NOTE: Our robot's center is about 8 inches from the wall. Max is actually 136
    private final int MAX_WALL_POS = 136;


    //All in inches...Not centimeters
    // Observation Zone is 0,0
    //Other Observation Zone is 144,144
    //0 PROBABLY intersects the fully coloOther bars
    // Pose goes in this order: Pose(x,y, Radians);
    private final Pose StartingPose = new Pose(8, 102, Math.toRadians(90));
    private final Pose Basket = new Pose(13.5,124.5, Math.toRadians(130));
    private final Pose OtherObservation = new Pose(120, 102, Math.toRadians(90));
    private final Pose OtherBasket = new Pose(120, 6, Math.toRadians(135));
    private final Pose Observation = new Pose(8,32, Math.toRadians(0));
    private final Pose TapeHangRobot = new Pose(72,78, Math.toRadians(90));
    private final Pose OtherTapeHangRobot = new Pose(72,30, Math.toRadians(270));
    private final Pose littleBack = new Pose(20, 112, Math.toRadians(130));
    private final Pose sample1  = new Pose(24.5, 126, Math.toRadians(0));
    private final Pose sample2 = new Pose(24.5,116, Math.toRadians(0));
    private final Pose sample3 = new Pose(45.5, 118.8, Math.toRadians(90));
    private final Pose hangBar = new Pose(80,92, Math.toRadians(270));
    private final Point BasketPoint = new Point(13.5, 122.5);
    private final Point hangBarPoint = new Point(60, 97);
    private final Point parkControlPoint = new Point(64, 120);

    private PathChain square;

    private PathChain sampleCollect1;
    private PathChain scoreBasket1;
    private PathChain JustBack1;
    private PathChain scoreBasket2;
    private PathChain sampleCollect2;
    private PathChain sampleCollect3;
    private PathChain scoreBasket3;
    private PathChain park;
    private Telemetry telemetryA;

    @Override
    public void init() {
        robot.init(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(StartingPose);
        state_timer = new Timer();
        Op_mode_timer = new Timer();
        Op_mode_timer.resetTimer();

        park = follower.pathBuilder()
                .addBezierCurve(BasketPoint, parkControlPoint, hangBarPoint)
                .setLinearHeadingInterpolation(Basket.getHeading(), hangBar.getHeading())
                .build();
        scoreBasket1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartingPose), new Point(Basket)))
                .setLinearHeadingInterpolation(StartingPose.getHeading(), Basket.getHeading())
                .build();
        JustBack1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Basket), new Point(littleBack)))
                .setLinearHeadingInterpolation(Basket.getHeading(), littleBack.getHeading())
                .build();

        sampleCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleBack), new Point(sample1)))
                .setLinearHeadingInterpolation(littleBack.getHeading(), sample1.getHeading())
                .build();
        scoreBasket2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample1), new Point(Basket)))
                .setLinearHeadingInterpolation(sample1.getHeading(), Basket.getHeading())
                .build();
        sampleCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleBack), new Point(sample2)))
                .setLinearHeadingInterpolation(littleBack.getHeading(), sample2.getHeading())
                .build();
        sampleCollect3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Basket), new Point(sample3)))
                .setLinearHeadingInterpolation(Basket.getHeading(), sample3.getHeading())
                .build();
        scoreBasket3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample3), new Point(Basket)))
                .setLinearHeadingInterpolation(sample3.getHeading(), Basket.getHeading())
                .build();
        square = follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartingPose), new Point(Basket)))
                .setLinearHeadingInterpolation(StartingPose.getHeading(), Basket.getHeading())
                .addPath(new BezierLine(new Point(Basket), new Point(OtherObservation)))
                .setLinearHeadingInterpolation(Basket.getHeading(), OtherObservation.getHeading())
                .addPath(new BezierLine(new Point(OtherObservation), new Point(OtherBasket)))
                .setLinearHeadingInterpolation(OtherObservation.getHeading(), OtherBasket.getHeading())
                .addPath(new BezierLine(new Point(OtherBasket), new Point(Observation)))
                .setLinearHeadingInterpolation(OtherBasket.getHeading(), Observation.getHeading())
                .addPath(new BezierLine(new Point(Observation), new Point(Basket)))
                .setLinearHeadingInterpolation(Observation.getHeading(), Basket.getHeading())
                .build();


        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.update();
    }
    public void next_state(){
        autoState += 1;
        state_timer.resetTimer();
    }
    @Override
    public void start() {
        Op_mode_timer.resetTimer();
        autoState = 0;
        Op_mode_timer.getElapsedTimeSeconds();
    }


    @Override
    public void loop() {
        follower.update();
        robot.setLastPose(follower.getPose());
        switch (autoState) {
            case 0: //closes the claw
                robot.wristHorizontal();
                robot.wristAct();
                robot.closeClaw();
                robot.closeMiniClaw();
                next_state();
                break;
            case 1: //raises the sliders than waits 1.5 seconds
                robot.setArmState(Robot2.armState.BASKET);
                robot.sliderNoTouchAct();
                if (state_timer.getElapsedTimeSeconds() > 0.7){
                    robot.allAct();
                    if(state_timer.getElapsedTimeSeconds()  > 2.0){
                        next_state();
                    }
                }
                break;
            case 2: //drives to the basket
                follower.setMaxPower(1.0);
                follower.followPath(scoreBasket1, true);
                next_state();
                break;
            case 3://drops the sample in the basket
                if(!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    next_state();
                }
                break;
            case 4: //moves back so we don't accidentally ascend.
                if(!follower.isBusy()) {
                    follower.followPath(JustBack1, true);
                    next_state();
                }
                break;
            case 5: //moves the arm to resting so we don't tip
                if (state_timer.getElapsedTimeSeconds() > 1.1){
                    robot.setArmState(Robot2.armState.RESTING);
                    robot.slidersAct();
                    next_state();
                }
                break;
            case 6: //moves to the first sample
                if(!follower.isBusy()) {
                    follower.followPath(sampleCollect1, true);
                    next_state();
                }
                break;
            case 7: //arm to the collection position
                if(!follower.isBusy()){
                    robot.setArmState(Robot2.armState.COLLECTION);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    next_state();
                }
                break;
            case 8: //grabs the sample
                if(state_timer.getElapsedTimeSeconds() > 1.4) {
                    robot.closeClaw();
                    robot.closeMiniClaw();
                    if (state_timer.getElapsedTimeSeconds() > 1.5) {
                        next_state();
                    }
                }
                break;
            case 9: //sets the arm to the basket position
                robot.setArmState(Robot2.armState.BASKET);
                robot.sliderNoTouchAct();
                if (state_timer.getElapsedTimeSeconds() > 0.2){
                    robot.allAct();
                    if(state_timer.getElapsedTimeSeconds()  > 2.0){
                        next_state();
                    }
                }
                break;
            case 10: //moves to scoring position
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.80);
                    follower.followPath(scoreBasket2, true);
                    next_state();
                }
                break;
            case 11://drops the sample in the bucket
                if (!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    next_state();
                }
                break;
            case 12: //moves back so we don't accidentally ascend.
                if(state_timer.getElapsedTimeSeconds() > 0.2) {
                    if (!follower.isBusy()) {
                        follower.setMaxPower(1.0);
                        follower.followPath(JustBack1);
                        next_state();
                    }
                }
                break;
            case 13://lowering the sliders
                if(state_timer.getElapsedTimeSeconds()  > 1.1) {
                    robot.setArmState(Robot2.armState.SPECIMEN);
                    robot.allAct();
                    next_state();
                }
                break;
            case 14://drives to the second sample
                if(!follower.isBusy()){
                    follower.followPath(sampleCollect2, true);
                    next_state();
                }
                break;
            case 15: //arm to the collection position
                if(!follower.isBusy()) {
                    robot.setArmState(Robot2.armState.COLLECTION);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    next_state();
                }
                break;
            case 16: //grabs the second sample
                if (state_timer.getElapsedTimeSeconds() > 1.4) {
                    robot.closeClaw();
                    robot.closeMiniClaw();
                    if (state_timer.getElapsedTimeSeconds() > 1.5) {
                        next_state();
                    }
                }
                break;
            case 17: //sets the arm to the basket position
                robot.setArmState(Robot2.armState.BASKET);
                robot.sliderNoTouchAct();
                if (state_timer.getElapsedTimeSeconds() > 0.2){
                    robot.allAct();
                    if(state_timer.getElapsedTimeSeconds()  > 2.0){
                        next_state();
                    }
                }
                break;
            case 18: //moves to scoring position
                if(!follower.isBusy()) {
                    follower.followPath(scoreBasket2, true);
                    next_state();
                }
                break;
            case 19://drops the sample in the bucket
                if (!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    next_state();
                }
                break;
            case 20://drives to the third sample
                if(!follower.isBusy()){
                    follower.followPath(sampleCollect3, true);
                    next_state();
                }
                break;
            case 21: //lowers the sliders
                if(state_timer.getElapsedTimeSeconds() > 1.1) {
                    robot.setArmState(Robot2.armState.SPECIMEN);
                    robot.allAct();
                    next_state();
                }
                break;
            case 22: //wrist lined up with the arm
                robot.setWristPosition(1.0);
                robot.wristAct();
                next_state();
                break;
            case 23: //arm to the collection position
                if(!follower.isBusy()){
                    robot.setArmState(Robot2.armState.COLLECTION);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    next_state();
                }
                break;
            case 24: //grabs the third sample
                if(state_timer.getElapsedTimeSeconds() > 1.8) {
                    robot.closeClaw();
                    robot.closeMiniClaw();
                    if (state_timer.getElapsedTimeSeconds() > 1.9) {
                        next_state();
                    }
                }
                break;
            case 25: //sets the arm to the basket position
                robot.setArmState(Robot2.armState.BASKET);
                robot.sliderNoTouchAct();
                if (state_timer.getElapsedTimeSeconds() > 0.2){
                    robot.allAct();
                    if(state_timer.getElapsedTimeSeconds()  > 2.0){
                        next_state();
                    }
                }
                break;
            case 26: //moves to scoring position
                if(!follower.isBusy()) {
                    follower.followPath(scoreBasket3, true);
                    next_state();
                }
                break;
            case 27://drops the sample in the bucket
                if (!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    next_state();
                }
                break;
            case 28: //parks
                if(!follower.isBusy()) {
                    follower.followPath(park);
                    next_state();
                }
                break;
            case 29: //sliders to parking
                if(state_timer.getElapsedTimeSeconds() > 0.3) {
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.allAct();
                    next_state();
                }
                break;
            case 30: // arm to parking
                robot.setShoulderPosition(200);
                robot.shoulderAct();
                next_state();
                break;

        }
        telemetry.addData("autoState", autoState);


        //follower.telemetryDebug(telemetryA);
    }
}