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
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
@Autonomous(name = "4Spec", group = "PedroAutos")
public class Spec4 extends OpMode {
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
    private boolean stateAdvance = false;


    //All in inches...Not centimeters
    // Observation Zone is 0,0
    //Other Observation Zone is 144,144
    //0 PROBABLY intersects the fully coloOther bars
    // Pose goes in this order: Pose(x,y, Radians);
    private final Pose StartingPose = new Pose(8, 56, Math.toRadians(0));
    private final Pose Basket = new Pose(24,120, Math.toRadians(270));
    private final Pose OtherObservation = new Pose(120, 120, Math.toRadians(90));
    private final Pose OtherBasket = new Pose(120, 24, Math.toRadians(135));
    private final Pose Observation = new Pose(8,40, Math.toRadians(0));
    private final Pose HangSpecimen = new Pose(37,74, Math.toRadians(0));
    private final Pose OtherHangSpecimen = new Pose(112,72,Math.toRadians(90));
    private final Pose TapeHangRobot = new Pose(72,96, Math.toRadians(90));
    private final Pose OtherTapeHangRobot = new Pose(72,48, Math.toRadians(270));
    private final Pose SpecPrepStep1 = new Pose(52, 25, Math.toRadians(180));
    private final Point SpecPrepStep1Point = new Point(70, 22);
    private final Point SpecPrepStep2Point = new Point(25, 20);
    private final Pose SpecPrepStep2 = new Pose(25, 20, Math.toRadians(180));
    private final Point littleBackPoint = new Point (25, 24);
    private final Pose littleBack = new Pose (25, 24, Math.toRadians(180));
    private final Pose littleRight = new Pose(37,70, Math.toRadians(0));
    private final Point littleRightPoint = new Point(37,70);
    private final Pose BlockPush1 = new Pose(20, 20, Math.toRadians(180));
    private final Pose BlockPush2 = new Pose(20, 16, Math.toRadians(180));
    private final Pose SpecGrab = new Pose(8.5, 36, Math.toRadians(190));
    private final Pose CurvePoseSpecGrab = new Pose(61, 28, Math.toRadians(180));
    private final Point CurveSpecGrab = new Point(61, 28);
    private final Point controlSpecCollect1Step1 = new Point(8, 55);
    private final Point controlSpecCollect2 = new Point(80, 11);

    private PathChain square;

    private PathChain specimenHang1;
    private PathChain JustRight;
    private PathChain SpecCollect1;
    private PathChain SpecCollect2;
    private PathChain BlockToBase1;
    private PathChain BlockToBase2;
    private PathChain JustBack;
    private PathChain preHang;
    private PathChain specimenHang2;
    private PathChain Park;
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

        specimenHang1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartingPose), new Point(HangSpecimen)))
                .setLinearHeadingInterpolation(StartingPose.getHeading(), HangSpecimen.getHeading())
                .build();
        specimenHang2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpecGrab), new Point(HangSpecimen)))
                .setLinearHeadingInterpolation(SpecGrab.getHeading(), HangSpecimen.getHeading())
                .build();
        JustRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HangSpecimen), new Point(littleRight)))
                .setLinearHeadingInterpolation(HangSpecimen.getHeading(), littleRight.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HangSpecimen), new Point(SpecGrab)))
                .setLinearHeadingInterpolation(HangSpecimen.getHeading(), SpecGrab.getHeading())
                .build();
        SpecCollect1 = follower.pathBuilder()
                .addBezierCurve(littleRightPoint, controlSpecCollect1Step1, CurveSpecGrab)
                .setLinearHeadingInterpolation(littleRight.getHeading(), CurvePoseSpecGrab.getHeading())
                .build();
        BlockToBase1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(CurvePoseSpecGrab), new Point(littleBack)))
                .setLinearHeadingInterpolation(CurvePoseSpecGrab.getHeading(), littleBack.getHeading())
                .addPath(new BezierLine(new Point(littleBack), new Point(SpecPrepStep1)))
                .setLinearHeadingInterpolation(littleBack.getHeading(), SpecPrepStep1.getHeading())
                .addBezierCurve(SpecPrepStep1Point, controlSpecCollect2, SpecPrepStep2Point)
                .setLinearHeadingInterpolation(SpecPrepStep1.getHeading(), SpecPrepStep2.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep2), new Point(SpecGrab)))
                .setLinearHeadingInterpolation(SpecPrepStep2.getHeading(), SpecGrab.getHeading())
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
                robot.closeMiniClaw();
                robot.closeClaw();
                if (state_timer.getElapsedTimeSeconds()  > 0.7) {
                    next_state();
                }
                break;
            case 1: //raises the arm and sliders to the above bar position
                robot.setArmState(Robot2.armState.ABOVE_BAR);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(state_timer.getElapsedTimeSeconds() > 0.5) {
                    next_state();
                }
                break;
            case 2: //drives to the bar
                if(!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    follower.followPath(specimenHang1, true);
                    next_state();
                }
                break;
            case 3: //clips the specimen
                if(!follower.isBusy()) {
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    if (state_timer.getElapsedTimeSeconds() > 2.2) {
                        next_state();
                    }
                }
                break;
            case 4: //nudges the specimen on the bar a little right
                if (!follower.isBusy()){
                    follower.followPath(JustRight);
                    next_state();
                }
                break;
            case 5: //releases the claw
                robot.openClaw();
                robot.openMiniClaw();
                if(state_timer.getElapsedTimeSeconds() > 0.8) {next_state();}
                break;
            case 6: //pushes the samples in
                if(!follower.isBusy()){
                    follower.setMaxPower(0.9);
                    follower.followPath(SpecCollect1);
                    next_state();
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    follower.setMaxPower(0.9);
                    follower.followPath(BlockToBase1);
                    next_state();
                }
                break;
            case 8: //goes to resting
                robot.setArmState(Robot2.armState.RESTING);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(!follower.isBusy()) {
                    next_state();
                }
                break;
            case 9: //closes the claw
                robot.closeMiniClaw();
                robot.closeClaw();
                if (state_timer.getElapsedTimeSeconds() > 0.7) {next_state();}
                break;
            case 10: //raises the arm and sliders to the above bar position
                robot.setArmState(Robot2.armState.ABOVE_BAR);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(state_timer.getElapsedTimeSeconds() > 0.5) {
                    next_state();
                }
                break;
            case 11: //drives to the bar
                if(!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    follower.followPath(specimenHang2, true);
                    next_state();
                }
                break;
            case 12: //clips the specimen
                if(!follower.isBusy()) {
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    if (state_timer.getElapsedTimeSeconds() > 2.0) {
                        next_state();
                    }
                }
                break;
            case 13: //nudges the specimen on the bar a little right
                if (!follower.isBusy()){
                    follower.followPath(JustRight);
                    next_state();
                }
                break;
            case 14: //releases the claw
                if(!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    if(state_timer.getElapsedTimeSeconds() > 0.8) {
                        next_state();
                    }
                }
                break;
            case 15: //goes to the collection position
                if(!follower.isBusy()){
                    follower.followPath(Park);
                    next_state();
                }
                break;
            case 16: //goes to resting
                robot.setArmState(Robot2.armState.RESTING);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(!follower.isBusy()) {
                    next_state();
                }
                break;
            case 17: //closes the claw
                robot.closeMiniClaw();
                robot.closeClaw();
                if (state_timer.getElapsedTimeSeconds() > 0.7) {next_state();}
                break;
            case 18: //raises the arm and sliders to the above bar position
                robot.setArmState(Robot2.armState.ABOVE_BAR);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(state_timer.getElapsedTimeSeconds() > 0.5) {
                    next_state();
                }
                break;
            case 19: //drives to the bar
                if(!follower.isBusy()) {
                    follower.followPath(specimenHang2, true);
                    next_state();
                }
                break;
            case 20: //clips the specimen
                if(!follower.isBusy()) {
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    if (state_timer.getElapsedTimeSeconds() > 2.0) {
                        next_state();
                    }
                }
                break;
            case 21: //nudges the specimen on the bar a little right
                if (!follower.isBusy()){
                    follower.followPath(JustRight);
                    next_state();
                }
                break;
            case 22: //releases the claw
                if(!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    if(state_timer.getElapsedTimeSeconds() > 0.8) {
                        next_state();
                    }
                }
                break;
            case 23: //goes to the collection position
                if(!follower.isBusy()){
                    follower.followPath(Park);
                    next_state();
                }
                break;
            case 24: //goes to resting
                robot.setArmState(Robot2.armState.RESTING);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(!follower.isBusy()) {
                    next_state();
                }
                break;
            case 25: //closes the claw
                robot.closeMiniClaw();
                robot.closeClaw();
                if (state_timer.getElapsedTimeSeconds() > 0.7) {next_state();}
                break;
            case 26: //raises the arm and sliders to the above bar position
                robot.setArmState(Robot2.armState.ABOVE_BAR);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(state_timer.getElapsedTimeSeconds() > 0.5) {
                    next_state();
                }
                break;
            case 27: //drives to the bar
                if(!follower.isBusy()) {
                    follower.followPath(specimenHang2, true);
                    next_state();
                }
                break;
            case 28: //clips the specimen
                if(!follower.isBusy()) {
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    if (state_timer.getElapsedTimeSeconds() > 2.0) {
                        next_state();
                    }
                }
                break;
            case 29: //nudges the specimen on the bar a little right
                if (!follower.isBusy()){
                    follower.followPath(JustRight);
                    next_state();
                }
                break;
            case 30: //releases the claw
                if(!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    if(state_timer.getElapsedTimeSeconds() > 0.8) {
                        next_state();
                    }
                }
                break;
            case 31: //goes to the collection position
                if(!follower.isBusy()){
                    follower.followPath(Park);
                    next_state();
                }
                break;
        }
            telemetry.addData("autoState", autoState);


        //follower.telemetryDebug(telemetryA);
    }
}
