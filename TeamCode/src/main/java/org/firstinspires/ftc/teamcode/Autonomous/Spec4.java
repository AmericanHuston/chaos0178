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
    private final Pose SpecPrepStep1 = new Pose(32, 40, Math.toRadians(180));
    private final Pose SpecPrepStep2 = new Pose(56, 30, Math.toRadians(180));
    private final Pose SpecimenCollect1 = new Pose(30, 20, Math.toRadians(180));
    private final Pose SpecimenCollect2 = new Pose(58, 7, Math.toRadians(180));
    private final Pose littleBack = new Pose (25, 84, Math.toRadians(0));
    private final Pose littleRight = new Pose(37,70, Math.toRadians(0));
    private final Pose BlockPush1 = new Pose(10, 33, Math.toRadians(180));
    private final Pose BlockPush2 = new Pose(5, 24, Math.toRadians(180));
    private final Pose SpecGrab = new Pose(10, 52, Math.toRadians(180));
    private final Pose controlHangSpec = new Pose(21, 88);
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
        JustRight = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HangSpecimen), new Point(littleRight)))
                .setLinearHeadingInterpolation(HangSpecimen.getHeading(), littleRight.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HangSpecimen), new Point(SpecGrab)))
                .setLinearHeadingInterpolation(HangSpecimen.getHeading(), SpecGrab.getHeading())
                .build();
        SpecCollect1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleRight), new Point(SpecPrepStep1)))
                .setLinearHeadingInterpolation(littleRight.getHeading(), SpecPrepStep1.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep1), new Point(SpecPrepStep2)))
                .setLinearHeadingInterpolation(SpecPrepStep1.getHeading(), SpecPrepStep2.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep2), new Point(SpecimenCollect1)))
                .setLinearHeadingInterpolation(SpecPrepStep2.getHeading(), SpecimenCollect1.getHeading())
                .build();
        BlockToBase1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpecimenCollect1), new Point(BlockPush1)))
                .setLinearHeadingInterpolation(SpecimenCollect1.getHeading(), BlockPush1.getHeading())
                .addPath(new BezierLine(new Point(BlockPush1), new Point(SpecGrab)))
                .setLinearHeadingInterpolation(BlockPush1.getHeading(), SpecGrab.getHeading())
                .build();
        preHang = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpecGrab), new Point(HangSpecimen)))
                .setLinearHeadingInterpolation(SpecGrab.getHeading(), HangSpecimen.getHeading())
                .build();
        SpecCollect2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleRight), new Point(SpecPrepStep1)))
                .setLinearHeadingInterpolation(littleRight.getHeading(), SpecPrepStep1.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep1), new Point(SpecPrepStep2)))
                .setLinearHeadingInterpolation(SpecPrepStep1.getHeading(), SpecPrepStep2.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep2), new Point(SpecimenCollect2)))
                .setLinearHeadingInterpolation(SpecPrepStep2.getHeading(), SpecimenCollect2.getHeading())
                .build();
        BlockToBase2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpecimenCollect2), new Point(BlockPush2)))
                .setLinearHeadingInterpolation(SpecimenCollect2.getHeading(), BlockPush2.getHeading())
                .addPath(new BezierLine(new Point(BlockPush2), new Point(SpecGrab)))
                .setLinearHeadingInterpolation(BlockPush2.getHeading(), SpecGrab.getHeading())
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
        switch (autoState) {
            case 0: //closes the claw than waits 1.4 seconds before moving to the next step
                robot.closeMiniClaw();
                robot.closeClaw();
                robot.miniClawAct();
                robot.clawAct();
                if (state_timer.getElapsedTimeSeconds() > 1.4) {
                    next_state();
                }
                break;
            case 1: //raises the arm and sliders to the above bar position
                robot.setArmState(Robot2.armState.ABOVE_BAR);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(state_timer.getElapsedTimeSeconds() > 1.5) {
                    next_state();
                }
                break;
            case 2: //drives to the bar
                if(!follower.isBusy()) {
                    follower.followPath(specimenHang1, true);
                    next_state();
                }
                break;
            case 3: //clips the specimen
                if(!follower.isBusy()) {
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    if (state_timer.getElapsedTimeSeconds() > 2.5) {
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
                if(!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    robot.miniClawAct();
                    robot.clawAct();
                    if(robot.isClawOpen()) {
                        next_state();
                    }
                }
                break;
            case 6: //follows the path to the first sample
                if(!follower.isBusy()){
                    follower.followPath(SpecCollect1);
                    next_state();
                }
                break;
            case 7: //goes to resting position
                robot.setArmState(Robot2.armState.RESTING);
                robot.sliderNoTouchAct();
                robot.allAct();
                next_state();
                break;
            case 8: // pushes the first sample to the HP zone
                if(!follower.isBusy()){
                    follower.followPath(BlockToBase1);
                    next_state();
                }
                break;
            case 9: //grabs the second specimen
                if(!follower.isBusy()) {
                    robot.closeMiniClaw();
                    robot.closeClaw();
                    robot.miniClawAct();
                    robot.clawAct();
                    if(state_timer.getElapsedTimeSeconds() > 1.0){
                        next_state();
                    }
                }
                break;
            case 10: //drives to the bar
                if(!follower.isBusy()){
                    follower.followPath(preHang, true);
                    next_state();
                }
                break;
            case 11: //arm above bar
                robot.setArmState(Robot2.armState.ABOVE_BAR);
                robot.sliderNoTouchAct();
                robot.allAct();
                if(state_timer.getElapsedTimeSeconds() > 2.5) {
                    next_state();
                }
                break;
            case 12: //clips the specimen
                if(!follower.isBusy()){
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    if(state_timer.getElapsedTimeSeconds() > 1.5){
                        next_state();
                    }
                }
                break;
            case 13: //moves a little to the right
                if(!follower.isBusy()){
                    follower.followPath(JustRight);
                    next_state();
                }
                break;
            case 14: // releases the claw
                if(!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    robot.miniClawAct();
                    robot.clawAct();
                    if(state_timer.getElapsedTimeSeconds() > 1.0) {
                        next_state();
                    }
                }
                break;
            case 15: // drives to the second sample
                if(!follower.isBusy()){
                    follower.followPath(SpecCollect2);
                    next_state();
                }
                break;
            case 16: // arm state resting for transport
                robot.setArmState(Robot2.armState.RESTING);
                robot.sliderNoTouchAct();
                robot.allAct();
                next_state();
                break;
            case 17: //drives it to base
                if(!follower.isBusy()){
                    follower.followPath(BlockToBase2);
                    next_state();
                }
                break;
            case 18://grabs the third specimen
                if(!follower.isBusy()) {
                    robot.closeMiniClaw();
                    robot.closeClaw();
                    robot.miniClawAct();
                    robot.clawAct();
                    if(state_timer.getElapsedTimeSeconds() > 1.0){
                        next_state();
                    }
                }
                break;
            case 19: //drives to the bar
                if(!follower.isBusy()){
                    follower.followPath(preHang, true);
                    next_state();
                }
                break;
            case 20: //arm above bar
                robot.setArmState(Robot2.armState.ABOVE_BAR);
                robot.sliderNoTouchAct();
                robot.allAct();
                next_state();
                break;
            case 21: //clips the specimen
                if(!follower.isBusy()){
                    robot.setArmState(Robot2.armState.BELOW_BAR);
                    robot.sliderNoTouchAct();
                    robot.allAct();
                    if(state_timer.getElapsedTimeSeconds() > 1.5){
                        next_state();
                    }
                }
                break;
            case 22: //moves a little to the right
                if(!follower.isBusy()){
                    follower.followPath(JustRight);
                    next_state();
                }
                break;
            case 23: // releases the claw
                if(!follower.isBusy()){
                    robot.openClaw();
                    robot.openMiniClaw();
                    robot.miniClawAct();
                    robot.clawAct();
                    if(state_timer.getElapsedTimeSeconds() > 1.0) {
                        next_state();
                    }
                }
                break;
            case 24: //drives to the HP zone
                if(!follower.isBusy()){
                    follower.followPath(Park);
                    next_state();
                }
            case 25: // arm state resting for transport
                robot.setArmState(Robot2.armState.RESTING);
                robot.sliderNoTouchAct();
                robot.allAct();
                break;






        }
            telemetry.addData("autoState", autoState);


        //follower.telemetryDebug(telemetryA);
    }
}
