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
import org.firstinspires.ftc.teamcode.EnvironmentLocations.Board0;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "SpecimenAuto3", group = "PedroAutos")
public class SpecimenAuto3 extends OpMode {
    Board0 board = new Board0();
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
    private final Pose StartingPose = new Pose(MIN_WALL_POS, t3, Math.toRadians(0));
    private final Pose Basket = new Pose(t1,t5, Math.toRadians(270));
    private final Pose OtherObservation = new Pose(t5, t5, Math.toRadians(90));
    private final Pose OtherBasket = new Pose(t5, t1, Math.toRadians(135));
    private final Pose Observation = new Pose(8,40, Math.toRadians(0));
    private final Pose HangSpecimen = new Pose(35,t3+12, Math.toRadians(0));
    private final Pose OtherHangSpecimen = new Pose(112,t3,Math.toRadians(90));
    private final Pose TapeHangRobot = new Pose(t3,t4, Math.toRadians(90));
    private final Pose OtherTapeHangRobot = new Pose(t3,t2, Math.toRadians(270));
    private final Pose SpecGrab = new Pose(t2-18, t2-8, Math.toRadians(180));
    private final Pose littleBack = new Pose (25, t3+12, Math.toRadians(0));
    private final Pose littleRight = new Pose(35,t3+8, Math.toRadians(0));
    private final Pose controlHangSpec = new Pose(21, 88);
    private final Pose SpecPrepStep1 = new Pose(32, 54, Math.toRadians(180));
    private final Pose SpecPrepStep2 = new Pose(56, 54, Math.toRadians(180));
    private final Pose SpecPrepStep3 = new Pose(56, 42, Math.toRadians(180));
    private final Pose SpecPrepStep4 = new Pose(10,40, Math.toRadians(180));
    private final Pose SpecCollect1 = new Pose(t3-14, t1-4, Math.toRadians(180));
    private final Pose SpecCollect2 = new Pose(t3-14, t1-12, Math.toRadians(180));
    private final Pose BlockPush = new Pose(10, t1, Math.toRadians(180));
    private final Pose sample1  = new Pose(t1+1, t1, Math.toRadians(0));
    private PathChain square;

    private PathChain pushBlock;
    private PathChain specimenHang1;
    private PathChain JustRight;
    private PathChain SpecimenCollect;
    private PathChain JustBack;
    private PathChain preHang;
    private PathChain specimenHang2;
    private PathChain Park;
    private PathChain SpecCollect;
    private PathChain BlockToBase1;
    private PathChain grabSample;
    private Telemetry telemetryA;

    @Override
    public void init() {
        board.init(hardwareMap);
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
        JustBack = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HangSpecimen), new Point(littleBack)))
                .setLinearHeadingInterpolation(HangSpecimen.getHeading(), littleBack.getHeading())
                .build();
        SpecimenCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleBack), new Point(SpecGrab)))
                .setLinearHeadingInterpolation(littleBack.getHeading(), SpecGrab.getHeading())
                .build();
        preHang = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpecGrab), new Point(littleBack)))
                .setLinearHeadingInterpolation(SpecGrab.getHeading(), littleBack.getHeading())
                .build();
        specimenHang2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleBack), new Point(HangSpecimen)))
                .setLinearHeadingInterpolation(littleBack.getHeading(), HangSpecimen.getHeading())
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HangSpecimen), new Point(Observation)))
                .setLinearHeadingInterpolation(HangSpecimen.getHeading(), Observation.getHeading())
                .build();
        SpecCollect = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleBack), new Point(SpecPrepStep1)))
                .setLinearHeadingInterpolation(littleBack.getHeading(), SpecPrepStep1.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep1), new Point(SpecPrepStep2)))
                .setLinearHeadingInterpolation(SpecPrepStep1.getHeading(), SpecPrepStep2.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep2), new Point(SpecCollect1)))
                .setLinearHeadingInterpolation(SpecPrepStep2.getHeading(), SpecCollect1.getHeading())
                .build();
        BlockToBase1 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(SpecCollect1), new Point(BlockPush)))
                .setLinearHeadingInterpolation(SpecCollect1.getHeading(), BlockPush.getHeading())
                .addPath(new BezierLine(new Point(BlockPush), new Point(SpecGrab)))
                .setLinearHeadingInterpolation(BlockPush.getHeading(), SpecGrab.getHeading())
                .build();
        grabSample = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleBack), new Point(sample1)))
                .setLinearHeadingInterpolation(littleBack.getHeading(),sample1.getHeading())
                .build();
        pushBlock = follower.pathBuilder()
                .addPath(new BezierLine(new Point(littleBack), new Point(SpecPrepStep1)))
                .setLinearHeadingInterpolation(littleBack.getHeading(), SpecPrepStep1.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep1), new Point(SpecPrepStep2)))
                .setLinearHeadingInterpolation(SpecPrepStep1.getHeading(), SpecPrepStep2.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep2), new Point(SpecPrepStep3)))
                .setLinearHeadingInterpolation(SpecPrepStep2.getHeading(), SpecPrepStep3.getHeading())
                .addPath(new BezierLine(new Point(SpecPrepStep3), new Point(SpecPrepStep4)))
                .setLinearHeadingInterpolation(SpecPrepStep3.getHeading(), SpecPrepStep4.getHeading())
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
        telemetryA.addLine("This is the Specimen auto."
                + "It scores two and parks right now."
                + "Chaos²");
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
                board.setClawState(Board0.clawPositions.CLAW_CLOSED);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);

                if (state_timer.getElapsedTimeSeconds() > 1.4) {
                    next_state();
                }
                break;
            case 1: //raises the sliders than waits 1.5 seconds
                board.setArmState(Board0.armStates.ABOVE_BAR);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                if (state_timer.getElapsedTimeSeconds() > 1.5) {
                    next_state();
                }
                break;
            case 2: //drives to the bar
                follower.followPath(specimenHang1, true);
                next_state();
                break;
            case 3: //lowers the slider, clipping the specimen
                if (!follower.isBusy()) {
                    board.setArmState(Board0.armStates.BELOW_BAR);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
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
            case 5: //opens the claw
                if(!follower.isBusy()) {
                    board.setClawState(Board0.clawPositions.CLAW_OPEN);
                    board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                    board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                    next_state();
                }

                break;
            case 6: //goes into resting mode for movement
                board.setArmState(Board0.armStates.RESTING);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                next_state();
                break;
            case 7: //Makes a short move back
                if (!follower.isBusy()){
                    follower.followPath(JustBack);
                    next_state();
                }
                break;
            case 8: //moves to the collection position
                if(!follower.isBusy()) {
                    follower.followPath(SpecimenCollect, true);
                    next_state();
                }
                break;
            case 9: //move the arm to collection position
                if(!follower.isBusy()) {
                    board.setArmState(Board0.armStates.COLLECTION);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
                    if (state_timer.getElapsedTimeSeconds() > 4.0) {
                        next_state();
                    }
                }
                break;
            case 10: //grabs the specimen
                board.setClawState(Board0.clawPositions.CLAW_CLOSED);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                if (state_timer.getElapsedTimeSeconds() > 1.0){
                    next_state();
                }
                break;
            case 11: //prepares to hang the specimen
                board.setArmState(Board0.armStates.ABOVE_BAR);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                if (state_timer.getElapsedTimeSeconds() > 1.4) {
                    next_state();
                }
                break;
            case 12: //drives to hang the specimen part one
                if (!follower.isBusy()){
                    follower.followPath(preHang);
                    next_state();
                }
                break;
            case 13: //drives to hang the specimen part two
                if (!follower.isBusy()){
                    follower.followPath(specimenHang2);
                    next_state();
                }
                break;
            case 14: //lowers the sliders after waiting for the robot to stop moving
                if(!follower.isBusy()) {
                    board.setArmState(Board0.armStates.BELOW_BAR);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
                    if (state_timer.getElapsedTimeSeconds() > 2.5) {
                        next_state();
                    }
                }
                break;
            case 15: //opens the claw
                board.setClawState(Board0.clawPositions.CLAW_OPEN);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                if(state_timer.getElapsedTimeSeconds() > 1.0){
                    next_state();
                }
                break;
            case 16: //goes to resting mode
                board.setArmState(Board0.armStates.RESTING);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                next_state();
                break;
            case 17://safe from the bar
                if(!follower.isBusy()){
                    follower.followPath(JustBack);
                    next_state();
                }
                break;
            case 18:
                if(!follower.isBusy()){
                    follower.followPath(pushBlock);
                    next_state();
                }
                break;
//            case 19:
//                if(!follower.isBusy()){
//                    board.setArmState(Board0.armStates.COLLECTION);
//                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
//                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
//                    if (state_timer.getElapsedTimeSeconds() > 1.5) {
//                        next_state();
//                    }
//                }
//                break;
//            case 20: //opens the claw
//                board.setClawState(Board0.clawPositions.CLAW_OPEN);
//                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
//                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
//                if(state_timer.getElapsedTimeSeconds() > 1.0){
//                    next_state();
//                }
//                break;
//            case 21: //goes to resting mode
//                board.setArmState(Board0.armStates.RESTING);
//                board.stateMachinesThink(Board0.stateMachineAct.ARM);
//                board.stateMachinesAct(Board0.stateMachineAct.ARM);
//                next_state();
//                break;
//            case 28: //parks!
//                if(!follower.isBusy()) {
//                    follower.followPath(Park, true);
//                    next_state();
//                }
//                break;





        }
            telemetry.addData("autoState", autoState);


        //follower.telemetryDebug(telemetryA);
    }
}
