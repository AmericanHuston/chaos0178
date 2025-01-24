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

@Autonomous(name = "BasketAuto", group = "PedroAutos")
public class BasketAuto extends OpMode {
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
    private final Pose StartingPose = new Pose(MIN_WALL_POS, t5, Math.toRadians(90));
    private final Pose Basket = new Pose(t1-8.5,t5+21.5, Math.toRadians(130));
    private final Pose OtherObservation = new Pose(t5, t5, Math.toRadians(90));
    private final Pose OtherBasket = new Pose(t5, t1, Math.toRadians(135));
    private final Pose Observation = new Pose(8,50, Math.toRadians(0));
    private final Pose TapeHangRobot = new Pose(t3,t4, Math.toRadians(90));
    private final Pose OtherTapeHangRobot = new Pose(t3,t2, Math.toRadians(270));
    private final Pose littleBack = new Pose(t1-3, t5-3, Math.toRadians(130));
    private final Pose sample1  = new Pose(t1+1, t5+14, Math.toRadians(0));
    private final Pose sample2 = new Pose(t1+1,t5+24, Math.toRadians(0));
    private final Pose hangBar = new Pose(t4,t4-5, Math.toRadians(270));

    private PathChain square;

    private PathChain sampleCollect1;
    private PathChain scoreBasket1;
    private PathChain JustBack1;
    private PathChain scoreBasket2;
    private PathChain sampleCollect2;
    private PathChain scoreBasket3;
    private PathChain park;
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
        scoreBasket3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(sample2), new Point(Basket)))
                .setLinearHeadingInterpolation(sample2.getHeading(), Basket.getHeading())
                .build();
        park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(Basket), new Point(Observation)))
                .setLinearHeadingInterpolation(Basket.getHeading(), Observation.getHeading())
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
        telemetryA.addLine("This is the Basket auto."
                + "It scores three and parks right now."
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
            case 0: //closes the claw than waits 0.5 seconds before moving to the next step
                board.setClawState(Board0.clawPositions.CLAW_CLOSED);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                if (state_timer.getElapsedTimeSeconds() > 1.0) {
                    next_state();
                }
                break;
            case 1: //raises the sliders than waits 1.5 seconds
                board.setArmState(Board0.armStates.BASKET);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                if (state_timer.getElapsedTimeSeconds() > 1.5) {
                    next_state();
                }
                break;
            case 2: //drives to the basket
                follower.setMaxPower(1.0);
                follower.followPath(scoreBasket1, true);
                if(state_timer.getElapsedTimeSeconds() > 0.7) {
                    next_state();
                }
                break;
            case 3://drops the sample in the basket
                if(!follower.isBusy()){
                    board.setClawState(Board0.clawPositions.CLAW_OPEN);
                    board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
                    if(state_timer.getElapsedTimeSeconds() > 0.5) {
                        next_state();
                    }
                }
                break;
            case 4: //moves back so we don't accidentally ascend.
                if(!follower.isBusy()) {
                    follower.followPath(JustBack1);
                    if(state_timer.getElapsedTimeSeconds() > 0.5) {
                        next_state();
                    }
                }
                break;
            case 5: //moves the arm to resting so we don't tip
                if (!follower.isBusy()){
                    board.setArmState(Board0.armStates.RESTING);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
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
                    board.setArmState(Board0.armStates.COLLECTION);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
                    if (state_timer.getElapsedTimeSeconds() > 3.0) {
                        next_state();
                    }
                }
                break;
            case 8: //grabs the sample
                board.setClawState(Board0.clawPositions.CLAW_CLOSED);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                if(state_timer.getElapsedTimeSeconds() > 0.5){
                    next_state();
                }
                break;
            case 9: //sets the arm to the basket position
                board.setArmState(Board0.armStates.BASKET);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                if (state_timer.getElapsedTimeSeconds() > 1.0){
                    next_state();
                }
                break;
            case 10: //moves to scoring position
                follower.setMaxPower(0.80);
                if(!follower.isBusy()) {
                    follower.followPath(scoreBasket2, true);
                    next_state();
                }
                break;
            case 11://drops the sample in the bucket
                if (!follower.isBusy() && state_timer.getElapsedTimeSeconds() > 0.5){
                    board.setClawState(Board0.clawPositions.CLAW_OPEN);
                    board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                    board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                    if(state_timer.getElapsedTimeSeconds() > 1.0){
                        next_state();
                    }
                }
                break;
            case 12: //moves back so we don't accidentally ascend.
                follower.setMaxPower(1.0);
                if(!follower.isBusy()) {
                    follower.followPath(JustBack1);
                    next_state();
                }
                break;
            case 13://drives to the second sample
                if(!follower.isBusy()){
                    follower.followPath(sampleCollect2, true);
                    next_state();
                }
                break;
            case 14: //arm to the collection position
                if(!follower.isBusy()){
                    board.setArmState(Board0.armStates.COLLECTION);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
                    if (state_timer.getElapsedTimeSeconds() > 5.3) {
                        next_state();
                    }
                }
                break;
            case 15: //grabs the second sample
                board.setClawState(Board0.clawPositions.CLAW_CLOSED);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                if(state_timer.getElapsedTimeSeconds() > 0.5){
                    next_state();
                }
                break;
            case 16: //sets the arm to the basket position
                board.setArmState(Board0.armStates.BASKET);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                if (state_timer.getElapsedTimeSeconds() > 1.0){
                    next_state();
                }
                break;
            case 17: //moves to scoring position
                if(!follower.isBusy()) {
                    follower.followPath(scoreBasket2, true);
                    next_state();
                }
                break;
            case 18://drops the sample in the bucket
                if (!follower.isBusy()){
                    board.setClawState(Board0.clawPositions.CLAW_OPEN);
                    board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                    board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                    if(state_timer.getElapsedTimeSeconds() > 0.5){
                        next_state();
                    }
                }
                break;
            case 19: //parks
                if(!follower.isBusy()) {
                    follower.followPath(park);
                    next_state();
                }
                break;
            case 20: //arm in resting for initialization
                if(state_timer.getElapsedTimeSeconds() > 1.5){
                    board.setArmState(Board0.armStates.RESTING);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
                    next_state();
                }
                break;

        }
        telemetry.addData("autoState", autoState);


        //follower.telemetryDebug(telemetryA);
    }
}