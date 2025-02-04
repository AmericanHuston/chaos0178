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
    Robot2 Robot = new Robot2();
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
    private final Pose SpecCollect1 = new Pose(t2-18, t2-8, Math.toRadians(180));
    private final Pose littleBack = new Pose (25, t3+12, Math.toRadians(0));
    private final Pose littleRight = new Pose(35,t3+8, Math.toRadians(0));
    private final Pose controlHangSpec = new Pose(21, 88);
    private PathChain square;

    private PathChain specimenHang1;
    private PathChain JustRight;
    private PathChain SpecimenCollect;
    private PathChain JustBack;
    private PathChain preHang;
    private PathChain specimenHang2;
    private PathChain Park;
    private Telemetry telemetryA;

    @Override
    public void init() {
        Robot.init(hardwareMap);
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
        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(HangSpecimen), new Point(Observation)))
                .setLinearHeadingInterpolation(HangSpecimen.getHeading(), Observation.getHeading())
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
                Robot.closeMiniClaw();
                Robot.miniClawAct();
                if (state_timer.getElapsedTimeSeconds() > 1.4) {
                    next_state();
                }
                break;
            case 1:
                follower.followPath(specimenHang1, true);
                if(state_timer.getElapsedTimeSeconds() > 1.0){
                    next_state();
                }
                break;

        }
            telemetry.addData("autoState", autoState);


        //follower.telemetryDebug(telemetryA);
    }
}
