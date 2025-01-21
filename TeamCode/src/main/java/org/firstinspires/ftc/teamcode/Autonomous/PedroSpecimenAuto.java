package org.firstinspires.ftc.teamcode.Autonomous;


import static android.os.SystemClock.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.EnvironmentLocations.Board0;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "PedroSpecimenAuto", group = "PedroAutos")
public class PedroSpecimenAuto extends OpMode {
    Board0 board = new Board0();
    private Follower follower;
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
    private final Pose Observation = new Pose(t1,t1, Math.toRadians(270));
    private final Pose HangSpecimen = new Pose(30,t3+12, Math.toRadians(0));
    private final Pose OtherHangSpecimen = new Pose(112,t3,Math.toRadians(90));
    private final Pose TapeHangRobot = new Pose(t3,t4, Math.toRadians(90));
    private final Pose OtherTapeHangRobot = new Pose(t3,t2, Math.toRadians(270));

    private PathChain square;

    private PathChain specimenHang;

    private Telemetry telemetryA;

    @Override
    public void init() {
        board.init(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(StartingPose);

        specimenHang = follower.pathBuilder()
                .addPath(new BezierLine(new Point(StartingPose), new Point(HangSpecimen)))
                .setLinearHeadingInterpolation(StartingPose.getHeading(), HangSpecimen.getHeading())
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
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }


    @Override
    public void loop() {
        follower.update();
        switch (autoState) {
            case 0:
                board.setClawState(Board0.clawPositions.CLAW_CLOSED);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                autoState += 1;
                sleep(1400);
                break;
            case 1:
                board.setArmState(Board0.armStates.ABOVE_BAR);
                board.stateMachinesThink(Board0.stateMachineAct.ARM);
                board.stateMachinesAct(Board0.stateMachineAct.ARM);
                sleep(1400);
                autoState += 1;
                break;
            case 2:
                follower.followPath(specimenHang, true);
                autoState += 1;
                break;
            case 3:
                if (!follower.isBusy()) {
                    board.setArmState(Board0.armStates.BELOW_BAR);
                    board.stateMachinesThink(Board0.stateMachineAct.ARM);
                    board.stateMachinesAct(Board0.stateMachineAct.ARM);
                }
                autoState += 1;
                break;
            case 4:
                sleep(1400);
                board.setClawState(Board0.clawPositions.CLAW_OPEN);
                board.stateMachinesThink(Board0.stateMachineAct.CLAW);
                board.stateMachinesAct(Board0.stateMachineAct.CLAW);
                autoState +=1;
                break;
        }
        telemetry.addData("autoState", autoState);
        follower.telemetryDebug(telemetryA);
    }
}
