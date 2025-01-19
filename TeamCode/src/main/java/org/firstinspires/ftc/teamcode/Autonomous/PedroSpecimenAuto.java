package org.firstinspires.ftc.teamcode.Autonomous;


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
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@Autonomous(name = "PedroSpecimenAuto", group = "PedroAutos")
public class PedroSpecimenAuto extends OpMode {
    private Follower follower;
    private final int MIN_WALL_POS = 8;
    private final int t1 = 24;
    private final int t2 = 48;
    private final int t3 = 72;
    private final int t4 = 96;
    private final int t5 = 120;
    private final int t6 = 144; //NOTE: Our robot's center is about 8 inches from the wall. Max is actually 136
    private final int MAX_WALL_POS = 136;


    //All in inches...Not centimeters
    //Blue Observation Zone is 0,0
    //Red Observation Zone is 144,144
    //0 PROBABLY intersects the fully colored bars
    // Pose goes in this order: Pose(x,y, Radians);
    private final Pose redStartingPose = new Pose(MAX_WALL_POS, t4, Math.toRadians(180));
    private final Pose blueStartingPose = new Pose(MIN_WALL_POS, t4, Math.toRadians(0));
    private final Pose blueBasket = new Pose(t1,t5, Math.toRadians(270));
    private final Pose redObservation = new Pose(t5, t5, Math.toRadians(90));
    private final Pose redBasket = new Pose(t5, t1, Math.toRadians(135));
    private final Pose blueObservation = new Pose(t1,t1, Math.toRadians(270));
    private final Pose blueHangSpecimen = new Pose(38,t3, Math.toRadians(90));
    private final Pose redHangSpecimen = new Pose(112,t3,Math.toRadians(90));
    private final Pose blueTapeHangRobot = new Pose(t3,t4, Math.toRadians(90));
    private final Pose redTapeHangRobot = new Pose(t3,t2, Math.toRadians(270));

    private PathChain square;

    private Telemetry telemetryA;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(blueStartingPose);

        square = follower.pathBuilder()
                .addPath(new BezierLine(new Point(blueStartingPose), new Point(blueBasket)))
                .setLinearHeadingInterpolation(blueStartingPose.getHeading(), blueBasket.getHeading())
                .addPath(new BezierLine(new Point(blueBasket), new Point(redObservation)))
                .setLinearHeadingInterpolation(blueBasket.getHeading(), redObservation.getHeading())
                .addPath(new BezierLine(new Point(redObservation), new Point(redBasket)))
                .setLinearHeadingInterpolation(redObservation.getHeading(), redBasket.getHeading())
                .addPath(new BezierLine(new Point(redBasket), new Point(blueObservation)))
                .setLinearHeadingInterpolation(redBasket.getHeading(), blueObservation.getHeading())
                .addPath(new BezierLine(new Point(blueObservation), new Point(blueBasket)))
                .setLinearHeadingInterpolation(blueObservation.getHeading(), blueBasket.getHeading())
                .build();

        follower.followPath(square);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run in a roughly triangular shape,"
                + "starting on the bottom-middle point. So, make sure you have enough "
                + "space to the left, front, and right to run the OpMode.");
        telemetryA.update();
    }

    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(square, true);
        }

        follower.telemetryDebug(telemetryA);
    }
}
