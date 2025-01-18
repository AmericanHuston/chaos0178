package org.firstinspires.ftc.teamcode.pedroPathing.examples;

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


/**
 * This is the Triangle autonomous OpMode.
 * It runs the robot in a triangle, with the starting point being the bottom-middle point.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @author Samarth Mahapatra - 1002 CircuitRunners Robotics Surge
 * @version 1.0, 12/30/2024
 */
@Autonomous(name = "Square", group = "Examples")
public class Square extends OpMode {
    private Follower follower;

    private final Pose startingPose = new Pose(8, 96, Math.toRadians(0));
    private final Pose blueBasket = new Pose(24,120, Math.toRadians(270));
    private final Pose redObservation = new Pose(120, 120, Math.toRadians(90));
    private final Pose redBasket = new Pose(120, 24, Math.toRadians(135));
    private final Pose blueObservation = new Pose(24,24, Math.toRadians(270));

    private PathChain square;

    private Telemetry telemetryA;

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        follower.update();

        if (follower.atParametricEnd()) {
            follower.followPath(square, true);
        }

        follower.telemetryDebug(telemetryA);
    }

    /**
     * This initializes the Follower and creates the PathChain for the "triangle". Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startingPose);

        square = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startingPose), new Point(blueBasket)))
                .setLinearHeadingInterpolation(startingPose.getHeading(), blueBasket.getHeading())
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

}
