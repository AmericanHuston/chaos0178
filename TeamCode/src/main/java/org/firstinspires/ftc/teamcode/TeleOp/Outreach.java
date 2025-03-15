package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "Outreach ", group = "TeleOp")
public class Outreach extends OpMode {

    private Follower follower;

    private PathChain ToSpecPickup;
    private PathChain ToBasket;

    Robot2 robot = new Robot2();

    @Override
    public void init() {
        robot.init(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(Robot2.getLastPose());
    }

    @Override
    public void start() {
        follower.startTeleopDrive();}

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        follower.update();
        robot.setAllLEDOff();
        //Driving------------------

        //Rewrite below----------
        if (gamepad1.back) {
            follower.setPose(new Pose(8, 71, Math.toRadians(0)));
            follower.update();
        }
        if (gamepad2.right_trigger > 0.01) {
            robot.setClawPosition(Range.scale(gamepad2.right_trigger, 0.0, 1.0, 0.5, 0.99));
            robot.setMiniClawPosition(Range.scale(gamepad2.right_trigger, 0.0, 1.0, 0.5, 0.99));
        }
        if(gamepad2.right_bumper && !robot.changedClaw){
            robot.toggleClaw();
            robot.changedClaw = true;
        } else if (!gamepad2.right_bumper) {
            robot.changedClaw = false;
        }

        if (gamepad2.left_trigger > 0.01) {
            robot.setWristPosition(Range.scale(gamepad2.left_trigger, 0.0, 1.0, 0.0, 1.0));
        }
        if (gamepad2.left_bumper && !robot.changedWrist) {
            if (robot.getWristPosition() <= 0.45) {
                robot.wristHorizontal();
                robot.changedWrist = true;
            } else {
                robot.wristVertical();
                robot.changedWrist = true;
            }
        } else if (!gamepad2.left_bumper) {
            robot.changedWrist = false;
        }
        if (gamepad2.right_stick_x > 0.01 || gamepad2.right_stick_x < -0.01) {
            int shoulder_change = (int) gamepad2.right_stick_x * 10;
            robot.setShoulderPosition(robot.getShoulderPosition() + shoulder_change);
        }

        if (gamepad2.y) {
            robot.setArmState(Robot2.armState.RESTING);
        }
        if (gamepad2.x) {
            robot.setArmState(Robot2.armState.BASKET);
        }
        if (gamepad2.a) {
            robot.setArmState(Robot2.armState.SPECIMEN);
        }
        if (gamepad2.b) {
            robot.setArmState(Robot2.armState.COLLECTION);
        }
        if (gamepad2.dpad_left) {
            robot.setArmState(Robot2.armState.ABOVE_BAR);
        }
        if (gamepad2.dpad_right) {
            robot.setArmState(Robot2.armState.BELOW_BAR);
        }
        if (gamepad2.dpad_up) {
            robot.setArmState(Robot2.armState.PREHANG);
        }
        if (gamepad2.dpad_down) {
            robot.setArmState(Robot2.armState.POSTHANG);
        }
        if (gamepad1.dpad_up) {
            robot.sliderMove(50);
            robot.sliderNoTouchAct();
        }
        if (gamepad1.dpad_down) {
            robot.sliderMove(-50);
            robot.sliderNoTouchAct();
        }
        if (robot.getClawPosition() >= 0.70){
            robot.setRedLED(true);
        }
        if (follower.isBusy()) {
            robot.setGreenLED(true);
        }
        robot.allAct();
        //Rewrite above----------

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("wrist pos", robot.getWristPosition());

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
    @Override
    public void stop() {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}
