package org.firstinspires.ftc.teamcode.TeleOp;

import android.widget.GridLayout;

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
import org.firstinspires.ftc.teamcode.arm;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

@TeleOp(name = "TwoArmTwoFurious", group = "TeleOp")
public class TwoArmTwoFurious extends OpMode {

    private Follower follower;
    private final Pose startPose = new Pose(8,72,0);
    private final Pose SpecGrab = new Pose(29, 40, Math.toRadians(180));
    private final Pose Basket = new Pose(15.5,140.5, Math.toRadians(130));

    private PathChain ToSpecPickup;

    Robot2 Robot = new Robot2();

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        follower.update();

        //Driving------------------
        if(gamepad1.a){
            follower.holdPoint(SpecGrab);
        }
        if (gamepad1.b){
            follower.holdPoint(Basket);
        }
        if(gamepad1.left_stick_button || gamepad1.right_stick_button){
            follower.startTeleopDrive();
        }
        if(gamepad1.x){
            ToSpecPickup = follower.pathBuilder()
                    .addPath(new BezierLine(new Point(startPose.getX(), startPose.getY()), new Point(SpecGrab)))
                    .setLinearHeadingInterpolation(startPose.getHeading(), SpecGrab.getHeading())
                    .build();
            follower.followPath(ToSpecPickup);
        }
        //Driving----------------

        //Rewrite below----------
        if (gamepad1.back) {
            Robot.resetIMU();
        }
        if(gamepad2.right_trigger > 0.01) {
            Robot.setClawPosition(Range.scale(gamepad2.right_trigger, 0.0, 1.0, 0.5, 0.99));
        }
        if(gamepad2.right_bumper && !changedClaw){
            if(openClaw) {
                desired_claw_position = 0.99;
                desired_miniClaw_position = 0.99;
            }else{
                desired_claw_position = 0.5;
                desired_miniClaw_position = 0.5;
            }
            changedClaw = true;
            openClaw = !openClaw;
        } else if (!gamepad2.right_bumper) {
            changedClaw = false;
        }

        if(gamepad2.left_trigger > 0.01){
            Robot.wrist45();
        }
        if(gamepad2.left_bumper && !changedWrist){
            if(openWrist) {
                desired_wrist_position = 0.5;
            }else{
                desired_wrist_position = 0.1;
            }
            changedWrist = true;
            openWrist = !openWrist;
        } else if (!gamepad2.left_bumper) {
            changedWrist = false;
        }
        if (gamepad2.right_stick_x> 0.01|| gamepad2.right_stick_x < -0.01)  {
            int shoulder_position = Shoulder.getTargetPosition();
            int shoulder_change = (int)gamepad2.right_stick_x * 10;
            desired_shoulder_position = shoulder_position + shoulder_change;
        }

        if (gamepad2.y) {Robot.setArmState(Robot2.armState.RESTING);}
        if (gamepad2.x) {Robot.setArmState(Robot2.armState.BASKET);}
        if (gamepad2.a) {Robot.setArmState(Robot2.armState.SPECIMEN);}
        if (gamepad2.b) {Robot.setArmState(Robot2.armState.COLLECTION);}
        if (gamepad2.dpad_left) {Robot.setArmState(Robot2.armState.ABOVE_BAR);}
        if (gamepad2.dpad_right) {Robot.setArmState(Robot2.armState.BELOW_BAR);}
        if (gamepad2.dpad_up) {Robot.setArmState(Robot2.armState.PREHANG);}
        if (gamepad2.dpad_down) {Robot.setArmState(Robot2.armState.POSTHANG);}
        if (gamepad1.dpad_up) {Robot.sliderMove(50);}
        if (gamepad1.dpad_down) {Robot.sliderMove(-50);}
        //Rewrite above----------










        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        /* Update Telemetry to the Driver Hub */
        telemetry.update();
    }
}
