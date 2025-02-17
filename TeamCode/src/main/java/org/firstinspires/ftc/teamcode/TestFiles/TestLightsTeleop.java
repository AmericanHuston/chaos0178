package org.firstinspires.ftc.teamcode.TestFiles;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

/**
 * This is an example teleop that showcases movement and field-centric driving.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 12/30/2024
 */
@Disabled
@TeleOp(name = "TestLightsTeleOp", group = "TestFiles")
public class TestLightsTeleop extends OpMode {
    Robot2 robot;
    boolean desired_green_on = false;
    boolean desired_red_on = false;
    /** This method is call once when init is played, it initializes the follower **/
    @Override
    public void init() {
        robot = new Robot2();
        robot.init(hardwareMap);
    }

    /** This method is called continuously after Init while waiting to be started. **/
    @Override
    public void init_loop() {
    }

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        desired_green_on = false;
        desired_red_on = false;
    }

    /** This is the main loop of the opmode and runs continuously after play **/
    @Override
    public void loop() {
        if (gamepad1.a) {
            desired_green_on = true;
        }
        if (gamepad1.b) {
            desired_red_on = true;
        }
        if (gamepad1.y) {
            desired_red_on = false;
            desired_green_on = false;
        }

        if (desired_green_on) {
            robot.leftLEDGreen.on();
            robot.rightLEDGreen.on();
        } else {
            robot.leftLEDGreen.off();
            robot.rightLEDGreen.off();
        }
        robot.leftLEDRed.enableLight(desired_red_on);
        robot.rightLEDRed.enableLight(desired_red_on);

        telemetry.addData("desired_green_on", desired_green_on);
        telemetry.addData("desired_red_on", desired_red_on);
    }

    /** We do not use this because everything automatically should disable **/
    @Override
    public void stop() {
    }
}