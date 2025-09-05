package org.firstinspires.ftc.teamcode.TestFiles;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot2;

@TeleOp(name = "TestDistanceSensor", group = "TeleOp")
public class testDistanceSensor extends OpMode {
    Robot2 Bot = new Robot2();
    private Follower follower;

    @Override
    public void init() {
        follower = new Follower(hardwareMap);
        Bot.init(hardwareMap);}

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        follower.update();
        telemetry.addData("Distance", Bot.getDistanceFromSensor());
        if (gamepad1.left_stick_button || gamepad1.right_stick_button) {
            follower.startTeleopDrive();
        }
    }
}
