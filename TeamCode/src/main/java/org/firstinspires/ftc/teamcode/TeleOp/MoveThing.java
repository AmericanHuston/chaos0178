package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MoveThing", group = "TeleOp")
public class MoveThing extends LinearOpMode {
    private Servo arm;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()) {
            arm = hardwareMap.get(Servo.class, "arm");
            if (arm.getPosition() < 0.2) {
                arm.setPosition(this.gamepad1.left_stick_y);
            }
        }
    }
}
