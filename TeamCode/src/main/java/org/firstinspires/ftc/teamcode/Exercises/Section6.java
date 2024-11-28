package org.firstinspires.ftc.teamcode.Exercises;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FakeHardware.FakeButton;
public class Section6 extends LinearOpMode {
    FakeButton button = new FakeButton();
    @Override
    public void runOpMode() throws InterruptedException {
        button.pressButton();
        if (button.isPressed()){
            telemetry.addData(">", "Button is pressed");
        }
        telemetry.update();
    }
}
