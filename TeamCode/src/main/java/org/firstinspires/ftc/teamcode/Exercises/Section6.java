package org.firstinspires.ftc.teamcode.Exercises;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FakeHardware.FakeButton;
import org.firstinspires.ftc.teamcode.RobotBoard.FakeBoard1;

public class Section6 extends LinearOpMode {
    FakeBoard1 board = new FakeBoard1();
    public void runOpMode(){
        board.pressButton();
        telemetry.addData("Touch", "button is " + board.isButtonPressedString());
        telemetry.update();
    }
}
