package org.firstinspires.ftc.teamcode.Exercises;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FakeHardware.FakeButton;
import org.firstinspires.ftc.teamcode.FakeHardware.FakeMotor;
import org.firstinspires.ftc.teamcode.FakeRobotBoard.FakeBoard2;

public class Section7 extends LinearOpMode {
    FakeBoard2 Board = new FakeBoard2();
    FakeButton Button0 = new FakeButton();
    FakeMotor Motor0 = new FakeMotor(1000);
    @Override
    public void runOpMode() throws InterruptedException {
        Board.pressButton(Button0);
        if(Board.isButtonPressed(Button0)){
            Board.setMotorPower(Motor0, 0.5);
        }else {
            Board.setMotorPower(Motor0, 0.0);
        }
        //Outputs data to telemetry
    }
}
