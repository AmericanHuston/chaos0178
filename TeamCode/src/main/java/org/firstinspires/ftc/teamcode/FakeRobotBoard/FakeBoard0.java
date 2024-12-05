package org.firstinspires.ftc.teamcode.FakeRobotBoard;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import org.firstinspires.ftc.teamcode.FakeHardware.FakeButton;

public class FakeBoard0 {
    FakeButton fakeButton = new FakeButton();
    public void pressButton(){
        fakeButton.pressButton();
    }
    public void isButtonPressed(){
        if (fakeButton.isPressed()){
            telemetry.addData(">", "The button is pressed");
        }else {
            telemetry.addData(">", "The button is not pressed");
        }
    }
}
