package org.firstinspires.ftc.teamcode.FakeRobotBoard;

import org.firstinspires.ftc.teamcode.FakeHardware.FakeButton;

public class FakeBoard1 {
    FakeButton fakeButton = new FakeButton();

    /*
    Presses (Or releases) the button
     */
    public void pressButton(){
        fakeButton.pressButton();
    }

    /*
    returns if the button is pressed, boolean
     */
    public boolean isButtonPressed(){
        return fakeButton.isPressed();
    }
    /*
    Returns string values, whether or not the button is pressed
     */
    public String isButtonPressedString(){
        if (fakeButton.isPressed()){
            return "Pressed";
        }else {
            return "Unpressed";
        }
    }
    /*
    Returns if the button is up
     */
    public boolean isButtonReleased(){
        return !fakeButton.isPressed();
    }
}