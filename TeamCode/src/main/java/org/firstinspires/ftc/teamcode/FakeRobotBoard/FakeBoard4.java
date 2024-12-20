package org.firstinspires.ftc.teamcode.FakeRobotBoard;

import org.firstinspires.ftc.teamcode.FakeHardware.FakeButton;
import org.firstinspires.ftc.teamcode.FakeHardware.FakeHardwareMap;
import org.firstinspires.ftc.teamcode.FakeHardware.FakeMotor;

public class FakeBoard4 {
    public FakeButton button;
    public void init(FakeHardwareMap hw){
        button = (FakeButton) hw.get(FakeButton.class, "aButton");
    }
    /*
    Presses (Or releases) the button
     */
    public void pressButton(FakeButton Button){
        Button.pressButton();
    }

    /*
    returns if the button is pressed, boolean
     */
    public boolean isButtonPressed(FakeButton Button){
        return Button.isPressed();
    }
    /*
    Returns string values, whether or not the button is pressed
     */
    public String isButtonPressedString(FakeButton Button){
        if (Button.isPressed()){
            return "Pressed";
        }else {
            return "Unpressed";
        }
    }
    /*
    Returns if the button is up
     */
    public boolean isButtonReleased(FakeButton Button){
        return !Button.isPressed();
    }
    /*
    Sets motor speed
     */
    public void setMotorPower(FakeMotor Motor, double speed){
        Motor.setPower(speed);
    }
    /*
    Sets the motor's runMode
     */
    public void setMotorMode(FakeMotor Motor, FakeMotor.mode mode){
        Motor.setRunMode(mode);
    }
    /*
    Sets the motor's Zero Power Behavior
     */
    public void setMotorZeroPowerBehavior(FakeMotor Motor, FakeMotor.ZeroPowerBehavior mode){
        Motor.setZeroPowerBehavior(mode);
    }
}