package org.firstinspires.ftc.teamcode.FakeHardware;

public class FakeButton {
    boolean pressed = false;

    public void pressButton(){
        boolean localPressed = this.pressed;
        this.pressed = !localPressed;
    }
    public boolean isPressed(){
        return this.pressed;
    }
}