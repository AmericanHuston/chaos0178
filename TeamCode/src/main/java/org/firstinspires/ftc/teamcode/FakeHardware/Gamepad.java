package org.firstinspires.ftc.teamcode.FakeHardware;

public class Gamepad {
    public enum gamepadButtons {
        a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right, bumper_left, bumper_right, back
    }
    FakeButton _a = new FakeButton();
    FakeButton _b = new FakeButton();
    FakeButton _x = new FakeButton();
    FakeButton _y = new FakeButton();
    FakeButton _dpad_up = new FakeButton();
    FakeButton _dpad_down = new FakeButton();
    FakeButton _dpad_right = new FakeButton();
    FakeButton _dpad_left = new FakeButton();
    FakeButton _bumper_left = new FakeButton();
    FakeButton _bumper_right = new FakeButton();
    FakeButton _back = new FakeButton();
/*
Redundant, maybe:
 */
//    public boolean a(){
//        return this._a.isPressed();
//    }
//    public boolean b(){
//        return this._b.isPressed();
//    }
//    public boolean x(){
//        return this._x.isPressed();
//    }
//    public boolean y(){
//        return this._y.isPressed();
//    }
//    public boolean dpad_up(){
//        return this._dpad_up.isPressed();
//    }
//    public boolean dpad_down(){
//        return this._dpad_down.isPressed();
//    }
//    public boolean dpad_right(){
//        return this._dpad_right.isPressed();
//    }
//    public boolean dpad_left(){
//        return this._dpad_left.isPressed();
//    }
//    public boolean left_bumper(){
//        return this._bumper_left.isPressed();
//    }
//    public boolean right_bumper(){
//        return this._bumper_right.isPressed();
//    }
//    public boolean back(){
//        return this._back.isPressed();
//    }
    public volatile boolean a = this._a.isPressed();
    public volatile boolean b = this._b.isPressed();
    public volatile boolean x = this._x.isPressed();
    public volatile boolean y = this._y.isPressed();
    public volatile boolean dpad_up = this._dpad_up.isPressed();
    public volatile boolean dpad_down = this._dpad_down.isPressed();
    public volatile boolean dpad_right = this._dpad_right.isPressed();
    public volatile boolean dpad_left = this._dpad_left.isPressed();
    public volatile boolean bumper_left = this._bumper_left.isPressed();
    public volatile boolean bumper_right = this._bumper_right.isPressed();
    public volatile boolean back = this._back.isPressed();

    public void pressButton(gamepadButtons e){
        switch (e){
            case a: this._a.pressButton();
            case b: this._b.pressButton();
            case x: this._x.pressButton();
            case y: this._y.pressButton();
            case dpad_up: this._dpad_up.pressButton();
            case dpad_down: this._dpad_down.pressButton();
            case dpad_left: this._dpad_left.pressButton();
            case dpad_right: this._dpad_right.pressButton();
            case bumper_left: this._bumper_left.pressButton();
            case bumper_right: this._bumper_right.pressButton();
            case back: this._back.pressButton();
        }
    }

    public boolean getButtonState(gamepadButtons e){
        switch (e){
            case a: return this._a.isPressed();
            case b: return this._b.isPressed();
            case x: return this._x.isPressed();
            case y: return this._y.isPressed();
            case dpad_up: return this._dpad_up.isPressed();
            case dpad_down: return this._dpad_down.isPressed();
            case dpad_left: return this._dpad_left.isPressed();
            case dpad_right: return this._dpad_right.isPressed();
            case bumper_left: return this._bumper_left.isPressed();
            case bumper_right: return this._bumper_right.isPressed();
            case back: return this._back.isPressed();
        }
        return false;
    }
}