package org.firstinspires.ftc.teamcode.FakeHardware;

public class FakeServo {
    enum mode {RUN_TO_POSITION, CONTINUOUS}
    mode currentMode = mode.RUN_TO_POSITION;
    double pos;
    double targetPos;
    double power;

    public void setPower(double power) {
        this.power = power;
    }
    public double getPower() {
        return power;
    }
    public double getTargetPos() {
        return targetPos;
    }
    public void setTargetPos(double targetPos){
        this.targetPos = targetPos;
    }
    public double getPos(){
        return this.pos;
    }
    public void setMode(mode Mode){
        this.currentMode = Mode;
    }
    public void changePos(){
        double power = getPower();
        if (this.currentMode == mode.RUN_TO_POSITION){
            while (getTargetPos() > getPos()) {
                if (targetPos < getPos() + power) {
                    this.pos = targetPos;
                }
                this.pos = getPos() + power;
            }
        } else if (this.currentMode == mode.CONTINUOUS) {
            this.pos = this.pos + power;
        }
    }
}