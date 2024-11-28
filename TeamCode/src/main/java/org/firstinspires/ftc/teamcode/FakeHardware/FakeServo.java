package org.firstinspires.ftc.teamcode.FakeHardware;

public class FakeServo {
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
    public void changePos(){
        double power = getPower();
        while (getTargetPos() > getPos()){
            if (targetPos < getPos() + power){
                this.pos = targetPos;
            }
            this.pos = getPos() + power;
        }
    }
}