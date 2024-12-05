package org.firstinspires.ftc.teamcode.FakeHardware;

import androidx.annotation.NonNull;

public class FakeMotor {
    public enum direction {FORWARD, BACKWARD}
    public enum mode {
        RUN_TO_POSITION,
        CONTINUOUS;
    }
    public enum ZeroPowerBehavior {
        BRAKE,
        FLOAT
    }
    public mode runMode = mode.RUN_TO_POSITION;
    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.FLOAT;

    int tick; //tick can be substituted for position, or rotation angle.
    int runDirection;
    int MAX_TICKS = 4000;
    int MIN_TICKS = 0;
    double MAX_POWER = 1.0;
    double MIN_POWER = -1.0;
    double MAX_RPM, power;

    public FakeMotor(double rpm){
        this.MAX_RPM = rpm;
    }

    public void setRunMode(mode runMode){
        this.runMode = runMode;
    }

    public void setRunDirection(direction dir){
        if (dir == direction.BACKWARD){
            this.runDirection = -1;
        }else if (dir == direction.FORWARD) {
            this.runDirection = 1;
        }
    }

    public int getTick() {
        return tick;
    }

    public void setPower(double power){
        int runDirection = this.runDirection;
        if (MAX_POWER > power && power > MIN_POWER) {
            this.power = power * runDirection;
        } else if (power > MAX_POWER){
            this.power = 1.0 * runDirection;
        }else if (power < MIN_POWER){
            this.power = -1.0 * runDirection;
        }
    }
    public double getPower(){
        return this.power;
    }
    @NonNull
    public String toString() {
        return "FakeMotor{" +
                "runMode=" + runMode +
                ", runDirection=" + runDirection +
                ", MAX_TICKS=" + MAX_TICKS +
                ", MIN_TICKS=" + MIN_TICKS +
                ", MAX_POWER=" + MAX_POWER +
                ", MIN_POWER=" + MIN_POWER +
                ", MAX_RPM=" + MAX_RPM +
                ", power=" + power +
                '}';
    }
    public void applyPowerToPosition(int targetPos){
        double currentTicks = getTick();
        double power = getPower();
        if(this.runMode != mode.RUN_TO_POSITION){
            if (this.runMode == mode.CONTINUOUS){
                applyPowerContinuous();
            }
        }else {
            while (tick != currentTicks){
                if (currentTicks + power > targetPos){
                    power = targetPos - currentTicks;
                }
                currentTicks = currentTicks + power;
            }
        }
    }

    public void applyPowerContinuous(){
        double currentPos = getTick();
        double power = getPower();
        currentPos = currentPos + getPower();
        this.tick = (int) Math.round(currentPos);
    }

    public void setZeroPowerBehavior(ZeroPowerBehavior z){
        this.zeroPowerBehavior = z;
    }
}