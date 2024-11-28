package org.firstinspires.ftc.teamcode.FakeHardware;

import androidx.annotation.NonNull;

public class FakeLED {
    boolean light = false;
    public void turnOnLED(){
        this.light = true;
    }
    public void turnOffLED(){
        this.light = false;
    }
    public boolean isLight(){
        return this.light;
    }
    @NonNull
    public String toString() {
        return "FakeLED{" +
                "light=" + light +
                '}';
    }
}
