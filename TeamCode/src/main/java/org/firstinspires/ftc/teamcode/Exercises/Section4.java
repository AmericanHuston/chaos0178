package org.firstinspires.ftc.teamcode.Exercises;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Section4 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        double currentPos;
        double previousPos = this.gamepad1.left_stick_y;
        if (this.gamepad1.a){ //turbo, theoretical
            currentPos = previousPos;
        }else {
            currentPos = previousPos*2;
        }
        if (currentPos > 1.0){
            currentPos = 1.0;
        }
    }
}
