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
        //"Crazy mode"

        float prevX = this.gamepad1.right_stick_x;
        float prevY = this.gamepad1.right_stick_y;
        float x = prevX;
        float y = prevY;
        if (this.gamepad1.a){
            x = prevY;
            y = prevX;
        }
        //Do all the control stuff
    }
}
