package org.firstinspires.ftc.teamcode.Exercises;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotBoard.RobotLocation;

public class Section5 extends LinearOpMode {
    RobotLocation robotLocation = new RobotLocation(0);

    @Override
    public void runOpMode() throws InterruptedException {
        if(gamepad1.a){
            robotLocation.turn(0.1);
        } else if (gamepad1.b){
            robotLocation.turn(-0.1);
        }

        if (this.gamepad1.dpad_left){
            robotLocation.changeX(-0.1);
        } else if (this.gamepad1.dpad_right){
            robotLocation.changeX(0.1);
        }

        if (this.gamepad1.dpad_up){
            robotLocation.changeY(0.1);
        } else if (this.gamepad1.dpad_down){
            robotLocation.changeY(-0.1);
        }

        telemetry.addData("Location:", robotLocation);
        telemetry.addData("Heading", robotLocation.getHeading());
        telemetry.update();
    }
}
