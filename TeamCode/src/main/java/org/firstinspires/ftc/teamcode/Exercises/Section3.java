package org.firstinspires.ftc.teamcode.Exercises;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Section3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        double difference = this.gamepad1.left_stick_y - this.gamepad1.right_stick_y;
        double total = this.gamepad1.left_trigger + this.gamepad1.right_trigger;
        if (difference < 0.0){
            difference = -difference;
        }
        telemetry.addData("Right Stick", this.gamepad1.right_stick_x);
        telemetry.addData("B Button", this.gamepad1.b);
        telemetry.addData("Sticks Difference", difference);
        telemetry.addData("Total Triggers", total);
        telemetry.update();
    }
}
