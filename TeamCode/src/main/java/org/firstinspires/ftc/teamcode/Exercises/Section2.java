package org.firstinspires.ftc.teamcode.Exercises;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Section2 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        String name = "Noah";
        int grade = 98;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        telemetry.addData("Hello", name + ", your current grade is: " + grade);
        telemetry.update();
    }
}
