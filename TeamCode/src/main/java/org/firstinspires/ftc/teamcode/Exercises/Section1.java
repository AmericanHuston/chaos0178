package org.firstinspires.ftc.teamcode.Exercises;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
@Disabled
@Autonomous(name = "Section1", group = "Autonomous")
public class Section1 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        String name = "Noah";
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        telemetry.addData("Hello", name);
        telemetry.update();
    }
}
