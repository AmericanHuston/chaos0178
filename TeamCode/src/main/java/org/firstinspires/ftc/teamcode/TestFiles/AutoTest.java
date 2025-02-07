package org.firstinspires.ftc.teamcode.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="AutoTest", group="Autonomous")
@Config
@Disabled
public class AutoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        //Start of fancy stuff
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        //Done with the fancy stuff
        waitForStart();
        int i = 0;
        while(opModeIsActive()){
            for (; i < 10000000; i++) {
                idle();
                telemetry.addData("CountingUp", i);
                telemetry.update();
            }
            requestOpModeStop();
        }
    }
}
