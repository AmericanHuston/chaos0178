package org.firstinspires.ftc.teamcode.TestFiles;

import static org.firstinspires.ftc.teamcode.TestFiles.Methods.Adding;
import static org.firstinspires.ftc.teamcode.TestFiles.Methods.Dividing;
import static org.firstinspires.ftc.teamcode.TestFiles.Methods.Multiplying;
import static org.firstinspires.ftc.teamcode.TestFiles.Methods.Subtracting;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@TeleOp
@Disabled
public class TestMethodUsage extends LinearOpMode {
    public void runOpMode(){
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        telemetry.addData("Adding", Adding(2,3));
        telemetry.addData("Subtracting", Subtracting(2,3));
        telemetry.addData("Multiplying", Multiplying(2,3));
        telemetry.addData("Division", Dividing(2,3));
        telemetry.update();
    }
}
