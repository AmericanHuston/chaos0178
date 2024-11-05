package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Config
public class TeleUpdateTest extends LinearOpMode {
    public void runOpMode() {
        float triggerPos;
        //Don't edit code below this point
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        //Don't edit code above this point
        waitForStart();
        while (opModeIsActive()) {
            if (this.gamepad1.left_trigger != 0.0) {
                triggerPos = this.gamepad1.left_trigger;
                telemetry.addData("position", triggerPos);
                telemetry.update();
            }
        }
    }
}