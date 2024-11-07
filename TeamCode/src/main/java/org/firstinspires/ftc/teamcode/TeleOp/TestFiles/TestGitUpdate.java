package org.firstinspires.ftc.teamcode.TeleOp.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class TestGitUpdate extends LinearOpMode {
    private DcMotor ElbowMotor;
    public void runOpMode() {
        ElbowMotor = hardwareMap.get(DcMotor.class,"ElbowMotor");
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
                if (triggerPos == 1.0){
                    ElbowMotor.setPower(0.5);
                }
            }
        }
    }
}