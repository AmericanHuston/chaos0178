package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TestSliders", group="TeleOp")
public class Sliders extends LinearOpMode {

    public DcMotor sliderLeft = hardwareMap.get(DcMotor.class, "sliderLeft");
    public DcMotor sliderRight = hardwareMap.get(DcMotor.class, "sliderRight");
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        double power = 0.35;
        while(opModeIsActive()){
            while (this.gamepad1.dpad_up){
                sliderLeft.setPower(power);
                sliderRight.setPower(power);
            }
            sliderRight.setPower(0.0);
            sliderLeft.setPower(0.0);
            while (this.gamepad1.dpad_down){
                sliderLeft.setPower(-power);
                sliderRight.setPower(-power);
            }
            sliderRight.setPower(0.0);
            sliderLeft.setPower(0.0);
        }
    }
}
