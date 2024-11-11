package org.firstinspires.ftc.teamcode.TeleOp.TestFiles;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MeasureSliderMovement", group = "TeleOp")
@Config
public class MeasureSliderMovement extends LinearOpMode {
    public DcMotor sliderLeft = hardwareMap.get(DcMotor.class, "SliderLeft");
    public DcMotor sliderRight = hardwareMap.get(DcMotor.class, "SliderRight");
    @Override
    public void runOpMode() throws InterruptedException {
        //Don't edit code below this point
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        int leftPos = sliderLeft.getCurrentPosition();
        int rightPos = sliderRight.getCurrentPosition();
        double power = 0.35;
        //Don't edit code above this point
        while(opModeIsActive()){
            while (this.gamepad2.dpad_up){
                sliderLeft.setPower(power);
                sliderRight.setPower(power);
                leftPos = sliderLeft.getCurrentPosition();
                rightPos = sliderRight.getCurrentPosition();
                telemetry.addData("leftPos", leftPos);
                telemetry.addData("rightPos", rightPos);
                telemetry.update();
            }
            sliderRight.setPower(0.0);
            sliderLeft.setPower(0.0);
            while (this.gamepad2.dpad_down){
                sliderLeft.setPower(-power);
                sliderRight.setPower(-power);
                leftPos = sliderLeft.getCurrentPosition();
                rightPos = sliderRight.getCurrentPosition();
                telemetry.addData("leftPos", leftPos);
                telemetry.addData("rightPos", rightPos);
                telemetry.update();
            }
            sliderRight.setPower(0.0);
            sliderLeft.setPower(0.0);
        }
    }
}
