package org.firstinspires.ftc.teamcode.TeleOp.TestFiles;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/*
 * This OpMode scans a single servo back and forward until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a Robot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Config
@TeleOp(name = "Test: Shoulder Rotation", group = "Test")
public class TestShoulderRotation extends LinearOpMode {

    static final double INCREMENT   = 0.1;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   100;     // period of each cycle
    public static double MAX_POS     =  10.0;     // Maximum rotational position
    public static double MIN_POS     =  0.0;     // Minimum rotational position

    public static double shoulderPower = 0.2;

    // Define class members
    DcMotorEx   shoulder;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        shoulder = hardwareMap.get(DcMotorEx.class, "Shoulder");
        shoulder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the start button
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){


            if (gamepad2.right_bumper) {
                MAX_POS += 1;
            }
            if (gamepad2.left_bumper) {
                MAX_POS -= 1;
            }

            if (gamepad2.dpad_left) {
                shoulderPower -= 0.01;
            }
            if (gamepad2.dpad_up) {
                MAX_POS += 1;
            }
            if (gamepad2.dpad_right) {
                shoulderPower += -0.01;
            }
            if (gamepad2.dpad_down) {
                MAX_POS -= 1;
            }

            position = Range.scale(gamepad2.left_stick_y, -1.0, 1.0, 0, MAX_POS);
            // Display the current value
            telemetry.addData("Target Position", "%5.2f", position);
            telemetry.addData("MotorCurrent ", shoulder.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("ShoulderPosition", shoulder.getCurrentPosition());
            telemetry.addData("ShoulderPower", shoulderPower);
            telemetry.addData("MAX_POS", MAX_POS);
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;
            shoulder.setTargetPosition((int)position);
            shoulder.setPower(shoulderPower);
            shoulder.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(CYCLE_MS);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }
}
