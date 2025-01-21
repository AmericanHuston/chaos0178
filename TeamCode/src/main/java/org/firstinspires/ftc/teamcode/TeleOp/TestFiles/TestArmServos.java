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
@Disabled
@Config
@TeleOp(name = "Test: Arm Servos", group = "Test")
public class TestArmServos extends LinearOpMode {

    static final double INCREMENT   = 0.1;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   100;     // period of each cycle
    public static double MAX_POS     =  1.0;     // Maximum rotational position
    public static double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo wrist;
    Servo claw;
    public static double  wrist_position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    public static double claw_position = (MAX_POS - MIN_POS) / 2;
    boolean rampUp = true;


    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Connect to servo (Assume Robot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        wrist = hardwareMap.get(Servo.class, "wrist");
        claw = hardwareMap.get(Servo.class, "claw");



        // Wait for the start button
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            wrist_position = gamepad2.left_stick_y;
            claw_position = gamepad2.right_stick_y;

            // Display the current value
            telemetry.addData("Target Wrist Position", "%5.2f", wrist_position);
            telemetry.addData("Target Claw Position", claw_position);
            telemetry.addData("MAX_POS", MAX_POS);
            telemetry.addData( "wrist_position", wrist.getPosition());
            telemetry.addData("claw_position", claw.getPosition());
            telemetry.addData(">", "Press Stop to end test." );
            telemetry.update();

            // Set the servo to the new position and pause;

            wrist.setPosition(wrist_position);
            claw.setPosition(claw_position);
        }
    }
}
