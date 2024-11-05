package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//isPrime was copied from: https://stackoverflow.com/a/24006293

@TeleOp(name="PrimeCruncher", group="Autonomous")
@Config
public class PrimeCruncher extends LinearOpMode{
    @Override
    public void runOpMode() {
        //Don't edit code below this point
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.updateConfig();
        waitForStart();
        //Don't edit code above this point
        int num = 0;
        int totalPrimesFound = 0;
        while (opModeIsActive()) {

            do {
                num = num + 1;
            } while (!isPrime(num));
            telemetry.addData("Prime ", num);
            totalPrimesFound = totalPrimesFound +1;
            telemetry.addData("Number of primes found ", totalPrimesFound);
            telemetry.update();
        }
    }
  // print the number

    /**
     * Checks to see if the requested value is prime.
     */
    private static boolean isPrime(int inputNum){
        if (inputNum <= 3 || inputNum % 2 == 0)
            return inputNum == 2 || inputNum == 3; //this returns false if number is <=1 & true if number = 2 or 3
        int divisor = 3;
        while ((divisor <= Math.sqrt(inputNum)) && (inputNum % divisor != 0))
            divisor += 2; //iterates through all possible divisors
        return inputNum % divisor != 0; //returns true/false
    }
}