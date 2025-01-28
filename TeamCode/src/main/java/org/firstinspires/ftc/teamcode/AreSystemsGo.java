package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.VarsAndBoards.Board1;

@TeleOp(name = "AreSystemsGo", group = "SystemChecks")
public class AreSystemsGo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Board1 board = new Board1();
        board.init(hardwareMap);

        while (opModeIsActive()){
            sleep(1000);
            //Claw
            board.setClawState(Board1.clawPositions.CLAW_OPEN);
            board.DO_ALL();

            sleep(1000);
            //Arm
            board.setArmState(Board1.armStates.COLLECTION);
            board.DO_ALL();

            sleep(1000);
            //Sliders
            board.setArmState(Board1.armStates.ABOVE_BAR);
            board.DO_ALL();
        }
    }
}
