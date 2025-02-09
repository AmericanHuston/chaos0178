package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot2;
import org.firstinspires.ftc.teamcode.VarsAndBoards.Utils.DataLogger;
import org.firstinspires.ftc.teamcode.VarsAndBoards.Utils.DataReader;
import org.firstinspires.ftc.teamcode.VarsAndBoards.Utils.PoseComposer;
import org.firstinspires.ftc.teamcode.VarsAndBoards.Utils.WriteToTextFile;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

import java.io.IOException;
import java.util.List;

public class MakePathsTele extends OpMode {

    private Follower follower;
    private DataLogger Logger;
    private PoseComposer Composer;
    private DataReader Reader;
    private WriteToTextFile Writer;

    Robot2 robot = new Robot2();

    @Override
    public void init() {
        robot.init(hardwareMap);
        Constants.setConstants(FConstants.class, LConstants.class);
        Logger = new DataLogger();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(robot.getLastPose());
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y/2, -gamepad1.left_stick_x/2, -gamepad1.right_stick_x/2, false);
        follower.update();

        if (gamepad1.a){
            Logger.addData(
                    follower.getPose().getX() + ';' +
                    follower.getPose().getY() + ';' +
                    follower.getPose().getHeading() + ';'
            );
            telemetry.addData("Last Added Pose", follower.getPose().toString());
            Logger.update();
            Logger.addData(
                Logger.stringBuilder(follower.getPose().getX())
                    .stringBuilder(follower.getPose().getY())
                    .stringBuilder(follower.getPose().getHeading())
                    .buildString()
            );
            Logger.update();
            List<String> listOfStrings = Composer.ComposePose(Reader.read());
            try {
                Writer.writeToFile(listOfStrings);
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
