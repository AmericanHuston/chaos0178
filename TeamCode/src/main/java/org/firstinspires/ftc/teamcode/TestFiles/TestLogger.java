package org.firstinspires.ftc.teamcode.TestFiles;

import static android.os.SystemClock.sleep;

import android.os.Environment;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.VarsAndBoards.Utils.DataLogger;

@Autonomous(name = "TestLogger", group = "Tests")
public class TestLogger extends OpMode {
    @Override
    public void init() {
    }

    @Override
    public void loop() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DataLogger Logger = new DataLogger();
        Logger.addData(new String[] {"HelloWorld!", "What's up?"});
        Logger.update();
        Logger.addData(new String[]{"The second line", "Awe yeah man"});
        Logger.update();
        telemetry.addData("Dir: ", Environment.getExternalStorageDirectory().getPath());
        sleep(1000);
        telemetry.addData("Info", Logger.read(0,0));
        telemetry.update();
        return;
    }
}
