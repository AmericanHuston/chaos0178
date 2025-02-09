package org.firstinspires.ftc.teamcode.VarsAndBoards.Utils;

import android.os.Build;
import android.os.Environment;

import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.StandardOpenOption;
import java.util.List;

public class WriteToTextFile {
    public void writeToFile(String file, List<String> data) throws IOException {
        data.add("-----------------------------------");
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            Files.write(Paths.get(file), data, StandardCharsets.UTF_8, StandardOpenOption.APPEND);
        }
    }

    public void writeToFile(List<String> data) throws IOException {
        data.add("-----------------------------------");
        Path file = null;
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            file = Paths.get(Environment.getExternalStorageDirectory().getPath()+"/FIRST/defaultFileToWrite.txt");
        }
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            Files.write(file, data, StandardCharsets.UTF_8, StandardOpenOption.APPEND);
        }
    }
}
