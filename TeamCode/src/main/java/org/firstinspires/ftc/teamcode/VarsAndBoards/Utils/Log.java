package org.firstinspires.ftc.teamcode.VarsAndBoards.Utils;

import android.os.Environment;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;

import kotlin.Deprecated;
import kotlin.ReplaceWith;

/**
 * Created by Gabriel on 2018-01-03.
 * A simple way to log data to a file.
 * HOMAR-FTC-Library
 * Almost all code is original, aside from some small changes
 * <a href="https://github.com/StPaulAcademy/HOMAR-FTC-Library/tree/master">Credit</a>
 */
@Deprecated(message = "Now use DataLogger.java")
public class Log {
    private static final String BASE_FOLDER_NAME = "FIRST";
    private Writer fileWriter;
    private String line;
    private boolean logTime;
    private long startTime;
    private boolean disabled = false;
    public Log(String filename, boolean logTime) {
        if (logTime) startTime = System.nanoTime();
        this.logTime = logTime;
        String directoryPath = Environment.getExternalStorageDirectory().getPath()+"/"+BASE_FOLDER_NAME;
        File directory = new File(directoryPath);
        //noinspection ResultOfMethodCallIgnored
        directory.mkdir();
        try {
            fileWriter = new FileWriter(directoryPath+"/"+filename+".csv");
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public boolean isDisabled() {
        return disabled;
    }

    public void setDisabled(boolean disabled) {
        this.disabled = disabled;
    }

    public void close() {
        try {
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void update() {
        if (disabled) return;
        try {
            if (logTime) {
                long timeDifference = System.nanoTime()-startTime;
                line = timeDifference/1E9+","+line;
            }
            fileWriter.write(line+"\n");
            line = "";
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addData(String data) {
        if (disabled) return;
        if (!line.isEmpty()) line += ",";
        line += data;
    }
    public void addData(Object data) {
        addData(data.toString());
    }
    public void addData(boolean data) {
        addData(String.valueOf(data));
    }
    public void addData(byte data) {
        addData(String.valueOf(data));
    }
    public void addData(char data) {
        addData(String.valueOf(data));
    }
    public void addData(short data) {
        addData(String.valueOf(data));
    }
    public void addData(int data) {
        addData(String.valueOf(data));
    }
    public void addData(long data) {
        addData(String.valueOf(data));
    }
    public void addData(float data) {
        addData(String.valueOf(data));
    }
    public void addData(double data) {
        addData(String.valueOf(data));
    }
}