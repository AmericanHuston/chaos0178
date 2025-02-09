package org.firstinspires.ftc.teamcode.VarsAndBoards.Utils;

import android.os.Environment;

import java.io.*;
import java.util.*;
import com.opencsv.*;

public class DataLogger {
    List<String[]> allData = new ArrayList<>();
    String constructionString;

    public DataLogger(){
    }

    /**
     * +1 Overload Default Value is FIRST directory
     * Basically always use the default
     **/
    public void update(){
        update(Environment.getExternalStorageDirectory().getPath()+"/FIRST/PosLog.csv");
    }

    /**
     * Overload
     * Use standard update() with no params
     * @param output File to write out to
     */
    public void update(String output)
    {
        File file = new File(output);
        try {
            FileWriter outputFile = new FileWriter(file);

            CSVWriter writer = new CSVWriter(outputFile, ';',
                    CSVWriter.NO_QUOTE_CHARACTER,
                    CSVWriter.DEFAULT_ESCAPE_CHARACTER,
                    CSVWriter.DEFAULT_LINE_END);

            writer.writeAll(allData);

            writer.close();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Use for when you only want to have one piece of information per line
     * @param data The stuff you want to put in
     */
    public void addData(String data) {
        addData(new String[] {data});
    }
    public void addData(String[] data) {
        allData.add(data);
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

    public DataLogger stringBuilder(String data) {
        this.constructionString += data + ';';
        return this;
    }
    public DataLogger stringBuilder(String[] data) {
        stringBuilder(data);
        return this;
    }
    public DataLogger stringBuilder(boolean data) {
        stringBuilder(String.valueOf(data));
        return this;
    }
    public DataLogger stringBuilder(byte data) {
        stringBuilder(String.valueOf(data));
        return this;
    }
    public DataLogger stringBuilder(char data) {
        stringBuilder(String.valueOf(data));
        return this;
    }
    public DataLogger stringBuilder(short data) {
        stringBuilder(String.valueOf(data));
        return this;
    }
    public DataLogger stringBuilder(int data) {
        stringBuilder(String.valueOf(data));
        return this;
    }
    public DataLogger stringBuilder(long data) {
        stringBuilder(String.valueOf(data));
        return this;
    }
    public DataLogger stringBuilder(float data) {
        stringBuilder(String.valueOf(data));
        return this;
    }
    public DataLogger stringBuilder(double data) {
        stringBuilder(String.valueOf(data));
        return this;
    }

    public String buildString(){
        return this.constructionString;
    }
}