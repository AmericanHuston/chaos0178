package org.firstinspires.ftc.teamcode.VarsAndBoards.Utils;

import android.os.Environment;
import java.io.*;
import java.util.*;
import com.opencsv.*;

public class DataLogger {
    List<String[]> allData = new ArrayList<>();
    List<String[]> readData;
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

    /**
     * REMEMBER THAT ARRAYS START AT 0 AAAA
     * @return returns a list of strings
     */
    public List<String[]> read(){
        return read(Environment.getExternalStorageDirectory().getPath()+"/FIRST/PosLog.csv");
    }
    public List<String[]> read(String file)
    {
        try {
            FileReader filereader = new FileReader(file);
            CSVParser parser = new CSVParserBuilder().withSeparator(';').build();
            CSVReader csvReader = new CSVReaderBuilder(filereader)
                    .withCSVParser(parser)
                    .build();
            readData = csvReader.readAll();
            return readData;
        }
        catch (Exception e) {
            e.printStackTrace();
        }
        return readData;
    }

    /**
     * ARRAYS START AT 0
     * @param line which line to look on - start at 0
     * @param cell which cell to look on - start at 0
     * @return returns a single string
     */
    public String read(int line, int cell){
        String[] lineData = read().get(line); //On this line
        return lineData[cell]; //At this position
    }
}