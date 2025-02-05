package org.firstinspires.ftc.teamcode.VarsAndBoards.Utils;

import android.os.Environment;
import java.io.*;
import java.util.*;
import com.opencsv.*;

public class DataLogger {
    static List<String[]> allData = new ArrayList<String[]>();
    static List<String[]> readData;
    /**
     * +1 Overload Default Value is FIRST directory
     **/
    public static void update(){
        update(Environment.getExternalStorageDirectory().getPath()+"/FIRST");
    }

    /**
     * Overload
     * @param output File to write out to
     */
    public static void update(String output)
    {
        File file = new File(output);
        try {
            FileWriter outputFile = new FileWriter(file);

            CSVWriter writer = new CSVWriter(outputFile, ';',
                    CSVWriter.NO_QUOTE_CHARACTER,
                    CSVWriter.DEFAULT_ESCAPE_CHARACTER,
                    CSVWriter.DEFAULT_LINE_END);

            writer.writeAll(allData);

            // closing writer connection
            writer.close();
            allData.clear();
        }
        catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void addData(String data) {
        String[] s = data.split("");
        addData(s);
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

    public List<String[]> read(String file)
    {
        if (!readData.isEmpty()){
            readData.clear();
        }
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
        return null;
    }
    public String read(int line, int cell){
        String[] lineData = readData.get(line); //On this line
        return lineData[cell]; //At this position
    }
}