package org.firstinspires.ftc.teamcode.VarsAndBoards.Utils;

import android.os.Environment;

import com.opencsv.CSVParser;
import com.opencsv.CSVParserBuilder;
import com.opencsv.CSVReader;
import com.opencsv.CSVReaderBuilder;

import java.io.FileReader;
import java.util.List;

public class DataReader {
    List<String[]> readData;
    /**
     * REMEMBER THAT ARRAYS START AT 0
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
    public String readCell(int line, int cell){
        return read().get(line)[cell]; //On this line, at this position "[]"
    }
    /**
     * @param line The line to read from (starts at 0)
     * @return Returns a String
     */
    public String readLine(int line){
        StringBuilder result = new StringBuilder();
        String[] lineData = read().get(line);
        for (String lineDatum : lineData) {
            result.append(lineDatum);
        }
        return result.toString();
    }
}