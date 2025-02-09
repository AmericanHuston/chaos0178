package org.firstinspires.ftc.teamcode.VarsAndBoards.Utils;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import kotlin.collections.ArrayDeque;

public class PoseComposer {

    List<String[]> things;
    DataReader Reader = new DataReader();

    public void ReadFromAll(){
        this.things = Reader.read();
    }

    public List<String[]> getThings() {
        return things;
    }

    public String ComposePose(int x, int y, int heading){
        return "Pose("+x+","+y+"," + "Math.toRadians("+heading+"));";
    }

    public List<String> ComposePose(List<String[]> listOfStrings){
        List<String> newStrings = new ArrayList<>();
        int size = listOfStrings.size();
        for (int i = 0; i < size; i++){
            String x, y, heading;
            x = Reader.read(i,0);
            y = Reader.read(i, 1);
            heading = Reader.read(i,2);
            newStrings.add(ComposePose(Integer.parseInt(x),Integer.parseInt(y),Integer.parseInt(heading)));
        }
        return newStrings;
    }
}
