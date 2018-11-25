//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by Fernflower decompiler)
//

package org.firstinspires.ftc.teamcode.control;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.util.ArrayList;

public class RobotLiveData {
    public final int RIN;
    ArrayList<String> dataNames;
    ArrayList<String> data;
    ArrayList<String> fileNames;
    ArrayList<File> files;
    FileInputStream live = null;

    public RobotLiveData(int roundNumber) {
        this.RIN = roundNumber;
        this.dataNames = new ArrayList();
        this.data = new ArrayList();
        this.fileNames = new ArrayList();
        this.files = new ArrayList();
    }

    void reset() {
        this.dataNames = new ArrayList();
        this.data = new ArrayList();
        this.fileNames = new ArrayList();
        this.files = new ArrayList();
    }

    public void addStringData(String name, String str) {
        this.dataNames.add(name);
        this.data.add("T:" + str);
    }

    public void addStringData(String name, double d) {
        this.dataNames.add(name);
        this.data.add("T:" + d);
    }

    public void addChartData(String name, ArrayList<String> xCoords, ArrayList<String> yCoords) {
        if (xCoords.size() == yCoords.size()) {
            this.dataNames.add(name);
            String dataLine = "C:";
            int size = xCoords.size();

            for(int i = 0; i < size; ++i) {
                dataLine = dataLine + (String)xCoords.get(i) + "-" + (String)yCoords.get(i);
                if (i + 1 < size) {
                    dataLine = dataLine + "~";
                }
            }

            this.data.add(dataLine);
        }

    }

    public void addFile(String name, File f) {
        this.fileNames.add(name);
        this.files.add(f);
    }

    public void addLiveImage(File f) {
        try {
            this.live = new FileInputStream(f);
        } catch (FileNotFoundException var3) {
            var3.printStackTrace();
        }

    }

    public void addLiveImage(FileInputStream f) {
        this.live = f;
    }
}
