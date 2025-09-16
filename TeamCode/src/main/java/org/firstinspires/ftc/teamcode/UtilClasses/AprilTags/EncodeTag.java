package org.firstinspires.ftc.teamcode.UtilClasses.AprilTags;


import java.util.ArrayList;

public class EncodeTag extends AprilTag {

    ArrayList<Character> code = new ArrayList<Character>();

    public EncodeTag(int id, String family, char[] code) {
        super(id, family);

        for (char c : code) {
            this.code.add(c);
        }
    }

    @Override
    public String toString() {
        String output = "";
        output += super.toString();
        output += "\nEncodeTag: " + code;
        return output;
    }


}
