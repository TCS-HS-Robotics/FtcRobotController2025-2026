package org.firstinspires.ftc.teamcode.UtilClasses.AprilTags;

public class AprilTag {

    public int id;
    String family;

    public AprilTag(int id, String family) {
        this.id = id;
        this.family = family;
    }

    @Override
    public String toString() {
        String output = "";
        output += "ID: " + id;
        output += "\nFamily: " + family;
        return output;
    }
}
