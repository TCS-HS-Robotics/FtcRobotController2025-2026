package org.firstinspires.ftc.teamcode.UtilClasses.AprilTags;

public class ColorTag extends AprilTag {

    String color;

    public ColorTag(int id, String family, String color) {
        super(id, family);

        this.color = color;
    }

    @Override
    public String toString() {
        String output = "";
        output += super.toString();
        output += "\nColorTag: " + color;
        return output;
    }

}
