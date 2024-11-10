package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 6;

        c.setStrokeWidth(1);
        c.strokeCircle(t.getX(), t.getY(), ROBOT_RADIUS);

        Transform2d transform2d = new Transform2d(new Translation2d(Math.cos(t.getHeading()), Math.sin(t.getHeading())), new Rotation2d());
        Transform2d halfv = transform2d.times(0.5 * ROBOT_RADIUS);
        Pose2d p1 = t.plus(halfv);
        Pose2d p2 = p1.plus(halfv);
        c.strokeLine(p1.getX(), p1.getY(), p2.getX(), p2.getY());
    }
}