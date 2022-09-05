package org.firstinspires.ftc.teamcode.constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {



    public static double kP = 0.02; // "P" PID value for strafe direction PID
    public static double kI = 0.00; // "I" PID value for strafe direction PID
    public static double kD = 0.00; // "D" PID value for strafe direction PID

    public static double autoRotatePotentialConstant = 0.018; // potential value for rotate method in autonomous

    public static double clawClose = 0.95;
    public static double clawOpen = 0.3;

    public static double intakeVelocity = 2000;

    public static double leftCatDown = 1;
    public static double leftCatUp = 0.3;

    public static double rightCatDown = 0;
    public static double rightCatUp = 0.7;

    public static int liftLevel1 = 0;
    public static int liftLevel2 = 920;
    public static int liftLevel3 = 1850;
    public static int liftLevel4 = 0;

    public static int rackRed = -1450;
    public static int rackBlue = 1450;

    public static int r1XBlue = 17;
    public static int r1YBlue = 175;
    public static int r2XBlue = 180;
    public static int r2YBlue = 170;
    public static int r3XBlue = 270;
    public static int r3YBlue = 175;
    public static int blueVisionThreshold = 120;

    public static int r1XRed = 4;
    public static int r1YRed = 100;
    public static int r2XRed = 120;
    public static int r2YRed = 125;
    public static int r3XRed = 270;
    public static int r3YRed = 150;
    public static int redVisionThreshold = 120;

    public static double liftSpeed = 1;
    public static double intakeSpeed = 1;
    public static double rackSpeed = 1;







}
