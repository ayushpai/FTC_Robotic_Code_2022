package org.firstinspires.ftc.teamcode.Hardware;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class HardwareMaster {


    //Defining Drive Motors
    public              DcMotor         lF                       = null;
    public              DcMotor         rF                       = null;
    public              DcMotor         lR                       = null;
    public              DcMotor         rR                       = null;
    
    //Defining Subsystem Motors
    public              DcMotor         duckMotor                = null;
    public              DcMotor         liftMotor                = null;
    public              DcMotor         intakeMotor              = null;
    public              DcMotor         rackAndPinionMotor       = null;

    public              Servo           clawServo                = null;
    public              Servo           leftCatEar               = null;
    public              Servo           rightCatEar              = null;
    public              Servo           verticalTurret           = null;
    public              Servo           horizontalTurret         = null;
    public              Servo           tapeMeasure              = null;


    //Defining Gyros
    public              BNO055IMU           gyro0;
    public              BNO055IMU           gyro1;
    
    //Miscellaneous Hardware Definitions
    public              HardwareMap         hwMap             = null;
    private static      HardwareMaster      robot             = null;
    public              WebcamName          webcam            = null;

    public ColorRangeSensor colorSensor       = null;
    public ColorRangeSensor bottomColorSensor = null;

    public DigitalChannel magnet = null;
    public DigitalChannel LED = null;




    //Create Instance To Be Referred By Other Classes
    public static HardwareMaster getInstance() {
        if(robot == null) {
            robot = new HardwareMaster();
        }
        return robot;
    }

    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;


        try{ //Left Front Wheel
            lF = hwMap.get(DcMotor.class, "lF");
            lF.setDirection(DcMotorSimple.Direction.REVERSE);
            lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lF.setPower(0);
            lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            lF = null;
        }

        try{ //Right Front Wheel
            rF = hwMap.get(DcMotor.class, "rF");
            rF.setDirection(DcMotorSimple.Direction.FORWARD);
            rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rF.setPower(0);
            rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            rF = null;
        }

        try{ // Left Rear Wheel
            lR = hwMap.get(DcMotor.class, "lR");
            lR.setDirection(DcMotorSimple.Direction.REVERSE);
            lR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lR.setPower(0);
            lR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            lR = null;
        }


        try{ // Right Rear Wheel
            rR = hwMap.get(DcMotor.class, "rR");
            rR.setDirection(DcMotorSimple.Direction.FORWARD);
            rR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rR.setPower(0);
            rR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            rR = null;
        }

        try{ // Color Sensor
            colorSensor = hwMap.get(ColorRangeSensor.class, "color");
        }
        catch (Exception p_exception) {
            colorSensor = null;
        }

        try{ // Color Sensor
            bottomColorSensor = hwMap.get(ColorRangeSensor.class, "bColor");
        }
        catch (Exception p_exception) {
            bottomColorSensor = null;
        }

        try{ // Duck Motor
            duckMotor = hwMap.get(DcMotor.class, "duck");
            duckMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            duckMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            duckMotor.setPower(0);
            duckMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            duckMotor = null;
        }

        try{ // Lift Motor
            liftMotor = hwMap.get(DcMotor.class, "lift");
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            liftMotor.setPower(0);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            liftMotor = null;
        }

        try{ // Intake Motor
            intakeMotor = hwMap.get(DcMotor.class, "intake");
            intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setPower(0);
            intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            intakeMotor = null;
        }

        try{ // Rack and Pinion Motor
            rackAndPinionMotor = hwMap.get(DcMotor.class, "rp");
            rackAndPinionMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            rackAndPinionMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rackAndPinionMotor.setPower(0);
            rackAndPinionMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
        catch (Exception p_exception) {
            rackAndPinionMotor = null;
        }

        try{
            clawServo = hwMap.get(Servo.class, "claw");
        }
        catch (Exception p_exception) {
            clawServo = null;
        }

        try{
            leftCatEar = hwMap.get(Servo.class, "lC");
        }
        catch (Exception p_exception) {
            leftCatEar = null;
        }

        try{
            rightCatEar = hwMap.get(Servo.class, "rC");
        }
        catch (Exception p_exception) {
            rightCatEar = null;
        }

        try{
            verticalTurret = hwMap.get(Servo.class, "vT");
        }
        catch (Exception p_exception) {
            verticalTurret = null;
        }

        try{
            horizontalTurret = hwMap.get(Servo.class, "hT");
        }
        catch (Exception p_exception) {
            horizontalTurret = null;
        }

        try{
            tapeMeasure = hwMap.get(Servo.class, "tape");
        }
        catch (Exception p_exception) {
            tapeMeasure = null;
        }

        try{
            magnet = hwMap.get(DigitalChannel.class, "d0");
            magnet.setMode(DigitalChannel.Mode.INPUT);
        }
        catch (Exception p_exception) {
            magnet = null;
        }

        try{
            LED = hwMap.get(DigitalChannel.class, "led");
            LED.setMode(DigitalChannel.Mode.OUTPUT);
        }
        catch (Exception p_exception) {
            LED = null;
        }
        


        try { // Gyro 0
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro0 = hwMap.get(BNO055IMU.class, "g0");
            gyro0.initialize(parameters);
        }
        catch (Exception p_exception) {
            gyro0 = null;
        }

        try { // Gyro 1
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            gyro1 = hwMap.get(BNO055IMU.class, "g1");
            gyro1.initialize(parameters);
        }
        catch (Exception p_exception) {
            gyro1 = null;
        }





    }

    //Teleop Drive Method


    public void setDrivePower(double leftBackDrivePower, double leftFrontDrivePower, double rightBackDrivePower, double rightFrontDrivePower) { // Send power to wheels
        if (lR != null) {
            leftBackDrivePower = Range.clip(leftBackDrivePower, -1, 1);
            lR.setPower(leftBackDrivePower);


        }
        if (lF != null) {
            leftFrontDrivePower = Range.clip(leftFrontDrivePower, -1, 1);
            lF.setPower(leftFrontDrivePower);
        }
        if (rR != null) {
            rightBackDrivePower = Range.clip(rightBackDrivePower, -1, 1);
            rR.setPower(rightBackDrivePower);
        }
        if (rF != null) {
            rightFrontDrivePower = Range.clip(rightFrontDrivePower, -1, 1);
            rF.setPower(rightFrontDrivePower);
        }
    }


}