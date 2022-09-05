package org.firstinspires.ftc.teamcode.OLD_CODE.OLDAUTOS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMaster;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.TimeUnit;

//@Config
//@Autonomous(name = "Blue Side Auto2", group = "Autonomous")
public class AutoBlue2_OLD extends LinearOpMode {

    public HardwareMaster robot = HardwareMaster.getInstance();
    private volatile boolean doTelemetry = true;
    private double angleOffset = 2;
    ElapsedTime elapsedTime = new ElapsedTime(); // Measure timing
    ExpansionHubMotor shooterMotor;
    ExpansionHubEx expansionHub;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    public enum RingPosition
    {
        FOUR,
        ONE,
        NONE
    }
    // Volatile since accessed by OpMode thread w/o synchronization
    OpenCvCamera webcam;
    enum Direction {left, right}
    PIDCoefficients PID;

    int numberofRingsShot;
    boolean oneRingShot = false;
    boolean twoRingsShot = false;
    boolean threeRingsShot = false;
    double currentTimer;
    double startTimer;


    @Override
    public void runOpMode() throws InterruptedException {


        /*==========================INIT ROBOT FOR AUTONOMOUS==========================*/
        robot.init(hardwareMap);
        resetEncoders();

        //expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");

        //PID = shooterMotor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
       /* PID.p = RobotConstants.p;
        PID.i = RobotConstants.i;
        PID.d = RobotConstants.d;

        */


       /* webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"));
        pipeline = new SkystoneDeterminationPipeline();
        webcam.setPipeline(pipeline);

        */

/*
        webcam.openCameraDevice();
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                CameraStreamSource cameraStreamSource = webcam;
                webcam.openCameraDevice();
                FtcDashboard.getInstance().startCameraStream(cameraStreamSource, 60);
            }
        });

 */







        waitForStart(); //Waits for the human to press the "play" button
        /*==========================START OF AUTONOMOUS PROGRAM==========================*/
        Thread update = new Thread() {
            @Override
            public synchronized void run() {
                while (opModeIsActive()) {
                    try {
                        String tmy = "Motors" + "\n";
                        tmy += "    lF: " + robot.lF.getCurrentPosition() + "\n";
                        tmy += "    rF: " + robot.rF.getCurrentPosition() + "\n";
                        tmy += "    lR: " + robot.lR.getCurrentPosition() + "\n";
                        tmy += "    rR: " + robot.rR.getCurrentPosition() + "\n";
                        telemetry.addData("", tmy);

                        dashboardTelemetry.update();
                    } catch (Exception p_exception) {
                        telemetry.addData("Uh oh", p_exception);
                    }
                    telemetry.update();
                }
            }
        };
        update.start();

        //strafeDirection(0, 0.5, 1000, true);


        encoderTest(-51, 64, 60, -42, 0.4, 2.5, true);
        duck(true);
        wait(4000);
        resetEncoders();
        encoderTest(238, -228, -169, 170, 0.3, 2.5, true);
        resetEncoders();
        encoderTest(-246, -287, -260, -276, 0.6, 2.5, true);
        resetEncoders();
        encoderTest(786, -715, 614, -585, 0.6, 2.5, true);
        resetEncoders();
        encoderTest(2052, 2032, 2034, 2086, 0.75, 2.5, true);
        resetEncoders();
        encoderTest(182, -507, -471, 135, 0.6, 2.5, true);
        resetEncoders();
        encoderTest(1552, 1532, 1534, 1586, 13, 3, true);



    }

    /*==========================CUSTOM METHODS==========================*/



    public void moveInches(int inches, double speed){

        int numberOfTicks = inches * 35;
        strafeDirection(0, speed, numberOfTicks, true);


    }

    public void duck(boolean on){
        if(on) {
            robot.duckMotor.setPower(0.35);

        }
        else{
            robot.duckMotor.setPower(0);
        }
    }


    private void wait(int milliseconds){
        try { //adjust
            Thread.sleep(milliseconds);
        }
        catch (Exception p_exception) {
        }
    }
    static double[] currentAngle = new double[2];

    /**
     * Allows the robot to strafe at any angle
     * @param imuAngle the imu angle that you want the robot to travel at
     * @param power the max power you want to give the motors
     * @param tick the average number of ticks you want to transpire over the course of the strafe per motor
     * @param zeroPowerOn if on sets motor power to zero when strafe is complete
     */
    public void strafeDirection(double imuAngle, double power, int tick, boolean zeroPowerOn) {
        resetEncoders();
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startAngle = getAngle();

        //resetEncoders();

        imuAngle += 90;
        //k1 = FLBR, k2 = FRBL
        double k1 = 0, k2 = 0;
        while (imuAngle < 0 && opModeIsActive()) imuAngle += 360;
        imuAngle = imuAngle % 360;

        if (imuAngle > 0 && imuAngle < 90) {
            k1 = (Math.tan(Math.toRadians(imuAngle)) - 1) / (1 + Math.tan(Math.toRadians(imuAngle)));
            k2 = 1;
        } else if (imuAngle > 90 && imuAngle < 180) {
            k1 = 1;
            k2 = (1 + Math.tan(Math.toRadians(imuAngle))) / (Math.tan(Math.toRadians(imuAngle)) - 1);
        } else if (imuAngle > 180 && imuAngle < 270) {
            k1 = -(Math.tan(Math.toRadians(imuAngle)) - 1) / (1 + Math.tan(Math.toRadians(imuAngle)));
            k2 = -1;
        } else if (imuAngle > 270 && imuAngle < 360) {
            k1 = -1;
            k2 = -(1 + Math.tan(Math.toRadians(imuAngle))) / (Math.tan(Math.toRadians(imuAngle)) - 1);
        } else if (imuAngle == 0) {
            k1 = -1;
            k2 = 1;
        } else if (imuAngle == 90) {
            k1 = 1;
            k2 = 1;
        } else if (imuAngle == 180) {
            k1 = 1;
            k2 = -1;
        } else if (imuAngle == 270) {
            k1 = -1;
            k2 = -1;
        }

        k1 *= power;
        k2 *= power;

        k1 = Range.clip(k1, -1, 1);
        k2 = Range.clip(k2, -1, 1);

        currentAngle[0] = getAngle();
        currentAngle[1] = getAngle();

        int[] startTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        int[] currentTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        double avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 3.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;

        while (avgDifference < tick && opModeIsActive()) {


            currentTick[0] = robot.lF.getCurrentPosition();
            currentTick[1] = robot.rF.getCurrentPosition();
            currentTick[2] = robot.lR.getCurrentPosition();
            currentTick[3] = robot.rR.getCurrentPosition();


            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.072;

            currentAngle[0] = getAngle();
            startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            k1 = Range.clip(k1, -1, 1);
            k2 = Range.clip(k2, -1, 1);

            //LF LB RB RF
            robot.setDrivePower(Range.clip(k2 - stabilizeCoefficient, -1, 1), Range.clip(k1 - stabilizeCoefficient, -1, 1), Range.clip(k1 + stabilizeCoefficient, -1, 1), Range.clip(k2 + stabilizeCoefficient, -1, 1));

        }
        if (zeroPowerOn) {
            robot.lF.setPower(0);
            robot.rR.setPower(0);
            robot.rF.setPower(0);
            robot.lR.setPower(0);
        }
    }

    public void strafeDirection(double imuAngle, double power, int tick, boolean zeroPowerOn, String armPosition) {
        resetEncoders();
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startAngle = getAngle();

        //resetEncoders();

        imuAngle += 90;
        //k1 = FLBR, k2 = FRBL
        double k1 = 0, k2 = 0;
        while (imuAngle < 0 && opModeIsActive()) imuAngle += 360;
        imuAngle = imuAngle % 360;

        if (imuAngle > 0 && imuAngle < 90) {
            k1 = (Math.tan(Math.toRadians(imuAngle)) - 1) / (1 + Math.tan(Math.toRadians(imuAngle)));
            k2 = 1;
        } else if (imuAngle > 90 && imuAngle < 180) {
            k1 = 1;
            k2 = (1 + Math.tan(Math.toRadians(imuAngle))) / (Math.tan(Math.toRadians(imuAngle)) - 1);
        } else if (imuAngle > 180 && imuAngle < 270) {
            k1 = -(Math.tan(Math.toRadians(imuAngle)) - 1) / (1 + Math.tan(Math.toRadians(imuAngle)));
            k2 = -1;
        } else if (imuAngle > 270 && imuAngle < 360) {
            k1 = -1;
            k2 = -(1 + Math.tan(Math.toRadians(imuAngle))) / (Math.tan(Math.toRadians(imuAngle)) - 1);
        } else if (imuAngle == 0) {
            k1 = -1;
            k2 = 1;
        } else if (imuAngle == 90) {
            k1 = 1;
            k2 = 1;
        } else if (imuAngle == 180) {
            k1 = 1;
            k2 = -1;
        } else if (imuAngle == 270) {
            k1 = -1;
            k2 = -1;
        }

        k1 *= power;
        k2 *= power;

        k1 = Range.clip(k1, -1, 1);
        k2 = Range.clip(k2, -1, 1);

        currentAngle[0] = getAngle();
        currentAngle[1] = getAngle();

        int[] startTick = {robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lF.getCurrentPosition()};
        int[] currentTick = {robot.lF.getCurrentPosition(), robot.rR.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lF.getCurrentPosition()};
        double avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;

        while (avgDifference < tick && opModeIsActive()) {


            currentTick[0] = robot.lF.getCurrentPosition();
            currentTick[1] = robot.rR.getCurrentPosition();
            currentTick[2] = robot.rF.getCurrentPosition();
            currentTick[3] = robot.lR.getCurrentPosition();

            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[2] - startTick[2])) / 4.0;

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.072;

            currentAngle[0] = getAngle();
            startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            k1 = Range.clip(k1, -1, 1);
            k2 = Range.clip(k2, -1, 1);

            //LF LB RB RF
            robot.setDrivePower(Range.clip(k2 - stabilizeCoefficient, -1, 1), Range.clip(k1 - stabilizeCoefficient, -1, 1), Range.clip(k1 + stabilizeCoefficient, -1, 1), Range.clip(k2 + stabilizeCoefficient, -1, 1));

        }
        if (zeroPowerOn) {
            robot.lF.setPower(0);
            robot.rR.setPower(0);
            robot.rF.setPower(0);
            robot.lR.setPower(0);
        }
    }

    public void strafeDirection_PID(double imuAngle, double power, int tick, boolean zeroPowerOn) {
        resetEncoders();
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startAngle = getAngle();

        //resetEncoders();

        imuAngle += 90;
        //k1 = FLBR, k2 = FRBL
        double k1 = 0, k2 = 0;
        while (imuAngle < 0 && opModeIsActive()) imuAngle += 360;
        imuAngle = imuAngle % 360;

        if (imuAngle > 0 && imuAngle < 90) {
            k1 = (Math.tan(Math.toRadians(imuAngle)) - 1) / (1 + Math.tan(Math.toRadians(imuAngle)));
            k2 = 1;
        } else if (imuAngle > 90 && imuAngle < 180) {
            k1 = 1;
            k2 = (1 + Math.tan(Math.toRadians(imuAngle))) / (Math.tan(Math.toRadians(imuAngle)) - 1);
        } else if (imuAngle > 180 && imuAngle < 270) {
            k1 = -(Math.tan(Math.toRadians(imuAngle)) - 1) / (1 + Math.tan(Math.toRadians(imuAngle)));
            k2 = -1;
        } else if (imuAngle > 270 && imuAngle < 360) {
            k1 = -1;
            k2 = -(1 + Math.tan(Math.toRadians(imuAngle))) / (Math.tan(Math.toRadians(imuAngle)) - 1);
        } else if (imuAngle == 0) {
            k1 = -1;
            k2 = 1;
        } else if (imuAngle == 90) {
            k1 = 1;
            k2 = 1;
        } else if (imuAngle == 180) {
            k1 = 1;
            k2 = -1;
        } else if (imuAngle == 270) {
            k1 = -1;
            k2 = -1;
        }

        k1 *= power;
        k2 *= power;

        k1 = Range.clip(k1, -1, 1);
        k2 = Range.clip(k2, -1, 1);

        currentAngle[0] = getAngle();
        currentAngle[1] = getAngle();

        int[] startTick = {robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lF.getCurrentPosition()};
        int[] currentTick = {robot.lF.getCurrentPosition(), robot.rR.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lF.getCurrentPosition()};
        double avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;

        double[] averageDifferences = {0, avgDifference};
        while (avgDifference < tick && opModeIsActive()) {


            currentTick[0] = robot.lF.getCurrentPosition();
            currentTick[1] = robot.rR.getCurrentPosition();
            currentTick[2] = robot.rF.getCurrentPosition();
            currentTick[3] = robot.lR.getCurrentPosition();

            averageDifferences[1] = avgDifference;
            endTime = System.nanoTime() / 1000000000.0;
            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[2] - startTick[2])) / 4.0;
            endTime = System.nanoTime() / 1000000000.0;
            averageDifferences[0] = avgDifference;


            //PID stuff
            double potential = tick - avgDifference;
            double derivative = (averageDifferences[1] - averageDifferences[0]) / (endTime - startTime);
            double integral = (tick - avgDifference) * (endTime - startTime);

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.072;

            currentAngle[0] = getAngle();
            startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            k1 = Range.clip(k1, -1, 1);
            k2 = Range.clip(k2, -1, 1);

            //LF LB RB RF
            robot.setDrivePower(Range.clip((potential * RobotConstants.kP + derivative * RobotConstants.kD + integral * RobotConstants.kI) * (k2 - stabilizeCoefficient), -1, 1), Range.clip((potential * RobotConstants.kP + derivative * RobotConstants.kD + integral * RobotConstants.kI) * (k1 - stabilizeCoefficient), -1, 1), Range.clip((potential * RobotConstants.kP + derivative * RobotConstants.kD + integral * RobotConstants.kI) * (k1 + stabilizeCoefficient), -1, 1), Range.clip((potential * RobotConstants.kP + derivative * RobotConstants.kD + integral * RobotConstants.kI) * (k2 + stabilizeCoefficient), -1, 1));
        }
        if (zeroPowerOn) {
            robot.lF.setPower(0);
            robot.rR.setPower(0);
            robot.rF.setPower(0);
            robot.lR.setPower(0);
        }
    }



    public void rotate(double rotateAngle, double power, double timeLimit) {

        ElapsedTime timer = new ElapsedTime();

        double[] time = new double[2];
        int[] heading = new int[2];

        time[0] = 0;
        heading[0] = 0;

        rotateAngle += 180; //adjust for Ayush's arguments

        while (rotateAngle < 0 && opModeIsActive()) rotateAngle += 360;

        double rotatePower;

        //Timer used as time intervals to calculate Angular Velocity of the Robot
        ElapsedTime angularVelocityChecker = new ElapsedTime();
        angularVelocityChecker.reset();

        //Timer to confirm robot has been at the correct position for a period of time to make sure robot doesn't overshoot angle.
        ElapsedTime completionCheck = new ElapsedTime();

        timer.reset();
        double stallCorrection = 1, angularVelocity;
        boolean complete = false;
        double startTime = elapsedTime.time();
        while ((!complete) && (elapsedTime.time() < startTime + timeLimit) && opModeIsActive()) {

            if (angularVelocityChecker.time() > 0.075) {
                time[1] = time[0];
                heading[1] = heading[0];
                time[0] = System.currentTimeMillis() / 1000.0;
                heading[0] = (int) (getAngle() + 180);

                angularVelocity = (heading[0] - heading[1]) / (time[0] - time[1]);
                telemetry.addData("Anglular Velocity", angularVelocity);
                angularVelocityChecker.reset();

                if (angularVelocity == 0) stallCorrection *= 1.085;
            }

            rotatePower = power * Range.clip((Math.abs(rotateAngle - (getAngle() + 180))) * (RobotConstants.autoRotatePotentialConstant/power) * stallCorrection, -1, 1);

            double lF = Range.clip(rotatePower, -1, 1);
            double rR = Range.clip(rotatePower, -1, 1);
            double rF = Range.clip(rotatePower, -1, 1);
            double lR = Range.clip(rotatePower, -1, 1);

            //Telemetry
            telemetry.addData("Desired Angle:", rotateAngle);
            telemetry.addData("Current Angle:", (getAngle() + 180));
            telemetry.addData("Stall Correction: ", stallCorrection);
            telemetry.addData("Calculated Rotate Power", (Math.abs(rotateAngle - (getAngle() + 180))) * (RobotConstants.autoRotatePotentialConstant/power)  * stallCorrection);
            telemetry.addData("Actual Rotate Power", rotatePower);
            telemetry.update();

            if (((getAngle() + 180) < rotateAngle || (getAngle() + 180) >= ((rotateAngle + 180) % 360)) && !((getAngle() + 180) > rotateAngle || (getAngle() + 180) < ((rotateAngle - 180) % 360))) {
                robot.lF.setPower(lF);
                robot.rR.setPower(-rR);
                robot.rF.setPower(-rF);
                robot.lR.setPower(lR);
            } else {

                robot.lF.setPower(-lF);
                robot.rR.setPower(rR);
                robot.rF.setPower(rF);
                robot.lR.setPower(-lR);
            }

            double error = 1.5;

            //makes sure bot is on desired angle and not zooming past it
            if ((getAngle()+ 180) >= rotateAngle - error && (getAngle()+ 180) <= rotateAngle + error) {
                if (completionCheck.time() > 0.5) complete = true;
            } else {
                completionCheck.reset();
            }
        }
    }

    public void rotate(double rotateAngle, double autoRotatePotentialConstant, double power, double timeLimit) {

        ElapsedTime timer = new ElapsedTime();

        double[] time = new double[2];
        int[] heading = new int[2];

        time[0] = 0;
        heading[0] = 0;

        rotateAngle += 180; //adjust for Ayush's arguments

        while (rotateAngle < 0 && opModeIsActive()) rotateAngle += 360;

        double rotatePower;

        //Timer used as time intervals to calculate Angular Velocity of the Robot
        ElapsedTime angularVelocityChecker = new ElapsedTime();
        angularVelocityChecker.reset();

        //Timer to confirm robot has been at the correct position for a period of time to make sure robot doesn't overshoot angle.
        ElapsedTime completionCheck = new ElapsedTime();

        timer.reset();
        double stallCorrection = 1, angularVelocity;
        boolean complete = false;
        double startTime = elapsedTime.time();
        while ((!complete) && (elapsedTime.time() < startTime + timeLimit) && opModeIsActive()) {

            if (angularVelocityChecker.time() > 0.02) {
                time[1] = time[0];
                heading[1] = heading[0];
                time[0] = System.currentTimeMillis() / 1000.0;
                heading[0] = (int) (getAngle() + 180);

                angularVelocity = (heading[0] - heading[1]) / (time[0] - time[1]);
                telemetry.addData("Anglular Velocity", angularVelocity);
                angularVelocityChecker.reset();

                if (angularVelocity == 0) stallCorrection *= 1.085;
            }

            rotatePower = power * Range.clip((Math.abs(rotateAngle - (getAngle() + 180))) * (autoRotatePotentialConstant/power) * stallCorrection, -1, 1);

            double lF = Range.clip(rotatePower, -1, 1);
            double rR = Range.clip(rotatePower, -1, 1);
            double rF = Range.clip(rotatePower, -1, 1);
            double lR = Range.clip(rotatePower, -1, 1);

            //Telemetry
            telemetry.addData("Desired Angle:", rotateAngle);
            telemetry.addData("Current Angle:", (getAngle() + 180));
            telemetry.addData("Stall Correction: ", stallCorrection);
            telemetry.addData("Calculated Rotate Power", (Math.abs(rotateAngle - (getAngle() + 180))) * (autoRotatePotentialConstant/power)  * stallCorrection);
            telemetry.addData("Actual Rotate Power", rotatePower);
            telemetry.update();

            if (((getAngle() + 180) < rotateAngle || (getAngle() + 180) >= ((rotateAngle + 180) % 360)) && !((getAngle() + 180) > rotateAngle || (getAngle() + 180) < ((rotateAngle - 180) % 360))) {
                robot.lF.setPower(lF);
                robot.rR.setPower(-rR);
                robot.rF.setPower(-rF);
                robot.lR.setPower(lR);
            } else {

                robot.lF.setPower(-lF);
                robot.rR.setPower(rR);
                robot.rF.setPower(rF);
                robot.lR.setPower(-lR);
            }

            double error = 1.5;

            //makes sure bot is on desired angle and not zooming past it
            if ((getAngle()+ 180) >= rotateAngle - error && (getAngle()+ 180) <= rotateAngle + error) {
                if (completionCheck.time() > 0.5) complete = true;
            } else {
                completionCheck.reset();
            }
        }
    }

    private double getAngle() {

        double robotAngle;
        Orientation g0angles = null;
        Orientation g1angles = null;
        if (robot.gyro0 != null) {
            g0angles = robot.gyro0.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Get z axis angle rFom first gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (robot.gyro1 != null) {
            g1angles = robot.gyro1.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES); // Get z axis angle rFom second gyro (in radians so that a conversion is unnecessary for proper employment of Java's Math class)
        }
        if (g0angles != null && g1angles != null) {
            robotAngle = ((g0angles.firstAngle + g1angles.firstAngle) / 2); // Average angle measures to determine actual robot angle
        } else if (g0angles != null) {
            robotAngle = g0angles.firstAngle;
        } else if (g1angles != null) {
            robotAngle = g1angles.firstAngle;
        } else {
            robotAngle = 0;
        }
        return robotAngle;
    }

    private void turn(double angle, double time) {
        if(Math.abs(angle) < 0.1 || Math.abs(angle) + 0.1 % 360 < 0.2) { // Detects if turn is too insignificant
            return;
        }
        angle += (angleOffset * Math.abs(angle) / angle);
        double oldAngle;
        double angleIntended;
        double robotAngle;
        double lastError;
        double error = 0;
        ElapsedTime turnTime = new ElapsedTime();
        robot.rR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robotAngle = getAngle();
        oldAngle = robotAngle;
        angleIntended = robotAngle + angle;
        if (angleIntended < robotAngle) { // Left turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                if(oldAngle > 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    lastError = error;
                    error = Math.abs(robotAngle - angleIntended);
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(Math.min(-0.0075 * error, -0.1), Math.min(-0.0075 * error, -0.1), Math.max(0.0075 * error, 0.1), Math.max(0.0075 * error, 0.1));
                robotAngle = getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }
        else if(opModeIsActive() && angleIntended > robotAngle) { // Right turn
            if(angleIntended > 180) {
                angleIntended -= 360;
            }
            else if(angleIntended < -180) {
                angleIntended += 360;
            }
            while(opModeIsActive() && !(angleIntended - angleOffset  < robotAngle && angleIntended + angleOffset > robotAngle) && turnTime.time() < time) {
                if(oldAngle < 0 || (Math.abs(angleIntended) == angleIntended && Math.abs(robotAngle) == robotAngle) || (Math.abs(angleIntended) != angleIntended && Math.abs(robotAngle) != robotAngle)) {
                    error = Math.abs(robotAngle - angleIntended);
                    if(error > 180) {
                        lastError = error;
                        error = Math.abs(robotAngle + angleIntended);
                        if(lastError != 0 && error > lastError) {
                            error = lastError;
                        }
                    }
                }
                else {
                    lastError = error;
                    error = Math.abs(robotAngle - (angleIntended + (360 * -(Math.abs(angleIntended) / angleIntended))));
                    if(lastError != 0 && error > lastError) {
                        error = lastError;
                    }
                }
                robot.setDrivePower(Math.max(0.75 * error, 0.1), Math.max(0.75 * error, 0.1), Math.min(-0.75 * error, -0.1), Math.min(-0.75 * error, -0.1));
                robotAngle = getAngle();
            }
            robot.setDrivePower(0, 0, 0, 0);
        }

    }
    private void encoderTest(int lF, int rF, int lR, int rR, double speed, double timeLimit, boolean motorZero) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (opModeIsActive() && robot.rR != null && robot.rF != null && robot.lR != null && robot.lF != null ) {
            robot.rR.setTargetPosition(rR);
            robot.rF.setTargetPosition(rF);
            robot.lF.setTargetPosition(lF);
            robot.lR.setTargetPosition(lR);

            robot.rR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double lFhalf = robot.lF.getTargetPosition()/1.2;


            try {

                while (Math.abs(robot.lF.getCurrentPosition()) < Math.abs(lFhalf)){
                    robot.setDrivePower(-(lR / Math.abs(lR)) * speed, -(lF / Math.abs(lF)) * speed, -(rR / Math.abs(rR)) * speed, -(rF / Math.abs(rF)) * speed);
                }


            }
            catch(Exception p_exception) {
                robot.setDrivePower(speed, speed, speed, speed);
            }
            ElapsedTime limitTest = new ElapsedTime();
            while ((robot.rR.isBusy() || robot.rF.isBusy() || robot.lR.isBusy() || robot.lF.isBusy()) && opModeIsActive() && limitTest.time() < timeLimit) {}
            if(limitTest.time() > timeLimit) {
                robot.rR.setTargetPosition((robot.rR.getCurrentPosition()));
                robot.rF.setTargetPosition((robot.rF.getCurrentPosition()));
                robot.lR.setTargetPosition((robot.lR.getCurrentPosition()));
                robot.lF.setTargetPosition((robot.lF.getCurrentPosition()));
            }
            if(motorZero) {
                robot.setDrivePower(0, 0, 0, 0);
            }

        }
    }
    private void resetEncoders() {
        robot.rR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


}


