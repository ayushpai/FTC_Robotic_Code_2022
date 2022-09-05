package org.firstinspires.ftc.teamcode.Autonomous_NEW;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMaster;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.TimeUnit;

//@Config
@Autonomous(name = "BLUE CYCLE AUTO", group = "Autonomous")
public class AutoBlue_CYCLE extends LinearOpMode {
    private             HardwareMaster      robot             = HardwareMaster.getInstance();
    private             double              angleOffset       = 2;
    private             ElapsedTime         elapsedTime       = new ElapsedTime();
    private             OpenCvCamera        webcam;
    private             FreightDetermination pipeline;

    private             FtcDashboard        dashboard                       = FtcDashboard.getInstance();
    private             Telemetry           dashboardTelemetry              = dashboard.getTelemetry();

    private ExpansionHubMotor intakeMotor;
    private ExpansionHubEx expansionHub;

    ElapsedTime timer = new ElapsedTime();

    double intakeVoltage = 0.5;

    double endingTick = 0;

    double startingTick = 0;
    static double[] currentAngle = new double[2];

    boolean lineChecker = true;
    double avgDifference;

    double accelerationConstant;
    boolean stopMethod = false;


    @Override
    public void runOpMode() throws InterruptedException {
        /*==========================INIT ROBOT FOR AUTONOMOUS==========================*/
        robot.init(hardwareMap);
        telemetry.addData("Initialization", "STARTING DO NOT TOUCH");
        telemetry.update();
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"); // Initialize EHub
        intakeMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake"); // Initialize Intake
        resetEncoders();
        robot.clawServo.setPosition(RobotConstants.clawClose);
        visionInit();
        wait(2000);
        if(pipeline.position.equals(AutoBlue_CYCLE.FreightDetermination.FreightPosition.LEFT)){
            telemetry.addData("Position: ", "Left");
        }
        else if(pipeline.position.equals(AutoBlue_CYCLE.FreightDetermination.FreightPosition.CENTER)){
            telemetry.addData("Position: ", "Center");
        }
        else if(pipeline.position.equals(AutoBlue_CYCLE.FreightDetermination.FreightPosition.RIGHT)){
            telemetry.addData("Position: ", "Right");
        }
        telemetry.update();
        waitForStart();
        timer.reset();

        /*==========================START OF AUTONOMOUS PROGRAM==========================*/


        if(pipeline.position.equals(FreightDetermination.FreightPosition.LEFT)){
            level1Auto();
        }
        else if(pipeline.position.equals(FreightDetermination.FreightPosition.CENTER)){
            level2Auto();
        }
        else if(pipeline.position.equals(FreightDetermination.FreightPosition.RIGHT)){
            level3Auto();
        }





    }

    private void level1Auto(){
        strafeDirection(114, 0.7 , 950, true, true, false, 1); // go to hub
        clawOpen(true, true);
        wait(200);
        resetLiftExtension(true); // reset the lift and extension
        intake(false, true);
        strafeDirection(294, 0.8, 985, true, false,false, 0); // go back to wall


        /** Cycle 2**/
        strafeDirection(270, 1, 40, false, false,false, 0); // line up w/ wall
        strafeDirection(355, 1, 200,  false, false,false, 0); // go forward fast
        intake(true, false);
        clawOpen(true, false);
        strafeDirection(355, 1, 700,  true, false,false, 0); // go forward fast
        strafeDirection(355, 0.25, 850,  true, false,true,  0); // go forward until intake spikes
        wait(250);
        goToWhiteLine(185);
        goBackToStartingPosition();
        strafeDirection(114, 1 , 1000, true, true, false, 3); // go to hub
        clawOpen(true, false);
        wait(400);
        clawOpen(false, false);
        intake(false, true);
        strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


        /** Cycle 3**/
        strafeDirection(270, 1, 40, false, false,false, 0); // line up w/ wall
        strafeDirection(355, 1, 200,  false, false,false, 0); // go forward fast
        intake(true, false);
        clawOpen(true, false);
        strafeDirection(355, 1, 700,  true, false,false, 0); // go forward fast
        strafeDirection(10, 0.2, 900,  true, false, true, 0); // go forward until intake spikes
        wait(200);
        if(timer.seconds() <= 24) {
            moveLiftLow(180, 250);
            moveLiftLow(250, 200);
            goToWhiteLine(190);
            goBackToStartingPosition();
            strafeDirection(114, 1, 1000, true, true, false, 3); // go to hub
            clawOpen(true, false);
            wait(400);
            clawOpen(false, false);
            intake(false, true);
            strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall

            /**Cycle 4**/
            strafeDirection(270, 1, 40, false, false, false, 0); // line up w/ wall
            strafeDirection(355, 1, 200, false, false, false, 0); // go forward fast
            intake(true, false);
            clawOpen(true, false);
            strafeDirection(355, 1, 700, true, false, false, 0); // go forward fast
            /*strafeDirection(30, 0.8, 900, true, false, true, 0); // go forward until intake spikes
            wait(200);
            if (timer.seconds() <= 24) {
                moveLiftLow(180, 250);
                moveLiftLow(270, 1200);
                goToWhiteLine(190);
                goBackToStartingPosition();
                strafeDirection(114, 1, 1000, true, true, false, 3); // go to hub
                clawOpen(true, false);
                wait(400);
                clawOpen(false, false);
                intake(false, true);
                strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


                //Line up and park
                strafeDirection(270, 1, 40, false, false, false, 0); // line up w/ wall
                intake(true, false);
                strafeDirection(355, 1, 1500, true, false, false, 0); // go forward fast
                clawOpen(true, false);
                strafeDirection(0, 0.6, 200, true, false, false, 0); // go forward until intake spikes

            }*/
        }
    }

    private void level2Auto(){
        strafeDirection(116, 1 , 1000, true, true, false, 2); // go to hub
        wait(500);
        clawOpen(true, false);
        wait(350);
        clawOpen(false, false);
        intake(false, true);
        strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


        /** Cycle 2**/
        strafeDirection(270, 1, 40, false, false,false, 0); // line up w/ wall
        strafeDirection(355, 1, 200,  false, false,false, 0); // go forward fast
        intake(true, false);
        clawOpen(true, false);
        strafeDirection(355, 1, 700,  true, false,false, 0); // go forward fast
        strafeDirection(355, 0.25, 850,  true, false,true,  0); // go forward until intake spikes
        wait(250);
        goToWhiteLine(185);
        goBackToStartingPosition();
        strafeDirection(114, 1 , 1000, true, true, false, 3); // go to hub
        clawOpen(true, false);
        wait(400);
        clawOpen(false, false);
        intake(false, true);
        strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


        /** Cycle 3**/
        strafeDirection(270, 1, 40, false, false,false, 0); // line up w/ wall
        strafeDirection(355, 1, 200,  false, false,false, 0); // go forward fast
        intake(true, false);
        clawOpen(true, false);
        strafeDirection(355, 1, 700,  true, false,false, 0); // go forward fast
        strafeDirection(10, 0.2, 900,  true, false, true, 0); // go forward until intake spikes
        wait(200);
        if(timer.seconds() <= 24) {
            moveLiftLow(180, 250);
            moveLiftLow(250, 200);
            goToWhiteLine(190);
            goBackToStartingPosition();
            strafeDirection(114, 1, 1000, true, true, false, 3); // go to hub
            clawOpen(true, false);
            wait(400);
            clawOpen(false, false);
            intake(false, true);
            strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall

            /**Cycle 4**/
            strafeDirection(270, 1, 40, false, false, false, 0); // line up w/ wall
            strafeDirection(355, 1, 200, false, false, false, 0); // go forward fast
            intake(true, false);
            clawOpen(true, false);
            strafeDirection(355, 1, 700, true, false, false, 0); // go forward fast
            /*strafeDirection(30, 0.8, 900, true, false, true, 0); // go forward until intake spikes
            wait(200);
            if (timer.seconds() <= 24) {
                moveLiftLow(180, 250);
                moveLiftLow(270, 1200);
                goToWhiteLine(190);
                goBackToStartingPosition();
                strafeDirection(114, 1, 1000, true, true, false, 3); // go to hub
                clawOpen(true, false);
                wait(400);
                clawOpen(false, false);
                intake(false, true);
                strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


                //Line up and park
                strafeDirection(270, 1, 40, false, false, false, 0); // line up w/ wall
                intake(true, false);
                strafeDirection(355, 1, 1500, true, false, false, 0); // go forward fast
                clawOpen(true, false);
                strafeDirection(0, 0.6, 200, true, false, false, 0); // go forward until intake spikes

            }*/
        }


    }

    private void level3Auto(){
        strafeDirection(116, 1 , 1000, true, true, false, 3); // go to hub
        wait(500);
        clawOpen(true, false);
        wait(350);
        clawOpen(false, false);
        intake(false, true);
        strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


        /** Cycle 2**/
        strafeDirection(270, 1, 40, false, false,false, 0); // line up w/ wall
        strafeDirection(355, 1, 200,  false, false,false, 0); // go forward fast
        intake(true, false);
        clawOpen(true, false);
        strafeDirection(355, 1, 700,  true, false,false, 0); // go forward fast
        strafeDirection(355, 0.25, 850,  true, false,true,  0); // go forward until intake spikes
        wait(250);
        goToWhiteLine(185);
        goBackToStartingPosition();
        strafeDirection(114, 1 , 1000, true, true, false, 3); // go to hub
        clawOpen(true, false);
        wait(400);
        clawOpen(false, false);
        intake(false, true);
        strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


        /** Cycle 3**/
        strafeDirection(270, 1, 40, false, false,false, 0); // line up w/ wall
        strafeDirection(355, 1, 200,  false, false,false, 0); // go forward fast
        intake(true, false);
        clawOpen(true, false);
        strafeDirection(355, 1, 700,  true, false,false, 0); // go forward fast
        strafeDirection(10, 0.2, 900,  true, false, true, 0); // go forward until intake spikes
        wait(200);
        if(timer.seconds() <= 24) {
            moveLiftLow(180, 250);
            moveLiftLow(250, 200);
            goToWhiteLine(190);
            goBackToStartingPosition();
            strafeDirection(114, 1, 1000, true, true, false, 3); // go to hub
            clawOpen(true, false);
            wait(400);
            clawOpen(false, false);
            intake(false, true);
            strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall

            /**Cycle 4**/
            strafeDirection(270, 1, 40, false, false, false, 0); // line up w/ wall
            strafeDirection(355, 1, 200, false, false, false, 0); // go forward fast
            intake(true, false);
            clawOpen(true, false);
            strafeDirection(355, 1, 700, true, false, false, 0); // go forward fast
            /*strafeDirection(30, 0.8, 900, true, false, true, 0); // go forward until intake spikes
            wait(200);
            if (timer.seconds() <= 24) {
                moveLiftLow(180, 250);
                moveLiftLow(270, 1200);
                goToWhiteLine(190);
                goBackToStartingPosition();
                strafeDirection(114, 1, 1000, true, true, false, 3); // go to hub
                clawOpen(true, false);
                wait(400);
                clawOpen(false, false);
                intake(false, true);
                strafeDirection(294, 0.875, 985, false, false, true, false, 0); // go back to wall


                //Line up and park
                strafeDirection(270, 1, 40, false, false, false, 0); // line up w/ wall
                intake(true, false);
                strafeDirection(355, 1, 1500, true, false, false, 0); // go forward fast
                clawOpen(true, false);
                strafeDirection(0, 0.6, 200, true, false, false, 0); // go forward until intake spikes

            }*/
        }

    }


    public void moveLiftLow(int angle, int encoderTick) {
        resetEncoders();
        double imuAngle = angle;
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startAngle = getAngle();

        //resetEncoders();

        imuAngle += 90;
        //k1 = FLBR, k2 = FRBL
        double k1 = 0, k2 = 0;
        while (180 < 0 && opModeIsActive()) imuAngle += 360;
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

        k1 *= 0.8;
        k2 *= 0.8;

        k1 = Range.clip(k1, -1, 1);
        k2 = Range.clip(k2, -1, 1);

        currentAngle[0] = getAngle();
        currentAngle[1] = getAngle();

        int[] startTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        int[] currentTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;




        while (avgDifference < encoderTick && opModeIsActive()) {

            dashboardTelemetry.addData("avgDifference < tick", avgDifference < 2000);
            dashboardTelemetry.update();


            moveLift(700, 0.7, 0.2);
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
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;


            currentAngle[0] = getAngle();
            startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            k1 = Range.clip(k1, -1, 1);
            k2 = Range.clip(k2, -1, 1);

            //LF LB RB RF
            //tick = target, avgDifference = current

            int target = 2000;
            double current = avgDifference;



            double lRPower = Range.clip(k2 - stabilizeCoefficient, -1, 1);
            double lFPower = Range.clip(k1 - stabilizeCoefficient, -1, 1);
            double rRPower = Range.clip(k1 + stabilizeCoefficient, -1, 1);
            double rFPower = Range.clip(k2 + stabilizeCoefficient, -1, 1);

            robot.setDrivePower(lRPower, lFPower, rRPower, rFPower);


        }

        robot.lF.setPower(0);
        robot.rR.setPower(0);
        robot.rF.setPower(0);
        robot.lR.setPower(0);
        lineChecker = true;

    }


    public void strafeDirection(double imuAngle, double power, int tick, boolean zeroPowerOn, boolean lift, boolean collection, int liftLevel) {
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
        avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;




        while (avgDifference < tick && opModeIsActive() && !stopMethod) {

            if(lift) {
                if(liftLevel == 2 || liftLevel == 3) {
                    liftUp(true, liftLevel);
                    if (robot.liftMotor.getCurrentPosition() > 300) {
                        extensionOut(true, RobotConstants.rackBlue);
                    }
                }
                else if(liftLevel == 1){
                    if(avgDifference < 350){
                        liftUp(true, 2);
                        if (robot.liftMotor.getCurrentPosition() > 500) {
                            extensionOut(true, RobotConstants.rackBlue);
                        }
                    }
                    else{
                        robot.liftMotor.setPower(0);
                    }

                }

            }

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
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;


            currentAngle[0] = getAngle();
            startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            k1 = Range.clip(k1, -1, 1);
            k2 = Range.clip(k2, -1, 1);

            //LF LB RB RF
            //tick = target, avgDifference = current

            int target = tick;
            double current = avgDifference;

            if(power > 0.4) {
                if (current < target * 0.05) {
                    accelerationConstant = 0.6;
                } else if (current < target * 0.1) {
                    accelerationConstant = 0.65;
                } else if (current < target * 0.15) {
                    accelerationConstant = 0.7;
                } else if (current < target * 0.2) {
                    accelerationConstant = 0.75;
                } else if (current < target * 0.25) {
                    accelerationConstant = 0.8;
                } else if (current < target * 0.3) {
                    accelerationConstant = 0.85;
                } else if (current < target * 0.4) {
                    accelerationConstant = 0.9;
                } else if (current < target * 0.5) {
                    accelerationConstant = 0.95;
                } else if (current < target * 0.6) {
                    accelerationConstant = 1;
                } else if (current < target * 0.7) {
                    accelerationConstant = 1;
                } else if (current < target * 0.75) {
                    accelerationConstant = 0.95;
                } else if (current < target * 0.8) {
                    accelerationConstant = 0.9;
                } else if (current < target * 0.85) {
                    accelerationConstant = 0.85;
                } else if (current < target * 0.9) {
                    accelerationConstant = 0.8;
                } else if (current < target * 0.95) {
                    accelerationConstant = 0.75;
                }
            }
            else{
                accelerationConstant = 1;
            }




            double lRPower = accelerationConstant * Range.clip(k2 - stabilizeCoefficient, -1, 1);
            double lFPower = accelerationConstant * Range.clip(k1 - stabilizeCoefficient, -1, 1);
            double rRPower = accelerationConstant * Range.clip(k1 + stabilizeCoefficient, -1, 1);
            double rFPower = accelerationConstant * Range.clip(k2 + stabilizeCoefficient, -1, 1);

            robot.setDrivePower(lRPower, lFPower, rRPower, rFPower);

            if(robot.colorSensor.red() > 20 && collection){
                stopMethod = true;
            }

        }

        if(collection){
            clawOpen(false, false);
            wait(250);
            intake(false, true);
            wait(100);
            stopMethod = false;
        }
        if (zeroPowerOn) {
            robot.lF.setPower(0);
            robot.rR.setPower(0);
            robot.rF.setPower(0);
            robot.lR.setPower(0);
        }
    }

    public void strafeDirection(double imuAngle, double power, int tick, boolean zeroPowerOn, boolean lift, boolean liftDown, boolean collection, int liftLevel) {
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
        avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;




        while (avgDifference < tick && opModeIsActive() && !stopMethod) {

            if(lift) {
                if(liftLevel == 2 || liftLevel == 3) {
                    liftUp(true, liftLevel);
                    if (robot.liftMotor.getCurrentPosition() > 300) {
                        extensionOut(true, RobotConstants.rackBlue);
                    }
                }
                else if(liftLevel == 1){
                    if(avgDifference < 350){
                        liftUp(true, 2);
                        if (robot.liftMotor.getCurrentPosition() > 500) {
                            extensionOut(true, RobotConstants.rackBlue +50);
                        }
                    }
                    else{
                        robot.liftMotor.setPower(0);
                    }

                }

            }
            else if(liftDown){


                extensionOut(false, 10);
                if(Math.abs(robot.rackAndPinionMotor.getCurrentPosition()) > 20) {
                    liftUp(true, 3);
                }
                else{
                    //robot.liftMotor.setPower(0);
                    moveLift(0, 1, 3);
                }

            }

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
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;


            currentAngle[0] = getAngle();
            startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            k1 = Range.clip(k1, -1, 1);
            k2 = Range.clip(k2, -1, 1);

            //LF LB RB RF
            //tick = target, avgDifference = current

            int target = tick;
            double current = avgDifference;

            if(power > 0.4) {
                if (current < target * 0.05) {
                    accelerationConstant = 0.63;
                } else if (current < target * 0.1) {
                    accelerationConstant = 0.65;
                } else if (current < target * 0.15) {
                    accelerationConstant = 0.7;
                } else if (current < target * 0.2) {
                    accelerationConstant = 0.75;
                } else if (current < target * 0.25) {
                    accelerationConstant = 0.8;
                } else if (current < target * 0.3) {
                    accelerationConstant = 0.85;
                } else if (current < target * 0.4) {
                    accelerationConstant = 0.9;
                } else if (current < target * 0.5) {
                    accelerationConstant = 0.95;
                } else if (current < target * 0.6) {
                    accelerationConstant = 1;
                } else if (current < target * 0.7) {
                    accelerationConstant = 1;
                } else if (current < target * 0.75) {
                    accelerationConstant = 0.95;
                } else if (current < target * 0.8) {
                    accelerationConstant = 0.9;
                } else if (current < target * 0.85) {
                    accelerationConstant = 0.85;
                } else if (current < target * 0.9) {
                    accelerationConstant = 0.8;
                } else if (current < target * 0.95) {
                    accelerationConstant = 0.75;
                }
            }
            else{
                accelerationConstant = 1;
            }




            double lRPower = accelerationConstant * Range.clip(k2 - stabilizeCoefficient, -1, 1);
            double lFPower = accelerationConstant * Range.clip(k1 - stabilizeCoefficient, -1, 1);
            double rRPower = accelerationConstant * Range.clip(k1 + stabilizeCoefficient, -1, 1);
            double rFPower = accelerationConstant * Range.clip(k2 + stabilizeCoefficient, -1, 1);

            robot.setDrivePower(lRPower, lFPower, rRPower, rFPower);

            if(robot.colorSensor.red() > 20 && collection){
                stopMethod = true;
            }

        }

        if(collection){
            clawOpen(false, false);
            wait(250);
            intake(false, true);
            wait(100);
            stopMethod = false;
        }
        if (zeroPowerOn) {
            robot.lF.setPower(0);
            robot.rR.setPower(0);
            robot.rF.setPower(0);
            robot.lR.setPower(0);
        }
    }

    public void goToWhiteLine(int angle) {
        resetEncoders();
        double imuAngle = angle;
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startAngle = getAngle();

        //resetEncoders();

        imuAngle += 90;
        //k1 = FLBR, k2 = FRBL
        double k1 = 0, k2 = 0;
        while (180 < 0 && opModeIsActive()) imuAngle += 360;
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

        k1 *= 0.7;
        k2 *= 0.7;

        k1 = Range.clip(k1, -1, 1);
        k2 = Range.clip(k2, -1, 1);

        currentAngle[0] = getAngle();
        currentAngle[1] = getAngle();

        int[] startTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        int[] currentTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;




        while (avgDifference < 3000 && opModeIsActive()) {

            dashboardTelemetry.addData("avgDifference < tick", avgDifference < 2000);
            dashboardTelemetry.update();
            if (robot.bottomColorSensor.green() > 100 && lineChecker){
                avgDifference = 1000000000;
                lineChecker = false;
            }
            else {

                moveLift(700, 0.7, 0.2);
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
                double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;


                currentAngle[0] = getAngle();
                startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

                k1 = Range.clip(k1, -1, 1);
                k2 = Range.clip(k2, -1, 1);

                //LF LB RB RF
                //tick = target, avgDifference = current

                int target = 2000;
                double current = avgDifference;



                double lRPower = Range.clip(k2 - stabilizeCoefficient, -1, 1);
                double lFPower = Range.clip(k1 - stabilizeCoefficient, -1, 1);
                double rRPower = Range.clip(k1 + stabilizeCoefficient, -1, 1);
                double rFPower = Range.clip(k2 + stabilizeCoefficient, -1, 1);

                robot.setDrivePower(lRPower, lFPower, rRPower, rFPower);
            }

        }

        robot.lF.setPower(0);
        robot.rR.setPower(0);
        robot.rF.setPower(0);
        robot.lR.setPower(0);
        lineChecker = true;

    }

    public void goBackToStartingPosition() {
        resetEncoders();
        double imuAngle = 185;
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double startAngle = getAngle();

        //resetEncoders();

        imuAngle += 90;
        //k1 = FLBR, k2 = FRBL
        double k1 = 0, k2 = 0;
        while (180 < 0 && opModeIsActive()) imuAngle += 360;
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

        k1 *= 1;
        k2 *= 1;

        k1 = Range.clip(k1, -1, 1);
        k2 = Range.clip(k2, -1, 1);

        currentAngle[0] = getAngle();
        currentAngle[1] = getAngle();

        int[] startTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        int[] currentTick = {robot.lF.getCurrentPosition(), robot.rF.getCurrentPosition(), robot.lR.getCurrentPosition(), robot.rR.getCurrentPosition()};
        avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;




        while (avgDifference < 500 && opModeIsActive()) {

            dashboardTelemetry.addData("avgDifference < tick", avgDifference < 2000);
            dashboardTelemetry.update();

            moveLift(500, 0.7, 0.2);
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
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;


            currentAngle[0] = getAngle();
            startTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            k1 = Range.clip(k1, -1, 1);
            k2 = Range.clip(k2, -1, 1);

            //LF LB RB RF
            //tick = target, avgDifference = current

            int target = 2000;
            double current = avgDifference;



            double lRPower = Range.clip(k2 - stabilizeCoefficient, -1, 1);
            double lFPower = Range.clip(k1 - stabilizeCoefficient, -1, 1);
            double rRPower = Range.clip(k1 + stabilizeCoefficient, -1, 1);
            double rFPower = Range.clip(k2 + stabilizeCoefficient, -1, 1);

            robot.setDrivePower(lRPower, lFPower, rRPower, rFPower);


        }

        robot.lF.setPower(0);
        robot.rR.setPower(0);
        robot.rF.setPower(0);
        robot.lR.setPower(0);
        lineChecker = true;

    }

    private void clawOpen(boolean open, boolean safeDelay){
        if(safeDelay){
            wait(500);
        }
        if(open){
            robot.clawServo.setPosition(RobotConstants.clawOpen);
        }
        else{
            robot.clawServo.setPosition(RobotConstants.clawClose);
        }
        if(safeDelay){
            wait(500);
        }


    }

    public void resetLiftExtension(){
        liftUp(true, 3);
        extensionOut(false, -80);
        clawOpen(false, true);
        wait(350);
        robot.liftMotor.setPower(0);
    }
    public void resetLiftExtension(boolean levelOne){
        liftUp(true, 3);
        clawOpen(false, true);
        extensionOut(false, -80);

        wait(350);
        robot.liftMotor.setPower(0);
    }

    public void duck(boolean on){
        if(on) {
            robot.duckMotor.setPower(-0.45);

        }
        else{
            robot.duckMotor.setPower(0);
        }
    }

    public void intake(boolean on){
        if(on) {
            robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
        }
        else{
            robot.intakeMotor.setPower(0);
        }
    }

    public void intake(boolean on, boolean outtake){
        if(on) {
            robot.intakeMotor.setPower(RobotConstants.intakeSpeed);
            robot.LED.setState(false);
        }
        else if(outtake){
            robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
            robot.LED.setState(true);
        }
        else{
            robot.intakeMotor.setPower(0);
        }
    }


    public void liftUp(boolean up, int liftLevel){
        if(up) {
            if(liftLevel == 1){
                moveLift(RobotConstants.liftLevel2, 0.7, 3);
            }
            else if(liftLevel == 2){
                moveLift(RobotConstants.liftLevel2 -30, 0.7, 3);

            }
            else if(liftLevel == 3){
                moveLift(RobotConstants.liftLevel3 - 150, 0.7, 3);

            }
        }
        else{
            robot.liftMotor.setPower(0);
        }
    }

    public void extensionOut(boolean out, int rackTick){
        if(out) {
            moveRack(rackTick, 0.7, 3);
        }
        else{
            moveRack(rackTick, 1, 2);
        }
    }

    private void moveLift(int tick, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (robot.liftMotor != null) {
            robot.liftMotor.setTargetPosition(tick);


            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            try {
                robot.liftMotor.setPower(speed);
            } catch (Exception p_exception) {
                robot.liftMotor.setPower(speed);
            }

        }
    }

    private void moveRack(int tick, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (robot.rackAndPinionMotor != null) {
            robot.rackAndPinionMotor.setTargetPosition(tick);


            robot.rackAndPinionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            try {
                robot.rackAndPinionMotor.setPower(speed);
            } catch (Exception p_exception) {
                robot.rackAndPinionMotor.setPower(speed);
            }


        }
    }



    private void visionInit(){
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "rWebcam"));
        pipeline = new FreightDetermination();
        webcam.setPipeline(pipeline);
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

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Camera Broken", "");
            }
        });

    }



    private void wait(int milliseconds){
        try { //adjust
            Thread.sleep(milliseconds);
        }
        catch (Exception p_exception) {
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

    private void resetEncoders() {
        robot.rR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }


    public static class FreightDetermination extends OpenCvPipeline{

        public enum FreightPosition
        {
            LEFT,
            CENTER,
            RIGHT,
            NONE
        }


        static final Scalar BLUE = new Scalar(0, 0, 255);
        static final Scalar GREEN = new Scalar(0, 255, 0);

        static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(RobotConstants.r1XBlue,RobotConstants.r1YBlue);
        static Point REGION1_TOPLEFT_ANCHOR_POINT2 = new Point(RobotConstants.r2XBlue,RobotConstants.r2YBlue);
        static Point REGION1_TOPLEFT_ANCHOR_POINT3 = new Point(RobotConstants.r3XBlue,RobotConstants.r3YBlue);
        static final int REGION_WIDTH = 50;
        static final int REGION_HEIGHT = 55;

        final int FOUR_RING_THRESHOLD = 135;
        final int ONE_RING_THRESHOLD = 129;

        Point region1_pointA = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x,
                REGION1_TOPLEFT_ANCHOR_POINT.y);
        Point region1_pointB = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

        static final Scalar BLUE2 = new Scalar(0, 0, 255);
        static final Scalar GREEN2 = new Scalar(0, 255, 0);

        /*
         * The core values which define the location and size of the sample regions
         */


        static final int REGION_WIDTH2 = 50;
        static final int REGION_HEIGHT2 = 55;


        Point region1_pointA2 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT2.x,
                REGION1_TOPLEFT_ANCHOR_POINT2.y);
        Point region1_pointB2 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT2.x + REGION_WIDTH2,
                REGION1_TOPLEFT_ANCHOR_POINT2.y + REGION_HEIGHT2);



        static final int REGION_WIDTH3 = 50;
        static final int REGION_HEIGHT3 = 55;

        Point region1_pointA3 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT3.x,
                REGION1_TOPLEFT_ANCHOR_POINT3.y);
        Point region1_pointB3 = new Point(
                REGION1_TOPLEFT_ANCHOR_POINT3.x + REGION_WIDTH3,
                REGION1_TOPLEFT_ANCHOR_POINT3.y + REGION_HEIGHT3);

        /*
         * Working variables
         */
        Mat region1_Cb, region2_Cb, region3_Cb;

        Mat YCrCb = new Mat();
        Mat Cb = new Mat();
        int avg1;
        int avg2;
        int avg3;

        public volatile FreightPosition position = FreightPosition.LEFT;


        void inputToCb(Mat input)
        {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 1);
        }

        @Override
        public void init(Mat firstFrame)
        {
            inputToCb(firstFrame);

            region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
            region2_Cb = Cb.submat(new Rect(region1_pointA2, region1_pointB2));
            region3_Cb = Cb.submat(new Rect(region1_pointA3, region1_pointB3));
        }

        @Override
        public Mat processFrame(Mat input)
        {
            inputToCb(input);


            avg1 = (int) Core.mean(region1_Cb).val[0];
            avg2 = (int) Core.mean(region2_Cb).val[0];
            avg3 = (int) Core.mean(region3_Cb).val[0];


            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA2, // First point which defines the rectangle
                    region1_pointB2, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA3, // First point which defines the rectangle
                    region1_pointB3, // Second point which defines the rectangle
                    BLUE, // The color the rectangle is drawn in
                    2); // Thickness of the rectangle lines

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA, // First point which defines the rectangle
                    region1_pointB, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA2, // First point which defines the rectangle
                    region1_pointB2, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            Imgproc.rectangle(
                    input, // Buffer to draw on
                    region1_pointA3, // First point which defines the rectangle
                    region1_pointB3, // Second point which defines the rectangle
                    GREEN, // The color the rectangle is drawn in
                    -1); // Negative thickness means solid fill

            if(avg1 < avg2 && avg1 < avg3 && avg1 <= RobotConstants.blueVisionThreshold){
                position = FreightPosition.LEFT;
            }
            else if(avg2 < avg1 && avg2 < avg3 && avg2 <= RobotConstants.blueVisionThreshold){
                position = FreightPosition.CENTER;
            }
            else if(avg3 < avg1 && avg3 < avg2 && avg3 <= RobotConstants.blueVisionThreshold){
                position = FreightPosition.RIGHT;
            }
            else{
                position = FreightPosition.RIGHT;
            }




            return input;
        }

        public int getAnalysis1()
        {
            return avg1;
        }
        public int getAnalysis2()
        {
            return avg2;
        }
        public int getAnalysis3()
        {
            return avg3;
        }
        /*public String getPosition(){
            if(avg1 > avg2 && avg1 > avg3 && avg1 >= 140){
                return "left";
            }
            else if(avg2 > avg1 && avg2 > avg3 && avg2 >= 140){
                return "center";
            }
            else if(avg3 > avg1 && avg3 > avg2 && avg3 >= 140){
                return "right";
            }
            else{
                return "none";
            }
        }*/
    }



}


