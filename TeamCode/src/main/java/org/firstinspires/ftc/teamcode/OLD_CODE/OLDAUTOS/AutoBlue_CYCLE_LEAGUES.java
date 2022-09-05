package org.firstinspires.ftc.teamcode.OLD_CODE.OLDAUTOS;

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
import org.firstinspires.ftc.teamcode.Autonomous_NEW.AutoBlue_DUCK;
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
//@Autonomous(name = "BLUE CYCLE AUTO LEAGUES", group = "Autonomous")
public class AutoBlue_CYCLE_LEAGUES extends LinearOpMode {
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
        if(pipeline.position.equals(AutoBlue_DUCK.FreightDetermination.FreightPosition.LEFT)){
            telemetry.addData("Position: ", "Left");
        }
        else if(pipeline.position.equals(AutoBlue_DUCK.FreightDetermination.FreightPosition.CENTER)){
            telemetry.addData("Position: ", "Center");
        }
        else if(pipeline.position.equals(AutoBlue_DUCK.FreightDetermination.FreightPosition.RIGHT)){
            telemetry.addData("Position: ", "Right");
        }
        telemetry.update();
        waitForStart();
        timer.reset();

        /*==========================START OF AUTONOMOUS PROGRAM==========================*/


        if(pipeline.position.equals(FreightDetermination.FreightPosition.LEFT)){
            level3Auto();
        }
        else if(pipeline.position.equals(FreightDetermination.FreightPosition.CENTER)){
            level2Auto();
        }
        else if(pipeline.position.equals(FreightDetermination.FreightPosition.RIGHT)){
            level3Auto();
        }


    }


    private void level1Auto(){
        /** Cycle 1**/
        strafeDirectionLIFT(110, 0.7 , 870, true, 2, RobotConstants.rackBlue +100); // go to hub
        liftUp(false, 2);
        wait(2000);
        clawOpen(true, true);
        liftUp(true, 3);
        resetLiftExtensionLOW();
        intake(true, false);
        strafeDirection(290, 0.7, 985,  true); // go back to wall
        clawOpen(true, true);

        /** Cycle 2**/
        strafeDirection(270, 0.8, 50, false); // line up w/ wall
        strafeDirection(355, 0.7, 1300,  true); // go forward until intake spikes
        strafeDirectionCOLLECTION(355, 0.2, 800,  true); // go forward until intake spikes
        clawOpen(false, true);
        intake(false, true);
        wait(200);
        robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        strafeDirection(270, 0.8, 20, false); //rub on wall
        strafeDirectionLIFTLOW(180, 0.5, 1700,  true); // go back
        endingTick = 0;
        strafeDirectionLIFT(113, 0.7 , 1050, true, 3, RobotConstants.rackBlue); // go to hub
        clawOpen(true, true); // open claw
        resetLiftExtension(); // reset the lift and extension
        intake(true, false);
        intakeVoltage = 0;
        strafeDirection(292, 0.7, 1200,  true); // go back to wall
        clawOpen(true, false); // open claw
        strafeDirection(270, 0.4, 50,  false); // line up
        strafeDirection(355, 0.5, 1600, true); // park

    }

    private void level2Auto(){
        /** Cycle 1**/
        strafeDirectionLIFT(114, 0.7 , 1050, true, 2, RobotConstants.rackBlue); // go to hub
        clawOpen(true, true);
        resetLiftExtension(); // reset the lift and extension
        intake(true, false);
        strafeDirection(294, 0.7, 1200,  true); // go back to wall
        clawOpen(true, false); // open claw
        //robot.leftCatEar.setPosition(RobotConstants.leftCatDown);
        /** Cycle 2**/
        strafeDirection(270, 0.8, 50, false); // line up w/ wall;
        strafeDirection(355, 0.6, 1200,  true); // go forward until intake spikes
        strafeDirectionCOLLECTION(355, 0.2, 800,  true); // go forward until intake spikes
        clawOpen(false, true);
        intake(false, true);
        wait(200);
        //robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        strafeDirection(270, 0.8, 20, false); //rub on wall
        robot.intakeMotor.setPower(-1);
        strafeDirectionLIFTLOW(180, 0.5, 1700,  true); // go back
        robot.intakeMotor.setPower(-1);
        endingTick = 0;
        strafeDirectionLIFT(113, 0.7 , 1050, true, 3, RobotConstants.rackBlue); // go to hub
        clawOpen(true, true); // open claw
        resetLiftExtension(); // reset the lift and extension
        intake(true, false);
        intakeVoltage = 0;
        strafeDirection(292, 0.7, 1200,  true); // go back to wall
        clawOpen(true, false); // open claw

        //robot.leftCatEar.setPosition(RobotConstants.leftCatDown);

        /** Cycle 3**/
        strafeDirection(270, 0.8, 50, false); // line up w/ wall
        strafeDirection(355, 0.7, 1300,  true); // go forward until intake spikes
        strafeDirectionCOLLECTION(5, 0.2, 700,  true); // go forward until intake spikes
        clawOpen(false, true);
        intake(false, true);
        wait(200);
        robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        strafeDirection(270, 0.8, 20, false); // line up
        strafeDirectionLIFTLOW(180, 0.7, 1700,  true); // go back
        robot.intakeMotor.setPower(-1);
        endingTick = 0;
        if(timer.time(TimeUnit.SECONDS) <= 25) {
            wait(250);
            strafeDirectionLIFT(110 , 0.7, 1025, true, 3, RobotConstants.rackBlue); // go to hub
            clawOpen(true, true); // open claw
            resetLiftExtension(); // reset the lift and extension
            strafeDirection(292, 1, 1025, true); // go back to wall
            clawOpen(true, false); // open claw
            strafeDirection(355, 0.65, 1600, true); //park
        }
        else{
            strafeDirection(270, 0.4, 50,  false); // line up
            strafeDirection(355, 0.5, 1600, true); // park
        }

    }

    private void level3Auto(){
        /** Cycle 1**/
        strafeDirectionLIFT(114, 0.7 , 1050, true, 3, RobotConstants.rackBlue); // go to hub
        clawOpen(true, true);
        resetLiftExtension(); // reset the lift and extension
        intake(true, false);
        strafeDirection(294, 0.7, 1200,  true); // go back to wall
        clawOpen(true, false); // open claw
        //robot.leftCatEar.setPosition(RobotConstants.leftCatDown);
        /** Cycle 2**/
        strafeDirection(270, 0.8, 50, false); // line up w/ wall;
        strafeDirection(355, 0.6, 1200,  true); // go forward until intake spikes
        strafeDirectionCOLLECTION(355, 0.2, 800,  true); // go forward until intake spikes
        clawOpen(false, true);
        intake(false, true);
        wait(200);
        //robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        strafeDirection(270, 0.8, 20, false); //rub on wall
        robot.intakeMotor.setPower(-1);
        strafeDirectionLIFTLOW(180, 0.5, 1700,  true); // go back
        robot.intakeMotor.setPower(-1);
        endingTick = 0;
        strafeDirectionLIFT(113, 0.7 , 1050, true, 3, RobotConstants.rackBlue); // go to hub
        clawOpen(true, true); // open claw
        resetLiftExtension(); // reset the lift and extension
        intake(true, false);
        intakeVoltage = 0;
        strafeDirection(292, 0.7, 1200,  true); // go back to wall
        clawOpen(true, false); // open claw

        //robot.leftCatEar.setPosition(RobotConstants.leftCatDown);

        /** Cycle 3**/
        strafeDirection(270, 0.8, 50, false); // line up w/ wall
        strafeDirection(355, 0.7, 1300,  true); // go forward until intake spikes
        strafeDirectionCOLLECTION(5, 0.2, 700,  true); // go forward until intake spikes
        clawOpen(false, true);
        intake(false, true);
        wait(200);
        robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        strafeDirection(270, 0.8, 20, false); // line up
        strafeDirectionLIFTLOW(180, 0.7, 1700,  true); // go back
        robot.intakeMotor.setPower(-1);
        endingTick = 0;
        if(timer.time(TimeUnit.SECONDS) <= 25) {
            wait(250);
            strafeDirectionLIFT(110 , 0.7, 1025, true, 3, RobotConstants.rackBlue); // go to hub
            clawOpen(true, true); // open claw
            resetLiftExtension(); // reset the lift and extension
            strafeDirection(292, 1, 1025, true); // go back to wall
            clawOpen(true, false); // open claw
            strafeDirection(355, 0.65, 1600, true); //park
        }
        else{
            strafeDirection(270, 0.4, 50,  false); // line up
            strafeDirection(355, 0.5, 1600, true); // park
        }

    }


    public void moveInches(int inches, double speed){

        int numberOfTicks = inches * 35;
        strafeDirection(0, speed, numberOfTicks, true);


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
        extensionOut(false, -80);
        clawOpen(false, true);
        wait(350);
        robot.liftMotor.setPower(0);
    }

    public void resetLiftExtensionLOW(){
        extensionOut(false, -50);
        clawOpen(false, true);
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
        }
        else if(outtake){
            robot.intakeMotor.setPower(-RobotConstants.intakeSpeed);
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
                moveLift(RobotConstants.liftLevel2, 0.7, 3);

            }
            else if(liftLevel == 3){
                moveLift(RobotConstants.liftLevel3, 0.7, 3);

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
            moveRack(50, 1, 2);
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
        double avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;

        while (avgDifference < tick && opModeIsActive()) {


            currentTick[0] = robot.lF.getCurrentPosition();
            currentTick[1] = robot.rF.getCurrentPosition();
            currentTick[2] = robot.lR.getCurrentPosition();
            currentTick[3] = robot.rR.getCurrentPosition();


            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2])+ Math.abs(currentTick[3] - startTick[3])) / 4.0;

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;

            telemetry.addData("Current Angle", currentAngle[1]);
            telemetry.addData("Start Angle", startAngle);
            telemetry.update();
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



    public void strafeDirectionCOLLECTION(double imuAngle, double power, int tick, boolean zeroPowerOn) {
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
        double avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;



        while (avgDifference < tick && opModeIsActive() && intakeVoltage < 1.25) {
            intakeVoltage = intakeMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
            endingTick = avgDifference;

            currentTick[0] = robot.lF.getCurrentPosition();
            currentTick[1] = robot.rF.getCurrentPosition();
            currentTick[2] = robot.lR.getCurrentPosition();
            currentTick[3] = robot.rR.getCurrentPosition();

            /*if(intakeVoltage > 1.25){
                intake(false, true);
            }

             */

            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2])+ Math.abs(currentTick[3] - startTick[3])) / 4.0;

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;

            telemetry.addData("Current Angle", currentAngle[1]);
            telemetry.addData("Start Angle", startAngle);
            telemetry.update();
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

    public void strafeDirectionLIFT(double imuAngle, double power, int tick, boolean zeroPowerOn, int liftLevel, int rackTick) {
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
        double avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;

        while (avgDifference < tick && opModeIsActive()) {


            dashboardTelemetry.addData("Rack Motor", robot.rackAndPinionMotor.getCurrentPosition());
            dashboardTelemetry.addData("Lift Motor", robot.liftMotor.getCurrentPosition());
            dashboardTelemetry.update();
            liftUp(true, liftLevel);
            if(robot.liftMotor.getCurrentPosition() > 300) {
                extensionOut(true, rackTick);
            }



            currentTick[0] = robot.lF.getCurrentPosition();
            currentTick[1] = robot.rF.getCurrentPosition();
            currentTick[2] = robot.lR.getCurrentPosition();
            currentTick[3] = robot.rR.getCurrentPosition();


            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2])+ Math.abs(currentTick[3] - startTick[3])) / 4.0;

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;

            telemetry.addData("Current Angle", currentAngle[1]);
            telemetry.addData("Start Angle", startAngle);
            telemetry.update();
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

    public void strafeDirectionLIFTLOW(double imuAngle, double power, int tick, boolean zeroPowerOn) {
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
        double avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[3] - startTick[3])) / 4.0;

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double endTime = 0;
        double startTime = 0;

        while (avgDifference < tick && opModeIsActive()) {


            dashboardTelemetry.addData("Rack Motor", robot.rackAndPinionMotor.getCurrentPosition());
            dashboardTelemetry.addData("Lift Motor", robot.liftMotor.getCurrentPosition());
            dashboardTelemetry.update();
            moveLift(200, 0.7, 3);




            currentTick[0] = robot.lF.getCurrentPosition();
            currentTick[1] = robot.rF.getCurrentPosition();
            currentTick[2] = robot.lR.getCurrentPosition();
            currentTick[3] = robot.rR.getCurrentPosition();


            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2])+ Math.abs(currentTick[3] - startTick[3])) / 4.0;

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * -0.072;

            telemetry.addData("Current Angle", currentAngle[1]);
            telemetry.addData("Start Angle", startAngle);
            telemetry.update();
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

            avgDifference = (Math.abs(currentTick[0] - startTick[0]) + Math.abs(currentTick[1] - startTick[1]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[2] - startTick[2]) + Math.abs(currentTick[2] - startTick[2])) / 5;

            currentAngle[1] = getAngle();
            endTime = timer.time(TimeUnit.NANOSECONDS) / 1000000000.0;

            //double angularVelocity = (currentAngle[1] - currentAngle[0]) / (endTime - startTime);
            double angularVelocity = (currentAngle[1] - currentAngle[0]);

            //was working: double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) * 0.03;
            double stabilizeCoefficient = ((currentAngle[1] - startAngle) / (Math.abs(angularVelocity) + 2)) *0.072;

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
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rackAndPinionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


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

            if(avg1 > avg2 && avg1 > avg3 && avg1 >= RobotConstants.redVisionThreshold){
                position = FreightPosition.LEFT;
            }
            else if(avg2 > avg1 && avg2 > avg3 && avg2 >= RobotConstants.redVisionThreshold){
                position = FreightPosition.CENTER;
            }
            else if(avg3 > avg1 && avg3 > avg2 && avg3 >= RobotConstants.redVisionThreshold){
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


