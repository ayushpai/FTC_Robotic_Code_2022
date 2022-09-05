package org.firstinspires.ftc.teamcode.OLD_CODE.OLDTELEOPS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.HardwareMaster;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;


//@TeleOp(name = "RED ALLIANCE SPECIFIC", group = "Teleop")
//  @Config
public class Teleop_RED_ALLIANCE_SPECIFIC extends OpMode {

    /*================================ROBOT INIT================================*/
    public HardwareMaster robot                               = HardwareMaster.getInstance();
    private volatile    boolean         doTelemetry                         = true;
    private             ElapsedTime     runTime                             = new ElapsedTime();

    boolean duck = false;
    boolean duckDOUBLECLICK = false;
    boolean reversedDrive = false;
    boolean reversedDriveDOUBLECLICK = false;
    boolean clawOpen = true;
    boolean clawOpenDOUBLECLICK = true;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    boolean dpadUp = false;
    boolean dpadUpDOUBLECLICK = false;
    boolean dpadDownDOUBLECLICK = false;
    boolean dpadDown = false;
    boolean extension = false;
    boolean extensionDOUBLECLICK = false;

    boolean intaked = false;

    int liftReset = 0;
    boolean cubeInRobot = false;

    double startTimer = 0;
    double currentTimer = 0;
    ElapsedTime elapsedTime = new ElapsedTime();



    ElapsedTime limitTest;
    int liftCounter = 0;
    int extendCounter = 0;

    boolean intakeOn = true;
    boolean intakeOnDOUBLECLICK = true;

    ExpansionHubMotor intakeMotor;
    ExpansionHubEx expansionHub;

    boolean liftDown = false;
    boolean intakeLoopStart = true;

    boolean pinpoint = false;

    double intakeTreshold = 1.0;

    boolean initiateSpikeDetector = false;
    boolean timerBoolean = true;

    boolean leftBumper = false;
    boolean leftBumperDOUBLECLICK = false;

    boolean rightBumper = false;
    boolean rightBumperDOUBLECLICK = false;

    boolean outake = false;


    boolean rightClawAdjustment = false;
    boolean rightClawAdjustmentDOUBLECLICK = false;

    boolean extensionDone = false;




    @Override
    public void init() {
        robot.init(hardwareMap);
        //robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        clawOpen = true;
        elapsedTime.reset();
        resetEncoders();
        telemetry.addData("Init", "Completed");
        telemetry.update();

        robot.rightCatEar.setPosition(RobotConstants.rightCatUp);
        robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intakeMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");

    }

    @Override
    public void start() {
        runTime.reset();
    }

    @Override
    public void loop() {
        /*==========================TESTING MODE BOOLEANS==========================*/
        standardDriving();
        reversedDrive = toggle(gamepad1.b , reversedDrive, reversedDriveDOUBLECLICK)[0];
        reversedDriveDOUBLECLICK = toggle(gamepad1.b, reversedDrive, reversedDriveDOUBLECLICK)[1];

        dpadDown = toggle(gamepad2.dpad_down, dpadDown, dpadDownDOUBLECLICK)[0];
        dpadDownDOUBLECLICK = toggle(gamepad2.dpad_down, dpadDown, dpadDownDOUBLECLICK)[1];

        dpadUp = toggle(gamepad2.dpad_up, dpadUp, dpadUpDOUBLECLICK)[0];
        dpadUpDOUBLECLICK = toggle(gamepad2.dpad_up, dpadUp, dpadUpDOUBLECLICK)[1];

        intakeOn = toggle(gamepad2.a, intakeOn, intakeOnDOUBLECLICK)[0];
        intakeOnDOUBLECLICK = toggle(gamepad2.a, intakeOn, intakeOnDOUBLECLICK)[1];

        duck = toggle(gamepad2.y,duck, duckDOUBLECLICK)[0];
        duckDOUBLECLICK = toggle(gamepad2.y, duck, duckDOUBLECLICK)[1];

        extension = toggle(gamepad2.x, extension, extensionDOUBLECLICK)[0];
        extensionDOUBLECLICK = toggle(gamepad2.x, extension, extensionDOUBLECLICK)[1];

        clawOpen = toggle(gamepad2.b, clawOpen, clawOpenDOUBLECLICK)[0];
        clawOpenDOUBLECLICK = toggle(gamepad2.b, clawOpen, clawOpenDOUBLECLICK) [1];

        rightBumper = toggle(gamepad2.right_bumper, rightBumper, rightBumperDOUBLECLICK)[0];
        rightBumperDOUBLECLICK = toggle(gamepad2.right_bumper, rightBumper, rightBumperDOUBLECLICK)[1];

        leftBumper = toggle(gamepad2.left_bumper, leftBumper, leftBumperDOUBLECLICK)[0];
        leftBumperDOUBLECLICK = toggle(gamepad2.left_bumper, leftBumper, leftBumperDOUBLECLICK)[1];

        rightClawAdjustment = toggle(gamepad2.right_bumper, rightClawAdjustment, rightClawAdjustmentDOUBLECLICK)[0];
        rightClawAdjustmentDOUBLECLICK = toggle(gamepad2.right_bumper, rightClawAdjustment, rightClawAdjustmentDOUBLECLICK)[1];

        dashboardTelemetry.addData("Intake Voltage", intakeMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        dashboardTelemetry.addData("Intake On", intakeOn);
        dashboardTelemetry.addData("clawOpen", clawOpen);

        dashboardTelemetry.addData("Lift Down", liftDown);
        dashboardTelemetry.update();
        /*if(gamepad1.x){
            resetEncoders();
        }

         */
        double extendPower = gamepad2.left_stick_y;
        double armPower = gamepad2.right_stick_x;
        //Attachments: Arm, Extension, Claw

        currentTimer = System.currentTimeMillis();

        /* Manual Controls
        double maxPower = Math.max(1.0, Math.max(Math.abs(extendPower), Math.abs(armPower)));

        //double extendPower = extend;

        if (maxPower > 0) {
            extendPower /= maxPower;
            armPower /= maxPower;

        }
        robot.liftMotor.setPower(extendPower);
        robot.rackAndPinionMotor.setPower(armPower);

         */
        if(dpadDown){
            dpadUp = false;
            outake = false;
            extendCounter = 0;
            moveExtension2(50,0.7, 3);
            if(Math.abs(robot.rackAndPinionMotor.getCurrentPosition()) < 200) {
                robot.liftMotor.setPower(0);
            }
            clawOpen = false;
            if(liftDown){
                intakeOn = true;
                clawOpen = true;
                timerBoolean = true;
                dpadDown = false;
            }
        }

        if(dpadUp){
            dpadDown = false;
            liftCounter = 0;
            moveLift(2000, 0.8, 3);
            extendCounter = 0;
            if(robot.liftMotor.getCurrentPosition() > 750) {
                moveExtension2(-1450, 0.7, 3);
            }
            outake = true;
        }

        if(robot.magnet.getState()){
            liftDown = false;//lift up
        }
        else{
            liftDown = true;
        }

        if(duck){
            robot.duckMotor.setPower(-0.7);
            intakeOn = false;
        }
        else{
            robot.duckMotor.setPower(0);
        }



        if(clawOpen){
            robot.clawServo.setPosition(RobotConstants.clawOpen);
        }
        else{
            robot.clawServo.setPosition(RobotConstants.clawClose);
        }
        if(intakeOn){
            robot.intakeMotor.setPower(0.65);
        }
        else{
            robot.intakeMotor.setPower(0);
        }

        if(gamepad2.a){
            timerBoolean = true;
        }



        //robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        //            robot.rightCatEar.setPosition(RobotConstants.rightCatUp); robot.leftCatEar.setPosition(RobotConstants.leftCatDown);
        //            robot.rightCatEar.setPosition(RobotConstants.rightCatDown);

        if(leftBumper){
            robot.leftCatEar.setPosition(RobotConstants.leftCatDown);
            intakeOn = false;
        }else{
            robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        }
        if(rightBumper){
            robot.rightCatEar.setPosition(RobotConstants.rightCatDown);
            intakeOn = false;
        } else{
            robot.rightCatEar.setPosition(RobotConstants.rightCatUp);
        }

        /*if(outake){
            robot.intakeMotor.setPower(-0.7);
        }
        else if(!liftDown){
            intakeOn = false;
        }


         */



        automaticClaw(intakeTreshold);
    }



    private void automaticClaw(double threshold){
        double intakeVoltage = intakeMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
        dashboardTelemetry.addData("IntakeMotor.getVelocity() > RobotConstants.intakeVelocity - 100", intakeMotor.getVelocity() > RobotConstants.intakeVelocity - 100);
        dashboardTelemetry.addData("intakeVoltage < intakeTreshold", intakeVoltage < intakeTreshold);
        dashboardTelemetry.addData("Intake Velocity", intakeMotor.getVelocity());


        //if(intakeMotor.getVelocity() > RobotConstants.intakeVelocity - 100) { //if intake is almost at full velocity

            if(intakeVoltage < intakeTreshold) {
                initiateSpikeDetector = true;
                if(timerBoolean){
                    startTimer = System.currentTimeMillis();
                    clawOpen = true;
                    timerBoolean = false;
                }
            }
        //}

        dashboardTelemetry.addData("Intake Voltage > Treshold", intakeVoltage > threshold);
        dashboardTelemetry.addData("Current Timer > Start Timer", currentTimer > startTimer + 500);
        dashboardTelemetry.addData("Initiate Spike", initiateSpikeDetector);


        if(initiateSpikeDetector && !gamepad2.a){
            if(intakeVoltage > threshold){
                if(currentTimer > startTimer + 1000) {
                    clawOpen = false;

                }
                if(currentTimer > startTimer + 1000) {
                    intakeOn = false;
                }
            }


        }

    }




    /**
     * Toggles between true and false for a mode using a
     * button or a combo of buttons and prevents double clicking (where it just
     * constantly switches from true to false when holding
     * the button down.)
     *
     * @param button      Gamepad Button
     * @param mode        Boolean variable for mode which will be toggled
     * @param doubleClick DoubleClick boolean specific for this mode
     * @return Element 1 is what the toggled "mode" became and Element 2 is what the doubleClick boolean became.
     */
    public boolean[] toggle(boolean button, boolean mode, boolean doubleClick) {
        if (button && !doubleClick && !mode) mode = true;
        else if (button && !doubleClick && mode) mode = false;
        if (button) doubleClick = true;
        else doubleClick = false;

        boolean[] output = {mode, doubleClick};
        return output;
    }
    private void standardDriving() {

        /*-------------------Drive Controls-------------------*/

        double drive = 0;
        double angle = 0;

        double turn = 0;
        if(!reversedDrive){
            turn = gamepad1.right_stick_x;
            angle = gamepad1.left_stick_x;
            drive = -gamepad1.left_stick_y;
        }
        else {
            turn = -gamepad1.right_stick_x;
            angle = gamepad1.left_stick_x;
            drive = -gamepad1.left_stick_y;
        }


        /*------------------Driver 1 Controls (Wheels)-------------------*/

        //Slow Motor Power based on Right Trigger
        double scaleFactor = (1 / (1 + 3 * gamepad1.right_trigger));

        //Set motor power based on gamepad input
        robot.setDrivePower(scaleFactor * (drive + turn - angle ), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));

    }
    private void resetEncoders() {
        robot.rR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rackAndPinionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void moveLift(int tick, double speed, double timeLimit) {
        // Move encoders towards target position until the position is reached, or the time limit expires
        if (robot.liftMotor != null) {
            robot.liftMotor.setTargetPosition(tick);

            if(liftCounter == 0){
                limitTest = new ElapsedTime();
            }

            robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            try {
                robot.liftMotor.setPower(speed);
            } catch (Exception p_exception) {
                robot.liftMotor.setPower(speed);
            }

            if (limitTest.time() > timeLimit) {
                robot.liftMotor.setTargetPosition((robot.liftMotor.getCurrentPosition()));

            }
            liftCounter++;
        }
    }
    private void moveExtension(int tickTarget, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (robot.rackAndPinionMotor != null) {
            robot.rackAndPinionMotor.setTargetPosition(tickTarget);
            int currentTick = robot.rackAndPinionMotor.getCurrentPosition();

            robot.rackAndPinionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            double tickRatio =  1 - Math.pow(Math.abs(currentTick / tickTarget), 1.25);

            robot.rackAndPinionMotor.setPower(speed * tickRatio);

            ElapsedTime limitTest = new ElapsedTime();
            while ((robot.rackAndPinionMotor.isBusy() && limitTest.time() < timeLimit)) {
            }
            if (limitTest.time() > timeLimit) {
                robot.rackAndPinionMotor.setTargetPosition((currentTick));

            }
            extendCounter++;
            //robot.liftMotor.setPower(0);

        }
    }
    private void moveExtension2(int tick, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (robot.rackAndPinionMotor != null) {



            if(extendCounter == 0){
                limitTest = new ElapsedTime();
                robot.rackAndPinionMotor.setTargetPosition(tick);
                robot.rackAndPinionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



            try {
                robot.rackAndPinionMotor.setPower(speed);
            } catch (Exception p_exception) {
                robot.rackAndPinionMotor.setPower(speed);
            }


            if (limitTest.time() > timeLimit) {
                robot.rackAndPinionMotor.setTargetPosition((robot.rackAndPinionMotor.getCurrentPosition()));

            }
            //robot.liftMotor.setPower(0);
            extendCounter++;

        }
    }


}