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


//@TeleOp(name = "RED ALLIANCE SPECIFIC 2", group = "Teleop")
//  @Config
public class Teleop_RED_ALLIANCE_SPECIFIC_ALEX extends OpMode {

    /*================================ROBOT INIT================================*/

                                /** Toggle Enums **/
    private             enum CAPSTONE_TYPE    {LEFT, NONE, RIGHT};
    private             CAPSTONE_TYPE       capstone_status                 = CAPSTONE_TYPE.NONE;


                                /** Toggle Booleans **/
    private             boolean             duck                            = false;
    private             boolean             duckDOUBLECLICK                 = false;
    private             boolean             reversedDrive                   = false;
    private             boolean             reversedDriveDOUBLECLICK        = false;
    private             boolean             clawOpen                        = true;
    private             boolean             clawOpenDOUBLECLICK             = true;
    private             boolean             dpadUp                          = false;
    private             boolean             dpadUpDOUBLECLICK               = false;
    private             boolean             dpadDownDOUBLECLICK             = false;
    private             boolean             dpadDown                        = false;
    private             boolean             extension                       = false;
    private             boolean             extensionDOUBLECLICK            = false;
    private             boolean             initiateSpikeDetector           = false;
    private             boolean             timerBoolean                    = true;
    private             boolean             leftBumper                      = false;
    private             boolean             leftBumperDOUBLECLICK           = false;
    private             boolean             rightBumper                     = false;
    private             boolean             rightBumperDOUBLECLICK          = false;
    private             boolean             rightClawAdjustment             = false;
    private             boolean             rightClawAdjustmentDOUBLECLICK  = false;
    private             boolean             intakeOn                        = true;
    private             boolean             intakeOnDOUBLECLICK             = true;

    private             boolean             capstoneChangeStatus            = false;
    private             boolean             capstoneChangeStatusDOUBLECLICK = false;


                                /** Logic Booleans **/
    private             boolean             liftDown                        = false;
    private             boolean             colorDetectionStart             = false;
    private             boolean             highLevel                       = true;

                                   /** Numbers **/
    private             double              startTimer                      = 0;
    private             double              currentTimer                    = 0;
    private             double              intakeThreshold                 = 1.0;

    private             int                 liftCounter                     = 0;
    private             int                 extendCounter                   = 0;

                              /** Essential Variables **/
    private             FtcDashboard        dashboard                       = FtcDashboard.getInstance();
    private             Telemetry           dashboardTelemetry              = dashboard.getTelemetry();
    private             ElapsedTime         elapsedTime                     = new ElapsedTime();
    private             ElapsedTime         limitTest;
    public              HardwareMaster      robot                           = HardwareMaster.getInstance();
    private             ElapsedTime         runTime                         = new ElapsedTime();
    private             ExpansionHubMotor intakeMotor;
    private             ExpansionHubEx expansionHub;







    @Override
    public void init() {
        robot.init(hardwareMap); // Initialize Hardware Config
        clawOpen = true; // Open the claw
        elapsedTime.reset(); // Reset Timer
        //resetEncoders(); // Reset Encoders

        robot.rightCatEar.setPosition(RobotConstants.rightCatUp); // Cat Ears Up
        robot.leftCatEar.setPosition(RobotConstants.leftCatUp); // Cat Ears Up
        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2"); // Initialize EHub
        intakeMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake"); // Initialize Intake
        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rackAndPinionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    @Override
    public void start() {
        runTime.reset();
    }

    @Override
    public void loop() {
        currentTimer = System.currentTimeMillis();
        standardDriving();
        toggles();

        //DBT();

        /** Capstone Control **/




        //CHANGE STATUS BASED ON CONTROLS
        if (!capstoneChangeStatusDOUBLECLICK) {
            if (gamepad2.dpad_right) {
                if (capstone_status == CAPSTONE_TYPE.LEFT) {
                    capstone_status = CAPSTONE_TYPE.NONE;
                } else if (capstone_status == CAPSTONE_TYPE.NONE) {
                    capstone_status = CAPSTONE_TYPE.RIGHT;
                }
            } else if (gamepad2.dpad_left) {
                if (capstone_status == CAPSTONE_TYPE.RIGHT) {
                    capstone_status = CAPSTONE_TYPE.NONE;
                } else if (capstone_status == CAPSTONE_TYPE.NONE) {
                    capstone_status = CAPSTONE_TYPE.LEFT;
                }
            }
        }

        dashboardTelemetry.addData("Capstone Status", capstone_status);
        dashboardTelemetry.addData("Double Click", capstoneChangeStatusDOUBLECLICK);
        dashboardTelemetry.update();

        //DO SOMETHING BASED ON STATUS
        if (capstone_status == CAPSTONE_TYPE.NONE) {

        } else if (capstone_status == CAPSTONE_TYPE.RIGHT) {

        } else if (capstone_status == CAPSTONE_TYPE.LEFT) {

        }

        /** DPad Up Control **/
        if(dpadUp){
            dpadDown = false;
            liftCounter = 0;

            moveLift(RobotConstants.liftLevel3 + 200, 1, 4); // move lift to level 3


            extendCounter = 0;
            if(robot.liftMotor.getCurrentPosition() > 750) {
                moveExtension2(RobotConstants.rackRed, 0.9, 3); // move extension out once the lift is high enough
            }

        }

        /** DPad Down Control **/
        if(dpadDown){
            dpadUp = false;
            extendCounter = 0;
            moveExtension2(0,0.7, 3); // Bring the rack back in the robot
            if(Math.abs(robot.rackAndPinionMotor.getCurrentPosition()) < 200) {
                robot.liftMotor.setPower(0); // bring the lift down
            }
            clawOpen = false; // close the claw
            if(liftDown){ //once the lift is down (magnet sensor)....
                intakeOn = true; // turn intake on
                clawOpen = true; // open the claw
                timerBoolean = true;
                dpadDown = false;
            }
        }



        if(robot.magnet.getState()){
            liftDown = false;   //lift up
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

        if(leftBumper){
            robot.leftCatEar.setPosition(RobotConstants.leftCatDown);
        }else{
            robot.leftCatEar.setPosition(RobotConstants.leftCatUp);
        }
        if(rightBumper){
            robot.rightCatEar.setPosition(RobotConstants.rightCatDown);
        } else{
            robot.rightCatEar.setPosition(RobotConstants.rightCatUp);
        }



        automaticClaw(intakeThreshold);
    }

    private void automaticClaw(double threshold){
        double intakeVoltage = intakeMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);
        dashboardTelemetry.addData("IntakeMotor.getVelocity() > RobotConstants.intakeVelocity - 100", intakeMotor.getVelocity() > RobotConstants.intakeVelocity - 100);
        dashboardTelemetry.addData("intakeVoltage < intakeThreshold", intakeVoltage < intakeThreshold);
        dashboardTelemetry.addData("Intake Velocity", intakeMotor.getVelocity());


        //if(intakeMotor.getVelocity() > RobotConstants.intakeVelocity - 100) { //if intake is almost at full velocity

            if(intakeVoltage < intakeThreshold) {
                initiateSpikeDetector = true;
                if(timerBoolean){
                    startTimer = System.currentTimeMillis();
                    clawOpen = true;
                    timerBoolean = false;
                }
            }
        //}

        dashboardTelemetry.addData("Intake Voltage > Threshold", intakeVoltage > threshold);
        dashboardTelemetry.addData("Current Timer > Start Timer", currentTimer > startTimer + 500);
        dashboardTelemetry.addData("Initiate Spike", initiateSpikeDetector);
        dashboardTelemetry.addData("colorDetectionStart", colorDetectionStart);


        if(initiateSpikeDetector && !gamepad2.a){
            if(intakeVoltage > threshold && currentTimer > startTimer + 1000){
                colorDetectionStart = true;
            }

            if(colorDetectionStart){
                if(robot.colorSensor.red() > 25){
                    if(currentTimer > startTimer + 1000) {
                        clawOpen = false;
                        //moveLift(100, 0.5, 2);
                    }
                    if(currentTimer > startTimer + 1400) {
                        intakeOn = false;

                    }
                    colorDetectionStart = false;
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

    public void DBT(){
        dashboardTelemetry.addData("Intake Voltage", intakeMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        dashboardTelemetry.addData("Intake On", intakeOn);
        dashboardTelemetry.addData("clawOpen", clawOpen);

        dashboardTelemetry.addData("Lift Down", liftDown);
        dashboardTelemetry.update();
    }

    public void toggles(){
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

        capstoneChangeStatus = toggle(gamepad2.dpad_left || gamepad2.dpad_right, capstoneChangeStatus, capstoneChangeStatusDOUBLECLICK)[0];
        capstoneChangeStatusDOUBLECLICK = toggle(gamepad2.dpad_left || gamepad2.dpad_right, capstoneChangeStatus, capstoneChangeStatusDOUBLECLICK)[1];
    }


}