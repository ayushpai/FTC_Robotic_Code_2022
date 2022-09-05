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

import java.util.concurrent.TimeUnit;


//@TeleOp(name = "Teleop Blue Alliance" , group = "Teleop")
//@Config
public class TeleopBlue2 extends OpMode {

    /*================================ROBOT INIT================================*/
    public HardwareMaster robot                               = HardwareMaster.getInstance();
    private volatile    boolean         doTelemetry                         = true;

    boolean codePinPointer = false;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    ElapsedTime elapsedTime = new ElapsedTime();
    double startIntakeTime;
    double currentIntakeTime;
    double startClawTime;
    double currentClawTime;
    double startLiftClawTime;
    double currentLiftClawTime;
    int initialColorValue = 0;
    ElapsedTime limitTest;
    int liftCounter = 0;

    boolean detectedElement = false;

    boolean duck = false;
    boolean duckDOUBLECLICK = false;

    boolean reversedDrive = false, reversedDriveDOUBLECLICK = false;
    boolean sidewaysDriving = false, sidewaysDrivingDOUBLECLICK = false;

    boolean stoptake = false;
    boolean stoptakeDOUBLECLICK = false;
    boolean extend = false;
    boolean extendDOUBLECLICK = false;
    boolean liftMax = false;
    boolean liftMaxDOUBLECLICK = false;
    boolean freightIntaked = false;
    boolean intakeLoopStart = true;
    boolean liftLoopStart = true;
    boolean resetInitiation = false;

    boolean clawLoopStart = true;



    boolean intakeOn = true;
    boolean intakeOnDOUBLECLICK = false;
    boolean outakeOn = false;
    boolean outakeOnDOUBLECLICK = false;
    boolean clawOpen = true;
    boolean clawOpenDOUBLECLICK = false;

    boolean seekingElement = true;
    boolean seekingElementDOUBLECLICK = false;

    boolean dpad_upBOOLEAN = false;
    boolean dpad_upDOUBLECLICK = false;
    boolean dpad_downBOOLEAN = false;
    boolean dpad_downDOUBLECLICK = false;



    ExpansionHubMotor intakeMotor;
    ExpansionHubEx expansionHub;

    int liftCount = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        //robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //telemetry.addData("Init", "Completed");
        //telemetry.update();
        robot.clawServo.setPosition(RobotConstants.clawOpen);
        robot.intakeMotor.setPower(0.5);


        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.rackAndPinionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        expansionHub = hardwareMap.get(ExpansionHubEx.class, "Expansion Hub 2");
        intakeMotor = (ExpansionHubMotor) hardwareMap.dcMotor.get("intake");






    }

    @Override
    public void start() {
        elapsedTime.reset();
        Thread update = new Thread() {
            @Override
            public synchronized void run() {
                while (doTelemetry) {
                    try {

                            String tmy = "Motors" + "\n";
                            telemetry.addData("Lift Motor", robot.liftMotor.getCurrentPosition());
                             telemetry.addData("Rack and Pinion", robot.liftMotor.getCurrentPosition());

                        telemetry.addData("", tmy);


                    } catch (Exception p_exception) {
                        telemetry.addData("Uh oh", p_exception);
                    }
                    telemetry.update();
                }
            }
        };
        update.start();
    }

    @Override
    public void loop() {
        /*==========================TESTING MODE BOOLEANS==========================*/

        if (sidewaysDriving) sidewaysDriving();
        else standardDriving();

        setDBT();
        toggles();
        intakeOutake();

        if(duck){
            robot.duckMotor.setPower(0.70);
        }
        else{
            robot.duckMotor.setPower(0);
        }

        clawIntakeManualAutoSync(1.2);








       if(liftMax){
            if(liftLoopStart) {
                /*if(liftCount % 2 == 0) {
                    moveLift(1875, 0.85, 2);
                }
                else{
                    moveLift(-1875, 0.85, 2);

                }

                 */
                liftCounter = 0;
                robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                moveLift(1875, 0.85, 2);

                liftCount++;

                liftLoopStart = false;

            }

        }
        else{
            currentLiftClawTime = elapsedTime.time(TimeUnit.MILLISECONDS);
            if(!liftLoopStart) {
                startLiftClawTime = elapsedTime.time(TimeUnit.MILLISECONDS);
                clawOpen = false;
                robot.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                robot.liftMotor.setPower(0);
                liftLoopStart = true;
            }
            if(currentLiftClawTime > startLiftClawTime + 1500 && !seekingElement){
                codePinPointer = true;
                //clawOpen = true;
                resetInitiation = false;


            }

        }

        double extendPower = gamepad2.left_stick_x;


        //Attachments: Arm, Extension, Claw
        double maxPower = Math.max(1.0, Math.abs(extendPower));

        //double extendPower = extend;

        if (maxPower > 0) {
            extendPower /= maxPower;

        }
        robot.rackAndPinionMotor.setPower(extendPower);

















    }


    private void intakeOutake2() {

       if(intakeOn){
           robot.intakeMotor.setPower(0.5);
           if(outakeOn) {
               intakeOn = false;
           }
       }
       else if (outakeOn){
           robot.intakeMotor.setPower(-0.5);
           if(intakeOn) {
               outakeOn = false;
           }
       }
       else {
           robot.intakeMotor.setPower(0);

       }

    }

    private void intakeOutake() {

        if (intakeOn && !outakeOn) { //intake is on
            robot.intakeMotor.setPower(0.4);
            if (dpad_downBOOLEAN) { //turn on outake
                intakeOn = false;
                outakeOn = true;
                dpad_downBOOLEAN = false;

            }
            else if (dpad_upBOOLEAN) { //turn off intake
                intakeOn = false;
                dpad_upBOOLEAN = false;
            }
        }
        else if (!intakeOn && outakeOn) { //outtake is on
            robot.intakeMotor.setPower(-0.4);
            if (dpad_upBOOLEAN) { //turn on intake
                outakeOn = false;
                intakeOn = true;
                dpad_upBOOLEAN = false;
            }
            else if (dpad_downBOOLEAN) { //turn off outtake
                outakeOn = false;
                dpad_downBOOLEAN = false;
            }
        }
        else {
            robot.intakeMotor.setPower(0);
            if (dpad_downBOOLEAN) { //turn on outtake
                outakeOn = true;
                dpad_downBOOLEAN = false;
            }
            else if (dpad_upBOOLEAN) { //turn on intake
                intakeOn = true;
                dpad_upBOOLEAN = false;
            }
        }


    }

    public void clawOpen(){
        if(clawOpen){
            robot.clawServo.setPosition(RobotConstants.clawOpen);
        }
        else{
            robot.clawServo.setPosition(RobotConstants.clawClose);
        }
    }

    public void clawIntakeManualAutoSync(double threshold) {
        if(seekingElement) {
            outakeOn = false;
            if (intakeOn && !outakeOn && clawOpen) {
                currentIntakeTime = elapsedTime.time(TimeUnit.MILLISECONDS);
                intakeOutake();
                if(intakeLoopStart) {
                    startIntakeTime = elapsedTime.time(TimeUnit.MILLISECONDS);
                    intakeLoopStart = false;
                }

                boolean intakeIsStarting = currentIntakeTime < startIntakeTime + 500;

                if(!intakeIsStarting){
                    double intakeVoltage = intakeMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS);


                    if (intakeVoltage > threshold && !detectedElement) { //happens too fast, must fix
                        startClawTime = elapsedTime.time(TimeUnit.MILLISECONDS);
                        detectedElement = true;
                    }

                    currentClawTime = elapsedTime.time(TimeUnit.MILLISECONDS);
                    if (currentClawTime > startClawTime + 2 && detectedElement) {
                        intakeLoopStart = true;
                        seekingElement = false;
                        clawOpen = false;
                        detectedElement = false;
                    }
                    if (currentClawTime > startClawTime + 500 && detectedElement) {
                        intakeOn = false;
                    }
                }
                seekingElement = toggle(gamepad2.left_bumper, seekingElement, seekingElementDOUBLECLICK)[0];
                seekingElementDOUBLECLICK = toggle(gamepad2.left_bumper, seekingElement, seekingElementDOUBLECLICK)[1];
            } else if (!intakeOn && outakeOn && !clawOpen) {
                clawOpen = toggle(gamepad2.b, clawOpen, clawOpenDOUBLECLICK)[0];
                clawOpenDOUBLECLICK = toggle(gamepad2.b, clawOpen, clawOpenDOUBLECLICK) [1];
                clawOpen();
            }
        } else if (!seekingElement) {
            seekingElement = toggle(gamepad2.left_bumper, seekingElement, seekingElementDOUBLECLICK)[0];
            seekingElementDOUBLECLICK = toggle(gamepad2.left_bumper, seekingElement, seekingElementDOUBLECLICK)[1];
            if (seekingElement) {
                clawOpen = true;
                intakeOn = true;
                outakeOn = false;
            }
        }
        clawOpen();
    }


    private void setDBT(){
        /*dashboardTelemetry.addData("Intake Voltage", intakeMotor.getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS));
        dashboardTelemetry.addData("Freight Intaked", freightIntaked);
        dashboardTelemetry.addData("Code Pinpointer", codePinPointer);
        dashboardTelemetry.addData("Intake Loop Start", intakeLoopStart);
        dashboardTelemetry.addData("LiftMax", liftMax);
        dashboardTelemetry.addData("liftLoopStart", liftLoopStart);
        dashboardTelemetry.addData("resetInitiation", resetInitiation);

         */

        dashboardTelemetry.addData("Intake On", intakeOn);
        dashboardTelemetry.addData("Outake On", outakeOn);
        dashboardTelemetry.addData("Seeking Element", seekingElement);
        dashboardTelemetry.addData("ClawOpen", clawOpen);
        dashboardTelemetry.addData("pinpointer", codePinPointer);


        dashboardTelemetry.update();




        //dashboardTelemetry.update();
    }

    //max - 1600

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

    private void moveExtension(int tick, double speed, double timeLimit) { // Move encoders towards target position until the position is reached, or the time limit expires
        if (robot.rackAndPinionMotor != null) {
            robot.rackAndPinionMotor.setTargetPosition(tick);


            robot.rackAndPinionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            try {
                robot.rackAndPinionMotor.setPower(speed);
            } catch (Exception p_exception) {
                robot.rackAndPinionMotor.setPower(speed);
            }
            ElapsedTime limitTest = new ElapsedTime();
            while ((robot.rackAndPinionMotor.isBusy() && limitTest.time() < timeLimit)) {
            }
            if (limitTest.time() > timeLimit) {
                robot.rackAndPinionMotor.setTargetPosition((robot.rackAndPinionMotor.getCurrentPosition()));

            }
            //robot.liftMotor.setPower(0);

        }
    }

    private void ringIntakeDetector(){

        int currentColorValue = robot.colorSensor.red();
        if(currentColorValue > initialColorValue + 5){

        }

    }




    /*===========================METHODS===========================*/

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

        int reverse;
        if (reversedDrive) {
            reverse = -1;
        } else {
            reverse = 1;
        }

        double drive = 0;
        double angle = 0;

        double turn = 0;

        turn = gamepad1.right_stick_x;
        angle = gamepad1.left_stick_x;
        drive = reverse * -gamepad1.left_stick_y;



        /*------------------Driver 1 Controls (Wheels)-------------------*/

        //Slow Motor Power based on Right Trigger
        double scaleFactor = (1 / (1 + 3 * gamepad1.right_trigger));

        //Set motor power based on gamepad input
        robot.setDrivePower(scaleFactor * (drive + turn - angle ), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));

    }

    private void sidewaysDriving() {

        /*-------------------Drive Controls-------------------*/

        int reverse;
        if (reversedDrive) {
            reverse = -1;
        } else {
            reverse = 1;
        }

        double drive = 0;
        double angle = 0;

        double turn = 0;
        turn = gamepad1.right_stick_x;
        angle = gamepad1.left_stick_y;
        drive = reverse * gamepad1.left_stick_x;


        /*------------------Driver 1 Controls (Wheels)-------------------*/

        //Slow Motor Power based on Right Trigger
        double scaleFactor = (1 / (1 + 3 * gamepad1.right_trigger));

        //Set motor power based on gamepad input
        robot.setDrivePower(scaleFactor * (drive + turn - angle ), scaleFactor * (drive + turn + angle), scaleFactor * (drive - turn + angle), scaleFactor * (drive - turn - angle));

    }

    private void toggles(){

        duck = toggle(gamepad2.right_bumper, duck, duckDOUBLECLICK)[0];
        duckDOUBLECLICK = toggle(gamepad2.right_bumper, duck, duckDOUBLECLICK) [1];

        clawOpen = toggle(gamepad2.b, clawOpen, clawOpenDOUBLECLICK)[0];
        clawOpenDOUBLECLICK = toggle(gamepad2.b, clawOpen, clawOpenDOUBLECLICK) [1];

        dpad_downBOOLEAN = toggle(gamepad2.dpad_down, dpad_downBOOLEAN, dpad_downDOUBLECLICK)[0];
        dpad_downDOUBLECLICK = toggle(gamepad2.dpad_down, dpad_downBOOLEAN, dpad_downDOUBLECLICK)[1];

        dpad_upBOOLEAN = toggle(gamepad2.dpad_up, dpad_upBOOLEAN, dpad_upDOUBLECLICK)[0];
        dpad_upDOUBLECLICK = toggle(gamepad2.dpad_up, dpad_upBOOLEAN, dpad_upDOUBLECLICK)[1];

        liftMax = toggle(gamepad2.y, liftMax, liftMaxDOUBLECLICK)[0];
        liftMaxDOUBLECLICK = toggle(gamepad2.y, liftMax, liftMaxDOUBLECLICK) [1];

        extend = toggle(gamepad2.right_bumper, extend, extendDOUBLECLICK)[0];
        extendDOUBLECLICK = toggle(gamepad2.right_bumper, extend, extendDOUBLECLICK) [1];

        seekingElement = toggle(gamepad2.left_bumper, seekingElement, seekingElementDOUBLECLICK)[0];
        seekingElementDOUBLECLICK = toggle(gamepad2.left_bumper, seekingElement, seekingElementDOUBLECLICK) [1];

        sidewaysDriving = toggle(gamepad1.dpad_right, sidewaysDriving, sidewaysDrivingDOUBLECLICK)[0];
        sidewaysDrivingDOUBLECLICK = toggle(gamepad1.dpad_right, sidewaysDriving, sidewaysDrivingDOUBLECLICK)[1];

        reversedDrive = toggle(gamepad1.dpad_left , reversedDrive, reversedDriveDOUBLECLICK)[0];
        reversedDriveDOUBLECLICK = toggle(gamepad1.dpad_left, reversedDrive, reversedDriveDOUBLECLICK)[1];





        //freightIntakedBoolean = toggle(gamepad2.left_bumper, freightIntakedBoolean, freightIntakedBooleanDOUBLECLICK)[0];
        //freightIntakedBooleanDOUBLECLICK = toggle(gamepad2.left_bumper, freightIntakedBoolean, freightIntakedBooleanDOUBLECLICK) [1];

    }


}