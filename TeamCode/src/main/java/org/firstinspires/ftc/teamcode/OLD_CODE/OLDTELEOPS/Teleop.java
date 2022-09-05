package org.firstinspires.ftc.teamcode.OLD_CODE.OLDTELEOPS;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.HardwareMaster;
import org.firstinspires.ftc.teamcode.constants.RobotConstants;

//@TeleOp(name = "Testing Teleop", group = "Teleop DEP")
//  @Config
public class Teleop extends OpMode {

    /*================================ROBOT INIT================================*/
    public HardwareMaster robot                               = HardwareMaster.getInstance();
    private volatile    boolean         doTelemetry                         = true;
    private             ElapsedTime     runTime                             = new ElapsedTime();

    boolean duck = false;
    boolean duckDOUBLECLICK = false;
    boolean reversedDrive = false;
    boolean reversedDriveDOUBLECLICK = false;
    boolean claw = false;
    boolean clawDOUBLECLICK = false;


    @Override
    public void init() {
        robot.init(hardwareMap);
        //robot.wobbleMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Init", "Completed");
        telemetry.update();

        robot.rR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.lR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    @Override
    public void start() {
        runTime.reset();
        Thread update = new Thread() {
            @Override
            public synchronized void run() {
                while (doTelemetry) {
                    try {

                            String tmy = "Motors" + "\n";
                        tmy += "    lF: " + robot.lF.getCurrentPosition() + "\n";
                        tmy += "    rF: " + robot.rF.getCurrentPosition() + "\n";
                        tmy += "    lR: " + robot.lR.getCurrentPosition() + "\n";
                        tmy += "    rR: " + robot.rR.getCurrentPosition() + "\n";
                        tmy += "    lift: " + robot.liftMotor.getCurrentPosition() + "\n";
                        tmy += "    rpt: " + robot.rackAndPinionMotor.getCurrentPosition() + "\n";

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
        standardDriving();
        reversedDrive = toggle(gamepad1.b , reversedDrive, reversedDriveDOUBLECLICK)[0];
        reversedDriveDOUBLECLICK = toggle(gamepad1.b, reversedDrive, reversedDriveDOUBLECLICK)[1];

        claw = toggle(gamepad2.b, claw, clawDOUBLECLICK)[0];
        clawDOUBLECLICK = toggle(gamepad2.b, claw, clawDOUBLECLICK) [1];

        if(gamepad1.x){
            resetEncoders();
        }

        if(claw){
            robot.clawServo.setPosition(RobotConstants.clawClose);
        }
        else{
            robot.clawServo.setPosition(RobotConstants.clawOpen);

        }

        double extendPower = gamepad2.left_stick_y;
        double armPower = gamepad2.right_stick_x;


        //Attachments: Arm, Extension, Claw
        double maxPower = Math.max(1.0, Math.max(Math.abs(extendPower), Math.abs(armPower)));

        //double extendPower = extend;

        if (maxPower > 0) {
            extendPower /= maxPower;
            armPower /= maxPower;

        }
        robot.liftMotor.setPower(extendPower);
        robot.rackAndPinionMotor.setPower(armPower);





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


}