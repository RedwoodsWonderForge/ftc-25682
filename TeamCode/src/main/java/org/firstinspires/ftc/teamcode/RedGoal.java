package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "RedGoal")
public class RedGoal extends LinearOpMode {


    private DcMotor FL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor BR_MOTOR;
    double turn;
    double forward;
    double maxDrivePower;
    double strafe;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");

        initalSetup();
        waitForStart();
        autoDrive();
    }

    /**
     * Describe this function...
     */
    private void initalSetup() {
        maxDrivePower = 1;
        turn = 0;
        forward = 0;
        strafe = 0;
        // "really weird" -Misty

    }

    /**
     * Describe this function...
     */
    private void autoDrive() {
        backward();
        turn();
        backward();
    }

    /**
     * Describe this function...
     */
    private void prosessInputsAndSleep(int duration) {
        // "This makes the code cleaner"??
        processDriveInputs();
        sleep(duration);
        // Stop all movement after sleep
        turn = 0;
        forward = 0;
        strafe = 0;
        processDriveInputs();
    }

    /**
     * callculates BL BR FL FR motors (Btw. Does not stop)
     */
    private void processDriveInputs() {
        turn = turn * maxDrivePower;
        forward = forward * maxDrivePower;
        strafe = strafe * maxDrivePower;
        // Combine inputs to create drive and turn (or both)
        FL_MOTOR.setPower(forward + turn + strafe);
        FR_MOTOR.setPower((forward - turn) - strafe);
        BL_MOTOR.setPower((forward + turn) - strafe);
        BR_MOTOR.setPower((forward - turn) + strafe);
    }


    /**
     * Describe this function...
     */
    private void backward() {
        forward = -1;
        prosessInputsAndSleep(1200);
    }
    private void turn() {
        turn = -1;
        prosessInputsAndSleep(280);
    }

}