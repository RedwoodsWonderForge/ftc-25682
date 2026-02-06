package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
public class LeaveGoal{


    private DcMotor FlMotor;
    private DcMotor FR_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor BR_MOTOR;
    double turn;
    private double forward;
    double maxDrivePower;
    double strafe;
    private String Direction;

    public LeaveGoal(DcMotor flMotor, DcMotor frMotor, DcMotor blMotor, DcMotor brMotor, String direction) {
        FlMotor = flMotor;
        FR_MOTOR = frMotor;
        BL_MOTOR = blMotor;
        BR_MOTOR = brMotor;
        Direction = direction;
    }

    /**
     * Describe this function...
     */
    public void initalSetup() {
        maxDrivePower = 1;
        turn = 0;
        forward = 0;
        strafe = 0;
        // "really weird" -Misty

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
        FlMotor.setPower(forward + turn + strafe);
        FR_MOTOR.setPower((forward - turn) - strafe);
        BL_MOTOR.setPower((forward + turn) - strafe);
        BR_MOTOR.setPower((forward - turn) + strafe);
    }


    /**
     * Describe this function...
     */
    public void backward() {
        forward = 1;
        prosessInputsAndSleep(400);
    }
    public void turn(String direction) {
        if (direction == "CCW") {
            turn = -1;
        } else {
            turn = 1;
        }

        prosessInputsAndSleep(280);
    }

    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds); // Pause for 'milliseconds'
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore the interrupted status
        }
    }
}
