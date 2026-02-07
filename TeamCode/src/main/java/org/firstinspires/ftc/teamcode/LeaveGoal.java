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
    public void forward(int moveTime,double power) {

        forward =-power;
        prosessInputsAndSleep(moveTime);
    }
    public void turn(String direction, int turnTime, double power) {
        if (direction == "CW") {
            turn = -Math.abs(power);
        } else {
            turn = Math.abs(power);
        }
        prosessInputsAndSleep(turnTime);
    }

    public void strafe(int moveTime, double power){
        strafe = power;
        prosessInputsAndSleep(moveTime);
    }



    private void sleep(int milliseconds) {
        try {
            Thread.sleep(Math.abs(milliseconds)); // Pause for 'milliseconds'
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore the interrupted status
        }
    }
}
