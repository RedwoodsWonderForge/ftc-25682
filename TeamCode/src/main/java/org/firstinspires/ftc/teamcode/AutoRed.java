package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "testautoRed (Blocks to Java)")
public class AutoRed extends LinearOpMode {

    private Servo deflecLeft;
    private DcMotor LAUNCHER;
    private Servo deflecRight;
    private DcMotor FL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor BR_MOTOR;
    private CRServo RIGHT;
    private CRServo LEFT;

    int turn;
    boolean IsShooting;
    int forward;
    double ShootPower;
    int maxDrivePower;
    int strafe;

    /**
     * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
     * Comment Blocks show where to place Initialization code (runs once, after touching the
     * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
     * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
     * Stopped).
     */
    @Override
    public void runOpMode() {
        deflecLeft = hardwareMap.get(Servo.class, "deflecLeft");
        LAUNCHER = hardwareMap.get(DcMotor.class, "LAUNCHER");
        deflecRight = hardwareMap.get(Servo.class, "deflecRight");
        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");
        RIGHT = hardwareMap.get(CRServo.class, "RIGHT");
        LEFT = hardwareMap.get(CRServo.class, "LEFT");

        initalSetup();
        telemetry.addLine("waiting for start");
        telemetry.update();
        waitForStart();
        telemetry.addLine("auto drive");
        telemetry.update();
        autoDrive();
    }

    /**
     * Describe this function...
     */
    private void initalSetup() {
        double duration;
        boolean aprallag;

        duration = 0.8;
        IsShooting = false;
        ShootPower = 1;
        maxDrivePower = 1;
        turn = 0;
        forward = 0;
        strafe = 0;
        deflecLeft.setDirection(Servo.Direction.REVERSE);
        // "really weird" -Misty
        aprallag = false;
    }

    /**
     * Describe this function...
     */
    private void autoDrive() {
        LAUNCHER.setPower(ShootPower);
        deflecLeft.setPosition(0.62);
        deflecRight.setPosition(0.2);
        telemetry.addLine("driving to goal");
        telemetry.update();
        driveToGoal();
        telemetry.addLine("shooting 3 artifacts");
        telemetry.update();
        shootThreeArtifacts();
        telemetry.addLine("go home");
        telemetry.update();
        driveToPlayerStationAndBack();
        telemetry.addLine("last 3 artifacts");
        telemetry.update();
        shootThreeArtifacts();
        telemetry.addLine("auto-drive complete");
        telemetry.update();
    }

    /**
     * Describe this function...
     */
    private void prosessInputsAndSleep(int duration) {
        telemetry.addLine("driving");
        telemetry.addData("duration", duration);
        telemetry.update();
        // "This makes the code cleaner"??
        processDriveInputs();
        sleep((long) duration);
        telemetry.addLine("done driving");
        telemetry.update();
        // Stop all movement after sleep
        turn = 0;
        forward = 0;
        strafe = 0;
        processDriveInputs();
    }

    /**
     * Describe this function...
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
    private void Shoot() {
        IsShooting = true;
        RIGHT.setPower(-1);
        LEFT.setPower(1);
        sleep(750);
        LEFT.setPower(0);
        RIGHT.setPower(0);
        IsShooting = false;
    }

    /**
     * Describe this function...
     */
    private void shootThreeArtifacts() {
        int NArtifacts;

        NArtifacts = 3;
        while (opModeIsActive() && NArtifacts > 0) {
            if (!IsShooting) {
                Shoot();
                NArtifacts += -1;
            }
        }
        LAUNCHER.setPower(0);
    }

    /**
     * Describe this function...
     */
    private void driveToGoal() {
        forward = 1;
        prosessInputsAndSleep(1000);
        turn = -1;
        prosessInputsAndSleep(280);
        sleep(500);
    }

    /**
     * Describe this function...
     */
    private void driveToPlayerStationAndBack() {
        forward = 1;
        prosessInputsAndSleep(670);
        LAUNCHER.setPower(ShootPower);
        sleep(2700);
        forward = -1;
        prosessInputsAndSleep(660);
    }
}