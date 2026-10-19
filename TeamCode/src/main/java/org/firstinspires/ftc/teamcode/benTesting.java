package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "omniDrive")
public class benTesting extends LinearOpMode {

    private DcMotor FL_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BR_MOTOR;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    double max = 0;
    double forward;
    double turn;
    double strafe;
    double maxDrivePower;
    // int testing = 0:

    private void initalSetup() {
    }

    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        double axial;
        double lateral;
        double yaw;


        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");


        runtime = new ElapsedTime();

        FL_MOTOR.setDirection(DcMotor.Direction.REVERSE);
        BL_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        FR_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        BR_MOTOR.setDirection(DcMotor.Direction.REVERSE);

        FL_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        initalSetup();

        telemetry.update();
        waitForStart();        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            axial = gamepad1.left_stick_y;
            lateral = -gamepad1.left_stick_x;


            yaw = -gamepad1.right_stick_x;

            frontLeftPower = axial + lateral + yaw;
            frontRightPower = (axial - lateral) - yaw;
            backLeftPower = (axial - lateral) + yaw;
            backRightPower = (axial + lateral) - yaw;
            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = JavaUtil.maxOfList(JavaUtil.createListWith(Math.abs(frontLeftPower), Math.abs(frontRightPower), Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (max > 1) {
                frontLeftPower = frontLeftPower / max;
                frontRightPower = frontRightPower / max;
                backLeftPower = backLeftPower / max;
                backRightPower = backRightPower / max;
            }

            FL_MOTOR.setPower(frontLeftPower);
            FR_MOTOR.setPower(frontRightPower);
            BL_MOTOR.setPower(backLeftPower);
            BR_MOTOR.setPower(backRightPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));

            telemetry.update();
        }
    }


}
