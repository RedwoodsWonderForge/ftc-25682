package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "teleop-main")
public class teleop_main extends LinearOpMode {

    private DcMotorEx FL_MOTOR;
    private DcMotorEx BL_MOTOR;
    private DcMotorEx FR_MOTOR;
    private DcMotorEx BR_MOTOR;
    private DcMotorEx INTAKE;
    private DcMotorEx CAROUSEL;
    private DcMotorEx LAUNCHER;
    public Servo FLICKER;
    public Servo EXPANSION;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    double max = 0;
    double carouselPos = 0;
    double carouselTicks = 356.275; //356 is the number of ticks for a quarter of a rotation on a 117RPM motor (well 356.275 but it only works in int so)

    double launchPower = 0.6;
    double increment = 0.01;
    double forward;
    double turn;
    double strafe;
    double maxDrivePower;
    boolean RPadPress;
    boolean LPadPress;
    boolean aPress;
    boolean yPress;
    // int testing = 0:
    public PIDCounterforce launchPID = new PIDCounterforce(1,0,0);

    private void initalSetup() {
    }

    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        double axial;
        double lateral;
        double yaw;


        FL_MOTOR = hardwareMap.get(DcMotorEx.class, "FL_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotorEx.class, "BL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotorEx.class, "FR_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotorEx.class, "BR_MOTOR");

        INTAKE = hardwareMap.get(DcMotorEx.class, "INTAKE");
        LAUNCHER = hardwareMap.get(DcMotorEx.class, "LAUNCHER");
        CAROUSEL = hardwareMap.get(DcMotorEx.class, "CAROUSEL");

        FLICKER = hardwareMap.get(Servo.class, "FLICKER");
        EXPANSION = hardwareMap.get(Servo.class, "EXPANSION");


        runtime = new ElapsedTime();

        FL_MOTOR.setDirection(DcMotorEx.Direction.REVERSE);
        BL_MOTOR.setDirection(DcMotorEx.Direction.FORWARD);
        FR_MOTOR.setDirection(DcMotorEx.Direction.REVERSE);
        BR_MOTOR.setDirection(DcMotorEx.Direction.REVERSE);

        FL_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BR_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LAUNCHER.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        CAROUSEL.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        initalSetup();

        telemetry.update();
        waitForStart();        // Run until the end of the match (driver presses STOP)


        while (opModeIsActive()) {

            axial = gamepad2.left_stick_y;
            lateral = -gamepad2.left_stick_x;
            yaw = -gamepad2.right_stick_x;

            frontLeftPower = axial + lateral + yaw;
            frontRightPower = (axial - lateral) - yaw;
            backLeftPower = (axial - lateral) + yaw;
            backRightPower = (axial + lateral) - yaw;

            if (gamepad1.b) {
                EXPANSION.setPosition(0);
            } else EXPANSION.setPosition(0.7);

            if (gamepad1.dpad_right) {
                RPadPress = true;
            }
            if (!gamepad1.dpad_right && RPadPress) {
                carouselPos += carouselTicks; //was 10
                RPadPress = false;
            }
            if (gamepad1.dpad_left) {
                LPadPress = true;
            }
            if (!gamepad1.dpad_left && LPadPress) {
                carouselPos -= carouselTicks;
                LPadPress = false;
            }
            if (gamepad1.dpad_down) {
                if (gamepad1.left_stick_x > 0.02) {
                    carouselPos += 0.5;
                } else if (gamepad1.left_stick_x < -0.02) {
                    carouselPos -= 0.5;
                }
            }


            if (gamepad1.a) {
                aPress = true;
            }
            if (!gamepad1.a && aPress) {
                launchPower += increment;
                aPress = false;
            }
            if (gamepad1.y) {
                yPress = true;
            }
            if (!gamepad1.y && yPress) {
                launchPower -= increment;
                yPress = false;
            }
            //launchPID.setSetPoint(-0.09);
            if (gamepad1.right_trigger > 0.1) {
                LAUNCHER.setVelocity(launchPower * 2360);

                //LAUNCHER.setPower(launchPID.update(LAUNCHER.getVelocity())/-1);
            } else LAUNCHER.setPower(0);

            if (gamepad1.right_bumper) {
                FLICKER.setPosition(0.63);
            } else FLICKER.setPosition(0.91);

            if (gamepad1.left_bumper) {
                INTAKE.setPower(1);
            } else if (gamepad1.dpad_up) {
                INTAKE.setPower(-1);
            } else INTAKE.setPower(0);
            CAROUSEL.setTargetPosition((int) carouselPos);
            CAROUSEL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            CAROUSEL.setPower(1);


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
            telemetry.addData("counter", +carouselPos / 1425.1);
            telemetry.addData("carousel position", +CAROUSEL.getCurrentPosition() / 1425.1);
            telemetry.addData("launch power: ", +launchPower * 21);
            telemetry.addData("launch speed: ", +LAUNCHER.getVelocity());
            telemetry.addData("launcher power", +LAUNCHER.getPower());
            telemetry.update();
        }
    }
}
