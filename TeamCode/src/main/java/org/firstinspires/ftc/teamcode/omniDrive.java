package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "omniDrive")
public class omniDrive extends LinearOpMode {

    private DcMotor FL_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BR_MOTOR;
    private DcMotor INTAKE;
    private DcMotorEx FEEDER;
    private DcMotorEx launcherOne;
    private DcMotorEx launcherTwo;
    public Servo Deflector;
    private Limelight3A limelight;
    private IMU imu;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    AimController aimController;
    double forward;
    double turn;
    double strafe;
    double maxDrivePower;
    // int testing = 0;
    double counter = 50.0;

    private void initalSetup() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        imu = hardwareMap.get(IMU.class, "imu");
        aimController = new AimController(limelight, imu,2);

    }

    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        double axial;
        double lateral;
        double yaw;
        double deflecPos = 0.97;
        double max;
        double[] solution;
        boolean yPressed = false;
        boolean aPressed = false;
        boolean xPressed = false;
        boolean aimMode;

        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");
        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");
        FEEDER = hardwareMap.get(DcMotorEx.class, "FEEDER");
        launcherTwo = hardwareMap.get(DcMotorEx.class, "LAUNCHER_2");
        launcherOne = hardwareMap.get(DcMotorEx.class, "LAUNCHER_1");

        PIDCounterforce launchPID = new PIDCounterforce(0.01, 0.0, 0);
        //LAUNCHER.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        Deflector = hardwareMap.get(Servo.class, "Deflector");
        runtime = new ElapsedTime();

        FL_MOTOR.setDirection(DcMotor.Direction.REVERSE);
        BL_MOTOR.setDirection(DcMotor.Direction.REVERSE);
        FR_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        BR_MOTOR.setDirection(DcMotor.Direction.FORWARD);

        FL_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        BL_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FR_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FL_MOTOR.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        FEEDER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launcherOne.setDirection(DcMotorEx.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");

        telemetry.update();
        initalSetup();

        telemetry.update();
        waitForStart();
        aimMode = false;
        limelight.start();
        runtime.reset();
        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Note: pushing stick forward gives negative value
            double fineAim = 1;
            // axial = gamepad1.left_stick_y;
            // lateral = gamepad1.left_stick_x;
            // yaw = gamepad1.right_stick_x/fineAim;
            double launchPower = gamepad1.left_trigger;
            double feeder = 0;
            double testing = 0;

            if(gamepad1.y){
                yPressed = true;
            } if (!gamepad1.y && yPressed){
                counter ++; //was 10
                yPressed = false;
            }
            if(gamepad1.a){
                aPressed = true;
            } if (!gamepad1.a && aPressed){
                counter --; //was 10
                aPressed = false;
            }
            if(gamepad1.x){
                xPressed = true;
            }
            if (!gamepad1.x && xPressed){
                aimMode = !aimMode;
                xPressed = false;
            }
            if (counter>100.0){
                counter = 100.0;
            }
            if (counter<0){
                counter = 0;
            }

            if (gamepad1.left_bumper) {
                launchPower = counter*28; //was first 30 then 21
            }

            if (gamepad1.left_trigger > 0.1) {
                fineAim = 4;
            } else fineAim = 1;

            // Send calculated power to wheels and launcher.
            if (gamepad1.x) {
                aimController.refreshPosition();
               solution = aimController.fireControlSolution();
               launchPower = solution[0];
               deflecPos = solution[1];
                // do auto aiming
            } else {
                aimController.clearCache();
            }
           launchPID.setSetPoint(launchPower);
            double pidPower = Math.max(launchPID.update(launcherOne.getVelocity()), -0.1);
            launcherTwo.setPower(pidPower);
            launcherOne.setPower(pidPower);
//           LAUNCHER.setVelocity(launchPower);

            //Sets power to feeding servos.


            if (gamepad1.right_trigger > 0.1) {
                feeder = -1;
                INTAKE.setPower(1);
            } else if (gamepad1.right_bumper) {
                INTAKE.setPower(1);
            } else if (gamepad1.dpad_left) {
                INTAKE.setPower(-1);
            } else {
                INTAKE.setPower(0);
                feeder = 0;
            }

            FEEDER.setPower(feeder);

            if (gamepad1.dpad_down){
                deflecPos = 0.5;
            }

            if (gamepad1.dpad_up){
                deflecPos = 0.97;
            }
            Deflector.setPosition(deflecPos);
            axial = gamepad1.left_stick_y/fineAim;
            lateral = -gamepad1.left_stick_x/fineAim;

            //if( auto = true ) {get limelight} else {use right stick x} 
            yaw = gamepad1.x ? aimController.recalcualateYaw(): -gamepad1.right_stick_x/fineAim;

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
            telemetry.addData("Left Trigger" , counter + " ");
            telemetry.addData("Feeder" , feeder + " ");
            telemetry.addData("Current launch power" ,  pidPower + " ");
            telemetry.addData("LimeLight Ta (range)" , aimController.remapRange(aimController.refreshPosition()[0],2.7,0.21,50.0,80.0) + " ");
            telemetry.addData("LimeLight Ta (range)" , aimController.refreshPosition()[0] + " ");
            //telemetry.addData("LimeLight Ta (range)" , aimController.refreshPosition()[0] + " ");
            telemetry.addData("LimeLight Tx (deviation)" , aimController.refreshPosition()[1] + " ");
            telemetry.addData("LimeLight Active", aimMode);
            telemetry.addData("testing",launcherOne.getVelocity() + " ");
            telemetry.update();
        }
    }


}
