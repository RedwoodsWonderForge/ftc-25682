package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "omniDriveRed")
public class omniDriveRed extends LinearOpMode {

    private DcMotor FL_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BR_MOTOR;
    private DcMotorEx LAUNCHER;
    public Servo deflecRight;
    public Servo deflecLeft;
    public CRServo LEFT;
    public CRServo RIGHT;
    private Limelight3A limelight;
    private IMU imu;

    double frontLeftPower;
    double backLeftPower;
    double frontRightPower;
    double backRightPower;
    double forward;
    double turn;
    double strafe;
    double maxDrivePower;
    // int testing = 0;
    double counter = 85.0;
    /**
     * This OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
     * This code will work with either a Mecanum-Drive or an X-Drive train.
     * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
     *
     * Also note that it is critical to set the correct rotation direction for each motor. See details below.
     *
     * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
     * Each motion axis is controlled by one Joystick axis.
     *
     * 1) Axial -- Driving forward and backward -- Left-joystick Forward/Backward
     * 2) Lateral -- Strafing right and left -- Left-joystick Right and Left
     * 3) Yaw -- Rotating Clockwise and counter clockwise -- Right-joystick Right and Left
     *
     * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
     * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
     * the direction of all 4 motors (see code below).
     */
    @Override
    public void runOpMode() {
        ElapsedTime runtime;
        float axial;
        float lateral;
        float yaw;
        double deflecPos = 0.3;
        double max;
        boolean yPressed = false;
        boolean aPressed = false;
        boolean xPressed = false;
        boolean aimMode;


        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");
        LAUNCHER = hardwareMap.get(DcMotorEx.class, "LAUNCHER");

        LAUNCHER.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        LEFT = hardwareMap.get(CRServo.class, "LEFT");
        RIGHT = hardwareMap.get(CRServo.class, "RIGHT");
        deflecRight = hardwareMap.get(Servo.class, "deflecRight");
        deflecLeft = hardwareMap.get(Servo.class, "deflecLeft");
        runtime = new ElapsedTime();

        // ########################################################################################
        // !!! IMPORTANT Drive Information. Test your motor directions. !!!!!
        // ########################################################################################
        //
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot
        // (the wheels turn the same direction as the motor shaft).
        //
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction. So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        //
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward.
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        // <--- Click blue icon to see important note re. testing motor directions.
        FL_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        BL_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        FR_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        BR_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        LAUNCHER.setDirection(DcMotorEx.Direction.FORWARD);

        deflecLeft.setDirection(Servo.Direction.REVERSE);
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
            float fineAim = 1;
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
                launchPower = counter*30; //was first 30 then 21
            } else launchPower = gamepad1.left_trigger;

            if (launchPower > 0.1) {
                fineAim = 3;
            } else fineAim = 1;

            // Send calculated power to wheels and launcher.
            if (gamepad1.x) {
                // do auto aiming
                aim();
//               aimMode = false;
            } else {

                FL_MOTOR.setPower(frontLeftPower);
                FR_MOTOR.setPower(frontRightPower);
                BL_MOTOR.setPower(backLeftPower);
                BR_MOTOR.setPower(backRightPower);
            }
            LAUNCHER.setVelocity(launchPower);
            //Sets power to feeding servos.
            if (gamepad1.right_trigger > 0.1) {
                feeder = 1;
            } else feeder = 0;
            LEFT.setPower(feeder);
            RIGHT.setPower(feeder*-1);

            if (gamepad1.dpad_down){
                deflecPos = 0.16;
            }
            if (gamepad1.dpad_left){
                deflecPos = 0.3;
            }
            if (gamepad1.dpad_right){
                deflecPos = 0.2;
            }
            if (gamepad1.dpad_up){
                deflecPos = 0.26;
            }
            deflecLeft.setPosition(deflecPos+0.42);
            deflecRight.setPosition(deflecPos);
            axial = gamepad1.left_stick_y/fineAim;
            lateral = gamepad1.left_stick_x/fineAim;

            //if( auto = true ) {get limelight} else {use right stick x}
            yaw = gamepad1.right_stick_x/fineAim;

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



            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime);
            telemetry.addData("Front left/Right", JavaUtil.formatNumber(frontLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(frontRightPower, 4, 2));
            telemetry.addData("Back  left/Right", JavaUtil.formatNumber(backLeftPower, 4, 2) + ", " + JavaUtil.formatNumber(backRightPower, 4, 2));
            telemetry.addData("Left Trigger" , launchPower + " ");
            telemetry.addData("Feeder" , feeder + " ");
            telemetry.addData("Feeder" , feeder + " ");
            telemetry.addData("testing" , testing + " ");
            telemetry.addData("LimeLight Active", aimMode);
            telemetry.update();
        }
    }

    /**
     * This function is used to test your motor directions.
     *
     * Each button should make the corresponding motor run FORWARD.
     *
     *   1) First get all the motors to take to correct positions on the robot
     *      by adjusting your Robot Configuration if necessary.
     *
     *   2) Then make sure they run in the correct direction by modifying the
     *      the setDirection() calls above.
     */
    private void initalSetup() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    private void testMotorDirections() {
        frontLeftPower = gamepad1.x ? 1 : 0;
        backLeftPower = gamepad1.a ? 1 : 0;
        frontRightPower = gamepad1.y ? 1 : 0;
        backRightPower = gamepad1.b ? 1 : 0;
    }
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
    private void processInputsAndSleep(int duration) {
        // "This makes the code cleaner"??
        processDriveInputs();
        sleep(duration);
        // Stop all movement after sleep
        turn = 0;
        //forward = 0;
        //strafe = 0;
        processDriveInputs();
    }
    public void aim() {
        maxDrivePower = .15;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        telemetry.addData("TagFound",llResult != null && llResult.isValid());
        if (llResult != null && llResult.isValid()) {
//            Pose3D botPose = llResult.getBotpose_MT2();
            double y = llResult.getTy();
            while (y < -3 || y > 3) {
                telemetry.addData("Ty", y);
                turn = (y > 0) ? -1 : 1;
                processInputsAndSleep(100);
                orientation = imu.getRobotYawPitchRollAngles();
                limelight.updateRobotOrientation(orientation.getYaw());
                llResult = limelight.getLatestResult();
                y = llResult.getTy();
            }
        }
        maxDrivePower = 1;
        // getLimelightData
        //getTyData
        //move y to range (during shooting and going to loading)

    }
}
