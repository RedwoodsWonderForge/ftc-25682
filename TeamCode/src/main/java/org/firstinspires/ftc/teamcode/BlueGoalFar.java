package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueGoalFar")
public class BlueGoalFar extends LinearOpMode {
    AutoDrive autoDrive;
    Shoot shootUtil;
    private DcMotor FL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor BR_MOTOR;
    private DcMotor FEEDER;
    private Servo Deflector;
    private DcMotorEx LAUNCHER_ONE;
    private DcMotorEx LAUNCHER_TWO;
    private DcMotor INTAKE;

    @Override
    public void runOpMode() {
        Deflector = hardwareMap.get(Servo.class, "Deflector");

        Deflector.setPosition(0.97);

        LAUNCHER_ONE = hardwareMap.get(DcMotorEx.class, "LAUNCHER_1");
        LAUNCHER_TWO = hardwareMap.get(DcMotorEx.class, "LAUNCHER_2");


        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");

        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");

        FL_MOTOR.setDirection(DcMotor.Direction.REVERSE);
        BL_MOTOR.setDirection(DcMotor.Direction.REVERSE);
        FR_MOTOR.setDirection(DcMotor.Direction.FORWARD);
        BR_MOTOR.setDirection(DcMotor.Direction.FORWARD);

        FEEDER = hardwareMap.get(DcMotorEx.class, "FEEDER");

        LAUNCHER_ONE.setDirection(DcMotorEx.Direction.REVERSE);
        FEEDER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo rgbOne = hardwareMap.get(Servo.class, "RGB_ONE");
        Servo rgbTwo = hardwareMap.get(Servo.class, "RGB_TWO");
        shootUtil = new Shoot(FEEDER, LAUNCHER_ONE, LAUNCHER_TWO, INTAKE,Deflector);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        AimController aimController = new AimController(limelight, imu,1);
        RGBIndicator rgb1 = new RGBIndicator(rgbOne);
        RGBIndicator rgb2 = new RGBIndicator(rgbTwo);
        PIDCounterforce aimPID = new PIDCounterforce(0.75, 0.5, 0);
        aimPID.setSetPoint(0.0);

        autoDrive = new AutoDrive(FL_MOTOR, FR_MOTOR, BL_MOTOR, BR_MOTOR, "CW");
        limelight.start();
        autoDrive.initalSetup();
        waitForStart();
//        shootUtil.sleep(10000);
        shootUtil.sleep(1000);
        shootUtil.sleep(2000);
        rgb2.setColor("red");
        rgb1.setColor("red");
        autoAim(aimController,aimPID);
        shootUtil.pewPewPew(aimController);
        rgb2.setColor("blue");
        rgb1.setColor("blue");
        autoDrive.turn("CWW", 200, 0.5);
        autoDrive.strafe(1200,.5);

        rgb2.setColor("green");
        shootUtil.sleep(500);

    }
    public void autoAim(AimController aimC,PIDCounterforce aimPID){
        double[] result = aimC.refreshPosition();
        telemetry.addData("MMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMMM", result[1] + " ");
        telemetry.update();
        while (Math.abs(result[1])>1.7){
            result = aimC.refreshPosition();
            telemetry.addData("aim", Math.abs(result[1]) + " ");
            telemetry.update();
            autoDrive.turn("CCW", 255, -aimPID.update(aimC.recalcualateYaw()));
            if (Math.abs(result[1])<1.7){
                break;
            }
        }
    }


}