package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueGoalShoot")
public class BlueGoal extends LinearOpMode {
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

        FEEDER = hardwareMap.get(DcMotorEx.class, "FEEDER");

        LAUNCHER_ONE.setDirection(DcMotorEx.Direction.REVERSE);
        FEEDER.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Servo rgbOne = hardwareMap.get(Servo.class, "RGB_ONE");
        Servo rgbTwo = hardwareMap.get(Servo.class, "RGB_TWO");
        shootUtil = new Shoot(FEEDER, LAUNCHER_ONE, LAUNCHER_TWO, INTAKE,Deflector);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        IMU imu = hardwareMap.get(IMU.class, "imu");
        AimController aimController = new AimController(limelight, imu,2);
        RGBIndicator rgb1 = new RGBIndicator(rgbOne);
        RGBIndicator rgb2 = new RGBIndicator(rgbTwo);

        autoDrive = new AutoDrive(FL_MOTOR, FR_MOTOR, BL_MOTOR, BR_MOTOR, "CW");

        autoDrive.initalSetup();
        waitForStart();
//        shootUtil.sleep(10000);
        shootUtil.sleep(1000);
        autoDrive.forward(300,-1);
        shootUtil.sleep(2000);
        rgb2.setColor("red");
        rgb1.setColor("red");
        autoAim(aimController);
        shootUtil.pewPewPew(aimController);
        rgb2.setColor("blue");
        rgb1.setColor("blue");

//        autoDrive.forward(220,-.5);
//        autoDrive.turn("CW", 230,.5);
//        shootUtil.sleep(400);
//        autoDrive.strafe(600,-.5);
//        shootUtil.startIntake();
//        autoDrive.forward(1300,.4);
//        autoDrive.forward(600,-.5);
//        shootUtil.stopIntake();
//        shootUtil.prepareMotor();
//        autoDrive.strafe(500,.5);
//        autoDrive.turn("CCW", 255,.5);
//        shootUtil.sleep(2000);
//        rgb2.setColor("red");
//        autoAim(aimController);
//        shootUtil.shootThreeArtifacts();
//        rgb2.setColor("blue");
//
//        shootUtil.stopMotor();
//        autoDrive.turn("CW", 255,.5);
//        autoDrive.strafe(900,-.5);
        rgb2.setColor("green");
    }
    public void autoAim(AimController aimC){
        double[] result = aimC.refreshPosition();
        while (Math.abs(result[1])>4){
            autoDrive.turn("CCW", 255, aimC.recalcualateYaw());
        }
    }


}