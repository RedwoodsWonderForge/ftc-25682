package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "BlueGoalShoot")
public class BlueGoal extends LinearOpMode {
    LeaveGoal leaveGoal;
    Shoot shootUtil;
    private DcMotor FL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor BR_MOTOR;
    private CRServo RIGHT;
    private CRServo LEFT;
    private Servo Deflector;
    private DcMotorEx LAUNCHER_ONE;
    private DcMotorEx LAUNCHER_TWO;
    private DcMotor INTAKE;

    @Override
    public void runOpMode() {
        Deflector = hardwareMap.get(Servo.class, "Deflector");

        LAUNCHER_ONE = hardwareMap.get(DcMotorEx.class, "LAUNCHER_1");
        LAUNCHER_TWO = hardwareMap.get(DcMotorEx.class, "LAUNCHER_2");

        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");

        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");

        RIGHT = hardwareMap.get(CRServo.class, "RIGHT");
        LEFT = hardwareMap.get(CRServo.class, "LEFT");
        LAUNCHER_ONE.setDirection(DcMotorEx.Direction.REVERSE);
        shootUtil = new Shoot(LEFT, RIGHT, LAUNCHER_ONE, LAUNCHER_TWO, INTAKE);



        leaveGoal = new LeaveGoal(FL_MOTOR, FR_MOTOR, BL_MOTOR, BR_MOTOR, "CW");
        leaveGoal.initalSetup();
        waitForStart();
        shootUtil.sleep(10000);
        shootUtil.prepareMotor();
        shootUtil.sleep(5000);
        leaveGoal.backward(200);
        shootUtil.shootThreeArtifacts();
        leaveGoal.turn("CW");
        leaveGoal.backward(300);
    }


}