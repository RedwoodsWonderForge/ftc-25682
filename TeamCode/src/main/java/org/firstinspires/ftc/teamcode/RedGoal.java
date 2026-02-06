package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "RedGoalShoot")
public class RedGoal extends LinearOpMode {
    LeaveGoal leaveGoal;
    Shoot shootUtil;
    private DcMotor FL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor BR_MOTOR;
    private CRServo RIGHT;
    private CRServo LEFT;
    private Servo Deflector;
    private DcMotor LAUNCHER_ONE;
    private DcMotor LAUNCHER_TWO;
    private DcMotor INTAKE;

    @Override
    public void runOpMode() {
        Deflector = hardwareMap.get(Servo.class, "Deflector");

        LAUNCHER_ONE = hardwareMap.get(DcMotor.class, "LAUNCHER_ONE");
        LAUNCHER_TWO = hardwareMap.get(DcMotor.class, "LAUNCHER_TWO");

        INTAKE = hardwareMap.get(DcMotor.class, "INTAKE");

        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");

        RIGHT = hardwareMap.get(CRServo.class, "RIGHT");
        LEFT = hardwareMap.get(CRServo.class, "LEFT");
        shootUtil = new Shoot(LEFT, RIGHT, LAUNCHER_ONE, LAUNCHER_TWO, INTAKE);



        leaveGoal = new LeaveGoal(FL_MOTOR, FR_MOTOR, BL_MOTOR, BR_MOTOR, "CW");
        leaveGoal.initalSetup();
        waitForStart();
        shootUtil.prepareMotor();
        shootUtil.sleep(15000);
        leaveGoal.backward();
        shootUtil.shootThreeArtifacts();
        leaveGoal.turn("CCW");
        leaveGoal.backward();
    }


}