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
    private Servo deflecLeft;
    private DcMotor LAUNCHER;
    private Servo deflecRight;
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
        shootUtil = new Shoot(LEFT, RIGHT, LAUNCHER);



        leaveGoal = new LeaveGoal(FL_MOTOR, FR_MOTOR, BL_MOTOR, BR_MOTOR, "CCW");
        leaveGoal.initalSetup();
        waitForStart();
        shootUtil.prepareMotor();
        shootUtil.sleep(1000);
        shootUtil.shootThreeArtifacts();
        shootUtil.sleep(15000);
        leaveGoal.autoDrive();
    }


}