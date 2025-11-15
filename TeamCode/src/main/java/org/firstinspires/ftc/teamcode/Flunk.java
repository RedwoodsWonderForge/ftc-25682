package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
@Autonomous
public class Flunk extends LinearOpMode {
    private DcMotor FL_MOTOR;
    private DcMotor FR_MOTOR;
    private DcMotor BL_MOTOR;
    private DcMotor BR_MOTOR;

   @Override
    public void runOpMode() {

        FL_MOTOR = hardwareMap.get(DcMotor.class, "FL_MOTOR");
        FR_MOTOR = hardwareMap.get(DcMotor.class, "FR_MOTOR");
        BL_MOTOR = hardwareMap.get(DcMotor.class, "BL_MOTOR");
        BR_MOTOR = hardwareMap.get(DcMotor.class, "BR_MOTOR");
       //
       //FL_MOTOR.set
    waitForStart();
    while (opModeIsActive()){
        move();
        sleep(300);
        Stop();
        sleep(3000000);
    }
}public void move(){
       FL_MOTOR.setPower(1);
        FR_MOTOR.setPower(1);
        BL_MOTOR.setPower(1);
        BR_MOTOR.setPower(1);

    }
    public void Stop(){
        FL_MOTOR.setPower(0);
        FR_MOTOR.setPower(0);
        BL_MOTOR.setPower(0);
        BR_MOTOR.setPower(0);

    }

}
