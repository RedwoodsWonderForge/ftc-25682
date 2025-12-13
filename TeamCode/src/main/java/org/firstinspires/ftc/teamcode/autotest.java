package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "autotest (Blocks to Java)")
@Disabled
public class autotest extends LinearOpMode {

  private DcMotor RIGHT_MOTOR;
  private DcMotor LEFT_MOTOR;
  private DcMotor ARM_MOTOR;
  private Servo CLAW_LEFT;
  private Servo CLAW_RIGHT;

  /**
   * This sample contains the bare minimum Blocks for any regular OpMode. The 3 blue
   * Comment Blocks show where to place Initialization code (runs once, after touching the
   * DS INIT button, and before touching the DS Start arrow), Run code (runs once, after
   * touching Start), and Loop code (runs repeatedly while the OpMode is active, namely not
   * Stopped).
   */
  @Override
  public void runOpMode() {
    RIGHT_MOTOR = hardwareMap.get(DcMotor.class, "RIGHT_MOTOR");
    LEFT_MOTOR = hardwareMap.get(DcMotor.class, "LEFT_MOTOR");
    ARM_MOTOR = hardwareMap.get(DcMotor.class, "ARM_MOTOR");
    CLAW_LEFT = hardwareMap.get(Servo.class, "CLAW_LEFT");
    CLAW_RIGHT = hardwareMap.get(Servo.class, "CLAW_RIGHT");

    // Put initialization blocks here.
    RIGHT_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LEFT_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ARM_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ARM_MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LEFT_MOTOR.setDirection(DcMotor.Direction.REVERSE);
    CLAW_LEFT.setDirection(Servo.Direction.REVERSE);
    // drops block without setting claw position
    CLAW_LEFT.setPosition(-0.2);
    CLAW_RIGHT.setPosition(-0.2);
    waitForStart();
    telemetry.addData("right claw", CLAW_RIGHT.getPosition());
    telemetry.addData("left claw", CLAW_LEFT.getPosition());
    telemetry.addLine("===INITED===");
    if (opModeIsActive()) {
      telemetry.addLine("===OP MODE GOOOOO===");
      telemetry.update();
      // start at wall move toward submersible
      RIGHT_MOTOR.setTargetPosition(800);
      LEFT_MOTOR.setTargetPosition(800);
      RIGHT_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      LEFT_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      RIGHT_MOTOR.setPower(0.5);
      LEFT_MOTOR.setPower(0.5);
      while (opModeIsActive() && LEFT_MOTOR.isBusy() && RIGHT_MOTOR.isBusy()) {
        telemetry.addLine("get current position");
        // Put loop blocks here.
        telemetry.update();
      }
      telemetry.addLine("turnign");
      telemetry.update();
      // move arm up
      ARM_MOTOR.setTargetPosition(-500);
      ARM_MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      ARM_MOTOR.setPower(1);
      while (opModeIsActive() && ARM_MOTOR.isBusy()) {
        telemetry.addLine("turning");
        // Put loop blocks here.
        telemetry.update();
      }
    }
    while (opModeInInit()) {
      telemetry.update();
    }
  }
}
