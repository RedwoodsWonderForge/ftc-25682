package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Shoot {
    boolean IsShooting;
    private DcMotor Feeder;
    double ShootPower;
    private PIDCounterforce Launcher_pid;
    private Servo Deflector;
    private DcMotorEx launcherOne;
    private DcMotorEx launcherTwo;
    private DcMotor Intake;
//    private AimController aimController;
    public Shoot (DcMotor feeder, DcMotorEx launchMotor_1, DcMotorEx launchMotor_2, DcMotor intake, Servo Deflector){
        Feeder = feeder;
        this.Deflector = Deflector;
        this.Intake = intake;
        ShootPower = 5.2;
        launcherOne = launchMotor_1;
        launcherTwo = launchMotor_2;
        Launcher_pid = new PIDCounterforce( 0.003, 0, 0);

    }
//    private void feedShooter() {
//        IsShooting = true;
//        Feeder.setPower(-1);
//        Intake.setPower(1);
//        sleep(900);
//        Feeder.setPower(0);
//        Intake.setPower(0);
//        IsShooting = false;
//    }

    public void pewPewPew(AimController aimController){
        double pidPower = 0;
        long startTime = currentTimeMillis();
        //aim before pew
        //start launcher
//        launcherTwo.setPower(1);
//        launcherOne.setPower(1);
//        sleep(1000);
        boolean isShooting = false;

        while (currentTimeMillis() - startTime < 5000) {
            double[] solution;
            aimController.refreshPosition();
            solution = aimController.fireControlSolution();
            double launchPower = solution[0];
            double deflecPos = solution[1];
            Launcher_pid.setSetPoint(launchPower);
            pidPower = Math.max(Launcher_pid.update(launcherOne.getVelocity()), -0.1);
            Deflector.setPosition(deflecPos);
            launcherTwo.setPower(pidPower);
            launcherOne.setPower(pidPower);
            if (!isShooting && currentTimeMillis() - startTime > 2000 ) {

                Intake.setPower(1);
                Feeder.setPower(-1);
                isShooting = true;
            }


        }

        Intake.setPower(0);
        Feeder.setPower(0);
        launcherTwo.setPower(0);
        launcherOne.setPower(0);

    }
//    public void shootThreeArtifacts() {
//
//        int NArtifacts;
//        NArtifacts = 3;
//        while (NArtifacts > 0) {
//            if (!IsShooting) {
//                feedShooter();
//                NArtifacts --;
//                sleep(500);
//            }
//        }
//    }
//    public void prepareMotor(){
//        double[] solution;
//        aimController.refreshPosition();
//        solution = aimController.fireControlSolution();
//        launchPower = solution[0];
//        deflecPos = solution[1];
//
//        double pidPower = Math.max(Launcher_pid.update(launcherOne.getVelocity()), -0.1);
//        launcherTwo.setPower(pidPower);
//        launcherOne.setPower(pidPower);
//    }
//    public void stopMotor(){
//        22auncher_One.stop();
//        Launcher_Two.stop();
//    }
    public void startIntake(){
        Intake.setPower(1);
    }
    public void stopIntake(){
        Intake.setPower(0);
    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds); // Pause for 'milliseconds'
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore the interrupted status
        }
    }
}
