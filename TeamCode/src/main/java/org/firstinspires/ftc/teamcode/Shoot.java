package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shoot {
    boolean IsShooting;
    private DcMotor Feeder;
    double ShootPower;
    private PIDCounterforce Launcher_pid;

    private DcMotorEx launcherOne;
    private DcMotorEx launcherTwo;
    private DcMotor Intake;
    public Shoot (DcMotor feeder, DcMotorEx launchMotor_1, DcMotorEx launchMotor_2, DcMotor intake){
        Feeder = feeder;
        this.Intake = intake;
        ShootPower = 5.2;
        launcherOne = launchMotor_1;
        launcherTwo = launchMotor_2;
        Launcher_pid = new PIDCounterforce( 0.005, 0, 0);

    }
    private void feedShooter() {
        IsShooting = true;
        Feeder.setPower(-1);
        Intake.setPower(1);
        sleep(900);
        Feeder.setPower(0);
        IsShooting = false;
    }
    public void shootThreeArtifacts() {

        int NArtifacts;
        NArtifacts = 3;
        while (NArtifacts > 0) {
            if (!IsShooting) {
                feedShooter();
                NArtifacts += -1;
                sleep(500);
            }
        }
    }
    public void prepareMotor(){
        Launcher_pid.setSetPoint(ShootPower*22);
        Launcher_pid.update(launcherTwo.getVelocity());

        double pidPower = Math.max(Launcher_pid.update(launcherOne.getVelocity()), -0.1);
        launcherTwo.setPower(pidPower);
        launcherOne.setPower(pidPower);
    }
    public void stopMotor(){
        //22auncher_One.stop();
        //Launcher_Two.stop();
    }
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
