package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Shoot {
    boolean IsShooting;
    private CRServo FeederRight;
    private CRServo FeederLeft;
    double ShootPower;
    private PIDCounterforce Launcher_One;
    private PIDCounterforce Launcher_Two;
    private DcMotorEx launcherOne;
    private DcMotorEx launcherTwo;
    private DcMotor Intake;
    public Shoot (CRServo fLeft, CRServo fRight, DcMotorEx launchMotor_1, DcMotorEx launchMotor_2, DcMotor intake){
        FeederRight = fRight;
        FeederLeft = fLeft;
        this.Intake = intake;
        ShootPower = 5.2;
        launcherOne = launchMotor_1;
        launcherTwo = launchMotor_2;
        Launcher_One = new PIDCounterforce( 0.005, 0, 0);
        Launcher_Two = new PIDCounterforce( 0.005, 0, 0);
    }
    private void feedShooter() {
        IsShooting = true;
        FeederRight.setPower(-1);
        FeederLeft.setPower(1);
        Intake.setPower(1);
        sleep(900);
        FeederRight.setPower(0);
        FeederLeft.setPower(0);
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


        Launcher_One.setSetPoint(ShootPower*22);
        Launcher_One.update(launcherTwo.getVelocity());
        Launcher_Two.setSetPoint(ShootPower*22);
        Launcher_Two.update(launcherOne.getVelocity());
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
