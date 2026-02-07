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
    private DcMotor Intake;
    public Shoot (CRServo fLeft, CRServo fRight, DcMotorEx launchMotor_1, DcMotorEx launchMotor_2, DcMotor intake){
        FeederRight = fRight;
        FeederLeft = fLeft;
        this.Intake = intake;
        ShootPower = 5.5;
        Launcher_One = new PIDCounterforce(launchMotor_1, 0.005, 0, 0);
        Launcher_Two = new PIDCounterforce(launchMotor_2, 0.005, 0, 0);
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
        Launcher_One.setSetPoint(0);
        Launcher_One.update();
        Launcher_Two.setSetPoint(0);
        Launcher_Two.update();
    }
    public void prepareMotor(){


        Launcher_One.setSetPoint(ShootPower*22);
        Launcher_One.update();
        Launcher_Two.setSetPoint(ShootPower*22);
        Launcher_Two.update();
    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds); // Pause for 'milliseconds'
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore the interrupted status
        }
    }
}
