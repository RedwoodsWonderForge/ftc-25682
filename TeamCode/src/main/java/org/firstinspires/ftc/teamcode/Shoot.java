package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Shoot {
    boolean IsShooting;
    private CRServo FeederRight;
    private CRServo FeederLeft;
    double ShootPower;
    private DcMotor Launcher;
    public Shoot (CRServo fLeft, CRServo fRight, DcMotor launchMotor){
        FeederRight = fRight;
        FeederLeft = fLeft;
        Launcher = launchMotor;
        ShootPower = .65;
    }
    private void feedShooter() {
        IsShooting = true;
        FeederRight.setPower(-1);
        FeederLeft.setPower(1);
        sleep(750);
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
            }
        }
        Launcher.setPower(0);
    }
    public void prepareMotor(){
        Launcher.setPower(ShootPower);
    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds); // Pause for 'milliseconds'
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore the interrupted status
        }
    }
}
