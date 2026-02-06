package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

public class Shoot {
    boolean IsShooting;
    private CRServo FeederRight;
    private CRServo FeederLeft;
    double ShootPower;
    private DcMotor Launcher_One;
    private DcMotor Launcher_Two;
    private DcMotor Intake;
    public Shoot (CRServo fLeft, CRServo fRight, DcMotor launchMotor,DcMotor launchMotor_2, DcMotor intake){
        FeederRight = fRight;
        FeederLeft = fLeft;
        this.Intake = intake;
        Launcher_One = launchMotor;
        Launcher_Two = launchMotor_2;
        ShootPower = .45;
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
        Launcher_One.setPower(0);
        Launcher_Two.setPower(0);
    }
    public void prepareMotor(){
        Launcher_One.setPower(ShootPower);
        Launcher_Two.setPower(ShootPower);
    }
    public void sleep(int milliseconds) {
        try {
            Thread.sleep(milliseconds); // Pause for 'milliseconds'
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt(); // Restore the interrupted status
        }
    }
}
