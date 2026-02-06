package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AimController {
    private Limelight3A Limelight;
    private IMU Imu;
    private double MAX_DRIVE_POWER = .15;
    //CONSTRUCTOR
    public AimController(Limelight3A limelight, IMU imu){
        Limelight = limelight;
        Imu = imu;
    }
    public double recalcualateYaw(){
        return ((refreshPosition()[1] * 3) * MAX_DRIVE_POWER) /- 15;

    }
    public void start(){
        Limelight.start();
    }
    private void init() {

        Limelight.pipelineSwitch(2);
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        Imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
    public double[] refreshPosition() {
        YawPitchRollAngles orientation = Imu.getRobotYawPitchRollAngles();
        Limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = Limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
//            Pose3D botPose = llResult.getBotpose_MT2();
            return new double[] {llResult.getTa(), llResult.getTx()};
        }
        // getLimelightData
        //getTData
        //move y to range (during shooting and going to loading)

        return new double[] {0,0};
    }

    public double[] fireControlSolution() {
        double deflectAngle;
        double motorPower;
        double[] pos = refreshPosition();
         if (pos[0] < 4.0 && pos[0] > 0){
             motorPower = 22*remapRange((pos[0]),2.7,0.25,50.0,80.0);
             deflectAngle = remapRange((pos[0]),2.7,0.25,0.97,0.5);
         } else {motorPower = 0; deflectAngle = 0.97;}


        return new double[] {motorPower,deflectAngle};
    }
    public double remapRange (double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
