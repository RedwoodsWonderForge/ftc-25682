package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorEx;

public class PIDCounterforce {
    private double variable;
    private double setPoint = 0.0; // Desired position or velocity
    private double kp = 0.1; // Proportional gain
    private double ki = 0.01; // Integral gain
    private double kd = 0.05; // Derivative gain

    private double errorSum = 0.0;
    private double lastError = 0.0;

    public PIDCounterforce( double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;

    }

    public void setSetPoint(double setPoint) {
        this.setPoint = setPoint;
    }
    
   // public void stop() {
       // motor.setPower(0);
   // }

    public double update(double variable) {
        // Assuming you have a way to measure the current position or velocity
        // Calculate error
        double error = setPoint - variable;

        // Update integral error
        errorSum += error;

        // Calculate derivative error
        double derivativeError = error - lastError;
        lastError = error;

        // Calculate PID output
        double output = kp * error + ki * errorSum + kd * derivativeError *-1;

        // Apply output to the motor, limiting the power to a safe range
        double power = Math.min(Math.max(output, -1.0), 1.0);
        return power;
    }
}
