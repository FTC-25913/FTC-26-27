package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class PIDController {
    private double kp;
    private double ki;
    private double kd;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private boolean firstRun = true;
    private final ElapsedTime timer = new ElapsedTime();

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public void reset(){
        integralSum = 0.0;
        lastError = 0.0;
        firstRun = true;
        timer.reset();
    }

    public double update(double target, double current){
        double dt = timer.seconds();
        timer.reset();

        if (firstRun || dt <= 0.0) {
            lastError = target-current;
            firstRun = false;
            return Range.clip(kp * lastError, -1.0, 1.0);
        }

        double error = target - current;

        double pOut = kp * error;

        integralSum += error * dt;
        integralSum = Range.clip(integralSum, -500.0, 500.0);
        double iOut = ki * integralSum;

        double derivative = (error - lastError) / dt;
        double dOut = kd * derivative;

        lastError = error;

        return Range.clip(pOut + iOut + dOut, -1.0, 1.0);

    }
}
