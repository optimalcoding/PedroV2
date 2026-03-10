package org.firstinspires.ftc.teamcode.pedroPathing;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class aimassist {

    private DcMotorEx aim;

    private double kP = 0.0001;

    private double kD = 0.0000;

    private double goalX = 0;

    private double lasterror = 0;

    private double goalTolerance = 0.2;

    private final double maxPower = 0.8;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        aim = hwMap.get(DcMotorEx.class, "aim");

        aim.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newkP) {
        kP = newkP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newkD) {
        kD = newkD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }


    public void update(LLResult IDnow) {
        double deltaTime = timer.seconds();
        timer.reset();

        if (IDnow == null) {
            aim.setPower(0);
            lasterror = 0;
            return;

        }

        double error = goalX - IDnow.getTx();
        double pTerm = error * kP;


        double dTerm = 0;
        if (deltaTime > 0 ) {
            dTerm = (error-lasterror) * kD;
        }

        if (Math.abs(error) < goalTolerance) {
            power = 0;

        }
        else {
            power = Range.clip(pTerm + dTerm, -maxPower, maxPower);
        }

        aim.setPower(power);
        lasterror = error;

    }








}
