package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class aimassist {

    private DcMotorEx aim;

    private double kP = 0.0001;

    private double kD = 0.0000;

    private double goalX = 0;

    private double lasterror = 0;

    private double goalTolerance = 1.0;

    private final double maxPower = 0.8;

    private double power = 0;
    private final double MIN_TURRET_ANGLE = -150.0;
    private final double MAX_TURRET_ANGLE = 150.0;
    private final double TICKS_PER_DEGREE = 1.0655;

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

        if (IDnow.getFiducialResults().isEmpty()) {
            aim.setPower(0);
            lasterror = 0;
            return;

        }
        LLResultTypes.FiducialResult tag = IDnow.getFiducialResults().get(0);
        double tx = tag.getTx();

        double error = goalX - tx;
        double pTerm = error * kP;
        double safeTarget = Range.clip(targetTurretRelative, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
        double currentTurretAngle = aim.getCurrentPosition() / TICKS_PER_DEGREE;


        double dTerm = 0;
        if (deltaTime > 0 ) {
            dTerm = ((error-lasterror) / deltaTime) * kD;
        }

         if (Math.abs(error) < 1.5) { // 1.5 degree tolerance
            aim.setPower(0);
        } else {
            aim.setPower(power);
        }
        if (currentTurretAngle <= MIN_TURRET_ANGLE && error < 0) {
            error = 0;
        }
        if (currentTurretAngle >= MAX_TURRET_ANGLE && error > 0) {
            error = 0;
        }

        aim.setPower(power);
        lasterror = error;

    

}
