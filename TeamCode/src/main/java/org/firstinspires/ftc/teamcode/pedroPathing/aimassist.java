package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;




public class aimassist {

    private DcMotorEx aim;
    private GoBildaPinpointDriver pinpoint;

    private final double TICKS_PER_DEGREE = 1.0655;


    private double kP = 0.01;

    private double kD = 0.0001;

    private final double MIN_TURRET_ANGLE = -150.0;
    private final double MAX_TURRET_ANGLE = 150.0;



    private double lasterror = 0;

    private double goalTolerance = 2.0;

    private final double maxPower = 0.8;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {

        aim = hwMap.get(DcMotorEx.class, "aim");

        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aim.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        aim.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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
    public double getCurrentAngle() {
        return aim.getCurrentPosition() / TICKS_PER_DEGREE;
    }



    public void update(Pose2D pos, double goalX, double goalY) {

        double deltaTime = timer.seconds();
        timer.reset();
        if (deltaTime <= 0) deltaTime = 0.001;

        double dx = goalX - pos.getX(DistanceUnit.INCH);
        double dy = goalY - pos.getY(DistanceUnit.INCH);
        double robotHeading = pos.getHeading(AngleUnit.DEGREES);

        double targetAngleField = Math.toDegrees(Math.atan2(dy, dx));
        double targetTurretRelative = AngleUnit.normalizeDegrees(targetAngleField - robotHeading);
        double currentTurretAngle = aim.getCurrentPosition() / TICKS_PER_DEGREE;


        double error = AngleUnit.normalizeDegrees(targetTurretRelative - currentTurretAngle);


        double pTerm = error * kP;
        double dTerm = ((error - lasterror) / deltaTime) * kD;
        double power = Range.clip(pTerm + dTerm, -0.8, 0.8);


        dTerm = (deltaTime > 0) ? (error - lasterror) * kD : 0;
        if (Math.abs(error) < 1.5) { // 1.5 degree tolerance
            aim.setPower(0);
        } else {
            aim.setPower(power);
        }

        lasterror = error;

    }

}
