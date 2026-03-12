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


    private double kP = 0.01;

    private double kD = 0.0000;

    double goalX = 138;
    double goalY = 138;

    private double lasterror = 0;

    private double goalTolerance = 2.0;

    private final double maxPower = 0.8;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        
        aim = hwMap.get(DcMotorEx.class, "aim");
        
        
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0,-9.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

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

   
    
    public void update(Pose2D pos, double goalX, double goalY) {
        
        double deltaTime = timer.seconds();
        timer.reset();
        
        double dx = goalX - pos.getX(DistanceUnit.INCH);
        double dy = goalY - pos.getY(DistanceUnit.INCH);
        double robotHeading = pos.getHeading(AngleUnit.DEGREES);

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));


        double error = AngleUnit.normalizeDegrees(targetAngle - robotHeading);
        double pTerm = error * kP;


         double dTerm = (deltaTime > 0) ? (error - lasterror) * kD : 0;
         if (Math.abs(error) < goalTolerance) {
            power = 0;
                } else {
                    power = Range.clip(pTerm + dTerm, -maxPower, maxPower);
                }
        

        aim.setPower(power);
        lasterror = error;

    }

}
