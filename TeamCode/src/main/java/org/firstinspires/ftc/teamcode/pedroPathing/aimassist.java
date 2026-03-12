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
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;



import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class aimassist {

    private DcMotorEx aim;

    private double kP = 0.0001;

    private double kD = 0.0000;

    double goalX = 138;
    double goalY = 138;

    private double lasterror = 0;

    private double goalTolerance = 0.2;

    private final double maxPower = 0.8;

    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        aim = hwMap.get(DcMotorEx.class, "aim");
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        robot.pinpoint.setOffsets(0,-9.5, DistanceUnit.INCH);
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();

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

    Pose2D pos = robot.pinpoint.getPosition();
    
    public void update(Pose2D pos, double goalX, double goalY) {
        
        double deltaTime = timer.seconds();
        timer.reset();
        
        double dx = goalX - pos.getX(DistanceUnit.INCH);
        double dy = goalY - pos.getY(DistanceUnit.INCH);
        double rbootHeading = pos.getHeading(AngleUnits.Degrees);

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
