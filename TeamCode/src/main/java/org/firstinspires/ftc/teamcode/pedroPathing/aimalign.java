package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.List;

@TeleOp(name = "aimalign")

public class aimalign extends OpMode {

   

    private aimassist aim = new aimassist();

    double[] incre = {0.1,0.01,0.001,0.0001,0.00001};
    int increIndex = 2;


    @Override
    public void init(){
        aim.init(hardwareMap);
        robot.pinpoint.setOffsets(0,-9.5, DistanceUnit.INCH);
        robot.pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        robot.pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        robot.pinpoint.resetPosAndIMU();

        telemetry.addLine("All Systems Ready");
    }

    public void start(){
        aim.resetTimer();
    }

    @Override
    public void loop(){

        LLResult result = limelight.getLatestResult();
        if (result != null) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double tx = result.getTx();

                telemetry.addData("ID TAG", + id);
                telemetry.addData("ID X-Axis", + tx);
            }
        }
        else {
            telemetry.addLine("No Tags");
        }
        
        Pose2D pos = robot.pinpoint.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        double targetAngle = Math.atan2(pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH))/pos.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Target Angle", targetAngle);
        
        


        if (gamepad1.bWasPressed()) {
            increIndex = (increIndex + 1) % incre.length;
        }

        if (gamepad1.dpadRightWasPressed()) {
            aim.setkP(aim.getkP() - incre[increIndex]);
        }

        if (gamepad1.dpadUpWasPressed()) {
            aim.setkD(aim.getkD() + incre[increIndex]);
        }

        if (gamepad1.dpadDownWasPressed()) {
            aim.setkD(aim.getkD() - incre[increIndex] );
        }




        telemetry.addLine("---------------------------------");
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", aim.getkP());
        telemetry.addData("Tuning D", "%.5f (D-Pad U/D", aim.getkD());
        telemetry.addData("Incre Size", "%.5f (B Button)", incre[increIndex]);


    }


}
