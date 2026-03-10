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

import java.util.List;

//@TeleOp(name = "aimalign")

public class aimalign extends OpMode {

    private Limelight3A limelight;

    private aimassist aim = new aimassist();

    double[] incre = {0.1,0.01,0.001,0.0001,0.00001};
    int increIndex = 2;


    @Override
    public void init(){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.
         */
        limelight.start();
        aim.init(hardwareMap);

        telemetry.addLine("All Systems Ready");
    }

    public void start(){
        aim.resetTimer();
    }

    @Override
    public void loop(){
        LLResult result = limelight.getLatestResult();
        //AprilTagDetection id24 = result.getTx();

        aim.update(limelight.getLatestResult());

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

        if (result != null) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial

                telemetry.addData("ID TAG", + id);
            }
        }
        else {
            telemetry.addLine("No Tags");
        }


        telemetry.addLine("---------------------------------");
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", aim.getkP());
        telemetry.addData("Tuning D", "%.5f (D-Pad U/D", aim.getkD());
        telemetry.addData("Incre Size", "%.5f (B Button)", incre[increIndex]);


    }


}
