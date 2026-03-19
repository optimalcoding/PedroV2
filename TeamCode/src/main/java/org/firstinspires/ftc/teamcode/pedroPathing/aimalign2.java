package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.List;

//@TeleOp(name = "aimalign")

public class aimalign2 extends OpMode {

    private Limelight3A limelight;

    private aimassist aim = new aimassist();

    double[] incre = {0.1,0.01,0.001,0.0001,0.00001};
    int increIndex = 2;
    Robot robot = new Robot();


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
        robot.init(hardwareMap);

        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void start(){
        aim.resetTimer();
    }

    @Override
    public void loop(){
        
        double FrontLeftVal = -gamepad1.left_stick_y + gamepad1.left_stick_x + gamepad1.right_stick_x;
        double FrontRightVal = -gamepad1.left_stick_y - gamepad1.left_stick_x - gamepad1.right_stick_x;
        double BackLeftVal = -gamepad1.left_stick_y - gamepad1.left_stick_x + gamepad1.right_stick_x;
        double BackRightVal = -gamepad1.left_stick_y + gamepad1.left_stick_x - gamepad1.right_stick_x;

        // Normalize
        double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
        Arrays.sort(wheelPowers);
        if (wheelPowers[3] > 1) {
            FrontLeftVal /= wheelPowers[3];
            FrontRightVal /= wheelPowers[3];
            BackRightVal /= wheelPowers[3];
            BackLeftVal /= wheelPowers[3];
        }

        // Apply power
        robot.frontLeft.setPower(FrontLeftVal * 0.6);
        robot.frontRight.setPower(FrontRightVal * 0.6);
        robot.backLeft.setPower(BackLeftVal * 0.6);
        robot.backRight.setPower(BackRightVal * 0.6);

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
