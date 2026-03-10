package org.firstinspires.ftc.teamcode.pedroPathing;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


import java.util.Arrays;
import java.util.List;

@TeleOp(name = "Teleop")
public class Teleop extends LinearOpMode {




    final double FEED_TIME_SECONDS = 0.20;

    Robot robot = new Robot();
    int robotCycle = 0;



    double LAUNCHER_TARGET_VELOCITY = 1400;

    final double LAUNCHER_2ND = 1675;

    final double LAUNCHER_MIN_VELOCITY = 1400;

    final double FEEDER_TARGET_VELOCITY = 5000;
    final double FEEDER_MIN_VELOCITY = 750;
    final double LAUNCHER_REVERSE = -400;
    final double FEEDER_REVERSE = -1000;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0;
    final double goalX = 138;
    final double goalY = 138;

    private ElapsedTime reverseTimer = new ElapsedTime();
    private boolean isReversing = false;
    private final double REVERSE_DURATION = 0.450; // 450 milliseconds



    @Override
    public void runOpMode() throws InterruptedException {


        //initialization variables, notifying robot is initialized and shows how long robot ran for
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Runtime " + robot.runtime.toString());
        //telemetry.update();


        robot.init(hardwareMap);


        telemetry.setMsTransmissionInterval(11);

        robot.limelight.pipelineSwitch(0);

        robot.limelight.start();



        robot.frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.feeder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.aim.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.frontLeft.setZeroPowerBehavior(BRAKE);
        robot.frontRight.setZeroPowerBehavior(BRAKE);
        robot.backLeft.setZeroPowerBehavior(BRAKE);
        robot.backRight.setZeroPowerBehavior(BRAKE);


        robot.launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.launcher.setZeroPowerBehavior(BRAKE);
        robot.launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        waitForStart();



        while (opModeIsActive()) {
            LLStatus status = robot.limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());



            LLResult result = robot.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(), fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }



            telemetry.addData("Robot Cycle", robotCycle);
            //  telemetry.addData("Arm Encoder Value", robot.lift.getCurrentPosition());
            telemetry.addData("Match Time (s)", getRuntime());
            telemetry.addData("FL Count", robot.frontLeft.getCurrentPosition());
            telemetry.addData("FR Count", robot.frontRight.getCurrentPosition());
            telemetry.addData("BL Count", robot.backLeft.getCurrentPosition());
            telemetry.addData("BR Count", robot.backRight.getCurrentPosition());

            telemetry.addData("Status", "Resetting Values");




            //Powerplay controller configs for reference to centerstage indirect drive
            double FrontLeftVal = -gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x);
            double FrontRightVal = -gamepad1.left_stick_y - (gamepad1.left_stick_x) - (gamepad1.right_stick_x);
            double BackLeftVal = -gamepad1.left_stick_y - (gamepad1.left_stick_x) + (gamepad1.right_stick_x);
            double BackRightVal = -gamepad1.left_stick_y + (gamepad1.left_stick_x) - (gamepad1.right_stick_x);


            // change orientation bc going forward is backwards
            //Move range to between 0 and +1, if not already
            double[] wheelPowers = {FrontRightVal, FrontLeftVal, BackLeftVal, BackRightVal};
            Arrays.sort(wheelPowers);
            if (wheelPowers[3] > 1) {
                FrontLeftVal /= wheelPowers[3];
                FrontRightVal /= wheelPowers[3];
                BackRightVal /= wheelPowers[3];
                BackLeftVal /= wheelPowers[3];

            }

            robot.frontLeft.setPower(FrontLeftVal * 0.6);
            robot.frontRight.setPower(FrontRightVal * 0.6);
            robot.backLeft.setPower(BackLeftVal * 0.6);
            robot.backRight.setPower(BackRightVal * 0.6);
            robot.intake.setPower(gamepad2.left_stick_y);
            robot.aim.setPower(gamepad2.right_stick_y*0.3);


/*
            if (gamepad2.y) {
                robot.launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);


            } else if (gamepad2.b) { // stop flywheel
                robot.launcher.setVelocity(STOP_SPEED);

            }

       */

            if (gamepad2.y) {
                launchSequence();
            }
            else {
                robot.launcher.setVelocity(STOP_SPEED);
                robot.feeder.setVelocity(STOP_SPEED);
            }

            if (gamepad2.x) {
                robot.feeder.setVelocity(FEEDER_TARGET_VELOCITY);
            }
            else if (gamepad2.a) {
                robot.feeder.setVelocity(STOP_SPEED);
            }

           if (gamepad2.dpad_up) {
                Reverse(450);
            }
           else {
               robot.feeder.setVelocity(STOP_SPEED);
               robot.launcher.setVelocity(STOP_SPEED);
           }



            if (gamepad1.dpad_up) {
                robot.turret.setPosition(0.6);//raises hood

            }
            else if (gamepad1.dpad_down) {
                robot.turret.setPosition(0.2);//lowers hood

            }

            if (gamepad1.a){
                LAUNCHER_TARGET_VELOCITY = LAUNCHER_2ND;
            }
            if (gamepad1.b){
                LAUNCHER_TARGET_VELOCITY = LAUNCHER_MIN_VELOCITY;
            }

            telemetry.addData("feedmotor_speed", robot.feeder.getCurrentPosition());
            telemetry.addData("launcher_speed", robot.launcher.getVelocity());
            telemetry.update();

        }
    }
    void Reverse(int milliseconds) {
        robot.launcher.setVelocity(LAUNCHER_REVERSE);
        robot.feeder.setVelocity(FEEDER_REVERSE);
        sleep(milliseconds);
        robot.launcher.setVelocity(STOP_SPEED);
        robot.feeder.setVelocity(STOP_SPEED);
    }

    void launchSequence() {

        robot.launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
        // Only feed if wheels are at 95% speed
        if (robot.launcher.getVelocity() >= (LAUNCHER_TARGET_VELOCITY * 0.95)) {
            robot.feeder.setVelocity(FEEDER_TARGET_VELOCITY);
        } else {
            robot.feeder.setVelocity(0);
        }
    }
}