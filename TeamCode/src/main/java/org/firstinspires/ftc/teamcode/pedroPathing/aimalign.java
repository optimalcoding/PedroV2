package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.pedropathing.geometry.Pose;

import java.util.Locale;

@TeleOp(name = "aimalign")
public class aimalign extends OpMode {

    private Follower follower;

    private aimassist aim = new aimassist();
    private GoBildaPinpointDriver pinpoint;

    double[] incre = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int increIndex = 2;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(47.424317617866,96.43672456575682,Math.toRadians(135)));

        aim.init(hardwareMap);



        telemetry.addLine("All Systems Ready");
    }

    @Override
    public void start() {
        aim.resetTimer();
    }

    @Override
    public void loop() {

        follower.update(); // Pedro refreshes the Pinpoint data

        // Get the official position from Pedro
        Pose pedroPose = follower.getPose();
        Pose2D currentPos = new Pose2D(
                DistanceUnit.INCH,
                pedroPose.getX(),
                pedroPose.getY(),
                AngleUnit.DEGREES,
                Math.toDegrees(pedroPose.getHeading())
        );

        // Call aim assist update
        aim.update(currentPos, 138, 138);

        // Tuning controls
        if (gamepad1.b) {
            increIndex = (increIndex + 1) % incre.length;
        }

        if (gamepad1.dpad_right) {
            aim.setkP(aim.getkP() + incre[increIndex]);
        }

        if (gamepad1.dpad_left) {
            aim.setkP(aim.getkP() - incre[increIndex]);
        }

        if (gamepad1.dpad_up) {
            aim.setkD(aim.getkD() + incre[increIndex]);
        }

        if (gamepad1.dpad_down) {
            aim.setkD(aim.getkD() - incre[increIndex]);
        }

        telemetry.addData("Robot Pos", String.format(Locale.US, "X: %.1f Y: %.1f H: %.1f",
                currentPos.getX(DistanceUnit.INCH),
                currentPos.getY(DistanceUnit.INCH),
                currentPos.getHeading(AngleUnit.DEGREES)));

        // Telemetry
        telemetry.addLine("---------------------------------");
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", aim.getkP());
        telemetry.addData("Tuning D", "%.5f (D-Pad U/D)", aim.getkD());
        telemetry.addData("Increment Size", "%.5f (B Button)", incre[increIndex]);
    }
}
