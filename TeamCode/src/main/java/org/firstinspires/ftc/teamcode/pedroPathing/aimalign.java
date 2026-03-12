package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import java.util.Locale;

@TeleOp(name = "aimalign")
public class aimalign extends OpMode {

    private aimassist aim = new aimassist();
    private GoBildaPinpointDriver pinpoint;

    double[] incre = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int increIndex = 2;

    @Override
    public void init() {

        aim.init(hardwareMap);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(0, -9.5, DistanceUnit.INCH);
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        telemetry.addLine("All Systems Ready");
    }

    @Override
    public void start() {
        aim.resetTimer();
    }

    @Override
    public void loop() {

        // Read odometry pose
        Pose2D pos = pinpoint.getPosition();

        String data = String.format(
                Locale.US,
                "{X: %.2f in, Y: %.2f in, H: %.2f°}",
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                pos.getHeading(AngleUnit.DEGREES)
        );
        telemetry.addData("Position", data);

        // Example target angle telemetry (not used for control)
        double dx = 138 - pos.getX(DistanceUnit.INCH);
        double dy = 138 - pos.getY(DistanceUnit.INCH);
        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));
        telemetry.addData("Target Angle", "%.2f°", targetAngle);

        // Call aim assist update
        aim.update(pos, 138, 138);

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

        // Telemetry
        telemetry.addLine("---------------------------------");
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", aim.getkP());
        telemetry.addData("Tuning D", "%.5f (D-Pad U/D)", aim.getkD());
        telemetry.addData("Increment Size", "%.5f (B Button)", incre[increIndex]);
    }
}
