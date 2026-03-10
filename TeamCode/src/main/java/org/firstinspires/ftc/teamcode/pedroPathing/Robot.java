
package org.firstinspires.ftc.teamcode.pedroPathing;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class Robot {


    /* Public OpMode members. */
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor frontLeft;
    public DcMotor backLeft;
    public DcMotorEx launcher;
    public DcMotorEx feeder;
    public DcMotorEx intake;
    public Servo turret;
    public DcMotorEx aim;
    // Class-level declaration
    public HuskyLens huskyLens;
    public Limelight3A limelight;
    public GoBildaPinpointDriver pinpoint;






    //continuous rotation continuous rotation CR (Like a motor 360 degree rot.
    //Reg. servo (x degrees rot. limited so 180 degree but u can set specific angle to have servo rot.)

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime runtime = new ElapsedTime();

    /* Constructor */
    public Robot() {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {


        // Save reference to Hardware map
        hwMap = ahwMap;



        // Define and Initialize Devices
        frontRight = hwMap.get(DcMotor.class, "frontRight");
        backLeft = hwMap.get(DcMotor.class, "backLeft");
        frontLeft = hwMap.get(DcMotor.class, "frontLeft");
        backRight = hwMap.get(DcMotor.class, "backRight");
        launcher = hwMap.get(DcMotorEx.class, "launcher");
        feeder = hwMap.get(DcMotorEx.class, "feeder");
        intake = hwMap.get(DcMotorEx.class, "intake");
        turret = hwMap.get(Servo.class, "turret");
        aim = hwMap.get(DcMotorEx.class, "aim");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // In your OpMode's init() or before the run loop:
        // huskyLens = hwMap.get(HuskyLens.class, "huskylens");

        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // feeder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        aim.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);

        frontLeft.setZeroPowerBehavior(BRAKE);
        frontRight.setZeroPowerBehavior(BRAKE);
        backLeft.setZeroPowerBehavior(BRAKE);
        backRight.setZeroPowerBehavior(BRAKE);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        // Setting motor directions to turn
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        if (limelight != null) {
            limelight.setPollRateHz(100);

        }

        //  huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // forward = clockwise tetrix 0.0-1.0 counter is -1.0 to -0.0
        //no need to def direction bc its reg servo

    }

    public void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}