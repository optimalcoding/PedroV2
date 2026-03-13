package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous

public class BlueFar extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Robot robot = new Robot();

    final double LAUNCHER_VEL = 1675;
    final double FEEDER_VEL = -5000;
    final double INTAKE_VEL = -1250;
    final double TURRET_MAX = 0.6;
    final double TURRET_MIN = 0.2;

    private final Pose startPose = new Pose(56.53664596273291, 8.178881987577643, Math.toRadians(90));
    private final Pose scorePose = new Pose(61.71428571428571, 13.237267080745347, Math.toRadians(120));
    private final Pose wait1 = new Pose(47.93913043478261,35.39627329192547,Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(15.21242236024844,35.39627329192547,Math.toRadians(180));
/*    private final Pose wait2 = new Pose(96.7304347826087,59.19999999999996,Math.toRadians(0));
    private final Pose pickup2Pose = new Pose(121.43229813664597,59.19999999999996,Math.toRadians(0));
    private final Pose wait3 = new Pose(47.90570719602978,36.13151364764264, Math.toRadians(0));
    private final Pose pickup3Pose = new Pose(129.15155279503105,36.13151364764264,Math.toRadians(0));
    */
    private final Pose leavePose = new Pose(45.97267080745341,13.237267080745347,Math.toRadians(120));

    private Path scorePreload;

    private PathChain row_one_move, getp1, scorep1, goleave;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        row_one_move = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, wait1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), wait1.getHeading())
                .build();

        getp1 = follower.pathBuilder()
                .addPath(new BezierLine(wait1, pickup1Pose))
                .setLinearHeadingInterpolation(wait1.getHeading(), pickup1Pose.getHeading())

                .build();

        scorep1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();   

        goleave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Preload
                follower.followPath(scorePreload);
                setPathState(1);
                break;

            case 1: // Scoring Preload
                if (!follower.isBusy()) {
                    runLaunchRoutine(2);
                }
                break;

            case 2: // Move to Wait1 (Approach)
                follower.followPath(row_one_move);
                setPathState(3);
                break;

            case 3: // Intake + Move to Pickup1
                if (!follower.isBusy()) {
                    robot.intake.setVelocity(INTAKE_VEL);
                    robot.feeder.setVelocity(FEEDER_VEL);
                    follower.followPath(getp1);
                    setPathState(4);
                }
                break;

            case 4: // Dwell at Pickup1
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.setVelocity(0);
                        robot.feeder.setVelocity(0);
                        follower.followPath(scorep1);
                        setPathState(5);
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 5: // Scoring Pickup 1
                if (!follower.isBusy()) {
                    runLaunchRoutine(6);
                }
                break;

            
            case 6: // Park
                if (!follower.isBusy()) {
                    follower.followPath(goleave);
                    setPathState(-1);
                }
                break;
        }
    }

    private void runLaunchRoutine(int nextState) {
        double t = pathTimer.getElapsedTimeSeconds();
        if (t < 0.1) {
            robot.turret.setPosition(0.6);
            robot.launcher.setVelocity(-100);
            robot.feeder.setVelocity(-FEEDER_VEL);
        } else if (t < 0.3) {
            robot.launcher.setVelocity(0);
            robot.feeder.setVelocity(0);
        } else if (t < 1.3) { // 1.0s Sleep
            robot.launcher.setVelocity(LAUNCHER_VEL);
        } else if (t < 1.8) { // 0.5s Sleep
            robot.feeder.setVelocity(0);
        } else if (t < 2.0) { // 0.2s Sleep
            robot.intake.setVelocity(INTAKE_VEL);
        } else if (t < 3.8) { // 1.8s Sleep
            robot.feeder.setVelocity(FEEDER_VEL);
        } else {
            // End of routine
            robot.launcher.setVelocity(0);
            robot.intake.setVelocity(0);
            robot.feeder.setVelocity(0);
            setPathState(nextState);
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }





    @Override
    public void init() {

        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        robot.init(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);


    }

    public void loop(){

        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();

    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}
    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        PoseStorage.currentPose = follower.getPose();
        telemetry.addData("Auto Status", "Complete. Pose Saved.");
        telemetry.update();
    }

}
