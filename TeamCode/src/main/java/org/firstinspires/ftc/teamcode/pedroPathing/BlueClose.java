package org.firstinspires.ftc.teamcode.pedroPathing;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous

public class BlueClose extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private Robot robot = new Robot();

    final double LAUNCHER_VEL = 1250;
    final double FEEDER_VEL = -5000;
    final double INTAKE_VEL = -1250;

    private final Pose startPose = new Pose(20.803970223325063, 121.09181141439203, Math.toRadians(135));
    private final Pose scorePose = new Pose(47.96029776674937, 95.85111662531017, Math.toRadians(135));
    private final Pose wait1 = new Pose(48.02233250620347,83.32506203473947,Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(16.08933002481389,83.63027295285362,Math.toRadians(180));
    private final Pose wait2 = new Pose(47.97766749379652,95.90570719602978,Math.toRadians(180));
    private final Pose pickup2Pose = new Pose(21.384615384615387,60.35980148883376,Math.toRadians(180));
    private final Pose wait3 = new Pose(47.90570719602978,95.7791563275434, Math.toRadians(180));
    private final Pose pickup3Pose = new Pose(16.838709677419356,36.13151364764264,Math.toRadians(180));
    private final Pose leavePose = new Pose(29.657568238213393,71.46401985111663,Math.toRadians(180));

    private Path scorePreload;

    private PathChain row_one_move, getp1, scorep1, row_two_move, getp2, scorep2, row_3_move, getp3, scorep3, goleave;

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

        row_two_move = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,wait2))
                .setLinearHeadingInterpolation(scorePose.getHeading(), wait2.getHeading())
                .build();

        getp2 = follower.pathBuilder()
                .addPath(new BezierLine(wait2,pickup2Pose))
                .setLinearHeadingInterpolation(wait2.getHeading(),pickup2Pose.getHeading())
                .build();

        scorep2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        row_3_move = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,wait3))
                .setLinearHeadingInterpolation(scorePose.getHeading(), wait3.getHeading())
                .build();

        getp3 = follower.pathBuilder()
                .addPath(new BezierLine(wait3,pickup3Pose))
                .setLinearHeadingInterpolation(wait3.getHeading(), pickup3Pose.getHeading())
                .build();

        scorep3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose,scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
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

            case 6: // Move to Wait2
                follower.followPath(row_two_move);
                setPathState(7);
                break;

            case 7: // Intake + Move to Pickup2
                if (!follower.isBusy()) {
                    robot.intake.setVelocity(INTAKE_VEL);
                    robot.feeder.setVelocity(FEEDER_VEL);
                    follower.followPath(getp2);
                    setPathState(8);
                }
                break;

            case 8: // Dwell at Pickup2
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.setVelocity(0);
                        robot.feeder.setVelocity(0);
                        follower.followPath(scorep2);
                        setPathState(9);
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 9: // Scoring Pickup 2
                if (!follower.isBusy()) {
                    runLaunchRoutine(10);
                }
                break;

            case 10: // Move to Wait3
                follower.followPath(row_3_move);
                setPathState(11);
                break;

            case 11: // Intake + Move to Pickup3
                if (!follower.isBusy()) {
                    robot.intake.setVelocity(INTAKE_VEL);
                    robot.feeder.setVelocity(FEEDER_VEL);
                    follower.followPath(getp3);
                    setPathState(12);
                }
                break;

            case 12: // Dwell at Pickup3
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                        robot.intake.setVelocity(0);
                        robot.feeder.setVelocity(0);
                        follower.followPath(scorep3);
                        setPathState(13);
                    }
                } else { pathTimer.resetTimer(); }
                break;

            case 13: // Final Score
                if (!follower.isBusy()) {
                    runLaunchRoutine(14);
                }
                break;

            case 14: // Park
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
