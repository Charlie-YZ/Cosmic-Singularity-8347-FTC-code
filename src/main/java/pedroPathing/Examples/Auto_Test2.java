package pedroPathing.Examples;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.Constants.Constants;

@Autonomous
public class Auto_Test2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, OpmodeTimer;
    private int pathState;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;


    public void buildPath() {

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(124.240, 129.459), new Pose(95.967, 101.388))
                )
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(43))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.967, 101.388),
                                new Pose(83.446, 90.483),
                                new Pose(95.159, 81.193),
                                new Pose(114.142, 80.789)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(114.142, 80.789), new Pose(133.125, 80.789))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(133.125, 80.789), new Pose(96.169, 101.034))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0 :
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1 :
                follower.followPath(Path2);
                setPathState(2);
                break;

            case 2 :
                follower.followPath(Path3);
                setPathState(3);
                break;

            case 3 :
                follower.followPath(Path4);
                setPathState(4);
                break;
            case 4 :
                if(!follower.isBusy()){
                    setPathState(-1);
                }
                break;

        }

    }

    public void setPathState(int pstate) {
        pathState = pstate;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State",pathState);
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("Heading",follower.getPose().getHeading());
        telemetry.update();
    }
    @Override
    public void init() {
        pathTimer = new Timer();
        OpmodeTimer = new Timer();
        OpmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(122.392, 124.409,Math.toRadians(216)));
        buildPath();
    }

    @Override
    public void start(){
        OpmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop(){

    }
}
