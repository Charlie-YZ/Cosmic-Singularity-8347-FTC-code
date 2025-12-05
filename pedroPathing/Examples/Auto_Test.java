package pedroPathing.Examples;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import teamcode.subsystem.*;

import pedroPathing.Constants.Constants;

@Autonomous
public class Auto_Test extends OpMode {
    ///加
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private Vision vision;
    private boolean lastRBState = false;
    private boolean lastLBState = false;
    private boolean shooterON = false;
    private boolean intakeON = false;
    private boolean transferAuto = true;
    private int DEBOUNCE_TIME = 200;
    public int remainingShots = 0;
    public double shooterPower = 0;
    ///加结束
    private Follower follower;
    private Timer pathTimer, actionTimer, OpmodeTimer;
    private int pathState;

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    public PathChain Path11;
    public PathChain Path12;
    public void autoShootBall(){
        shooter.setPower(Math.max(shooterPower,0.40));
        shooter.openGate();
        intake.stop();
        sleep(300);
        //autoAlign()
        while(remainingShots > 0){
            transfer.pushOnce();
            transfer.update();
            if(!transfer.isRunning()){
                remainingShots--;
            }
        }
        intakeON = true;
        shooter.closeGate();
        shooter.setPower(0.0);
        shooterON = true;
    }
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
                                new Pose(91.524, 83.666),
                                new Pose(105.660, 80.435),
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

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.169, 101.034),
                                new Pose(95.361, 55.393),
                                new Pose(114.546, 56.605)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(114.546, 56.605), new Pose(134.135, 56.201))
                )
                .setTangentHeadingInterpolation()
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.135, 56.201), new Pose(96.775, 100.630))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.775, 100.630),
                                new Pose(92.534, 47.315),
                                new Pose(114.344, 47.517)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(0))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(114.344, 47.517), new Pose(134.539, 47.517))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(134.539, 47.517), new Pose(96.169, 100.832))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(43))
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(96.169, 100.832), new Pose(106.064, 67.308))
                )
                .setLinearHeadingInterpolation(Math.toRadians(43), Math.toRadians(270))
                .build();
    }

    public void autonomousPathUpdate(){
        switch (pathState){
            case 0 :
                follower.followPath(Path1);
                setPathState(1);
                break;

            case 1 :
                if(!follower.isBusy()){
                    follower.followPath(Path2);
                    setPathState(2);

                }
                break;

            case 2 :
                if(!follower.isBusy()) {
                    /// 加
                    intakeON = true;
                    /// 加完
                    follower.followPath(Path3);
                    setPathState(3);
                }
                break;

            case 3 :
                if(!follower.isBusy()){
                    follower.followPath(Path4);
                    setPathState(4);
                }
                break;

            case 4 :
                if(!follower.isBusy()) {
                    follower.followPath(Path5);
                    setPathState(5);
                }
                break;

            case 5 :
                if(!follower.isBusy()){
                    shooterPower = 0.44;
                    shooter.setPower(shooterPower);
                    follower.followPath(Path6);
                    setPathState(6);
                }
                break;

            case 6:
                if(!follower.isBusy()){
                    intakeON = false;
                    shooterON = true;
                }
                break;

            case 7 :
                if(!follower.isBusy()) {
                    follower.followPath(Path7);
                    setPathState(7);
                }
                break;

            case 8 :
                if(!follower.isBusy()){
                    follower.followPath(Path8);
                    setPathState(8);
                }
                break;

            case 9 :
                follower.followPath(Path9);
                setPathState(9);
                break;

            case 10 :
                if(!follower.isBusy()){
                    follower.followPath(Path10);
                    setPathState(10);
                }
                break;

            case 11 :
                if(!follower.isBusy()) {
                    follower.followPath(Path11);
                    setPathState(11);
                }
                break;

            case 12 :
                if(!follower.isBusy()){
                    follower.followPath(Path12);
                    setPathState(12);
                }
                break;

            case 13 :
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

        /// 加
        if(intakeON) {//intake 开关
            if(!gamepad1.left_bumper){intake.ballintake();}
            else{intake.ballouttake();}
        } else if(gamepad1.left_bumper){
            intake.ballouttake();
        } else {
            intake.stop();
        }

        if (shooterON) {//shooter 开关
            shooter.openGate();
            remainingShots = 3;//后面可通过transfer赋值
            autoShootBall();
            shooterON = false;
        } else {
            //shooter.stop();
        }
        /// 加结束
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
        drivetrain = new Drivetrain(
                hardwareMap.get(DcMotor.class, "front_left_drive"),
                hardwareMap.get(DcMotor.class, "front_right_drive"),
                hardwareMap.get(DcMotor.class, "back_left_drive"),
                hardwareMap.get(DcMotor.class, "back_right_drive"),
                hardwareMap.get(IMU.class, "imu"));

        intake   = new Intake(hardwareMap.get(DcMotor.class, "intake_motor"),
                hardwareMap.get(Servo.class, "servo1"),
                hardwareMap.get(Servo.class, "servo2"));

        shooter  = new Shooter(hardwareMap.get(DcMotorEx.class, "shooter_motor1"),
                hardwareMap.get(DcMotorEx.class, "shooter_motor2"),
                hardwareMap.get(Servo.class, "gate_servo"));

        transfer = new Transfer(hardwareMap.get(DcMotor.class, "transfer_motor"),
                hardwareMap.get(DigitalChannel.class, "switch1"),
                hardwareMap.get(DigitalChannel.class, "switch2"),
                gamepad2.dpad_left);

        vision   = new Vision(hardwareMap.get(Limelight3A.class, "limelight"));

        shooter.closeGate();

        OpmodeTimer.resetTimer();
        setPathState(0);
    }
    @Override
    public void stop(){

    }
}
