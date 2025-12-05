package pedroPathing.Examples;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
///加
import pedroPathing.Constants.Constants;
import teamcode.subsystem.Drivetrain;
import teamcode.subsystem.Intake;
import teamcode.subsystem.Shooter;
import teamcode.subsystem.Transfer;
import teamcode.subsystem.Vision;
import teamcode.subsystem.AutoVision;
import com.qualcomm.robotcore.hardware.configuration.LynxConstants;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import teamcode.subsystem.*;
/// 加结束
@Autonomous
public class Red_Front_nine extends OpMode {
    ///加
    private int tagID = 3;
    private boolean transfer1 = false;
    private Drivetrain drivetrain;
    private Intake intake;
    private Shooter shooter;
    private Transfer transfer;
    private AutoVision autovision;
    private boolean lastRBState = false;
    private boolean lastLBState = false;
    private boolean shooterON = true;
    private boolean intakeON = false;
    private boolean transferAuto = true;
    private int DEBOUNCE_TIME = 200;
    private boolean isStopRequested =false;
    public int remainingShots = 0;
    public double shooterPower = 0;

    private boolean readyToGo = false;
    ///加结束
    private Follower follower;
    private Timer pathTimer, actionTimer, OpmodeTimer;
    private int pathState;

    public PathChain Path1;//扫tag
    public PathChain Path2;//发射
    public PathChain Path3;//到一号位
    public PathChain Path4;//吸一号球
    public PathChain Path10;//吸一号球继续
    public PathChain Path5;//回发射位置，发射
    public PathChain Path6;//到二号位
    public PathChain Path7;//吸二号球
    public PathChain Path11;//吸二号球继续
    public PathChain Path8;//到发射位，发射
    public PathChain Path9;//结束


    private void autoShootBall(){
        shooterON = true;
        shooter.setPower(Math.max(shooterPower,0.40));
        shooter.openGate();
        intake.stop();
        sleep(500);
        //autoAlign()
        while(transfer.isRunning()){
            transfer.update();
        }
        while(remainingShots > 0){
            transfer.pushOnce();
            transfer.update();
            if(!transfer.isRunning()){
                remainingShots--;
                sleep(700);
            }
        }
        intakeON = true;
        shooter.closeGate();
        shooter.setPower(0.0);
        shooterON = false;
    }
    private void autoTransfer(int position){
        readyToGo = false;
        if(transfer.ballcontain == 3 && tagID < 3){
            int remainingTurns = position - tagID;
            if(remainingTurns<0){remainingTurns+=3;}
            while(transfer.isRunning()){
                transfer.update();
            }
            while(remainingTurns > 0){
                transfer.pushOnce();
                transfer.update();
                if(!transfer.isRunning()){
                    remainingTurns--;
                    sleep(200);
                }
            }
            readyToGo = true;
        }
        else{
            readyToGo = true;
        }
    }


    private void buildPath() {

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(117.988, 130.265),
                                new Pose(107.284, 108.301),
                                new Pose(82.846, 110.877)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(216), Math.toRadians(90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(82.846, 110.877), new Pose(88.703, 99.769))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(38))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.703, 99.769),
                                new Pose(89.309, 83.612),
                                new Pose(102.639, 78.056)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(102.639, 78.056), new Pose(110.000, 78.804))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(110.000, 78.804), new Pose(127.076, 78.804))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(127.076, 82.804), new Pose(88.703, 99.769))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(88.703, 99.769),
                                new Pose(85.667, 55.393),
                                new Pose(103.937, 57.406)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(0))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(103.937, 51.406), new Pose(107.437, 51.751))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(107.437, 51.751), new Pose(127.437,51.751))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(127.549, 51.751),
                                new Pose(94.000, 64.000),
                                new Pose(88.501, 99.769)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(38))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(88.501, 99.769), new Pose(100.208, 63.471))
                )
                .setLinearHeadingInterpolation(Math.toRadians(38), Math.toRadians(270))
                .build();





    }



    public void autonomousPathUpdate(){
        switch (pathState){
            case 0 :
                follower.followPath(Path1);
                /// 扫tag
                shooterPower = 0.43;
                shooter.setPower(shooterPower);
                intake.stop();
                setPathState(1);
                break;

            case 1 :
                if(!follower.isBusy()){
                    autoTransfer(1);
                    follower.followPath(Path2);
                    setPathState(2);
                }
                break;
            case 2://第一次发射
                shooterON = true;
                if(!follower.isBusy() && readyToGo){
                    remainingShots = 3;
                    autoShootBall();
                }
                if(!shooterON) {
                    setPathState(3);
                    break;
                }


            case 3 :
                if(!shooterON) {
                    follower.followPath(Path3);
                    setPathState(4);
                }
                break;

            case 4 :
                if(!follower.isBusy()){
                    follower.followPath(Path4);
                    setPathState(20);
                }
                break;

            case 20 :
                if(!follower.isBusy()&&transfer.ballcontain>0){
                    follower.followPath(Path10);
                    setPathState(5);
                }
                break;

            case 5 :
                if(!follower.isBusy()){
                    autoTransfer(2);
                    shooterPower = 0.43;
                    shooter.setPower(shooterPower);
                    follower.followPath(Path5);
                    setPathState(6);
                }
                break;

            case 6://第二次发射
                if(!follower.isBusy() && readyToGo == true){
                    intakeON = false;
                    shooterON = true;
                    remainingShots = 3;
                    autoShootBall();
                    setPathState(7);
                }
                break;

            case 7:
                if(!shooterON){
                    follower.followPath(Path6);
                    setPathState(8);
                }
                break;

            case 8 :
                if(!follower.isBusy()) {
                    follower.followPath(Path7);
                    setPathState(21);
                }
                break;

            case 21 :
                if(!follower.isBusy()&&transfer.ballcontain>0){
                    follower.followPath(Path11);
                    setPathState(9);
                }
                break;

            case 9 :
                if(!follower.isBusy()){
                    shooterPower = 0.43;
                    shooter.setPower(shooterPower);
                    autoTransfer(1);
                    follower.followPath(Path8);
                    setPathState(10);
                }
                break;

            case 10 ://第三次发射
                if(!follower.isBusy() && readyToGo == true) {
                    intakeON = false;
                    shooterON = true;
                    remainingShots = 3;
                    autoShootBall();
                    setPathState(11);
                }
                break;

            case 11 :
                if(!shooterON) {
                    setPathState(12);
                }
                break;

            case 12 :
                if(!follower.isBusy()) {
                    follower.followPath(Path9);
                    intake.autostop();
                    setPathState(13);
                }
                break;

            case 13:
                if(!follower.isBusy()){
                    intakeON = false;
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
            if(!gamepad1.left_bumper){intake.autoballintake();}
            else{intake.ballouttake();}
        } else if(gamepad1.left_bumper){
            intake.ballouttake();
        } else {
            intake.stop();
        }

        telemetry.addData("Path State",pathState);
        telemetry.addData("x",follower.getPose().getX());
        telemetry.addData("y",follower.getPose().getY());
        telemetry.addData("Heading",follower.getPose().getHeading());
        telemetry.addData("Tag ID",tagID+21);
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

        autovision   = new AutoVision(hardwareMap.get(Limelight3A.class, "limelight"));
    }

    @Override
    public void start(){

        OpmodeTimer.resetTimer();

        shooter.closeGate();

        setPathState(0);

        new Thread(() -> {
            while (!isStopRequested) {  // 使用 !isStopRequested() 检查 OpMode 是否活跃
                transfer.transferBall(gamepad2.dpad_right, shooterON);
                sleep(20);  // 每 20ms 休眠一次
            }
        }).start();
        new Thread(() -> {
            while (!isStopRequested) {  // 使用 !isStopRequested() 检查 OpMode 是否活跃
                tagID = Math.min(autovision.getID(),tagID);
                sleep(20);  // 每 20ms 休眠一次
            }
        }).start();
    }
    @Override
    public void stop(){
        isStopRequested = true;
    }
}