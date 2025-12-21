package pedroPathing.Examples;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import pedroPathing.Constants.Constants;
import teamcode.subsystem.AutoVision;
import teamcode.subsystem.Drivetrain;
import teamcode.subsystem.Intake;
import teamcode.subsystem.Shooter;
import teamcode.subsystem.Transfer;

/// 加结束
@Autonomous
public class Blue_Front_nine_auto extends OpMode {
    ///加
    private boolean transfer1 = false;
    private int tagID = 3;
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

    public int remainingTransfer =0;
    public double shooterPower = 0;

    ///加结束
    private Follower follower;
    private Timer pathTimer, actionTimer, OpmodeTimer;
    private int pathState;

    private int remainingTurns = 0;

    public PathChain Path1;//扫tag
    public PathChain Path2;//发射
    public PathChain Path3;//到一号位
    public PathChain Path4;//吸一号球
    public PathChain Path10;//吸一号球继续
    public PathChain Path20;//吸一号球继续
    public PathChain Path5;//回发射位置，发射
    public PathChain Path6;//到二号位
    public PathChain Path7;//吸二号球
    public PathChain Path11;//吸二号球继续
    public PathChain Path21;//吸一号球继续
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

    private void goodtransfer(){
        transfer1 = true;
        intake.stop();
        sleep(500);
        //autoAlign()
        while(transfer.isRunning()){
            transfer.update();
        }
        while(remainingTransfer > 0){
            transfer.pushOnce();
            transfer.update();
            if(!transfer.isRunning()){
                remainingTransfer--;
            }
        }
        transfer1 = false;
    }
    private void autoTransfer(int position){
        if(transfer.ballcontain == 3 && tagID < 3){
            remainingTurns = position - tagID;
            if(remainingTurns<0){remainingTurns+=3;}
        }

    }


    private void buildPath() {

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-117.988, 130.265),
                                new Pose(144-107.284, 108.301),
                                new Pose(144-82.846, 110.877)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-216+360), Math.toRadians(90))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-82.846, 110.877), new Pose(144-88.703, 99.769))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180-38))
                .build();

        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-88.703, 99.769),
                                new Pose(144-89.309, 83.612),
                                new Pose(144-102.639-3, 78.056+4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-38), Math.toRadians(180))
                .build();

        Path4 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-102.639-3, 78.056+4), new Pose(144-110.000, 78.804+4))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path10 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-110.000, 78.804+4), new Pose(144-110.000-8, 78.804+4))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path20 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-110.000-8, 78.804), new Pose(144-127.076, 78.804))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-127.076, 82.804), new Pose(144-88.703, 99.769))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-38))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-88.703, 99.769),
                                new Pose(144-85.667, 55.393),
                                new Pose(144-103.937-3, 57.406+4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-38), Math.toRadians(180))
                .build();

        Path7 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-103.937-3, 51.406+4), new Pose(144-107.437, 51.751+4))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path11 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-107.437, 51.751), new Pose(144-107.437-8,51.751))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path21 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-107.437-8, 51.751), new Pose(144-127.437,51.751))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        Path8 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(144-127.549, 51.751),
                                new Pose(144-94.000, 64.000),
                                new Pose(144-88.501, 99.769)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180-38))
                .build();

        Path9 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(144-88.501, 99.769), new Pose(144-100.208, 63.471))
                )
                .setLinearHeadingInterpolation(Math.toRadians(180-38), Math.toRadians(180-270+360))
                .build();





    }



    public void autonomousPathUpdate(){
        switch (pathState){
            //扫tag，
            case 0 :
                follower.followPath(Path1);

                shooterPower = 0.43;
                shooter.setPower(shooterPower);
                intake.stop();
                setPathState(1);
                break;
            //分色
            case 1 :
                if(!follower.isBusy()){

                    autoTransfer(2);
                    remainingTransfer = remainingTurns;
                    goodtransfer();

                    follower.followPath(Path2);
                }
                if(!transfer1) {
                    setPathState(2);
                    break;
                }
            //第一次发射
            case 2 :
                shooterON = true;
                if(!follower.isBusy()){
                    remainingShots = 3;
                    autoShootBall();
                }
                if(!shooterON) {
                    setPathState(3);
                    break;
                }

            case 3 :
                if(!follower.isBusy()) {
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
                if(!follower.isBusy()&&!transfer1){
                    remainingTransfer = 1;
                    sleep(500);
                    goodtransfer();
                    follower.followPath(Path10);
                    goodtransfer();
                    setPathState(30);
                }
                break;

            case 30 :
                if(!follower.isBusy()&&!transfer1){
                    remainingTransfer = 1;
                    goodtransfer();
                    sleep(500);
                    follower.followPath(Path20);
                    goodtransfer();
                    setPathState(5);
                }
                break;

            case 5 :
                if(!follower.isBusy()){
                    shooterPower = 0.43;
                    shooter.setPower(shooterPower);
                    autoTransfer(2);
                    remainingTransfer = remainingTurns;
                    goodtransfer();
                    follower.followPath(Path5);
                }
                if(!transfer1) {
                    setPathState(6);
                    break;
                }

            case 6://第二次发射
                if(!follower.isBusy()){
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
                if(!follower.isBusy()&&!transfer1){
                    remainingTransfer = 1;
                    goodtransfer();
                    sleep(500);
                    follower.followPath(Path11);
                    setPathState(31);
                }
                break;

            case 31 :
                if(!follower.isBusy()&&!transfer1){
                    remainingTransfer = 1;
                    goodtransfer();
                    sleep(500);
                    follower.followPath(Path11);
                    setPathState(9);
                }
                break;

            case 9 :
                if(!follower.isBusy()){
                    shooterPower = 0.43;
                    shooter.setPower(shooterPower);
                    autoTransfer(1);
                    remainingTransfer = remainingTurns;
                    goodtransfer();
                    follower.followPath(Path8);
                }
                if(!transfer1) {
                    setPathState(10);
                    break;
                }

            case 10 ://第三次发射
                if(!follower.isBusy()) {
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
        follower.setStartingPose(new Pose(144-122.392, 124.409,Math.toRadians(180-216+360)));
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