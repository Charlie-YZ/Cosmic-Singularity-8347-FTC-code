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
public class Red_Back_three extends OpMode {
    ///加
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
    private void goodtransfer(){

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
                        new BezierLine(new Pose(91.322, 1.675), new Pose(91.322, 18.437))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67))
                .build();

        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(91.322, 18.437), new Pose(130.096, 2.685))
                )
                .setLinearHeadingInterpolation(Math.toRadians(67), Math.toRadians(0))
                .build();
    }



    public void autonomousPathUpdate(){
        switch (pathState){
            case 0 :
                follower.followPath(Path1);
                /// 扫tag
                shooterPower = 0.60;
                shooter.setPower(shooterPower);
                intake.stop();
                sleep(1500);
                setPathState(1);
                break;

            case 1://第一次发射
                if(!follower.isBusy()){
                    remainingShots = 3;
                    autoShootBall();
                }
                if(!shooterON) {
                    setPathState(2);
                    break;
                }

            case 2 :
                if(!follower.isBusy()){
                    //autoTransfer(1);
                    follower.followPath(Path2);
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
        follower.setStartingPose(new Pose(91.322, 1.675,Math.toRadians(90)));
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