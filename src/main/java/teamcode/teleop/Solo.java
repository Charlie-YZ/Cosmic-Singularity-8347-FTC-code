package teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import teamcode.subsystem.Drivetrain;
import teamcode.subsystem.Intake;
import teamcode.subsystem.Shooter;
import teamcode.subsystem.Transfer;
import teamcode.subsystem.Vision;


@TeleOp(name = "Solo")
public class Solo extends LinearOpMode {
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
    private long lastToggleTime1 = 0;
    private long lastToggleTime2 = 0;
    public int remainingShots = 0;
    public double shooterPower = 0;
    public void autoShootBall(){
        shooter.setPower(Math.max(shooterPower,0.40));
        shooter.openGate();
        intake.stop();
        sleep(300);
        //autoAlign()
        while(remainingShots > 0){
            transfer.pushOnce();
            //transfer.update();
            if(!transfer.isRunning()){
                remainingShots--;
                sleep(10);
            }
            if (remainingShots==3){
                shooter.setPower(shooterPower + 0.11);}
            if (remainingShots==2){
                shooter.setPower(shooterPower+ 0.20);}
        }
        sleep(400);
        intakeON = true;
        shooter.closeGate();
        shooter.setPower(0.20);
        shooterON = false;
    }
    public void waitForBeep(){
        //DriverStation.getInstance().beep();
    }

    public void autoAlign(){
        double alignPower = vision.getAlignPower();
        while(alignPower > 0.0){
            alignPower = vision.getAlignPower();
            drivetrain.autoRotatePower = alignPower;
        }
    }

    @Override
    public void runOpMode() {
        // 硬件映射
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
                gamepad1.dpad_left);

        vision   = new Vision(hardwareMap.get(Limelight3A.class, "limelight"));

        shooter.closeGate();
        telemetry.addData(">", "INITIALIZED! Press 'Start' button to start");
        telemetry.update();

        waitForStart();

        // 底盘线程
        new Thread(() -> {
            while (opModeIsActive()) {
                drivetrain.teleopDrive(
                        gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        !gamepad1.dpad_up
                );
                sleep(20);
            }
        }).start();

        //transfer线程
        new Thread(() -> {
            while (opModeIsActive()) {
                transfer.transferBall(
                        gamepad1.dpad_right,
                        shooterON
                );
                sleep(20);
            }
        }).start();
        // 射击+视觉主线程
        while (opModeIsActive()) {
            // X 切换射击模式
            if(transfer.ballcontain >= 3){
                shooter.setPower(shooterPower);
                intakeON = false;
                waitForBeep();
            }
            boolean currentRB = gamepad1.right_bumper;
            boolean currentLB = gamepad1.left_bumper;
            if (gamepad1.x){
                shooterPower = 0.505;
                shooter.setPower(shooterPower);
                //intakeON = false;
            }else if (gamepad1.y){
                shooterPower = 0.45;
                shooter.setPower(shooterPower);
                //intakeON = false;
            }else if (gamepad1.right_trigger >= 0.5){
                shooter.setPower(0.0);
                autoAlign();
                //drivetrain.autoRotatePower = vision.getAlignPower();
            }else if (gamepad1.right_trigger < 0.5){
                drivetrain.autoRotatePower = 0.0;
            }

            if (currentRB && !lastRBState/* 防抖 */) {
                long now1 = System.currentTimeMillis();
                if (now1 - lastToggleTime1 >= DEBOUNCE_TIME) {
                    shooterON = !shooterON;
                    lastToggleTime1 = now1;
                }
            }
            lastRBState = currentRB;

            if (currentLB && !lastLBState /* 防抖 */) {
                long now2 = System.currentTimeMillis();
                if (now2 - lastToggleTime2 >= DEBOUNCE_TIME) {
                    intakeON = !intakeON;
                    lastToggleTime2 = now2;
                }
            }
            lastLBState = currentLB;

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
                remainingShots = 4;//后面可通过transfer赋值
                autoShootBall();
                shooterON = false;
            } else {
                //shooter.stop();
            }
            if (gamepad1.b) drivetrain.resetIMU();
            /*if (gamepad2.dpad_up){
                vision.kp += 0.001;
                sleep(200);
            }else if (gamepad2.dpad_down){
                vision.kp -= 0.001;
                sleep(200);
            }
            if (gamepad2.dpad_left){
                vision.kd += 0.0001;
                sleep(200);
            }else if (gamepad2.dpad_right){
                vision.kd -= 0.0001;
                sleep(200);
            }*/

            telemetry.addData("ShootingMode", shooterON);
            telemetry.addData("IntakingMode",intakeON);
            telemetry.addData("TransferAuto",transferAuto);

            telemetry.addData("Tag22 Visible", vision.isTag22Visible());
            telemetry.addData("Tag22 Tx", vision.getTxRaw());
            telemetry.addData("AlignPower", vision.getAlignPower());
            telemetry.addData("kp", vision.kp);
            telemetry.addData("kd", vision.kd);
            telemetry.update();
            sleep(10);
        }
    }
}