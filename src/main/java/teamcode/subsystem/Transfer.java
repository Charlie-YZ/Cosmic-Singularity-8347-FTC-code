// 文件路径：subsystem/Transfer.java
package teamcode.subsystem;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import teamcode.common.Constants;

public class Transfer {
    private final DcMotor motor;
    private final DigitalChannel limitSwitch1;
    private final DigitalChannel limitSwitch2;

    private long now = 0;
    private long lastTransferTime = 0;
    public int ballcontain = 0;
    private double lastToggleTime = 0;
    private boolean running = false;//在running的时候才进去
    private boolean dpad_left = false;
    private int targetTicks = 0;
    private int lastTicks = 0;
    private int count = 1;
    public boolean buttonready = false;
    // 新增：防堵转相关
    private long pushStartTime = 0;             // 记录开始推送时间
    private static final long PUSH_TIMEOUT_MS = 1000;  // 超时时间（1.5秒，根据实际测试调）
    private int lastEncoderPosition = 0;        // 上次编码器位置（用于检测卡住）
    private boolean isJammed = false;           // 是否检测到堵转
    private int jamRetryCount = 0;              // 重试次数（最多重试 3 次）
    private static final int MAX_JAM_RETRY = 3; // 最大重试次数
    public Transfer(DcMotor motor, DigitalChannel limitSwitch1,DigitalChannel limitSwitch2, boolean dpad_left) {//传入 transfer电机 和 transfer开关
        this.motor = motor;
        this.limitSwitch1 = limitSwitch1;
        this.limitSwitch2 = limitSwitch2;
        this.dpad_left = dpad_left;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // 只在初始化时归零一次
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//刹车并清零
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//刹车急停
    }

    public void pushOnce() {
        if (running){return;}
        if (!dpad_left){
            running = true;
            lastTicks = targetTicks;
            if(count % 5== 0){
                targetTicks -= 284;
            }
            else{
                targetTicks -= 283;
            }
            motor.setTargetPosition(targetTicks);  // 相对增量！
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(1);   // 稍微加大一点功率，防止惯性不足
            count ++;
            ballcontain++;
            running = false;
        }
        else{
            running = true;
            //motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            if(count % 5== 0){
                targetTicks -= 284;
            }
            else{
                targetTicks -= 283;
            }
            motor.setTargetPosition(0);
            motor.setTargetPosition(targetTicks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(0.5);
            count ++;
            ballcontain++;
            running = false;
        }

    }
    public void pushtest() {
        for(int i = 0; i<= 30; i++){
            pushOnce();
            for(int f = 0; f<= 50; f++){
                update();
            }
        }
    }
    public void antipushOnce() {
        if (running) return;
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(-Constants.TRANSFER_TICKS);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(Constants.TRANSFER_POWER);
        running = true;
    }

    public void update() {
        if (running) {
            if (!motor.isBusy()) { // run好了关 running设否
                motor.setPower(0.2); // 保持一点力防回退
                running = false;
            }
        }
    }
    public void transferBall(boolean dpad_right, boolean shooted) {
        now = System.currentTimeMillis();
        if ((!shooted && ballcontain <= 2)||dpad_right){
            if(dpad_right || ((isLimit1Pressed() || isLimit2Pressed()) && (now - lastTransferTime >= 300))) {
                update();
                pushOnce();
                buttonready = false;
                lastTransferTime = now;
            }
        }

        if (ballcontain > 2){
            if(isLimit1Pressed() || isLimit2Pressed()) {
                ballcontain = 3;
            }
        }
        if (shooted){
            ballcontain = 0;
        }

        if (limitSwitch2.getState()){
            buttonready = true;
        }


    }
    public boolean isLimit1Pressed() {
        return !limitSwitch1.getState();   // 低电平触发
    }
    public boolean isLimit2Pressed() {
        return !limitSwitch2.getState();   // 低电平触发
    }


    public boolean isRunning() { return running; }
}