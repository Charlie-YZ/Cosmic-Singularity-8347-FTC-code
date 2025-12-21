// 文件路径：subsystem/Vision.java   （Limelight 3A + 精准对准 Tag 22，已改为正确 PID）
package teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import teamcode.common.Constants;

public class Vision {
    private final Limelight3A limelight;

    // PID 状态变量（全部移到类成员，保持连续性）
    private double integral = 0.0;
    private double previousError = 0.0;
    private long lastUpdateTimeNs = 0;          // 用来自动算 dt

    public double kp = 0.0008;
    public double kd = 0.002;

    // 积分风积限制（建议比1小很多）
    private static final double MAX_INTEGRAL = 0.4;

    public Vision(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.setPollRateHz(100);   // Limelight 最快 100Hz
        limelight.start();
        limelight.pipelineSwitch(0);    // pipeline 0 = AprilTag
    }

    public boolean isTag22Visible() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;

        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == Constants.APRILTAG_TARGET_ID) {
                return true;
            }
        }
        return false;
    }

    /**
     * 返回用于对准 Tag 的旋转功率
     * 正值 = 向右转，负值 = 向左转，0 = 已对准或没看到
     */
    public double getAlignPower() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            resetPID();                 // 没看到 Tag 时清掉积分，防止下次爆冲
            return 0.0;
        }

        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == Constants.APRILTAG_TARGET_ID) {
                double tx = f.getTargetXDegrees();                     // 水平偏移角（度）

                // 死区判断
                if (Math.abs(tx) < Constants.ALIGN_DEADZONE_DEG) {
                    resetPID();
                    return 0.0;
                }

                // 自动计算 dt（秒）
                long now = System.nanoTime();
                double dt = (lastUpdateTimeNs == 0) ? 0.01 : (now - lastUpdateTimeNs) / 1_000_000_000.0;
                lastUpdateTimeNs = now;

                double power = calculatePID(tx, dt);

                // 最小功率 + 方向保持
                double finalPower = Math.signum(power) * Math.max(Math.abs(power), Constants.ALIGN_MIN_POWER);

                // 可选：整体功率上限，防止 PID 失控
                return Math.max(-0.6, Math.min(0.6, finalPower));
            }
        }

        resetPID();
        return 0.0;
    }

    /** 真正的 PID 计算（setpoint = 0°） */
    private double calculatePID(double error, double dt) {
        // P 项
        double P = error * kp;
        //double P = error * Constants.ALIGN_KP;

        // I 项（只在误差不太大时才累积，防止风积）
        if (Math.abs(error) < 20.0) {                 // 可根据实际情况调
            integral += error * dt;
        } else {
            // 误差太大时暂停积分，或者直接慢慢衰减也行
            integral *= 0.95;
        }
        integral = Math.max(-MAX_INTEGRAL, Math.min(MAX_INTEGRAL, integral));
        double I = integral * Constants.ALIGN_KI;

        // D 项
        double derivative = (dt < 0.001) ? 0.0 : (error - previousError) / dt;
        previousError = error;
        double D = derivative * kd;
        //double D = derivative * Constants.ALIGN_Kd;

        return (P + I + D);
    }

    /** 进入死区或丢失 Tag 时调用，防止积分残留 */
    private void resetPID() {
        integral = 0.0;
        previousError = 0.0;
        // lastUpdateTimeNs 不清，保持连续性即可
    }

    /** 原始 tx（不带任何处理），调试用 */
    public double getTxRaw() {
        float result = 0.0F;
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid() && !r.getFiducialResults().isEmpty()) {
            for (LLResultTypes.FiducialResult f : r.getFiducialResults()) {
                if (f.getFiducialId() == Constants.APRILTAG_TARGET_ID) {
                    result = (float) f.getTargetXDegrees();
                }
            }
        }else{
            result = 0.0F;
        }
        return result;
    }
}