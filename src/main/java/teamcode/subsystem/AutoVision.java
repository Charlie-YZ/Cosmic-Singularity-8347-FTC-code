
package teamcode.subsystem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import teamcode.common.Constants;

public class AutoVision {
    private final Limelight3A limelight;
    private int TID = 3;

    public AutoVision(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.setPollRateHz(100);   // Limelight 最快 100Hz
        limelight.start();
        limelight.pipelineSwitch(1);    // pipeline 0 = AprilTag
    }

    public boolean isTagVisible() {
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
    public int getID() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getFiducialResults().isEmpty()) {
            TID = 3;
        }
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == 21 || f.getFiducialId() == 22 || f.getFiducialId() == 23) {
                TID = f.getFiducialId() - 21;
            }
        }
        return TID;
    }
}