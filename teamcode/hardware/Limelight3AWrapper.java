package teamcode.hardware;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

public class Limelight3AWrapper {
    private final Limelight3A limelight;

    public Limelight3AWrapper(Limelight3A limelight) {
        this.limelight = limelight;
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0); // AprilTag pipeline
    }

    public boolean isTagVisible(int id) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return false;
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == id) return true;
        }
        return false;
    }

    public double getTxForTag(int id) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0.0;
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (f.getFiducialId() == id) {
                return f.getTargetXDegrees();
            }
        }
        return 0.0;
    }
}