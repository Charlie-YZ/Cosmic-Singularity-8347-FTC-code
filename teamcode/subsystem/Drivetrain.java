package teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.concurrent.locks.ReentrantLock;

public class Drivetrain {
    private final DcMotor fl, fr, bl, br;//四个轮子的映射 final表不可改变
    private final IMU imu;
    private final ReentrantLock lock = new ReentrantLock();
    public volatile double autoRotatePower = 0.0;

    public Drivetrain(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, IMU imu) {
        this.fl = fl; this.fr = fr; this.bl = bl; this.br = br;//把外部的通过Drivetrain(fl,fr,bl,br)输入进来
        this.imu = imu;

        fl.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);

        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);//刹车并清零
        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//刹车急停
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//刹车急停
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//刹车急停
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//刹车急停
        */
    }

    public void teleopDrive(double forward, double strafe, double rotate, boolean fieldCentric) {//最后一个boolean判断无头
        if (Math.abs(autoRotatePower) > 0.01) {
            rotate = autoRotatePower;//自旋设置
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);//通过yaw获得bearing偏角
        if (!fieldCentric) botHeading = 0;//如果不是无头，不需要偏角。
        double theta = Math.atan2(forward, strafe);
        double r = Math.hypot(strafe, forward);
        theta = AngleUnit.normalizeRadians(theta - botHeading);
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }
    public void drive(double forward, double right, double rotate) {
        double frontLeft  = forward + right + rotate;
        double frontRight = forward - right - rotate;
        double backRight  = forward + right - rotate;
        double backLeft   = forward - right + rotate;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeft), Math.max(Math.abs(frontRight),
                Math.max(Math.abs(backRight), Math.abs(backLeft)))));

        lock.lock();
        try {
            fl.setPower(frontLeft / max);
            fr.setPower(frontRight / max);
            bl.setPower(backLeft / max);
            br.setPower(backRight / max);
        } finally { lock.unlock(); }
    }

    public void resetIMU() { imu.resetYaw(); }
}