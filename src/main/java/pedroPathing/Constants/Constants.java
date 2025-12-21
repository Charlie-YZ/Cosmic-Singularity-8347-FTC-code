package pedroPathing.Constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {


    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14)
            .forwardZeroPowerAcceleration(-16.497280632982537)
            .lateralZeroPowerAcceleration(-72.77972692847821)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.2,0,0.019,0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8,0,0.04,0.025))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008,0,0.002,0.6,0.01))
            .centripetalScaling(0.0005)
            ;


    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            0.9,
            0.5);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorEx.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorEx.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorEx.Direction.FORWARD)
            .xVelocity(62.30580295352486)
            .yVelocity(48.023701825479826);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(99.5)
            .strafePodX(-126)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            //.customEncoderResolution(19)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)//
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
