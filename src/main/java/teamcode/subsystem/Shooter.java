package teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import teamcode.teleop.*;


public class Shooter {
    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final Servo gate;

    private double targetPower = 0.0;

    public Shooter(DcMotorEx m1, DcMotorEx m2, Servo gate) {//传入两个motor和gate
        this.motor1 = m1;
        this.motor2 = m2;
        this.gate = gate;
        motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setPower(double power) {  // 0.0 ~ 1.0 让motor跑起来
        targetPower = power;
        motor1.setPower(power);
        motor2.setPower(power);
    }

    public void openGate() {//开门
        gate.setPosition(0.80);
    }
    public void closeGate() {//关门
        gate.setPosition(0.25);
    }
    public void stop() {//急停（如需要）
        setPower(0.0);
        closeGate();
    }

    public double getPower() { return targetPower; }
}