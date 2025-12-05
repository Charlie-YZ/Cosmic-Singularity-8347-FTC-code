
package teamcode.subsystem;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private final DcMotor motor;
    private final Servo servo1;   // servo1
    private final Servo servo2;   // servo2
    public boolean ballstuck = false;

    private float servoValue = 0.5f;   // 0 → 吸，1 → 吐，0.5 → 停
    private float motorPower = 0;        // 1 吸，-1 吐，0 停

    public Intake(DcMotor motor, Servo servo1, Servo servo2) {
        this.motor = motor;
        this.servo1 = servo1;
        this.servo2 = servo2;
        motor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void ballintake() {           // 吸球
        servoValue = 0f;
        motorPower = 1;
        update();
    }

    public void autoballintake() {           // 吸球
        servoValue = 0f;
        motorPower = 0.7F;
        update();
    }
    public void ballouttake() {          // 吐球
        motorPower = -1;
        update();
    }
    public void stop() {             // 完全停（安全）
        //servoValue = 0.5f;
        servoValue = 0f;
        motorPower = 0;
        update();
    }
    public void autostop() {             // 完全停（安全）
        servoValue = 0.5f;
        motorPower = 0;
        update();
    }

    public void setServoRaw(float value) {
        servoValue = value;
        update();
    }

    public void setMotorRaw(int power) {  // 1 / 0 / -1
        motorPower = power;
        update();
    }

    public void ballstuckouttake() {  // 1 / 0 / -1
        servo1.setPosition(servoValue);
        servo2.setPosition(servoValue);
        motor.setPower(-0.2);
    }

    private void update() {
        if(!ballstuck){
            servo1.setPosition(servoValue);
            servo2.setPosition(1.0f - servoValue);
            motor.setPower(motorPower);
        }else{
            servo1.setPosition(servoValue);
            servo2.setPosition(servoValue);
            motor.setPower(motorPower-0.5);
        }

    }

    public float getServoValue() { return servoValue; }
    public float getMotorPower() { return motorPower; }
}