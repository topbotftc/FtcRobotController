package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.threeten.bp.DayOfWeek;
import org.threeten.bp.Instant;

public class servo{
    private Servo right;
    private Servo left;
    private Servo.Direction direction;
    private Instant hardware;
    private OpMode opMode;

    public void init(OpMode opMode) {
        this.opMode = opMode;
        left = opMode.hardwareMap.get(Servo.class,"leftServo");
        right = opMode.hardwareMap.get(Servo.class,"rightServo");
        left.setDirection(Servo.Direction.REVERSE);
    }

    public void setDirection(Servo.Direction direction) {
        this.direction = direction;
    }

    public Servo.Direction getDirection() {
        return direction;
    }
}
