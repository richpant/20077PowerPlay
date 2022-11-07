package org.firstinspires.ftc.teamcode; //.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import java.text.DecimalFormat;
//import com.qualcomm.robotcore.util.Range;

//https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController
//https://github.com/geomancer79/Tutorial_Ultimate_Goal/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DriverRelativeControls.java

@TeleOp(name = "Drive") /* Alastair is the worst*/
public class Drive extends OpMode {
    private DcMotor leftFront;  // 0
    private DcMotor leftRear;   // 2
    private DcMotor rightFront; // 1
    private DcMotor rightRear;  // 3
    private DcMotor lift;   // 0 expansion hub
    private Servo claw;         // s1
    private Servo claw2;         // s5
    private DigitalChannel touch;// expansion hub digital device-port 0

    //@Override
    public void runOpMode() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Encoder value", lift.getCurrentPosition());


        telemetry.update();

    }


    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class, "lift");

        claw = hardwareMap.get(Servo.class, "claw");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        claw.setPosition(0.64);
        claw2.setPosition(0.36);

        touch = hardwareMap.get(DigitalChannel.class, "touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);



    }

    @Override
    public void loop() {
        float lift_power = -gamepad2.left_stick_y;
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double armPower = -gamepad2.right_stick_y;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(0.7 * frontLeftPower);
        leftRear.setPower(0.7 * backLeftPower);
        rightFront.setPower(0.7 * frontRightPower);
        rightRear.setPower(0.7 * backRightPower);


        if (gamepad1.left_bumper) {
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);
        }

        lift.setPower(gamepad2.left_trigger - gamepad2.right_trigger);


        if (touch.getState() == true) {
            telemetry.addData("touch", "is not pressed");
            lift.setPower(lift_power);
        }
        if (touch.getState() == false) {
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setPower(0);
            resetRuntime();
            while (getRuntime() < 5 && touch.getState() == false) {
                lift.setPower(1);
            }

            telemetry.addData("touch", "pressed");
        } else {
            touch.setState(true);
        }


        //--------claw--------\\
        if (gamepad2.b) {//open
            claw.setPosition(0.64);//white
            telemetry.addData("claw1", claw.getPosition());
            claw2.setPosition(0.36);//black
            telemetry.addData("claw2", claw.getPosition());
        }
        if (gamepad2.y) {//close
            claw.setPosition(0.495);
            claw2.setPosition(0.505);
        }


    }
}