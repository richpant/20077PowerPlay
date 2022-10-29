package org.firstinspires.ftc.teamcode; //.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import java.text.DecimalFormat;
//import com.qualcomm.robotcore.util.Range;

//https://github.com/FIRST-Tech-Challenge/FtcRobotController/tree/master/FtcRobotController
//https://github.com/geomancer79/Tutorial_Ultimate_Goal/blob/master/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/DriverRelativeControls.java

@TeleOp(name = "Drive")
public class Drive extends OpMode {
    private DcMotor leftFront;  // 0
    private DcMotor leftRear;   // 2
    private DcMotor rightFront; // 1
    private DcMotor rightRear;  // 3
    private DcMotor lift;   // 0
    private Servo claw;         // s4
    private Servo claw2;         // s5
    private DigitalChannel touch;
    //\\//\\//\\//\\//\\//\\//\\//\\//\\
    //private DcMotorEx lift;

    //@Override
    public void runOpMode() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Encoder value", lift.getCurrentPosition());

        telemetry.update();

    }


    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class,"lift");

        claw = hardwareMap.get(Servo.class,"claw");
        claw2 = hardwareMap.get(Servo.class,"claw2");

        touch = hardwareMap.get(DigitalChannel.class,"touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);



        telemetry.addData("Say", "Hello Driver");

    }
    @Override
    public void loop() {

        //MECHANUM DRIVE
        float lift_power = -gamepad2.left_stick_y;
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        double armPower= -gamepad2.right_stick_y;
        // Denominator65 is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
       // FOR PRECISION DRIVING
        if (gamepad1.left_bumper) {
            leftFront.setPower(0.3 * frontLeftPower);
            leftRear.setPower(0.3 * backLeftPower);
            rightFront.setPower(0.3 * frontRightPower);
            rightRear.setPower(0.3 * backRightPower);
        }


        // BUTTON TO MAKE LIFT NOT GO TOO FAR DOWN AND WIND STRING ALL CRAZY
        if (touch.getState()==true) {
            telemetry.addData("touch","is not pressed");
            lift.setPower(lift_power);
        }
        if (touch.getState()==false){
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setPower(0);
            resetRuntime();
            while(getRuntime()<5 && touch.getState()== false){
                lift.setPower(.5);
            }

            telemetry.addData("touch", "pressed");}
        else{touch.setState(true);}
        
        //--------claw--------\\
        if (gamepad2.b) {
            claw.setPosition(.5);// think claw 1 and 2 need to be inverted
            claw2.setPosition(1);
        } if (gamepad2.y) {
            claw.setPosition(1); //the larger the number, the more it closes | for claw (0.285).
            claw2.setPosition(.5);
        }

    }

}
