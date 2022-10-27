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
   // private DcMotor armLift;  // 1
   // private DcMotor arm;  // 2
    private Servo claw;         // s0
    private Servo claw2;         // s2
    //private Servo stick;        // s1
    private Servo pivot;
    private DigitalChannel touch;
    //\\//\\//\\//\\//\\//\\//\\//\\//\\
    //private DcMotorEx lift;

    //@Override
    public void runOpMode() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        //armLift = hardwareMap.get(DcMotorEx.class,"armLift");
       // arm = hardwareMap.get(DcMotorEx.class,"arm");
        //bucket = hardwareMap.get(Servo.class,"bucket");
        //flipper = hardwareMap.get(CRServo.class, "flipper");
        //arm.setDirection(DcMotorSimple.Direction.REVERSE);
        // arm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Encoder value", lift.getCurrentPosition());
        //telemetry.addData("Encoder value", armLift.getCurrentPosition());
       // telemetry.addData("Encoder value", arm.getCurrentPosition());
        //telemetry.addData("Encoder value", arm2.getCurrentPosition());
        //telemetry.addData("servoPosiont", bucket.getPosition());

        telemetry.update();

    }


    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        lift = hardwareMap.get(DcMotor.class,"lift");
        //armLift = hardwareMap.get(DcMotor.class,"armLift");
        //arm = hardwareMap.get(DcMotor.class,"arm");
        claw = hardwareMap.get(Servo.class,"claw");
        claw2 = hardwareMap.get(Servo.class,"claw2");
       // stick = hardwareMap.get(Servo.class,"stick");
        touch = hardwareMap.get(DigitalChannel.class,"touch");
        touch.setMode(DigitalChannel.Mode.INPUT);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        //rightRear.setDirection(DcMotor.Direction.REVERSE);
        //lift.setDirection(DcMotor.Direction.REVERSE);
        //armLift.setDirection(DcMotor.Direction.REVERSE);
        //arm.setDirection(DcMotor.Direction.REVERSE);
        claw.setDirection(Servo.Direction.REVERSE);
        //claw2.setDirection(Servo.Direction.REVERSE);
        //stick.setDirection(Servo.Direction.REVERSE);

        telemetry.addData("Say", "Hello Driver");
        telemetry.addData("encoder",leftFront.getCurrentPosition());
    }
    @Override
    public void loop() {
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
        //telemetry.addData("encoder",leftFront.getCurrentPosition());
        //leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //telemetry.update();
        if (gamepad1.left_bumper) {
            leftFront.setPower(0.3 * frontLeftPower);
            leftRear.setPower(0.3 * backLeftPower);
            rightFront.setPower(0.3 * frontRightPower);
            rightRear.setPower(0.3 * backRightPower);
        }

        //lift.setPower(gamepad1.left_trigger - gamepad1.right_trigger);
        //armLift.setPower(gamepad2.right_trigger - gamepad2.left_trigger);

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
            claw.setPosition(0);
        } if (gamepad2.y) {
            claw.setPosition(1); //the larger the number, the more it closes | for claw (0.285).
        }
        //--------claw2--------\\
        if (gamepad2.b) {
            claw2.setPosition(0);
        } if (gamepad2.y) {
            claw2.setPosition(1);
        }

        /*
        if (gamepad1.a) { //tall
            move(3700);
        }
        if (gamepad1.b) { //medium
            move(2775);
        }
        if (gamepad1.y) { //small
            move(1850);
        }
        if (gamepad1.x) { //ground
            move(500);
        }
        if (gamepad1.right_bumper) { //bottom
            move(0);
        }
        telemetry.addData("LiftEncoder", lift.getCurrentPosition());
*/
    }
    /*private void move(float a){
        lift.setTargetPosition(Math.round(a));
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setPower(0.7);
    }*/

    /* Code to run ONCE after the driver hits STOP

    @Override
    public void stop() {
    } */
}
