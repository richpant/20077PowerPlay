package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ArmplusLiftCode")
public class ArmplusLiftCode extends OpMode {
    private DcMotor rightFront;
    //private DcMotor rightRear;
    private DcMotor leftFront;
    //private DcMotor leftRear;
    //private DcMotor lift;
    private DcMotor armLift;
    private DcMotor arm;
    private Servo pivot;
    private Servo claw;
    private Servo leftC;
    private Servo rightC;
    private DigitalChannel touch;
    //private Servo turretLR;
    //private Servo turretUD;
    //private CRServo tape;
    //RevBlinkinLedDriver lights;
    public static final double MID_SERVO = 0.5;
    //public static final double LIFT_UP_POWER    =  0.45 ;
    //public static final double LIFT_DOWN_POWER  = -0.45 ;
    double pivotOffset = 0.0;                  // Servo mid position
    final double pivot_SPEED = 0.02;                 // sets rate to move servo

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        //leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        //rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        pivot = hardwareMap.get(Servo.class, "pivot");
        claw = hardwareMap.get(Servo.class, "claw");
        leftC = hardwareMap.get(Servo.class, "leftC");
        rightC = hardwareMap.get(Servo.class, "rightC");
        //lift = hardwareMap.get(DcMotor.class, "lift");
        armLift = hardwareMap.get(DcMotor.class, "armLift");
        arm = hardwareMap.get(DcMotor.class, "arm");
        //turretLR = hardwareMap.get(Servo.class,"turretLR");
        //turretUD = hardwareMap.get(Servo.class,"turretUD");
        //tape = hardwareMap.get(CRServo.class, "tape");
        //lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightC.setDirection(Servo.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot.setPosition(1);
        //turretLR.setPosition(.5);
        //turretUD.setPosition(.5);
        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void loop() {
        //float lift_power = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;
        //double armPower= -gamepad1.right_stick_y;
        // Denominator65 is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        //leftRear.setPower(backLqgv eftPower);
        rightFront.setPower(frontRightPower);
        //rightRear.setPower(backRightPower);
        if (gamepad1.left_bumper) {
            leftFront.setPower(0.3 * frontLeftPower);
          //  leftRear.setPower(0.3 * backLeftPower);
            rightFront.setPower(0.3 * frontRightPower);
          //  rightRear.setPower(0.3 * backRightPower);
        }
        double left;
        double right;

        //--------leftC--------\\
        if (gamepad2.dpad_left) {
            leftC.setPosition(0);
        }
        if (gamepad2.dpad_right) {
            leftC.setPosition(1);
        }
        //--------rightC--------\\
        if (gamepad2.dpad_left) {
            rightC.setPosition(0);
        }
        if (gamepad2.dpad_right) {
            rightC.setPosition(1);
        }

        //--------claw--------\\
        if (gamepad2.dpad_left) {
            claw.setPosition(0);
        }
        if (gamepad2.dpad_right) {
            claw.setPosition(0.3);
        }
        {
            if (armLift.getPower() > .8) {
                armLift.setPower(.8);
            } else if (armLift.getPower() < -.8) {
                armLift.setPower(-.8);
            }

            /*if (gamepad1.dpad_left) {
                armLift.setPower(armLift.getPower() + .005);
            } else if (gamepad1.dpad_right) {
                armLift.setPower(armLift.getPower() - .005);
            } else {
                armLift.setPower(0);
            }
            if (gamepad1.left_bumper) {
                armLift.setPower(1);
            } else if (gamepad1.right_bumper) {
                armLift.setPower(-1);
            } else {
                armLift.setPower(0);
            }*/

            /*if (touch.getState()==true) {
                telemetry.addData("touch","is not pressed");
                //arm.setPower(arm_power);
            }*/
            /*if (touch.getState()==false){
                arm.setPower(0);
                resetRuntime();
                while(getRuntime()<5 && touch.getState()== false){
                    arm.setPower(.5);
                }
                telemetry.addData("touch", "pressed");}
            else{touch.setState(true);}*/
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            if (-gamepad1.left_stick_y > 0 && -gamepad1.right_stick_y < 0) {
                leftFront.setPower(left);
                //leftRear.setPower(left);
                rightFront.setPower(right);
                //rightRear.setPower(right);
            } else if (-gamepad1.left_stick_y < 0 && -gamepad1.right_stick_y > 0) {
                leftFront.setPower(left);
                //leftRear.setPower(left);
                rightFront.setPower(right);
                //rightRear.setPower(right);
            } else if (gamepad1.left_bumper || gamepad1.left_trigger > .9) {
                //------------NITRO---------------
                leftFront.setPower(left);
                //leftRear.setPower(left);
                rightFront.setPower(right);
                //rightRear.setPower(right);
            } else {
                leftFront.setPower(.65 * left);
                //leftRear.setPower(.65 * left);
                rightFront.setPower(.65 * right);
                //rightRear.setPower(.65 * right);
            }
            if (gamepad2.left_bumper) { //tall from behind
                move(2300);
                expand(1050);
            }
            if (gamepad2.a) { //tall
                move(1700);
                expand(1050);
            }
            if (gamepad2.b) { //medium
                move(1700);
                expand(0);
            }
            if (gamepad2.y) { //small
                move(990);
                expand(0);
            }
            if (gamepad2.x) { //ground
                move(250);
                expand(0);
            }
            if (gamepad2.right_bumper) { //bottom
                move(0);
                expand(0);
            }

            if (gamepad2.dpad_up) {
                arm.setTargetPosition(arm.getCurrentPosition() + 50);
            } else if (gamepad1.dpad_down) {
                arm.setTargetPosition(arm.getCurrentPosition() - 50);
            }

            if (gamepad2.x) {
                pivot.setPosition(pivot.getPosition() + .0025);
            } else if (gamepad1.y) {
                pivot.setPosition(pivot.getPosition() - .0025);
            }

            // arm ticks = 3895.9
            // servo ticks =
        }

        /*if (gamepad1.dpad_left) {
            turretLR.setPosition(turretLR.getPosition() - .0005);
        } else if (gamepad1.dpad_right) {
            turretLR.setPosition(turretLR.getPosition() + .0005);
        }
        if (gamepad1.dpad_up) {
            turretUD.setPosition(turretUD.getPosition() + .0005);
        } else if (gamepad1.dpad_down) {
            turretUD.setPosition(turretUD.getPosition() - .0005);
        }
        if (gamepad1.a) {
            tape.setPower(1);
        } else if (gamepad1.b) {
            tape.setPower(-1);
        } else {
            tape.setPower(0);
        }*/

        // Send telemetry message to signify robot running;
        telemetry.addData("left", "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("armEncoder", arm.getCurrentPosition());
        telemetry.addData("armLiftEncoder", armLift.getCurrentPosition());
        telemetry.addData("servo", pivot.getPosition());
        //telemetry.addData("turretLR", turretLR.getPosition());
        //telemetry.addData("turretUD", turretUD.getPosition());
    }

    //Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {
    }

    private void move(float a) {
        arm.setTargetPosition(Math.round(a));
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(0.7);
        pivot.setPosition(1 - a / 2900);
    }

    private void expand(float a) {
        armLift.setTargetPosition(Math.round(a));
        armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armLift.setPower(.5);

    }
}