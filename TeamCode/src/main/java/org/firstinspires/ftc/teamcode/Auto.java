package org.firstinspires.ftc.teamcode; //.OpModes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class Auto extends LinearOpMode {
    private DcMotor leftFront;
    private DcMotor leftRear;
    private DcMotor rightFront;
    private DcMotor rightRear;
  //  private Servo intake;
    private DcMotor lift;
    private BNO055IMU       imu         = null;      // Control/Expansion Hub IMU


    @Override

    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftRear = hardwareMap.get(DcMotor.class, "leftRear");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightRear = hardwareMap.get(DcMotor.class, "rightRear");
        //intake = hardwareMap.get(Servo.class,"intake");
        lift = hardwareMap.get(DcMotor.class,"lift");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
        //intake.setDirection(DcMotorSimple.Direction.REVERSE);
        //lift.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
           /* if (gamepad1.a) {
                resetRuntime();
                leftFront.setPower(1);
                sleep(500);
                leftFront.setPower(0);
                telemetry.addData("Left Front", "moving");
                telemetry.update();
            }
            if (gamepad1.b) {
                leftRear.setPower(1);
                sleep(500);
                leftRear.setPower(0);
                telemetry.addData("Left Rear", "moving");
                telemetry.update();
            }
            if (gamepad1.x) {
                resetRuntime();
                rightFront.setPower(1);
                sleep(500);
                rightFront.setPower(0);
                telemetry.addData("Right Front", "moving");
                telemetry.update();
            }
            if (gamepad1.y) {
                rightRear.setPower(1);
                sleep(500);
                rightRear.setPower(0);
                telemetry.addData("Right Rear", "moving");
                telemetry.update();
            }*/
            move(50, 50, -50, -50);
            telemetry.addData("leftFRont", leftFront.getCurrentPosition());
            move(1000, 1000, 1000, 1000);
            move(0, 0, 0, 0);
            lift(100);
            lift(0);


        }

    }


    //----------------------------encoder-----------------
    public void move(int rf, int rb, int lf, int lb) {
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setTargetPosition(rf);
        rightRear.setTargetPosition(rb);
        leftFront.setTargetPosition(lf);
        leftRear.setTargetPosition(lb);

        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightRear.setPower(0.3);
        rightFront.setPower(0.3);

        leftFront.setPower(0.3);
        leftRear.setPower(0.3);

        while (leftFront.isBusy() && leftRear.isBusy() && rightFront.isBusy() && rightRear.isBusy()) {
            sleep(25);

        }
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightRear.setPower(0);
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftRear.setPower(0);


    }
    //===============================lift====================




    //----------------------Lift------------
    public void lift(int encod) {
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setTargetPosition(encod);

        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(0.6);
        while (lift.isBusy()) {
            sleep(50);
        }
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lift.setPower(0);

    }

}

