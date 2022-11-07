package org.firstinspires.ftc.teamcode;//.OpModes;


import androidx.core.app.RemoteInput;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
public class ArmAndLift extends LinearOpMode {
    DcMotorEx arm;
    DcMotorEx armLift;
    Servo claw;

    @Override
    public void runOpMode() {
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        armLift = hardwareMap.get(DcMotorEx.class,"armLift");
        claw = hardwareMap.get(Servo.class,"claw");
        //arm.setDirection(DcMotorSimple.Direction.REVERSE);
        //arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        telemetry.addData("Encoder value", arm.getCurrentPosition());
        telemetry.addData("Encoder value", armLift.getCurrentPosition());
        telemetry.addData("servoPosiont", claw.getPosition());

        telemetry.update();

        while (opModeIsActive()) {
            EncoderControl(-100, gamepad1.a); //tall
            EncoderControl(-150, gamepad1.b); //medium
            EncoderControl(-340, gamepad1.y); //small
            EncoderControl(-575,gamepad1.x);  //ground
            /*if(gamepad1.dpad_left) {
                telemetry.addData("Encoder value", arm.getCurrentPosition());
                telemetry.addData("Encoder value", armLift.getCurrentPosition());
                telemetry.update();
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setTargetPosition(100);
                armLift.setTargetPosition(100);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setPower(.5);
                armLift.setPower(.5);
                while (arm.isBusy() && armLift.isBusy()) {
                    sleep(500);
                    claw.setPosition(0.0);
                    sleep(500);
                    flipper.setPower(1);
                    sleep(500);
                    flipper.setPower(0);
                    sleep(500);
                    telemetry.addData("Encoder value", arm.getCurrentPosition());
                    telemetry.addData("Encoder value", armLift.getCurrentPosition());
                    telemetry.update();
                }
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                arm.setPower(0);
                armLift.setPower(0);
            }*/
        }
    }
    //=============================Encoder controller===============
    public void EncoderControl(int encoder,boolean button){
        if(button) {
            telemetry.addData("Encoder value", arm.getCurrentPosition());
            telemetry.addData("Encoder value", armLift.getCurrentPosition());

            telemetry.update();
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            arm.setTargetPosition(encoder);
            armLift.setTargetPosition(encoder);


            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            arm.setPower(.3);

            /*while (arm.isBusy() && armLift.isBusy()) {
                sleep(500);
                telemetry.addData("Encoder value", arm.getCurrentPosition());
                telemetry.addData("Encoder value", armLift.getCurrentPosition());
                telemetry.update();
            }*/
            /*sleep(1000);
            claw.setPosition(1.0);
            sleep(1500);
            claw.setPosition(0.0);
            sleep(500); */


            telemetry.addData("Encoder value", arm.getCurrentPosition());
            telemetry.addData("Encoder value", armLift.getCurrentPosition());


            telemetry.update();
            arm.setTargetPosition(10);
            armLift.setTargetPosition(10);


            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            arm.setPower(.4);
            armLift.setPower(.1);
            arm.setTargetPosition(0);
            armLift.setTargetPosition(0);


            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            armLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            arm.setPower(0);

            armLift.setPower(0);
        }
    }
    //==================move

}