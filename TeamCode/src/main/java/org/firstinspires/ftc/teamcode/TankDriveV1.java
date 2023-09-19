package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.RamseteController;
import com.acmerobotics.roadrunner.TankKinematics;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;

public class TankDriveV1 extends LinearOpMode{
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double additionalYaw = 0;
    double leftYawCoolDown = runtime.,seconds();
    double rightYawCoolDown = runtime.seconds();
    double ViperSlideEncoderCoolDown = runtime.seconds();
    boolean servoClosed = false;

    robot.servo1.setPosition(0);
    double servo0pos = robot.servo1.getPosition();

    robot.servo1.setPosition(servo0pos+100);
    double servo100pos = robot.servo1.getPosition();
    robot.servo1.setPosition(servo0pos);

    while (opModeIsActive()){
        double axial = -gamepad1.left_stick_y*0.5;
        double lateral = gamepad1.left_stick_x*0.5;
        double yaw = gamepad1.right_stick_x*0.5;
        if (gamepad1.left_bumper){
            axial*=2;
            lateral*=2;
            yaw*=2;
        }

        double leftFrontPower = (axial+lateral+yaw);
        double rightFrontPower = (axial-lateral-yaw);
        double leftBackPower = (axial-lateral+yaw);
        double rightBackPower = (axial+lateral-yaw);
        double right_trig = gamepad1.right_trigger;
        double left_trig = gamepad1.left_trigger;
        double avgMotorPower = (leftBackPower+leftFrontPower+rightBackPower+rightFrontPower)/4;
        String VSPosition = "down";

        max = Math.max(Math.abs(leftFrontPower),Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (gamepad1.dpad_left && (runtime.second()-leftYawCoolDown)>1){
            additionalYaw -= 0.01;
            leftYawCoolDown = runtime.seconds();
        }

        if (gamepad1.dpad_right&&(runtime.seconds()-rightYawCoolDown)>1){
            additionalYaw += 0.01;
            rightYawCoolDown = runtime.seconds();
        }

        if (gamepad1.right_trigger > 0){
            leftFrontPower = right_trig;
            leftBackPower = -right_trig;
            rightFrontPower = -right_trig;
            rightBackPower = right_trig;
        }

        if (gamepad1.left_trigger > 0){
            leftFrontPower = -left_trig;
            leftBackPower = left_trig;
            rightFrontPower = left_trig;
            rightBackPower = -left_trig;
        }
    }
}
