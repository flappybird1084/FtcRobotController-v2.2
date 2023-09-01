package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Basic program that does teleop. This is supposed to be a function-based,
 * modular program.
 */

@TeleOp(name = "Omni Op Test Platform", group= "Linear Opmode")
public class TeleOpDynamicPlatform extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    private ElapsedTime runtime = new ElapsedTime();
    double additionalYaw = 0;
    double leftYawCoolDown = runtime.seconds();
    double rightYawCoolDown = runtime.seconds();


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // initalizes

        waitForStart();
        robot.init(hardwareMap);
        runtime.reset();
        //starts after initialization (press start)

        // run until stop is pressed
        while (opModeIsActive()) {
            double max; // get top wheel speed

            double regularSpeed = 0.5;
            double superSpeed = 1;
            //superspeed is used only when there's a stretch
            // of ground to be covered, normally not
            // to be used during matches

            double axial   = -gamepad1.left_stick_y * regularSpeed;  // Note: pushing stick forward gives negative value
            double lateral =  gamepad1.left_stick_x * regularSpeed;
            double yaw     =  gamepad1.right_stick_x* regularSpeed;

            if(gamepad1.left_bumper){
                axial   *=  superSpeed/regularSpeed; //recalculating all values, least year's method was less elegant
                lateral *=  superSpeed/regularSpeed;
                yaw     *=  superSpeed/regularSpeed;
            }

            double leftFrontPower  = (axial + lateral + yaw);
            double rightFrontPower = (axial - lateral - yaw);
            double leftBackPower   = (axial - lateral + yaw);
            double rightBackPower  = (axial + lateral - yaw);
            // movement algorithm

            double right_trig = gamepad1.right_trigger;
            double left_trig  = gamepad1.left_trigger;
            // for strafing

            double avgMotorPower = (leftBackPower+leftFrontPower+rightBackPower+rightFrontPower)/4;
            // additionalyaw resource. we need to turn proportional to average speed of bot, and while this isn't perfect it works


            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            // calculate top wheel speed

            if (gamepad1.dpad_left && (runtime.seconds()-leftYawCoolDown)>1){
                additionalYaw-=0.01;
                leftYawCoolDown = runtime.seconds();
            }
            // need a cooldown for additionalyaw or it'll keep adding

            if (gamepad1.dpad_right && (runtime.seconds()-rightYawCoolDown)>1){
                additionalYaw+=0.01;
                rightYawCoolDown = runtime.seconds();
            }

            if(gamepad1.right_trigger > 0){
                leftFrontPower = right_trig;
                leftBackPower = -right_trig;
                rightFrontPower = -right_trig;
                rightBackPower = right_trig;
            }

            if(gamepad1.left_trigger > 0){
                leftFrontPower = -left_trig;
                leftBackPower = left_trig;
                rightFrontPower = left_trig;
                rightBackPower = -left_trig;
            }

            telemetry.addData("vector to degree test: ",robot.vectorToDegrees(axial,lateral));
            // i believe this is calculating angle of the robot relative to starting point
        }
    }
}
