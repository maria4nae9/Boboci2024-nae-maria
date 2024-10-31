package org.firstinspires.ftc.teamcode.drive.opmodetele;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;


import org.firstinspires.ftc.teamcode.drive.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")

public class LinearDriveMode extends LinearOpMode {
    private Robot robot = null;
    int direction = 1; 
    //variabila direction ia valoarea initiala integer 1
    double servoPosSlides = 0.5;
    //variabila servoPosSlides este initialiata cu val 0.5 ,se foloseste tipul de date double, pt ca este nevoie de decimal point si un range mare de valori
    double servoPosGrippy = 0;
    //la fel ca mai sus
    public double calculateThrottle(float x) {
        int sign = -1;
        //variabila sign este initializata cu valoarea integer -1
        if (x > 0) sign = 1;
        //este pusa conditia ca x sa fie mai mare ca 0 iar daca aceasta este true variabila sign ia val 1;
        return sign * 3 * abs(x);
        //method ul public double calculateThrottle al class ului LinearDriveMode returneaza rezultatul inmultirii dintre variabila sign, 3 si abscisa lui x
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");
        telemetry.update();

        robot = new Robot(hardwareMap);
        while (robot.isInitialize() && opModeIsActive()) {
            idle();
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData(">", "Initialized");
        telemetry.update();



        waitForStart();
        if (isStopRequested()) return;


        while (opModeIsActive()) {



            if (gamepad2.left_bumper) {
                //gamepad2 reprezinta object ul
                //gamepad este un class
                //left_bumper este o proprietate a lui gamepad2 si este de tip boolean pt ca returneaza true sau false
                //daca se verifica conditia variabila de tip reference robot.crane.slidesDirection ia val 1
                robot.crane.slidesDirection = 1;
                robot.crane.setSlides(5);
                //setSlides este un membru al objectului robot
                if(robot.crane.slideEncoderLastPosition > robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension -= 3.3;
                }
            } else if (gamepad2.left_bumper)  {
                robot.crane.slidesDirection = -1;
                //ia valoarea -1;
                robot.crane.setSlides(5);
                if(robot.crane.slideEncoderLastPosition < robot.crane.slideEncoder.getVoltage()){
                    //este pusa conditi daca encoderLastPosition este mai mic decat slideEncoder
                    robot.crane.slideExtension += 3.3;
                }
            } else {
                //daca niciuna dintre conditiile cu gamepad2.left_bumper si gamepad2.left_bumper nu este valabila se indeplineste aceasta
               robot.crane.setSlides(0);
            }
            robot.crane.slideEncoderLastPosition = robot.crane.slideEncoder.getVoltage();
            //i se atribuie valoarea


            if(gamepad2.left_trigger > 0.1){
                //se verifica daca este mai mare ca 0.1
                robot.crane.craneTarget -= (int) calculateThrottle(gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger > 0.1){
                robot.crane.craneTarget += (int) calculateThrottle(gamepad2.right_trigger);
            }
            robot.crane.motorCrane1.setPower(robot.crane.cranePower(robot.crane.craneTarget));
            robot.crane.motorCrane2.setPower(robot.crane.cranePower(robot.crane.craneTarget));

            if (gamepad2.a) {
                robot.crane.gripperDirection = 1;
                robot.crane.setGripper(1);
            }
            else if (gamepad2.b) {
                robot.crane.gripperDirection = -1;
                robot.crane.setGripper(1);
            }
            else robot.crane.setGripper(0);

            robot.drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y),(-gamepad1.left_stick_x),(-gamepad1.right_stick_x)));








            telemetry.addData("crane target: ", robot.crane.craneTarget);
                telemetry.addData("right trigger: ", gamepad2.right_trigger);
                telemetry.addData("encoder value: ", robot.crane.slideEncoder.getVoltage());
                telemetry.addData("last position ", robot.crane.slideEncoderLastPosition);
                telemetry.addData("slide extension ", robot.crane.slideExtension);
                telemetry.addData("sensor touch: ", robot.crane.slideSensor.isPressed());
//                telemetry.addData("CRANE TICKS LEFT: ", robot.crane.motorCraneLeft.getCurrentPosition());
//                telemetry.addData("CRANE TICKS RIGHT: ", robot.crane.motorCraneRight.getCurrentPosition());
//                telemetry.addData("DIRECTION: ", direction);
//                telemetry.addData("SERVO GRIPPER: ", robot.crane.servoGrippy1.getPosition());
                telemetry.update();
            }

        }

    }



