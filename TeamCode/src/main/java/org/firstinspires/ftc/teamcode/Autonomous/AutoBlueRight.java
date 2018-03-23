package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AutoBlueRight", group = "Linear Opmode")
public class AutoBlueRight extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        align = true;
        // Set up our telemetry dashboard
//        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        vfValue();
        jewelKnockerBlue();

        setMotors(-.4, -.4);
        sleep(1000);
        stopMotors();
        sleep(100);

       try {
           turnCorr(0.1,-85, 7000);
       } catch (InterruptedException e) {
           e.printStackTrace();
       }
        sleep(1000);


        setMotorStrafe(-.4);
        sleep(800);
        stopMotors();

        setMotorStrafe(.4);
        sleep(825);
        stopMotors();

        vfMoveAlt();

        rangeMovePID(6, rangeF);

        manipAuto(-.75);
        sleep(500);

        manipAuto(-.75);

//back up, push forward, back up
        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotors(.3, .3);
        sleep(250);
        stopMotors();

        setMotors(-.2, -.2);
        sleep(250);
        stopMotors();
        manip.setPower(0);
        }
    }