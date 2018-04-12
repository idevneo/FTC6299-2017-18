package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Library.MyOpMode;

@Autonomous(name = "AutoBlueLeft", group = "Linear Opmode")
public class AutoBlueLeft extends MyOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        hMap(hardwareMap);
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
            telemetry.addLine("Initializing IMU...");
            telemetry.update();
        }

        align = true;
        // Set up our telemetry dashboard
//        composeTelemetry();

        // Wait until we're told to go
        waitForStart();
        runtime.reset();
/**---------------------------------------------------------------------------------------------------------------*/
        vfValue();
        jewelKnockerBlue();

        setMotorsAll(-.25,0,0);
        sleep(1550);
        stopMotors();

        setStrafeAuto(26.25, rangeR, 0, 0);
//        rangeMoveStrafe(26.25, rangeR,0);
        sleep(350);


        setTurnAuto(-178);
        sleep(500);
        vfMovePar('b',rangeL, 1);

        rangeMovePID(6, rangeF);

        manipAuto(-.75);
        sleep(500);
        manipAuto(-.75);

//back up, push forward, back up
        setMotorsAll(-.2,0,0);
        sleep(250);
        stopMotors();

        manip.setPower(-1);
        setMotorsAll(.4,0,0);
        sleep(250);
        stopMotors();

        setMotorsAll(-.2,0,0);
        sleep(250);
        stopMotors();
        manip.setPower(0);
    }
}