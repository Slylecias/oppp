package org.firstinspires.ftc.teamcode.Auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous
public class LebronJames extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-36, -72, Math.PI / 2);
        Pose2d initialPose2 = new Pose2d(-58, -60, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Pivot pivot = new Pivot(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Extends extend = new Extends(hardwareMap);


        // actionBuilder builds from the drive steps passed to it
        TrajectoryActionBuilder KendrickLamar = drive.actionBuilder(initialPose)
                .lineToY(-59)
                .setTangent(0)
                .lineToXLinearHeading(-57, (Math.PI) / 4);

        TrajectoryActionBuilder Weezer = drive.actionBuilder(initialPose2)
                .lineToY(-50)
                .setTangent(0)
                .lineToXLinearHeading(-38, (Math.PI) / 4);


        Action trajectoryActionCloseOut = KendrickLamar.endTrajectory().fresh()
                .build();


        waitForStart();
        if (isStopRequested()) return;


        Action trajectoryActionChosen;
        Action trajectoryActionChosen2;
        trajectoryActionChosen = KendrickLamar.build();
        trajectoryActionChosen2 = Weezer.build();


        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        pivot.PivotUp(),
                        extend.extend(),
                        claw.open(),
                        pivot.PivotDown(),
                        trajectoryActionChosen2,
                        trajectoryActionCloseOut
                )
        );
    }


    public class Pivot {
        private DcMotor lift;

        public Pivot(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotor.class, "AM");
        }

        public class PivotUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            public void MoveArm() {
                lift.setPower(-0.7);
                sleep(1100);
            }

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(-0.7);
                    initialized = true;
                }


                MoveArm();
                return false;

//                // checks lift's current position
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos < 2500) {
//                    // true causes the action to rerun
//                    return true;
//                } else {
//                    // false stops action rerun
//                    lift.setPower(0);
//                    return false;
//                }
//                // overall, the action powers the lift until it surpasses
//                // 3000 encoder ticks, then powers it off
//            }
            }

        }

        public Action PivotUp() {
            return new PivotUp();
        }

        public class PivotDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(0.7);
                    initialized = true;
                }

                sleep(900);
                lift.setPower(0);
                return false;
//                double pos = lift.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos > 1700) {
//                    return true;
//                } else {
//                    lift.setPower(0);
//                    return false;
//                }
            }
        }

        public Action PivotDown() {
            return new PivotDown();
        }
    }
    public class Claw{
        public Servo LS;
        public Servo RS;

        public DcMotor AM;
        public Claw(HardwareMap hardwareMap){
            LS = hardwareMap.get(Servo.class, "LS");
            RS = hardwareMap.get(Servo.class, "RS");

        }

        public class clawOpen implements Action{
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                LS.setPosition(0.1);
                RS.setPosition(0);
                return true;
            }
        }
        public Action open(){
            return new clawOpen();
        }

    }

    public class Extends{
        public DcMotor Ringo;
        public Extends(HardwareMap hardwareMap){
            Ringo = hardwareMap.get(DcMotor.class, "Ringo");
        }

        public class armExtends implements Action{

            public void MoveArm() {
                Ringo.setPower(-1);
                sleep(2700);
            }
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                MoveArm();
                return false;
            }
        }
        public Action extend(){
            return new armExtends();
        }

    }
}