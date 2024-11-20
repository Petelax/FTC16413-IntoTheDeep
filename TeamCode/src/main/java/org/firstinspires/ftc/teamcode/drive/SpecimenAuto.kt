package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandScheduler
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.commands.drivebase.AlignPIDToPosition
import org.firstinspires.ftc.teamcode.commands.subsystems.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.Deposit
import org.firstinspires.ftc.teamcode.subsystems.Elevator
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.Intake
import org.firstinspires.ftc.teamcode.subsystems.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.Drawing

@Config
@Autonomous
class SpecimenAuto: OpMode() {
    private lateinit var hubs: List<LynxModule>
    private lateinit var drive: SwerveDrivetrain
    private lateinit var elevator: Elevator
    private lateinit var horizontalExtension: HorizontalExtension
    private lateinit var horizontalArm: HorizontalArm
    private lateinit var horizontalWrist: HorizontalWrist
    private lateinit var intake: Intake
    private lateinit var verticalArm: VerticalArm
    private lateinit var verticalWrist: VerticalWrist
    private lateinit var deposit: Deposit
    private lateinit var dashboard: FtcDashboard

    private var lastTime = System.nanoTime()
    //@JvmField var pose = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0))
    @JvmField var startPose = Pose2d(78.0, 6.0, Rotation2d.fromDegrees(0.0))

    override fun init() {
        telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        // this just sets the bulk reading mode for each hub
        hubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in hubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //voltage = hardwareMap.getAll(PhotonLynxVoltageSensor::class.java).iterator().next()

        drive = SwerveDrivetrain(hardwareMap, 12.0)

        elevator = Elevator(hardwareMap)
        verticalArm = VerticalArm(hardwareMap)
        verticalWrist = VerticalWrist(hardwareMap)
        deposit = Deposit(hardwareMap)

        horizontalExtension = HorizontalExtension(hardwareMap)
        horizontalArm = HorizontalArm(hardwareMap)
        horizontalWrist = HorizontalWrist(hardwareMap)
        intake = Intake(hardwareMap)

        dashboard = FtcDashboard.getInstance()

        telemetry.addLine("init")

        CommandScheduler.getInstance().schedule(
            SequentialCommandGroup(
                SequentialCommandGroup(),
                WaitCommand(10),
                //PIDToPosition(drive, pose),
                ParallelCommandGroup(
                    AlignPIDToPosition(drive, Pose2d(-6.0, -24.0, Rotation2d.fromDegrees(-45.0))),
                    SequentialCommandGroup(
                        WaitCommand(600),
                        ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.TOP-0.5),
                    )
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(drive, Pose2d(-24.0, -12.0, Rotation2d.fromDegrees(0.0))),
                    ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.BOTTOM),
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(drive, Pose2d(-6.0, -24.0, Rotation2d.fromDegrees(-45.0))),
                    SequentialCommandGroup(
                        WaitCommand(500),
                        ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.TOP-0.5),
                    )
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(drive, Pose2d(-24.0, -24.0, Rotation2d.fromDegrees(0.0))),
                    ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.BOTTOM),
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(drive, Pose2d(-6.0, -24.0, Rotation2d.fromDegrees(-45.0))),
                    SequentialCommandGroup(
                        WaitCommand(500),
                        ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.TOP-0.5),
                    )
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(drive, Pose2d(-24.0, -36.0, Rotation2d.fromDegrees(0.0))),
                    ElevatorPIDCommand(elevator, VerticalConstants.ElevatorPositions.BOTTOM),
                ),

                WaitCommand(100)
            ),
        )

    }

    override fun loop() {
        val currentTime = System.nanoTime()
        val pose = drive.getPose()
        //val deltas = drive.getDelta()
        val packet = TelemetryPacket()
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        packet.put("elevator height", elevator.getPosition())

        /*
        packet.put("lf", deltas[0])
        packet.put("rf", deltas[1])
        packet.put("lr", deltas[2])
        packet.put("rr", deltas[3])
         */

        packet.put("ms", (currentTime-lastTime)/1E6)
        packet.fieldOverlay().setStroke("#3F51B5")
        Drawing.drawRobot(packet.fieldOverlay(), pose)
        dashboard.sendTelemetryPacket(packet)
        telemetry.update()

        //drive.drive(ChassisSpeeds(1.0, 0.0, 0.0))

        CommandScheduler.getInstance().run()

        lastTime = currentTime
    }

}