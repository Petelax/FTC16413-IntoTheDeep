package org.firstinspires.ftc.teamcode.drive

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.arcrobotics.ftclib.command.CommandOpMode
import com.arcrobotics.ftclib.command.InstantCommand
import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import com.arcrobotics.ftclib.geometry.Pose2d
import com.arcrobotics.ftclib.geometry.Rotation2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.commands.ftclib.drivebase.AlignPIDToPosition
import org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorPIDCommand
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Elevator
import org.firstinspires.ftc.teamcode.subsystems.ftclib.swerve.SwerveDrivetrain
import org.firstinspires.ftc.teamcode.utils.Drawing

@Config
@Autonomous(group = "test")
class PIDAuto: CommandOpMode() {
    private lateinit var drive: SwerveDrivetrain
    private lateinit var elevator: Elevator
    private lateinit var dashboard: FtcDashboard

    private var lastTime = System.nanoTime()
    //@JvmField var pose = Pose2d(0.0, 0.0, Rotation2d.fromDegrees(90.0))
    @JvmField var pose = Pose2d(24.0, 0.0, Rotation2d.fromDegrees(0.0))

    override fun initialize() {
        //telemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        drive = SwerveDrivetrain(hardwareMap)
        elevator = Elevator(hardwareMap)
        sleep(700)
        dashboard = FtcDashboard.getInstance()

        val packet = TelemetryPacket()
        packet.addLine("init")
        dashboard.sendTelemetryPacket(packet)

        val help = TelemetryPacket()
        help.addLine("end")

        schedule(
            SequentialCommandGroup(
                SequentialCommandGroup(),
                WaitCommand(10),
                //PIDToPosition(drive, pose),
                ParallelCommandGroup(
                    AlignPIDToPosition(
                        drive,
                        Pose2d(-6.0, -24.0, Rotation2d.fromDegrees(-45.0))
                    ),
                    SequentialCommandGroup(
                        WaitCommand(600),
                        ElevatorPIDCommand(
                            elevator,
                            VerticalConstants.ElevatorPositions.TOP - 0.5
                        ),
                    )
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(
                        drive,
                        Pose2d(-24.0, -12.0, Rotation2d.fromDegrees(0.0))
                    ),
                    ElevatorPIDCommand(
                        elevator,
                        VerticalConstants.ElevatorPositions.BOTTOM
                    ),
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(
                        drive,
                        Pose2d(-6.0, -24.0, Rotation2d.fromDegrees(-45.0))
                    ),
                    SequentialCommandGroup(
                        WaitCommand(500),
                        ElevatorPIDCommand(
                            elevator,
                            VerticalConstants.ElevatorPositions.TOP - 0.5
                        ),
                    )
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(
                        drive,
                        Pose2d(-24.0, -24.0, Rotation2d.fromDegrees(0.0))
                    ),
                    ElevatorPIDCommand(
                        elevator,
                        VerticalConstants.ElevatorPositions.BOTTOM
                    ),
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(
                        drive,
                        Pose2d(-6.0, -24.0, Rotation2d.fromDegrees(-45.0))
                    ),
                    SequentialCommandGroup(
                        WaitCommand(500),
                        ElevatorPIDCommand(
                            elevator,
                            VerticalConstants.ElevatorPositions.TOP - 0.5
                        ),
                    )
                ),
                WaitCommand(100),
                ParallelCommandGroup(
                    AlignPIDToPosition(
                        drive,
                        Pose2d(-24.0, -36.0, Rotation2d.fromDegrees(0.0))
                    ),
                    ElevatorPIDCommand(
                        elevator,
                        VerticalConstants.ElevatorPositions.BOTTOM
                    ),
                ),
                //AlignPIDToPosition(drive, Pose2d(24.0, 0.0, Rotation2d.fromDegrees(0.0))),

                //PIDToPosition(drive, Pose2d(24.0, 0.0, Rotation2d.fromDegrees(0.0))),
                //AlignModules(drive, ChassisSpeeds(1.0, 0.0, 0.0)),
                //PIDToPosition(drive, Pose2d(24.0, -24.0, Rotation2d.fromDegrees(0.0))),
                InstantCommand({dashboard.sendTelemetryPacket(help)}),
                WaitCommand(100)
            ),
        )

    }

    override fun run() {
        val currentTime = System.nanoTime()
        val pose = drive.getPose()
        val deltas = drive.getDelta()
        val packet = TelemetryPacket()
        packet.put("x", pose.x)
        packet.put("y", pose.y)
        packet.put("heading", pose.heading)
        packet.put("elevator height", elevator.getPosition())

        packet.put("lf", deltas[0])
        packet.put("rf", deltas[1])
        packet.put("lr", deltas[2])
        packet.put("rr", deltas[3])

        packet.put("ms", (currentTime-lastTime)/1E6)
        packet.fieldOverlay().setStroke("#3F51B5")
        //Drawing.drawRobot(packet.fieldOverlay(), pose)
        dashboard.sendTelemetryPacket(packet)
        telemetry.update()

        //drive.drive(ChassisSpeeds(1.0, 0.0, 0.0))

        lastTime = currentTime
        super.run()
    }

}