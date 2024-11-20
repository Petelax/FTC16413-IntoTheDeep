package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Elevator
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWristCommand

class VerticalSample(private val elevator: Elevator, private val verticalArm: VerticalArm, private val verticalWrist: VerticalWrist) : SequentialCommandGroup() {
    init {
        addRequirements(elevator, verticalArm, verticalWrist)

        if (elevator.getPosition() < VerticalConstants.ElevatorPositions.ARM) {
            addCommands(
                ParallelCommandGroup(
                    org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorPIDCommand(
                        elevator,
                        VerticalConstants.ElevatorPositions.TOP
                    ),
                    SequentialCommandGroup(
                        WaitCommand(250),
                        ParallelCommandGroup(
                            VerticalArmCommand(
                                verticalArm,
                                VerticalConstants.VerticalArmPositions.SAMPLE
                            ),
                            VerticalWristCommand(
                                verticalWrist,
                                VerticalConstants.VerticalWristPositions.SAMPLE
                            )
                        )
                    )
                )
            )
        } else {
            addCommands(
                ParallelCommandGroup(
                    org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorPIDCommand(
                        elevator,
                        VerticalConstants.ElevatorPositions.TOP
                    ),
                    VerticalArmCommand(verticalArm, VerticalConstants.VerticalArmPositions.SAMPLE),
                    VerticalWristCommand(verticalWrist, VerticalConstants.VerticalWristPositions.SAMPLE)
                )
            )

        }
    }
}