package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Elevator
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWristCommand

class VerticalSpecimenPickup(private val elevator: Elevator, private val verticalArm: VerticalArm, private val verticalWrist: VerticalWrist) : SequentialCommandGroup() {
    init {
        addRequirements(elevator, verticalArm, verticalWrist)

        addCommands(
            ParallelCommandGroup(
                org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorPIDCommand(
                    elevator,
                    VerticalConstants.ElevatorPositions.BOTTOM
                ),
                VerticalArmCommand(verticalArm, VerticalConstants.VerticalArmPositions.SPECIMEN),
                VerticalWristCommand(verticalWrist, VerticalConstants.VerticalWristPositions.SPECIMEN_PICKUP)
            )
        )
    }
}