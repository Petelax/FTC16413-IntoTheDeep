package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.constants.VerticalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Deposit
import org.firstinspires.ftc.teamcode.subsystems.ftclib.DepositCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.Elevator
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.VerticalWristCommand

class VerticalRetract(private val elevator: Elevator, private val verticalArm: VerticalArm, private val verticalWrist: VerticalWrist, private val deposit: Deposit) : SequentialCommandGroup() {
    init {
        addRequirements(elevator, verticalArm, verticalWrist, deposit)

        if (elevator.getPosition() > VerticalConstants.ElevatorPositions.ARM) {
            addCommands(
                ParallelCommandGroup(
                    org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorPIDCommand(
                        elevator,
                        VerticalConstants.ElevatorPositions.LOWER_LIMIT
                    ),
                    VerticalArmCommand(verticalArm, VerticalConstants.VerticalArmPositions.INTAKE),
                    VerticalWristCommand(
                        verticalWrist,
                        VerticalConstants.VerticalWristPositions.INTAKE
                    ),
                    DepositCommand(deposit, VerticalConstants.DepositPositions.OUT)
                )
            )
        } else {
            addCommands(
                ParallelCommandGroup(
                    WaitCommand(250),
                    VerticalArmCommand(verticalArm, VerticalConstants.VerticalArmPositions.INTAKE),
                    VerticalWristCommand(
                        verticalWrist,
                        VerticalConstants.VerticalWristPositions.INTAKE
                    ),
                    DepositCommand(deposit, VerticalConstants.DepositPositions.OUT)
                ),
                org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.ElevatorPIDCommand(
                    elevator,
                    VerticalConstants.ElevatorPositions.LOWER_LIMIT
                ),
            )

        }
    }
}