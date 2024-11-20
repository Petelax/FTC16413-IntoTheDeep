package org.firstinspires.ftc.teamcode.commands.ftclib.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.ftclib.HorizontalWristCommand

class HorizontalRetract(private val horizontalExtension: HorizontalExtension, private val horizontalArm: HorizontalArm, private val horizontalWrist: HorizontalWrist) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalExtension, horizontalArm)

        addCommands(
            ParallelCommandGroup(
                HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.IN),
                org.firstinspires.ftc.teamcode.commands.ftclib.subsystems.HorizontalExtensionPIDCommand(
                    horizontalExtension,
                    HorizontalConstants.HorizontalExtensionPositions.BOTTOM
                ),
                HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.IN)
            )
        )
    }
}