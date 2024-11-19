package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWrist
import org.firstinspires.ftc.teamcode.subsystems.HorizontalWristCommand

class HorizontalRetract(private val horizontalExtension: HorizontalExtension, private val horizontalArm: HorizontalArm, private val horizontalWrist: HorizontalWrist) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalExtension, horizontalArm)

        addCommands(
            ParallelCommandGroup(
                HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.IN),
                HorizontalExtensionPIDCommand(horizontalExtension, HorizontalConstants.HorizontalExtensionPositions.BOTTOM),
                HorizontalWristCommand(horizontalWrist, HorizontalConstants.HorizontalWristPositions.IN)
            )
        )
    }
}