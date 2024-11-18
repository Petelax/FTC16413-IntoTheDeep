package org.firstinspires.ftc.teamcode.commands.subsystems

import com.arcrobotics.ftclib.command.ParallelCommandGroup
import com.arcrobotics.ftclib.command.SequentialCommandGroup
import com.arcrobotics.ftclib.command.WaitCommand
import org.firstinspires.ftc.teamcode.constants.HorizontalConstants
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArm
import org.firstinspires.ftc.teamcode.subsystems.HorizontalArmCommand
import org.firstinspires.ftc.teamcode.subsystems.HorizontalExtension

class HorizontalArmExtend(private val horizontalExtension: HorizontalExtension, private val horizontalArm: HorizontalArm) : SequentialCommandGroup() {
    init {
        addRequirements(horizontalExtension, horizontalArm)

        addCommands(
            ParallelCommandGroup(
                HorizontalExtensionPIDCommand(horizontalExtension, HorizontalConstants.HorizontalExtensionPositions.TOP),
                SequentialCommandGroup(
                    WaitCommand(10),
                    HorizontalArmCommand(horizontalArm, HorizontalConstants.HorizontalArmPositions.OUT)
                )
            )
        )
    }
}