/*
 * Copyright (c) 2024 DigitalChickenLabs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.MovingStatistics;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
 * This OpMode illustrates how to use advanced features of the DigitalChickenLabs OctoQuad Quadrature Encoder & Pulse Width Interface Module
 *
 * The OctoQuad has 8 input channels that can used to read either Quadrature Encoder signals (like with most FTC motors)
 * or Pulse-Width style Absolute Encoder inputs (eg: REV Through Bore encoder pulse width output).
 *
 * This OpMode illustrates several of the more advanced features of an OctoQuad, including Pulse Width measurement and velocity measurement.
 * For a more basic OpMode just showing how to read encoder inputs, see the SensorOctoQuad sample.
 *
 * This OpMode assumes that the OctoQuad is attached to an I2C interface named "octoquad" in the robot configuration.
 *
 * One system that requires a lot of encoder inputs is a Swerve Drive system.
 * Such a drive requires two encoders per drive module: one for position & velocity of the Drive motor/wheel, and one for position/angle of the Steering motor.
 * The Drive motor usually requires a quadrature encoder, and the Steering motor requires an Absolute encoder.
 * This quantity and combination of encoder inputs is a challenge, but a single OctoQuad could be used to read them all.
 *
 * This sample will configure an OctoQuad for a swerve drive, as follows
 *  - Configure 4 Relative Quadrature Encoder inputs and 4 Absolute Pulse-Width Encoder inputs
 *  - Configure a velocity sample interval of 25 mSec
 *  - Configure a pulse width input range of 1-1024 uSec for a REV Through Bore Encoder's Absolute Pulse output.
 *
 * An OctoSwerveDrive class will be created to initialize the OctoQuad, and manage the 4 swerve modules.
 * An OctoSwerveModule class will be created to configure and read a single swerve module.
 *
 * Wiring:
 *  The OctoQuad will be configured to accept Quadrature encoders on the first four channels and Absolute (pulse width) encoders on the last four channels.
 *
 *  The standard 4-pin to 4-pin cable can be used to connect each Driver Motor encoder to the OctoQuad. (channels 0-3)
 *
 *  A modified version of the REV 6-4 pin cable (shipped with the encoder) connects the steering encoder to the OctoQuad. (channels 4-7)
 *  To connect the Absolute position signal from a REV Thru-Bore encoder to the OctoQuad, the Provided 6-pin to 4-pin cable will need to be modified.
 *    At the 6-pin connector end, move the yellow wire from its initial pin-3 position to the ABS pin-5 position. This can be done easily
 *    by using a small flathead screwdriver to lift the small white tab holding the metal wire crimp in place and gently pulling the wire out.
 *  See the OctoSwerveDrive() constructor below for the correct wheel/channel assignment.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 *
 * Note: If you prefer, you can move the two support classes from this file, and place them in their own files.
 *       But leaving them in place is simpler for this example.
 *
 * See the sensor's product page: https://www.tindie.com/products/35114/
 */
@TeleOp()

public class JCCoctoPulse extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Connect to the OctoQuad by looking up its name in the hardwareMap.
        OctoQuad octoquad = hardwareMap.get(OctoQuad.class, "octoquad");

        // Clear out all prior settings and encoder data before setting up desired configuration
        octoquad.resetEverything();

        // Make all channels pulse width
        octoquad.setChannelBankConfig(OctoQuad.ChannelBankConfig.ALL_PULSE_WIDTH);

        // now make sure the settings persist through any power glitches.
        octoquad.saveParametersToFlash();

        // Prepare an object to hold an entire OctoQuad encoder readable register bank
        OctoQuad.EncoderDataBlock encoderDataBlock = new OctoQuad.EncoderDataBlock();
        // Set channel 6 min and max pulse width.
        // Maxbotix MB1013 is 1 mm/1 uS with a max of 3000 mm
        octoquad.setSingleChannelPulseWidthParams (6, new OctoQuad.ChannelPulseWidthParams(1,3010));

        // Display the OctoQuad firmware revision
        telemetry.addLine("OctoQuad Firmware v" + octoquad.getFirmwareVersion());
        telemetry.addLine("\nPress START to read values");
        telemetry.update();

        waitForStart();

        // Configure the telemetry for optimal display of data.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(50);

        // Run stats to determine cycle times.
        MovingStatistics avgTime = new MovingStatistics(100);
        ElapsedTime elapsedTime = new ElapsedTime();

        while (opModeIsActive()) {

            // Read full OctoQuad data block
            octoquad.readAllEncoderData(encoderDataBlock);
            // Extract channel 6 from the data block and display
            int distance_mm = encoderDataBlock.positions[6];
            telemetry.addData("distance mm ", distance_mm);

            // Update cycle time stats
            avgTime.add(elapsedTime.nanoseconds());
            elapsedTime.reset();

            telemetry.addData("Loop time", "%.1f mS", avgTime.getMean()/1000000);
            telemetry.update();
        }
    }
}