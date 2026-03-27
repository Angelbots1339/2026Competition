// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.lib.util;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Leds extends SubsystemBase {

	private static Leds instance;
	private final CANdle m_candle = new CANdle(60);
	private static double globalTimer = 0.0;

	private Alliance alliance = Alliance.Blue;

	private Color defaultColor = Color.kGold;
	static final int unlitStripLength = 9;
	static final int stripLength = 38;

	public boolean lowbattery = false;
	public boolean criticallyLowbattery = false;
	public boolean driverStation_attached = false;
	public boolean hubStateChangeAlert = false;
	public boolean isHubActive = false;
	public boolean shooting = false;

	// LED Config
	private Frequency DSAttachFreq = Hertz.of(1);
	private Frequency hubAlertFreq = Hertz.of(3);
	private Time allianceBreathPeriod = Seconds.of(3);

	private int waveSlowCycleLength = 25;
	private Time waveSlowPeriod = Seconds.of(3.0);

	private int stripesLongLength = 10;
	private Time stripesFastPeriod = Seconds.of(0.5);

	// Effect configs
	private double waveExponent = 0.4;

	private AddressableLEDBuffer buf = new AddressableLEDBuffer(stripLength);

	public static Leds getInstance() {
		if (instance == null) {
			instance = new Leds();
		}
		return instance;
	}

	private Leds() {
		System.out.println("[Init] Creating LEDs");
		for (int i = 0; i < buf.getLength(); i++) {
			buf.setLED(i, Color.kBlack);
		}
	}

	@Override
	public void periodic() {
		globalTimer = Timer.getFPGATimestamp();
		if (DriverStation.isFMSAttached() || DriverStation.isDSAttached()) {
			driverStation_attached = true;
		}
		if (DriverStation.getAlliance().isPresent()) {
			alliance = DriverStation.getAlliance().get();
		}

		isHubActive = FieldUtil.isHubActive();

		if (FieldUtil.getShiftTimeLeft() <= 3 && DriverStation.isTeleop()
				&& DriverStation.getMatchTime() > FieldUtil.HubShiftTime.ENDGAME_START.time) {
			hubStateChangeAlert = true;
		} else {
			hubStateChangeAlert = false;
		}

		wave(Section.TOP, defaultColor, Color.kBlack, waveSlowCycleLength,
				waveSlowPeriod);

		if (DriverStation.isDisabled()) {
			if (lowbattery || criticallyLowbattery) {
				if (criticallyLowbattery)
					strobe(Section.TOP, Color.kOrange, Hertz.of(3));
				else
					solid(Color.kOrange);
			} else {
				switch (alliance) {
					case Red:
						if (!driverStation_attached)
							strobe(Section.TOP, Color.kRed, DSAttachFreq);
						else
							breath(Section.TOP, Color.kRed, Color.kBlack, allianceBreathPeriod);
						break;
					case Blue:
						if (!driverStation_attached)
							strobe(Section.TOP, Color.kBlue, DSAttachFreq);
						else
							breath(Section.TOP, Color.kBlue, Color.kBlack, allianceBreathPeriod);
						break;
					default:
						if (!driverStation_attached)
							strobe(Section.TOP, defaultColor, DSAttachFreq);
						else
							wave(Section.TOP, defaultColor, Color.kBlack, waveSlowCycleLength,
									waveSlowPeriod);
						break;
				}
			}
		} else {
			solid(isHubActive ? Color.kGreen : Color.kRed);
		}

		if (shooting) {
			stripes(Section.LEFT, List.of(Color.kWhite, Color.kBlack), stripesLongLength, stripesFastPeriod, true);
			stripes(Section.RIGHT, List.of(Color.kWhite, Color.kBlack), stripesLongLength, stripesFastPeriod);
		}
		if (hubStateChangeAlert)
			strobe(Section.TOP, isHubActive ? Color.kGreen : Color.kRed, hubAlertFreq);

		setLEDS();
	}

	public void setLEDS() {
		for (int i = 0; i < stripLength; i++) {
			// TODO: maybe optimize?
			m_candle.setControl(new SolidColor(i, i).withColor(new RGBWColor(buf.getLED(i))));
		}
	}

	public void solid(Color color) {
		solid(0, stripLength, color);
	}

	public void solid(int i, Color color) {
		solid(i, i + 1, color);
	}

	public void solid(Section section, Color color) {
		solid(section.start(), section.end(), color);
	}

	public void solid(int start, int end, Color color) {
		for (int i = start; i < end; i++) {
			buf.setRGB(i, (int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
		}
	}

	public void strobe(Section section, Color color, Frequency frequency) {
		strobe(section, color, 1.0 / frequency.in(Hertz));
	}

	public void strobe(Section section, Color color, double duration) {
		boolean on = ((globalTimer % duration) / duration) > 0.5;
		solid(section, on ? color : Color.kBlack);
	}

	private void breath(Section section, Color c1, Color c2, Time duration) {
		breath(section, c1, c2, duration.in(Seconds), globalTimer);
	}

	private void breath(Section section, Color c1, Color c2, double duration, double timestamp) {
		double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
		double ratio = (Math.sin(x) + 1.0) / 2.0;
		double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
		double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
		double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
		solid(section, new Color(red, green, blue));
	}

	private void wave(Section section, Color c1, Color c2, double cycleLength, Time period) {
		wave(section, c1, c2, cycleLength, period.in(Seconds));
	}

	private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
		double x = (1 - ((globalTimer % duration) / duration)) * 2.0 * Math.PI;
		double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
		for (int i = 0; i < section.end(); i++) {
			x += xDiffPerLed;
			if (i >= section.start()) {
				double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
				if (Double.isNaN(ratio)) {
					ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
				}
				if (Double.isNaN(ratio)) {
					ratio = 0.5;
				}
				double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
				double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
				double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
				solid(i, new Color(red, green, blue));
			}
		}
	}

	private void stripes(Section section, List<Color> colors, int length, Time period) {
		stripes(section, colors, length, period.in(Seconds), false);
	}

	private void stripes(Section section, List<Color> colors, int length, Time period, boolean reverse) {
		stripes(section, colors, length, period.in(Seconds), reverse);
	}

	private void stripes(Section section, List<Color> colors, int length, double duration, boolean reverse) {
		int offset = (int) (globalTimer % duration / duration * length * colors.size());
		for (int i = 0; i < section.end() - section.start(); i++) {
			int colorIndex = (int) (Math.floor((double) (i - offset) / (double) length) + colors.size())
					% colors.size();
			colorIndex = colors.size() - 1 - colorIndex;
			if (reverse)
				solid(section.end() - 1 - i, colors.get(colorIndex));
			else
				solid(section.start() + i, colors.get(colorIndex));
		}
	}

	private static enum Section {
		LEFT,
		RIGHT,
		TOP;

		private int start() {
			switch (this) {
				case TOP:
					return unlitStripLength;
				case LEFT:
					return unlitStripLength;
				case RIGHT:
					return unlitStripLength + (stripLength - unlitStripLength) / 2;
				default:
					return 0;
			}
		}

		// exclusive
		private int end() {
			switch (this) {
				case TOP:
					return stripLength;
				case LEFT:
					return unlitStripLength + (stripLength - unlitStripLength) / 2;
				case RIGHT:
					return stripLength;
				default:
					return 0;
			}
		}
	}
}
