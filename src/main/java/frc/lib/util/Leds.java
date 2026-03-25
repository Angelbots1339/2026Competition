// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
package frc.lib.util;

import static edu.wpi.first.units.Units.Hertz;

import java.util.List;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Leds extends SubsystemBase {

	private static Leds instance;
	private final CANdle m_candle = new CANdle(1);

	private Alliance alliance = Alliance.Blue;

	private Color defaultColor = Color.kGold;
	static final int stripLength = 20;

	public boolean lowbattery = false;
	public boolean criticallyLowbattery = false;
	public boolean driverStation_attached = false;
	public boolean hubStateChangeAlert = false;
	public boolean isHubActive = false;
	public boolean shooting = false;

	private Frequency DSAttachFreq = Hertz.of(1);
	private Frequency hubAlertFreq = Hertz.of(3);

	private double waveExponent = 0.4;

	public static Leds getInstance() {
		if (instance == null) {
			instance = new Leds();
		}
		return instance;
	}

	private Leds() {
		System.out.println("[Init] Creating LEDs");
	}

	@Override
	public void periodic() {
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

		wave(Section.TOP, defaultColor, Color.kBlack, 25, 3.0);

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
							solid(Color.kRed);

						break;
					case Blue:
						if (!driverStation_attached)
							strobe(Section.TOP, Color.kBlue, DSAttachFreq);
						else
							solid(Color.kBlue);
						break;
					default:
						if (!driverStation_attached)
							strobe(Section.TOP, defaultColor, DSAttachFreq);
						else
							wave(Section.TOP, defaultColor, Color.kBlack, 25, 3.0);
						break;
				}
			}
		} else {
			solid(isHubActive ? Color.kGreen : Color.kRed);
		}

		if (shooting) {
			stripes(Section.TOP, List.of(Color.kBlack, Color.kRed), 10, 0.5);
		}

		if (hubStateChangeAlert)
			strobe(Section.TOP, isHubActive ? Color.kGreen : Color.kRed, hubAlertFreq);
	}

	public void solid(Color color) {
		solid(0, 20, color);
	}

	public void solid(int i, Color color) {
		solid(i, i + 1, color);
	}

	public void solid(int start, int end, Color color) {
		m_candle.setControl(new SolidColor(start, end).withColor(new RGBWColor(color)));
	}

	public void strobe(Section section, Color color, Frequency frequency) {
		m_candle.setControl(
				new StrobeAnimation(section.start(), section.end()).withColor(new RGBWColor(color))
						.withFrameRate(frequency));
	}

	private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
		double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
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

	private void stripes(Section section, List<Color> colors, int length, double duration) {
		int offset = (int) (Timer.getFPGATimestamp() % duration / duration * length * colors.size());
		for (int i = section.start(); i < section.end(); i++) {
			int colorIndex = (int) (Math.floor((double) (i - offset) / length) + colors.size()) % colors.size();
			colorIndex = colors.size() - 1 - colorIndex;
			solid(i, colors.get(colorIndex));
		}
	}

	private static enum Section {
		TOP;

		private int start() {
			switch (this) {
				case TOP:
					return 0;
				default:
					return 0;
			}
		}

		private int end() {
			switch (this) {
				case TOP:
					return stripLength;
				default:
					return 0;
			}
		}
	}
}
