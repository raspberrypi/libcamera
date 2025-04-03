/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2023, Raspberry Pi Ltd
 *
 * AGC/AEC control algorithm
 */
#pragma once

#include <map>
#include <string>
#include <vector>

#include <libcamera/base/utils.h>

#include <libipa/pwl.h>

#include "../agc_status.h"
#include "../awb_status.h"
#include "../controller.h"

/* This is our implementation of AGC. */

namespace RPiController {

using AgcChannelTotalExposures = std::vector<libcamera::utils::Duration>;

struct AgcMeteringMode {
	std::vector<double> weights;
	int read(const libcamera::YamlObject &params);
};

struct AgcExposureMode {
	std::vector<libcamera::utils::Duration> exposureTime;
	std::vector<double> gain;
	int read(const libcamera::YamlObject &params);
};

struct AgcConstraint {
	enum class Bound { LOWER = 0,
			   UPPER = 1 };
	Bound bound;
	double qLo;
	double qHi;
	libcamera::ipa::Pwl yTarget;
	int read(const libcamera::YamlObject &params);
};

typedef std::vector<AgcConstraint> AgcConstraintMode;

struct AgcChannelConstraint {
	enum class Bound { LOWER = 0,
			   UPPER = 1 };
	Bound bound;
	unsigned int channel;
	double factor;
	int read(const libcamera::YamlObject &params);
};

struct AgcConfig {
	int read(const libcamera::YamlObject &params);
	std::map<std::string, AgcMeteringMode> meteringModes;
	std::map<std::string, AgcExposureMode> exposureModes;
	std::map<std::string, AgcConstraintMode> constraintModes;
	std::vector<AgcChannelConstraint> channelConstraints;
	libcamera::ipa::Pwl yTarget;
	double speed;
	uint16_t startupFrames;
	unsigned int convergenceFrames;
	double maxChange;
	double minChange;
	double fastReduceThreshold;
	double speedUpThreshold;
	std::string defaultMeteringMode;
	std::string defaultExposureMode;
	std::string defaultConstraintMode;
	double baseEv;
	libcamera::utils::Duration defaultExposureTime;
	double defaultAnalogueGain;
	double stableRegion;
	bool desaturate;
};

class AgcChannel
{
public:
	AgcChannel();
	int read(const libcamera::YamlObject &params,
		 const Controller::HardwareConfig &hardwareConfig);
	unsigned int getConvergenceFrames() const;
	std::vector<double> const &getWeights() const;
	void setEv(double ev);
	void setFlickerPeriod(libcamera::utils::Duration flickerPeriod);
	void setMaxExposureTime(libcamera::utils::Duration maxExposureTime);
	void setFixedExposureTime(libcamera::utils::Duration fixedExposureTime);
	void setFixedAnalogueGain(double fixedAnalogueGain);
	void setMeteringMode(std::string const &meteringModeName);
	void setExposureMode(std::string const &exposureModeName);
	void setConstraintMode(std::string const &contraintModeName);
	void enableAutoExposure();
	void disableAutoExposure();
	bool autoExposureEnabled() const;
	void enableAutoGain();
	void disableAutoGain();
	bool autoGainEnabled() const;
	void switchMode(CameraMode const &cameraMode, Metadata *metadata);
	void prepare(Metadata *imageMetadata);
	void process(StatisticsPtr &stats, DeviceStatus const &deviceStatus, Metadata *imageMetadata,
		     const AgcChannelTotalExposures &channelTotalExposures);

private:
	bool updateLockStatus(DeviceStatus const &deviceStatus);
	AgcConfig config_;
	void housekeepConfig();
	void fetchCurrentExposure(DeviceStatus const &deviceStatus);
	void fetchAwbStatus(Metadata *imageMetadata);
	void computeGain(StatisticsPtr &statistics, Metadata *imageMetadata,
			 double &gain, double &targetY);
	void computeTargetExposure(double gain);
	void filterExposure();
	bool applyChannelConstraints(const AgcChannelTotalExposures &channelTotalExposures);
	bool applyDigitalGain(double gain, double targetY, bool channelBound);
	void divideUpExposure();
	void writeAndFinish(Metadata *imageMetadata, bool desaturate);
	libcamera::utils::Duration limitExposureTime(libcamera::utils::Duration exposureTime);
	double limitGain(double gain) const;
	AgcMeteringMode *meteringMode_;
	AgcExposureMode *exposureMode_;
	AgcConstraintMode *constraintMode_;
	CameraMode mode_;
	uint64_t frameCount_;
	AwbStatus awb_;
	struct ExposureValues {
		ExposureValues();

		libcamera::utils::Duration exposureTime;
		double analogueGain;
		libcamera::utils::Duration totalExposure;
		libcamera::utils::Duration totalExposureNoDG; /* without digital gain */
	};
	ExposureValues current_; /* values for the current frame */
	ExposureValues target_; /* calculate the values we want here */
	ExposureValues filtered_; /* these values are filtered towards target */
	AgcStatus status_;
	int lockCount_;
	DeviceStatus lastDeviceStatus_;
	libcamera::utils::Duration lastTargetExposure_;
	/* Below here the "settings" that applications can change. */
	std::string meteringModeName_;
	std::string exposureModeName_;
	std::string constraintModeName_;
	double ev_;
	libcamera::utils::Duration flickerPeriod_;
	libcamera::utils::Duration maxExposureTime_;
	libcamera::utils::Duration fixedExposureTime_;
	double fixedAnalogueGain_;
};

} /* namespace RPiController */
