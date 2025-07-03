/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022-2023, Raspberry Pi Ltd
 *
 * Autofocus control algorithm
 */
#pragma once

#include "../af_algorithm.h"
#include "../af_status.h"
#include "../pdaf_data.h"

#include "libipa/pwl.h"

/*
 * This algorithm implements a hybrid of CDAF and PDAF, favouring PDAF.
 *
 * Whenever PDAF is available (and reports sufficiently high confidence),
 * it is used for continuous feedback control of the lens position. When
 * triggered in Auto mode, we enable the loop for a limited number of frames
 * (it may terminate sooner if the phase becomes small). In CAF mode, the
 * PDAF loop runs continuously. Very small lens movements are suppressed.
 *
 * When PDAF confidence is low (due e.g. to low contrast or extreme defocus)
 * or PDAF data are absent, fall back to CDAF with a programmed scan pattern.
 * A coarse and fine scan are performed, using the ISP's CDAF contrast FoM
 * to estimate the lens position with peak contrast. (This is slower due to
 * extra latency in the ISP, and requires a settling time between steps.)
 * The scan may terminate early if PDAF recovers and allows the zero-phase
 * lens position to be interpolated.
 *
 * In CAF mode, the fallback to a CDAF scan is triggered when PDAF fails to
 * report high confidence and a configurable number of frames have elapsed
 * since the last image change since either PDAF was working or a previous
 * scan found peak contrast. Image changes are detected using both contrast
 * and AWB statistics (within the AF window[s]).
 *
 * IR lighting can interfere with the correct operation of PDAF, so we
 * optionally try to detect it (from AWB statistics).
 */

namespace RPiController {

class Af : public AfAlgorithm
{
public:
	Af(Controller *controller = NULL);
	~Af();
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void initialise() override;

	/* IPA calls */
	void switchMode(CameraMode const &cameraMode, Metadata *metadata) override;
	void prepare(Metadata *imageMetadata) override;
	void process(StatisticsPtr &stats, Metadata *imageMetadata) override;

	/* controls */
	void setRange(AfRange range) override;
	void setSpeed(AfSpeed speed) override;
	void setMetering(bool use_windows) override;
	void setWindows(libcamera::Span<libcamera::Rectangle const> const &wins) override;
	void setMode(AfMode mode) override;
	AfMode getMode() const override;
	double getDefaultLensPosition() const override;
	void getLensLimits(double &min, double &max) const override;
	bool setLensPosition(double dioptres, int32_t *hwpos, bool force) override;
	std::optional<double> getLensPosition() const override;
	void triggerScan() override;
	void cancelScan() override;
	void pause(AfPause pause) override;

private:
	enum class ScanState {
		Idle = 0,
		Trigger,
		Pdaf,
		Coarse1,
		Coarse2,
		Fine,
		Settle
	};

	struct RangeDependentParams {
		double focusMin;       		/* lower (far) limit in dipotres */
		double focusMax;	       	/* upper (near) limit in dioptres */
		double focusDefault;		/* default setting ("hyperfocal") */

		RangeDependentParams();
		void read(const libcamera::YamlObject &params);
	};

	struct SpeedDependentParams {
		double stepCoarse;		/* in dioptres; used for scans */
		double stepFine;		/* in dioptres; used for scans */
		double contrastRatio;		/* used for scan termination and reporting */
		double retriggerRatio;          /* contrast and RGB ratio for re-triggering */
		uint32_t retriggerDelay;        /* frames of stability before re-triggering */
		double pdafGain;		/* coefficient for PDAF feedback loop */
		double pdafSquelch;		/* PDAF stability parameter (device-specific) */
		double maxSlew;			/* limit for lens movement per frame */
		uint32_t pdafFrames;		/* number of iterations when triggered */
		uint32_t dropoutFrames;		/* number of non-PDAF frames to switch to CDAF */
		uint32_t stepFrames;		/* frames to skip in between steps of a scan */

		SpeedDependentParams();
		void read(const libcamera::YamlObject &params);
	};

	struct CfgParams {
		RangeDependentParams ranges[AfRangeMax];
		SpeedDependentParams speeds[AfSpeedMax];
		uint32_t confEpsilon;	       	/* PDAF hysteresis threshold (sensor-specific) */
		uint32_t confThresh;	       	/* PDAF confidence cell min (sensor-specific) */
		uint32_t confClip;	       	/* PDAF confidence cell max (sensor-specific) */
		uint32_t skipFrames;	       	/* frames to skip at start or modeswitch */
		bool checkForIR;                /* Set this if PDAF is unreliable in IR light */
		libcamera::ipa::Pwl map;       	/* converts dioptres -> lens driver position */

		CfgParams();
		int read(const libcamera::YamlObject &params);
		void initialise();
	};

	struct ScanRecord {
		double focus;
		double contrast;
		double phase;
		double conf;
	};

	struct RegionWeights {
		unsigned rows;
		unsigned cols;
		uint32_t sum;
		std::vector<uint16_t> w;

		RegionWeights()
			: rows(0), cols(0), sum(0), w() {}
	};

	void computeWeights(RegionWeights *wgts, unsigned rows, unsigned cols);
	void invalidateWeights();
	bool getPhase(PdafRegions const &regions, double &phase, double &conf);
	double getContrast(const FocusRegions &focusStats);
	bool getAverageAndTestIr(const RgbyRegions &awbStats, double rgb[3]);
	void doPDAF(double phase, double conf);
	bool earlyTerminationByPhase(double phase);
	double findPeak(unsigned index) const;
	void doScan(double contrast, double phase, double conf);
	void doAF(double contrast, double phase, double conf);
	void updateLensPosition();
	void startAF();
	void startProgrammedScan();
	void goIdle();

	/* Configuration and settings */
	CfgParams cfg_;
	AfRange range_;
	AfSpeed speed_;
	AfMode mode_;
	bool pauseFlag_;
	libcamera::Rectangle statsRegion_;
	std::vector<libcamera::Rectangle> windows_;
	bool useWindows_;
	RegionWeights phaseWeights_;
	RegionWeights contrastWeights_;
	RegionWeights awbWeights_;

	/* Working state. */
	ScanState scanState_;
	bool initted_, irFlag_;
	double ftarget_, fsmooth_;
	double prevContrast_, oldSceneContrast_;
	double prevAverage_[3], oldSceneAverage_[3];
	double prevPhase_;
	unsigned skipCount_, stepCount_, dropCount_;
	unsigned sameSignCount_;
	unsigned sceneChangeCount_;
	unsigned scanMaxIndex_;
	double scanMaxContrast_, scanMinContrast_, scanStep_;
	std::vector<ScanRecord> scanData_;
	AfState reportState_;
};

} // namespace RPiController
