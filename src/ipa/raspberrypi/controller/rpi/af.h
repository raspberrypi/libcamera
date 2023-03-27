/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022-2023, Raspberry Pi Ltd
 *
 * af.h - Autofocus control algorithm
 */
#pragma once

#include "../af_algorithm.h"
#include "../af_status.h"
#include "../pdaf_data.h"
#include "../pwl.h"

/*
 * This algorithm implements a hybrid of CDAF and PDAF, favouring PDAF.
 *
 * Whenever PDAF is available, it is used in a continuous feedback loop.
 * When triggered in auto mode, we simply enable AF for a limited number
 * of frames (it may terminate early if the delta becomes small enough).
 *
 * When PDAF confidence is low (due e.g. to low contrast or extreme defocus)
 * or PDAF data are absent, fall back to CDAF with a programmed scan pattern.
 * A coarse and fine scan are performed, using ISP's CDAF focus FoM to
 * estimate the lens position with peak contrast. This is slower due to
 * extra latency in the ISP, and requires a settling time between steps.
 *
 * Some hysteresis is applied to the switch between PDAF and CDAF, to avoid
 * "nuisance" scans. During each interval where PDAF is not working, only
 * ONE scan will be performed; CAF cannot track objects using CDAF alone.
 *
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
	bool setLensPosition(double dioptres, int32_t *hwpos) override;
	std::optional<double> getLensPosition() const override;
	void triggerScan() override;
	void cancelScan() override;
	void pause(AfPause pause) override;

private:
	enum class ScanState {
		Idle = 0,
		Trigger,
		Pdaf,
		Coarse,
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
		double stepCoarse;		/* used for scans */
		double stepFine;		/* used for scans */
		double contrastRatio;		/* used for scan termination and reporting */
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
		Pwl map;		       	/* converts dioptres -> lens driver position */

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

	/* Working state. */
	ScanState scanState_;
	bool initted_;
	double ftarget_, fsmooth_;
	double prevContrast_;
	unsigned skipCount_, stepCount_, dropCount_;
	unsigned scanMaxIndex_;
	double scanMaxContrast_, scanMinContrast_;
	std::vector<ScanRecord> scanData_;
	AfState reportState_;
};

} // namespace RPiController
