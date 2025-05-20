/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2024, Raspberry Pi Ltd
 *
 * cam_helper_Mira220.cpp - camera information for Mira220 sensor
 */

 #include <assert.h>

 #include "cam_helper.h"
 
 using namespace RPiController;
 
 class CamHelperMira220 : public CamHelper
 {
 public:
	 CamHelperMira220();
	 uint32_t gainCode(double gain) const override;
	 double gain(uint32_t gainCode) const override;
	 unsigned int hideFramesModeSwitch() const override;
 
 private:
	 /*
	  * Smallest difference between the frame length and integration time,
	  * in units of lines.
	  */
	 static constexpr int frameIntegrationDiff = 4;
 };
 
 /*
  * Mira220 doesn't output metadata, so we have to use the delayed controls which
  * works by counting frames.
  */
 
 CamHelperMira220::CamHelperMira220()
	 : CamHelper({}, frameIntegrationDiff)
 {
 }
 
 uint32_t CamHelperMira220::gainCode(double gain) const
 {
	 return static_cast<uint32_t>(2048.0 - 2048.0 / gain);
 }
 
 double CamHelperMira220::gain(uint32_t gainCode) const
 {
	 return static_cast<double>(2048.0 / (2048 - gainCode));
 }
 
 unsigned int CamHelperMira220::hideFramesModeSwitch() const
 {
	 /* After a mode switch, we seem to get 1 bad frame. */
	 return 1;
 }
 
 static CamHelper *create()
 {
	 return new CamHelperMira220();
 }
 
 static RegisterCamHelper reg("mira220", &create);
 