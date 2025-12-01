/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2021 - 2023, Raspberry Pi Ltd
 *
 * stage.cpp - Tiling library stage definition
 */
#pragma once

#include <string>

#include "types.hpp"

namespace tiling
{

class Pipeline;

class TilingException : public std::exception
{
public:
	char const *what() const noexcept { return "Tiling Failed"; }
};

class Stage
{
public:
	Stage(char const *name, Pipeline *pipeline, int struct_offset);
	virtual ~Stage() {}
	Pipeline *GetPipeline() const;
	virtual Length2 GetInputImageSize() const = 0;
	virtual Length2 GetOutputImageSize() const = 0;
	virtual void SetDownstream(Stage *downstream) = 0;
	virtual void Reset() = 0;
	virtual void PushStartUp(int output_start, Dir dir) = 0;
	virtual int PushEndDown(int input_end, Dir dir) = 0;
	virtual void PushEndUp(int output_end, Dir dir) = 0;
	virtual void PushCropDown(Interval interval, Dir dir) = 0;
	virtual void CopyOut(void *dest, Dir dir) = 0;
	virtual bool GetBranchComplete() const = 0;
	virtual bool GetBranchInactive() const = 0;
	void MergeRegions(void *dest, void *x_src, void *y_src) const;

protected:
	std::string name_;
	Pipeline *pipeline_;
	int struct_offset_;
};

// BasicStage is a Stage that has a single upstream and a single downstream stage (though they can be NULL).

class BasicStage : public Stage
{
public:
	BasicStage(char const *name, Pipeline *pipeline, Stage *upstream, int struct_offset);
	virtual Length2 GetInputImageSize() const;
	virtual Length2 GetOutputImageSize() const;
	virtual void SetDownstream(Stage *downstream);
	virtual void Reset();
	virtual void CopyOut(void *dest, Dir dir);
	virtual bool GetBranchComplete() const;
	virtual bool GetBranchInactive() const;

protected:
	Stage *upstream_;
	Stage *downstream_;
	Interval input_interval_;
	Crop crop_;
	Interval output_interval_;
};

} // namespace tiling