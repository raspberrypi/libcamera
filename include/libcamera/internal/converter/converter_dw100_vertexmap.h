/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * DW100 vertex map interface
 */

#pragma once

#include <assert.h>
#include <cmath>
#include <optional>
#include <stdint.h>
#include <vector>

#include <libcamera/base/span.h>

#include <libcamera/geometry.h>
#include <libcamera/transform.h>

#include "libcamera/internal/matrix.h"
#include "libcamera/internal/vector.h"

namespace libcamera {

class Dw100VertexMap
{
public:
	enum ScaleMode {
		Fill = 0,
		Crop = 1,
	};

	struct DewarpParams {
		DewarpParams()
			: cm(Matrix<double, 3, 3>::identity()),
			  cmNew(Matrix<double, 3, 3>::identity()),
			  coefficients({})
		{
		}

		int setCoefficients(Span<const double> coeffs);

		Matrix<double, 3, 3> cm;
		Matrix<double, 3, 3> cmNew;

		struct {
			double k1;
			double k2;
			double p1;
			double p2;
			double k3;
			double k4;
			double k5;
			double k6;
			double s1;
			double s2;
			double s3;
			double s4;
		} coefficients;
	};

	void applyLimits();
	void setInputSize(const Size &size)
	{
		inputSize_ = size;
		scalerCrop_ = Rectangle(size);
	}

	void setSensorCrop(const Rectangle &rect) { sensorCrop_ = rect; }

	void setScalerCrop(const Rectangle &rect) { scalerCrop_ = rect; }
	const Rectangle &effectiveScalerCrop() const { return effectiveScalerCrop_; }

	void setOutputSize(const Size &size) { outputSize_ = size; }
	const Size &outputSize() const { return outputSize_; }

	void setTransform(const Transform &transform) { transform_ = transform; }
	const Transform &transform() const { return transform_; }

	void setScale(const float scale) { scale_ = scale; }
	float effectiveScale() const { return (effectiveScaleX_ + effectiveScaleY_) * 0.5; }

	void setRotation(const float rotation) { rotation_ = rotation; }
	float rotation() const { return rotation_; }

	void setOffset(const Point &offset) { offset_ = offset; }
	const Point &effectiveOffset() const { return effectiveOffset_; }

	void setMode(const ScaleMode mode) { mode_ = mode; }
	ScaleMode mode() const { return mode_; }

	void setDewarpParams(const DewarpParams &params) { dewarpParams_ = params; }

	void setLensDewarpEnable(bool enable) { lensDewarpEnable_ = enable; }
	bool lensDewarpEnable() { return lensDewarpEnable_; }

	std::vector<uint32_t> getVertexMap();

private:
	Vector<double, 2> dewarpPoint(const Vector<double, 2> &p);

	Rectangle scalerCrop_;
	Rectangle sensorCrop_;
	Transform transform_ = Transform::Identity;
	Size inputSize_;
	Size outputSize_;
	Point offset_;
	double scale_ = 1.0;
	double rotation_ = 0.0;
	ScaleMode mode_ = Fill;
	double effectiveScaleX_;
	double effectiveScaleY_;
	Point effectiveOffset_;
	Rectangle effectiveScalerCrop_;

	std::optional<DewarpParams> dewarpParams_;
	bool lensDewarpEnable_ = true;
};

} /* namespace libcamera */
