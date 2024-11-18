/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * CCM (colour correction matrix) control algorithm
 */
#pragma once

#include <vector>

#include <libipa/pwl.h>

#include "../ccm_algorithm.h"

namespace RPiController {

/* Algorithm to calculate colour matrix. Should be placed after AWB. */

struct Matrix3x3 {
	Matrix3x3(double m0, double m1, double m2, double m3, double m4, double m5,
	       double m6, double m7, double m8);
	Matrix3x3();
	double m[3][3];
	int read(const libcamera::YamlObject &params);
};
static inline Matrix3x3 operator*(double d, Matrix3x3 const &m)
{
	return Matrix3x3(m.m[0][0] * d, m.m[0][1] * d, m.m[0][2] * d,
		      m.m[1][0] * d, m.m[1][1] * d, m.m[1][2] * d,
		      m.m[2][0] * d, m.m[2][1] * d, m.m[2][2] * d);
}
static inline Matrix3x3 operator*(Matrix3x3 const &m1, Matrix3x3 const &m2)
{
	Matrix3x3 m;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m.m[i][j] = m1.m[i][0] * m2.m[0][j] +
				    m1.m[i][1] * m2.m[1][j] +
				    m1.m[i][2] * m2.m[2][j];
	return m;
}
static inline Matrix3x3 operator+(Matrix3x3 const &m1, Matrix3x3 const &m2)
{
	Matrix3x3 m;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m.m[i][j] = m1.m[i][j] + m2.m[i][j];
	return m;
}

struct CtCcm {
	double ct;
	Matrix3x3 ccm;
};

struct CcmConfig {
	std::vector<CtCcm> ccms;
	libcamera::ipa::Pwl saturation;
};

class Ccm : public CcmAlgorithm
{
public:
	Ccm(Controller *controller = NULL);
	char const *name() const override;
	int read(const libcamera::YamlObject &params) override;
	void setSaturation(double saturation) override;
	void initialise() override;
	void prepare(Metadata *imageMetadata) override;

private:
	CcmConfig config_;
	double saturation_;
};

} /* namespace RPiController */
