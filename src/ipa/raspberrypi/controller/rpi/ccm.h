/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi Ltd
 *
 * ccm.h - CCM (colour correction matrix) control algorithm
 */
#pragma once

#include <vector>

#include "../ccm_algorithm.h"
#include "../pwl.h"

namespace RPiController {

/* Algorithm to calculate colour matrix. Should be placed after AWB. */

struct Matrix {
	Matrix(double m0, double m1, double m2, double m3, double m4, double m5,
	       double m6, double m7, double m8);
	Matrix();
	double m[3][3];
	int read(const libcamera::YamlObject &params);
};
static inline Matrix operator*(double d, Matrix const &m)
{
	return Matrix(m.m[0][0] * d, m.m[0][1] * d, m.m[0][2] * d,
		      m.m[1][0] * d, m.m[1][1] * d, m.m[1][2] * d,
		      m.m[2][0] * d, m.m[2][1] * d, m.m[2][2] * d);
}
static inline Matrix operator*(Matrix const &m1, Matrix const &m2)
{
	Matrix m;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m.m[i][j] = m1.m[i][0] * m2.m[0][j] +
				    m1.m[i][1] * m2.m[1][j] +
				    m1.m[i][2] * m2.m[2][j];
	return m;
}
static inline Matrix operator+(Matrix const &m1, Matrix const &m2)
{
	Matrix m;
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			m.m[i][j] = m1.m[i][j] + m2.m[i][j];
	return m;
}

struct CtCcm {
	double ct;
	Matrix ccm;
};

struct CcmConfig {
	std::vector<CtCcm> ccms;
	Pwl saturation;
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
