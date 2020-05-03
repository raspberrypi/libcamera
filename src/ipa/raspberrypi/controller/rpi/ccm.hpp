/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2019, Raspberry Pi (Trading) Limited
 *
 * ccm.hpp - CCM (colour correction matrix) control algorithm
 */
#pragma once

#include <vector>
#include <atomic>

#include "../ccm_algorithm.hpp"
#include "../pwl.hpp"

namespace RPi {

// Algorithm to calculate colour matrix. Should be placed after AWB.

struct Matrix {
	Matrix(double m0, double m1, double m2, double m3, double m4, double m5,
	       double m6, double m7, double m8);
	Matrix();
	double m[3][3];
	void Read(boost::property_tree::ptree const &params);
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
	char const *Name() const override;
	void Read(boost::property_tree::ptree const &params) override;
	void SetSaturation(double saturation) override;
	void Initialise() override;
	void Prepare(Metadata *image_metadata) override;

private:
	CcmConfig config_;
	std::atomic<double> saturation_;
};

} // namespace RPi
