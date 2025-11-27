/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2025, Ideas on Board Oy
 *
 * DW100 vertex map implementation
 */

#include "libcamera/internal/converter/converter_dw100_vertexmap.h"

#include <algorithm>
#include <assert.h>
#include <cmath>
#include <stdint.h>
#include <utility>
#include <vector>

#include <libcamera/base/log.h>
#include <libcamera/base/span.h>

#include <libcamera/geometry.h>
#include <libcamera/transform.h>

#include "libcamera/internal/vector.h"

constexpr int kDw100BlockSize = 16;

namespace libcamera {

LOG_DECLARE_CATEGORY(Converter)
namespace {

using Vector2d = Vector<double, 2>;
using Vector3d = Vector<double, 3>;
using Matrix3x3 = Matrix<double, 3, 3>;

Matrix3x3 makeTranslate(const double tx, const double ty)
{
	Matrix3x3 m = Matrix3x3::identity();
	m[0][2] = tx;
	m[1][2] = ty;
	return m;
}

Matrix3x3 makeTranslate(const Vector2d &t)
{
	return makeTranslate(t.x(), t.y());
}

Matrix3x3 makeRotate(const double degrees)
{
	double rad = degrees / 180.0 * M_PI;
	double sa = std::sin(rad);
	double ca = std::cos(rad);

	Matrix3x3 m = Matrix3x3::identity();
	m[0][0] = ca;
	m[0][1] = -sa;
	m[1][0] = sa;
	m[1][1] = ca;
	return m;
}

Matrix3x3 makeScale(const double sx, const double sy)
{
	Matrix3x3 m = Matrix3x3::identity();
	m[0][0] = sx;
	m[1][1] = sy;
	return m;
}

/**
 * \param t The transform to apply
 * \param size The size of the rectangle that is transformed
 *
 * Create a matrix that represents the transform done by the \a t. It assumes
 * that the origin of the coordinate system is at the top left corner of of the
 * rectangle.
 */
Matrix3x3 makeTransform(const Transform &t, const Size &size)
{
	Matrix3x3 m = Matrix3x3::identity();
	double wm = size.width * 0.5;
	double hm = size.height * 0.5;
	m = makeTranslate(-wm, -hm) * m;

	if (!!(t & Transform::HFlip))
		m = makeScale(-1, 1) * m;

	if (!!(t & Transform::VFlip))
		m = makeScale(1, -1) * m;

	if (!!(t & Transform::Transpose)) {
		m = makeRotate(-90) * m;
		m = makeScale(1, -1) * m;
		std::swap(wm, hm);
	}

	m = makeTranslate(wm, hm) * m;

	return m;
}

/**
 * \param from The source rectangle
 * \param to The destination rectangle
 *
 * Create a matrix that transforms from the coordinate system of rectangle \a
 * from into the coordinate system of rectangle \a to, by overlaying the
 * rectangles.
 *
 * \see Rectangle::transformedBetween()
 */
Matrix3x3 makeTransform(const Rectangle &from, const Rectangle &to)
{
	Matrix3x3 m = Matrix3x3::identity();
	double sx = to.width / static_cast<double>(from.width);
	double sy = to.height / static_cast<double>(from.height);
	m = makeTranslate(-from.x, -from.y) * m;
	m = makeScale(sx, sy) * m;
	m = makeTranslate(to.x, to.y) * m;
	return m;
}

Vector2d transformPoint(const Matrix3x3 &m, const Vector2d &p)
{
	Vector3d p2{ { p.x(), p.y(), 1.0 } };
	p2 = m * p2;
	return { { p2.x() / p2.z(), p2.y() / p2.z() } };
}

Vector2d transformVector(const Matrix3x3 &m, const Vector2d &p)
{
	Vector3d p2{ { p.x(), p.y(), 0.0 } };
	p2 = m * p2;
	return { { p2.x(), p2.y() } };
}

Vector2d rotatedRectSize(const Vector2d &size, const double degrees)
{
	double rad = degrees / 180.0 * M_PI;
	double sa = sin(rad);
	double ca = cos(rad);

	return { { std::abs(size.x() * ca) + std::abs(size.y() * sa),
		   std::abs(size.x() * sa) + std::abs(size.y() * ca) } };
}

Vector2d point2Vec2d(const Point &p)
{
	return { { static_cast<double>(p.x), static_cast<double>(p.y) } };
}

int dw100VerticesForLength(const int length)
{
	return (length + kDw100BlockSize - 1) / kDw100BlockSize + 1;
}

} /* namespace */

/**
 * \class libcamera::Dw100VertexMap
 * \brief Helper class to compute dw100 vertex maps
 *
 * The vertex map class represents a helper for handling dewarper vertex maps.
 * There are 3 important sizes in the system:
 *
 * - The sensor size. The number of pixels of the whole sensor.
 * - The input rectangle to the dewarper. Describes the pixel data flowing into
 *   the dewarper in sensor coordinates.
 * - ScalerCrop rectangle. The rectangle that shall be used for all further
 *   stages. It is applied after lens dewarping but is in sensor coordinate
 *   space.
 * - The output size. This defines the size, the dewarper should output.
 *
 * +------------------------+
 * |Sensor size             |
 * |   +----------------+   |
 * |   |  Input rect    |   |
 * |   |  +-------------+   |
 * |   |  | ScalerCrop  |   |
 * |   |  |             |   |
 * |   +--+-------------+   |
 * +------------------------+
 *
 * This class implements a vertex map that forms the following pipeline:
 *
 * +-------------+    +-------------+    +------------+    +-----------------+
 * |             |    |             |    | Transform  |    | Pan/Zoom        |
 * | Lens Dewarp | -> | Scaler Crop | -> | (H/V Flip, | -> | (Offset, Scale, |
 * |             |    |             |    | Transpose) |    | Rotate)         |
 * +-------------+    +-------------+    +------------+    +-----------------+
 *
 * All parameters are clamped to valid values before creating the vertex map.
 *
 * The constraints process works as follows:
 * - The ScalerCrop rectangle is clamped to the input rectangle
 * - The ScalerCrop rectangle is transformed by the specified transform
 *   forming ScalerCropT
 * - A rectangle of output size is placed in the center of ScalerCropT
 *   (OutputRect).
 * - Rotate gets applied to OutputRect,
 * - Scale is applied, but clamped so that the OutputRect fits completely into
 *   ScalerCropT (Only regarding dimensions, not position)
 * - Offset is clamped so that the OutputRect lies inside ScalerCropT
 *
 * After applying the limits, the actual values used for processing are stored
 * effectiveXXX members and can be queried using the corresponding functions.
 *
 * The lens dewarp map is usually calibrated during tuning and is a map that
 * maps from incoming pixels to dewarped pixels.
 */

/**
 * \enum Dw100VertexMap::ScaleMode
 * \brief The scale modes available for a vertex map
 *
 * \var Dw100VertexMap::Fill
 * \brief Scale the input to fill the output
 *
 * This scale mode does not preserve aspect ratio. Offset and rotation are taken
 * into account.
 *
 * \var Dw100VertexMap::Crop
 * \brief Crop the input
 *
 * This scale mode preserves the aspect ratio. Offset, scale, rotation are taken
 * into account within the possible limits.
 */

/**
 * \brief Apply limits on scale and offset
 *
 * This function calculates \a effectiveScalerCrop_, \a effectiveScale_ and \a
 * effectiveOffset_ based on the requested scaler crop, scale, rotation, offset
 * and the selected scale mode, so that the whole output area is filled with
 * valid input data.
 */
void Dw100VertexMap::applyLimits()
{
	int ow = outputSize_.width;
	int oh = outputSize_.height;
	effectiveScalerCrop_ = scalerCrop_.boundedTo(sensorCrop_);

	/* Map the scalerCrop to the input pixel space */
	Rectangle localScalerCrop = effectiveScalerCrop_.transformedBetween(
		sensorCrop_, Rectangle(inputSize_));

	Size localCropSizeT = localScalerCrop.size();
	if (!!(transform_ & Transform::Transpose))
		std::swap(localCropSizeT.width, localCropSizeT.height);

	Vector2d size = rotatedRectSize(point2Vec2d({ ow, oh }), rotation_);

	if (mode_ != Crop && mode_ != Fill) {
		LOG(Converter, Error)
			<< "Unknown mode " << mode_ << ". Default to 'Fill'";
		mode_ = Fill;
	}

	/* Calculate constraints */
	double scale = scale_;
	if (mode_ == Crop) {
		/* Scale up if needed */
		scale = std::max(scale,
				 std::max(size.x() / localCropSizeT.width,
					  size.y() / localCropSizeT.height));
		effectiveScaleX_ = scale;
		effectiveScaleY_ = scale;

		size = size / scale;

	} else if (mode_ == Fill) {
		effectiveScaleX_ = size.x() / localCropSizeT.width;
		effectiveScaleY_ = size.y() / localCropSizeT.height;

		size.x() /= effectiveScaleX_;
		size.y() /= effectiveScaleY_;
	}

	/*
	 * Clamp offset. Due to rounding errors, size might be slightly bigger
	 * than scaler crop. Clamp the offset to 0 to prevent a crash in the
	 * next clamp.
	 */
	double maxoffX, maxoffY;
	maxoffX = std::max(0.0, (localCropSizeT.width - size.x())) * 0.5;
	maxoffY = std::max(0.0, (localCropSizeT.height - size.y())) * 0.5;
	if (!!(transform_ & Transform::Transpose))
		std::swap(maxoffX, maxoffY);

	/*
	 * Transform the offset from sensor space to local space, apply the
	 * limit and transform back.
	 */
	Vector2d offset = point2Vec2d(offset_);
	Matrix3x3 m;

	m = makeTransform(effectiveScalerCrop_, localScalerCrop);
	offset = transformVector(m, offset);
	offset.x() = std::clamp(offset.x(), -maxoffX, maxoffX);
	offset.y() = std::clamp(offset.y(), -maxoffY, maxoffY);
	m = makeTransform(localScalerCrop, effectiveScalerCrop_);
	offset = transformVector(m, offset);
	effectiveOffset_.x = offset.x();
	effectiveOffset_.y = offset.y();
}

/**
 * \fn Dw100VertexMap::setInputSize()
 * \brief Set the size of the input data
 * \param[in] size The input size
 *
 * To calculate a proper vertex map, the size of the input images must be set.
 */

/**
 * \fn Dw100VertexMap::setSensorCrop()
 * \brief Set the crop rectangle that represents the input data
 * \param[in] rect
 *
 * Set the rectangle that represents the input data in sensor coordinates. This
 * must be specified to properly calculate the vertex map.
 */

/**
 * \fn Dw100VertexMap::setScalerCrop()
 * \brief Set the requested scaler crop
 * \param[in] rect
 *
 * Set the requested scaler crop. The actually applied scaler crop can be
 * queried using \a Dw100VertexMap::effectiveScalerCrop() after calling
 * Dw100VertexMap::applyLimits().
 */

/**
 * \fn Dw100VertexMap::effectiveScalerCrop()
 * \brief Get the effective scaler crop
 *
 * \return The effective scaler crop
 */

/**
 * \fn Dw100VertexMap::setOutputSize()
 * \brief Set the output size
 * \param[in] size The size of the output images
 */

/**
 * \fn Dw100VertexMap::outputSize()
 * \brief Get the output size
 * \return The output size
 */

/**
 * \fn Dw100VertexMap::setTransform()
 * \brief Sets the transform to apply
 * \param[in] transform The transform
 */

/**
 * \fn Dw100VertexMap::transform()
 * \brief Get the transform
 * \return The transform
 */

/**
 * \fn Dw100VertexMap::setScale()
 * \brief Sets the scale to apply
 * \param[in] scale The scale
 *
 * Set the requested scale. The actually applied scale can be queried using \a
 * Dw100VertexMap::effectiveScale() after calling \a
 * Dw100VertexMap::applyLimits().
 */

/**
 * \fn Dw100VertexMap::effectiveScale()
 * \brief Get the effective scale
 *
 * Returns the actual scale applied to the input pixels in x and y direction. So
 * a value of [2.0, 1.5] means that every input pixel is scaled to cover 2
 * output pixels in x-direction and 1.5 in y-direction.
 *
 * \return The effective scale
 */

/**
 * \fn Dw100VertexMap::setRotation()
 * \brief Sets the rotation to apply
 * \param[in] rotation The rotation in degrees
 *
 * The rotation is in clockwise direction to allow the same transform as
 * CameraConfiguration::orientation
 */

/**
 * \fn Dw100VertexMap::rotation()
 * \brief Get the rotation
 * \return The rotation in degrees
 */

/**
 * \fn Dw100VertexMap::setOffset()
 * \brief Sets the offset to apply
 * \param[in] offset The offset
 *
 * Set the requested offset. The actually applied offset can be queried using \a
 * Dw100VertexMap::effectiveOffset() after calling \a
 * Dw100VertexMap::applyLimits().
 */

/**
 * \fn Dw100VertexMap::effectiveOffset()
 * \brief Get the effective offset
 *
 * Returns the actual offset applied to the input pixels in ScalerCrop
 * coordinates.
 *
 * \return The effective offset
 */

/**
 * \fn Dw100VertexMap::setMode()
 * \brief Sets the scaling mode to apply
 * \param[in] mode The mode
 */

/**
 * \fn Dw100VertexMap::mode()
 * \brief Get the scaling mode
 * \return The scaling mode
 */

/**
 * \brief Get the dw100 vertex map
 *
 * Calculates the vertex map as a vector of hardware specific entries.
 *
 * \return The vertex map
 */
std::vector<uint32_t> Dw100VertexMap::getVertexMap()
{
	int ow = outputSize_.width;
	int oh = outputSize_.height;
	int tileCountW = dw100VerticesForLength(ow);
	int tileCountH = dw100VerticesForLength(oh);

	applyLimits();

	/*
	 * libcamera handles all crop rectangles in sensor space. But the
	 * dewarper "sees" only the pixels it gets passed. Note that these might
	 * not cover exactly the max sensor crop, as there might be a crop
	 * between ISP and dewarper to crop to a format supported by the
	 * dewarper. effectiveScalerCrop_ is the crop in sensor space that gets
	 * fed into the dewarper. localScalerCrop is the sensor crop mapped to
	 * the data that is fed into the dewarper.
	 */
	Rectangle localScalerCrop = effectiveScalerCrop_.transformedBetween(
		sensorCrop_, Rectangle(inputSize_));
	Size localCropSizeT = localScalerCrop.size();
	if (!!(transform_ & Transform::Transpose))
		std::swap(localCropSizeT.width, localCropSizeT.height);

	/*
	 * The dw100 has a specialty in interpolation that has to be taken into
	 * account to use in a pixel perfect manner. To explain this, I will
	 * only use the x direction, the vertical axis behaves the same.
	 *
	 * Let's start with a pixel perfect 1:1 mapping of an image with a width
	 * of 64pixels. The coordinates of the vertex map would then be:
	 * 0 -- 16 -- 32 -- 48 -- 64
	 * Note how the last coordinate lies outside the image (which ends at
	 * 63) as it is basically the beginning of the next macro block.
	 *
	 * if we zoom out a bit we might end up with something like
	 * -10 -- 0 -- 32 -- 64 -- 74
	 * As the dewarper coordinates are unsigned it actually sees
	 * 0 -- 0 -- 32 -- 64 -- 74
	 * Leading to stretched pixels at the beginning and black for everything
	 * > 63
	 *
	 * Now lets rotate the image by 180 degrees. A trivial rotation would
	 * end up with:
	 *
	 * 64 -- 48 -- 32 -- 16 -- 0
	 *
	 * But as the first column now points to pixel 64 we get a single black
	 * line. So for a proper 180* rotation, the coordinates need to be
	 *
	 * 63 -- 47 -- 31 -- 15 -- -1
	 *
	 * The -1 is clamped to 0 again, leading to a theoretical slight
	 * interpolation error on the last 16 pixels.
	 *
	 * To create this proper transformation there are two things todo:
	 *
	 * 1. The rotation centers are offset by -0.5. This evens out for no
	 *    rotation, and leads to a coordinate offset of -1 on 180 degree
	 *    rotations.
	 * 2. The transformation (flip and transpose) need to act on a size-1
	 *    to get the same effect.
	 */
	Vector2d centerS{ { localCropSizeT.width * 0.5 - 0.5,
			    localCropSizeT.height * 0.5 - 0.5 } };
	Vector2d centerD{ { ow * 0.5 - 0.5,
			    oh * 0.5 - 0.5 } };

	LOG(Converter, Debug)
		<< "Apply vertex map for"
		<< " inputSize: " << inputSize_
		<< " outputSize: " << outputSize_
		<< " Transform: " << transformToString(transform_)
		<< "\n effectiveScalerCrop: " << effectiveScalerCrop_
		<< " localCropSizeT: " << localCropSizeT
		<< " scaleX: " << effectiveScaleX_
		<< " scaleY: " << effectiveScaleX_
		<< " rotation: " << rotation_
		<< " offset: " << effectiveOffset_;

	Matrix3x3 outputToSensor = Matrix3x3::identity();
	/* Move to center of output */
	outputToSensor = makeTranslate(-centerD) * outputToSensor;
	outputToSensor = makeRotate(-rotation_) * outputToSensor;
	outputToSensor = makeScale(1.0 / effectiveScaleX_, 1.0 / effectiveScaleY_) * outputToSensor;
	/* Move to top left of localScalerCropT */
	outputToSensor = makeTranslate(centerS) * outputToSensor;
	outputToSensor = makeTransform(-transform_, localCropSizeT.shrunkBy({ 1, 1 })) *
			 outputToSensor;
	/* Transform from "within localScalerCrop" to input reference frame */
	outputToSensor = makeTranslate(localScalerCrop.x, localScalerCrop.y) * outputToSensor;
	outputToSensor = makeTransform(localScalerCrop, effectiveScalerCrop_) * outputToSensor;
	outputToSensor = makeTranslate(point2Vec2d(effectiveOffset_)) * outputToSensor;

	Matrix3x3 sensorToInput = makeTransform(effectiveScalerCrop_, localScalerCrop);

	/*
	 * For every output tile, calculate the position of the corners in the
	 * input image.
	 */
	std::vector<uint32_t> res;
	res.reserve(tileCountW * tileCountH);
	for (int y = 0; y < tileCountH; y++) {
		for (int x = 0; x < tileCountW; x++) {
			Vector2d p{ { static_cast<double>(x) * kDw100BlockSize,
				      static_cast<double>(y) * kDw100BlockSize } };
			p = p.max(0.0).min(Vector2d{ { static_cast<double>(ow),
						       static_cast<double>(oh) } });

			p = transformPoint(outputToSensor, p);

			if (dewarpParamsValid_ && lensDewarpEnable_)
				p = dewarpPoint(p);

			p = transformPoint(sensorToInput, p);

			/* Convert to fixed point */
			uint32_t v = static_cast<uint32_t>(p.y() * 16) << 16 |
				     (static_cast<uint32_t>(p.x() * 16) & 0xffff);
			res.push_back(v);
		}
	}

	return res;
}

/**
 * \brief Set the dewarp parameters
 * \param cm The camera matrix
 * \param coeffs The dewarp coefficients
 *
 * Sets the dewarp parameters according to the commonly used dewarp model. See
 * https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html for further details
 * on the model. The parameter \a coeffs must either hold 4,5,8 or 12 values.
 * They represent the parameters k1,k2,p1,p2[,k3[,k4,k5,k6[,s1,s2,s3,s4]]] in
 * the model.
 *
 * \return A negative number on error, 0 otherwise
 */
int Dw100VertexMap::setDewarpParams(const Matrix<double, 3, 3> &cm,
				    const Span<const double> &coeffs)
{
	dewarpM_ = cm;
	dewarpCoeffs_.fill(0.0);

	if (coeffs.size() != 4 && coeffs.size() != 5 &&
	    coeffs.size() != 8 && coeffs.size() != 12) {
		LOG(Converter, Error)
			<< "Dewarp 'coefficients' must have 4, 5, 8 or 12 values";
		dewarpParamsValid_ = false;
		return -EINVAL;
	}
	std::copy(coeffs.begin(), coeffs.end(), dewarpCoeffs_.begin());

	dewarpParamsValid_ = true;
	return 0;
}

/**
 * \fn Dw100VertexMap::dewarpParamsValid()
 * \brief Returns if the dewarp parameters are valid
 *
 * \return True if the dewarp parameters are valid, false otherwise
 */

/**
 * \fn Dw100VertexMap::setLensDewarpEnable()
 * \brief Enables or disables lens dewarping
 * \param[in] enable Enable or disable lens dewarping
 */

/**
 * \fn Dw100VertexMap::lensDewarpEnable()
 * \brief Returns if lens dewarping is enabled
 */

/**
 * \brief Apply dewarp calculation to a point
 * \param p The point to dewarp
 *
 * Applies the dewarp transformation to point \a p according to the commonly
 * used dewarp model. See
 * https://docs.opencv.org/4.12.0/d9/d0c/group__calib3d.html for further details
 * on the model.
 *
 * \return The dewarped point
 */
Vector2d Dw100VertexMap::dewarpPoint(const Vector2d &p)
{
	double x, y;
	double k1 = dewarpCoeffs_[0];
	double k2 = dewarpCoeffs_[1];
	double p1 = dewarpCoeffs_[2];
	double p2 = dewarpCoeffs_[3];
	double k3 = dewarpCoeffs_[4];
	double k4 = dewarpCoeffs_[5];
	double k5 = dewarpCoeffs_[6];
	double k6 = dewarpCoeffs_[7];
	double s1 = dewarpCoeffs_[8];
	double s2 = dewarpCoeffs_[9];
	double s3 = dewarpCoeffs_[10];
	double s4 = dewarpCoeffs_[11];

	y = (p.y() - dewarpM_[1][2]) / dewarpM_[1][1];
	x = (p.x() - dewarpM_[0][2] - y * dewarpM_[0][1]) / dewarpM_[0][0];

	double r2 = x * x + y * y;
	double d = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) /
		   (1 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2);
	x = x * d + 2 * p1 * x * y + p2 * (r2 + 2 * x * x) + s1 * r2 + s2 * r2 * r2;
	y = y * d + 2 * p2 * x * y + p1 * (r2 + 2 * y * y) + s3 * r2 + s4 * r2 * r2;

	return { { x * dewarpM_[0][0] + y * dewarpM_[0][1] + dewarpM_[0][2],
		   y * dewarpM_[1][1] + dewarpM_[1][2] } };
}

} /* namespace libcamera */
