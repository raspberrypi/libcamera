/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Utkarsh Tiwari <utkarsh02t@gmail.com>
 *
 * cam_select_dialog.h - qcam - Camera Selection dialog
 */

#pragma once

#include <string>

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>
#include <libcamera/controls.h>
#include <libcamera/property_ids.h>

#include <QDialog>
#include <QString>

class QComboBox;
class QLabel;

class CameraSelectorDialog : public QDialog
{
	Q_OBJECT
public:
	CameraSelectorDialog(libcamera::CameraManager *cameraManager,
			     QWidget *parent);
	~CameraSelectorDialog();

	std::string getCameraId();

	/* Hotplug / Unplug Support. */
	void addCamera(QString cameraId);
	void removeCamera(QString cameraId);

	/* Camera Information */
	void updateCameraInfo(QString cameraId);

private:
	libcamera::CameraManager *cm_;

	/* UI elements. */
	QComboBox *cameraIdComboBox_;
	QLabel *cameraLocation_;
	QLabel *cameraModel_;
};
