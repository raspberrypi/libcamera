/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2022, Utkarsh Tiwari <utkarsh02t@gmail.com>
 *
 * cam_select_dialog.cpp - qcam - Camera Selection dialog
 */

#include "cam_select_dialog.h"

#include <libcamera/camera.h>
#include <libcamera/camera_manager.h>

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QString>

CameraSelectorDialog::CameraSelectorDialog(libcamera::CameraManager *cameraManager,
					   QWidget *parent)
	: QDialog(parent), cm_(cameraManager)
{
	/* Use a QFormLayout for the dialog. */
	QFormLayout *layout = new QFormLayout(this);

	/* Setup the camera id combo-box. */
	cameraIdComboBox_ = new QComboBox;
	for (const auto &cam : cm_->cameras())
		cameraIdComboBox_->addItem(QString::fromStdString(cam->id()));

	/* Setup the QDialogButton Box */
	QDialogButtonBox *buttonBox =
		new QDialogButtonBox(QDialogButtonBox::Ok |
				     QDialogButtonBox::Cancel);

	connect(buttonBox, &QDialogButtonBox::accepted,
		this, &QDialog::accept);
	connect(buttonBox, &QDialogButtonBox::rejected,
		this, &QDialog::reject);

	/* Set the layout. */
	layout->addRow("Camera:", cameraIdComboBox_);
	layout->addWidget(buttonBox);
}

CameraSelectorDialog::~CameraSelectorDialog() = default;

std::string CameraSelectorDialog::getCameraId()
{
	return cameraIdComboBox_->currentText().toStdString();
}
