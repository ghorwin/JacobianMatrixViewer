/*	JacobianMatrixViewer

	Copyright (c) 2017-today, Institut für Bauklimatik, TU Dresden, Germany

	Primary authors:
	  Andreas Nicolai  <andreas.nicolai -[at]- tu-dresden.de>

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.
*/

#include "MainDialog.h"

#include <QApplication>

#include <IBK_Exception.h>
#include <IBK_messages.h>
#include <IBK_ArgParser.h>
#include <IBK_BuildFlags.h>

const char * const LONG_VERSION = "1.0.0";

/*! qDebug() message handler function, redirects debug messages to IBK::IBK_Message(). */
void qDebugMsgHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg) {
	(void) type;
	std::string contextstr;
	if (context.file != nullptr && context.function != nullptr)
		contextstr = "[" + std::string(context.file) + "::" + std::string(context.function) + "]";
	IBK::IBK_Message(msg.toStdString(), IBK::MSG_DEBUG, contextstr.c_str(), IBK::VL_ALL);
}


int main(int argc, char *argv[]) {
	const char * const FUNC_ID = "[main]";

	// create QApplication
	QApplication a(argc, argv);

	// install message handler to catch qDebug()
	qInstallMessageHandler(qDebugMsgHandler);

	// *** Locale setup for Unix/Linux ***
#if defined(Q_OS_UNIX)
	setlocale(LC_NUMERIC,"C");
#endif

	// Compose program name using the always use the major.minor version variant,
	// since this string is used to identify the registry/config file location.
	const QString ProgramVersionName = QString("JacobianMatrixViewer %1").arg(LONG_VERSION);

	qApp->setApplicationName(ProgramVersionName);

	// disable ? button in windows
#if QT_VERSION >= 0x050A00
	QApplication::setAttribute(Qt::AA_DisableWindowContextHelpButton);
#elif QT_VERSION >= 0x050600
	QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

	// *** Setup and show MainDialog and start event loop ***
	MainDialog w;
	w.show();

	// start event loop
	int res = a.exec();

	// return exit code to environment
	return res;
}
