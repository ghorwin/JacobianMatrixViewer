# Session file for JacobianMatrixViewer
TEMPLATE = subdirs

SUBDIRS = \
		JacobianMatrixViewer \
		IBKMK \
		IBK

# where to find the sub projects
JacobianMatrixViewer.file = ../../JacobianMatrixViewer/projects/Qt/JacobianMatrixViewer.pro

IBK.file = ../../externals/IBK/projects/Qt/IBK.pro
IBKMK.file = ../../externals/IBKMK/projects/Qt/IBKMK.pro

# dependencies
JacobianMatrixViewer.depends = IBK IBKMK
IBKMK.depends = IBK

