# Session file for JacobianMatrixViewer
TEMPLATE = subdirs

SUBDIRS = \
		JacobianMatrixViewer \
		IBKMK \
		IBK

IBK.file = externals/IBK/IBK.pro
IBKMK.file = externals/IBKMK/IBKMK.pro

# dependencies
JacobianMatrixViewer.depends = IBK IBKMK
IBKMK.depends = IBK

