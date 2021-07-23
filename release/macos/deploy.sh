#!/bin/bash

# This script is expected to be executed from within release/mac
# The script will be configured by the python script.

OUTPUT_DIRECTORY_NAME=JacobianMatrixViewer-1.0
OUTPUT_FILE_BASENAME=JacobianMatrixViewer-1.0.1-macosx

# cleanup existing deployment directory and dmg file
if [ -d $OUTPUT_DIRECTORY_NAME ]; then
    rm -rf $OUTPUT_DIRECTORY_NAME
fi
if [ -e $OUTPUT_FILE_BASENAME.dmg ]; then
    rm $OUTPUT_FILE_BASENAME.dmg
fi

mkdir -p $OUTPUT_DIRECTORY_NAME &&

echo '*** Creating Application bundle and resources to '$OUTPUT_DIRECTORY_NAME &&

# Copy application bundle into folder

cp -R ../../bin/release/JacobianMatrixViewer.app  ./$OUTPUT_DIRECTORY_NAME &&

# Create missing Release-Path and subdirs

mkdir -p $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/resources &&
#mkdir -p $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/resources/translations &&

# Copy translations

#cp ../../JacobianMatrixViewer/resources/translations/JacobianMatrixViewer_de.qm $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/resources/translations/ &&
#cp ../../JacobianMatrixViewer/resources/translations/qt_de.qm $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/resources/translations/ &&

# Copy manual

# Not available, yet.
# cp ../../../JacobianMatrixViewer/resources/tutorial.pdf $RELEASE_BASE/Resources/Documentation/ &&

# Copy application icon

cp -R ../../JacobianMatrixViewer/resources/JacobianMatrixViewer.icns  $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/resources &&

# Copy qt_menu.nib

#cp -R qt_menu.nib $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/resources/ &&


# deploy Qt framework libraries within app bundle and relink shared libraries
~/Qt/5.11.3/clang_64/bin/macdeployqt $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app -always-overwrite &&
install_name_tool -rpath ~/Qt/5.11.3/clang_64/lib @executable_path/../Frameworks $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/MacOS/JacobianMatrixViewer

# Copy Info.plist (configured by Python script)

cp -f Info.plist $OUTPUT_DIRECTORY_NAME/JacobianMatrixViewer.app/Contents/ &&

echo '*** Creating Symlinks to Applications and Documentation directories' &&
ln -s /Applications $OUTPUT_DIRECTORY_NAME/Applications &&
#ln -s /Library/Documentation $OUTPUT_DIRECTORY_NAME/Documentation &&

# finally create dmg
echo '*** Creating dmg' &&
../scripts/mkdmg $OUTPUT_DIRECTORY_NAME &&
mv $OUTPUT_DIRECTORY_NAME.dmg $OUTPUT_FILE_BASENAME.dmg &&

echo "Created '$OUTPUT_FILE_BASENAME'.dmg"
