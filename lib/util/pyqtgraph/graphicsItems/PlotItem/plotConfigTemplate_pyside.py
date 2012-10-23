# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './graphicsItems/PlotItem/plotConfigTemplate.ui'
#
# Created: Sun Sep  9 14:41:32 2012
#      by: pyside-uic 0.2.13 running on PySide 1.1.0
#
# WARNING! All changes made in this file will be lost!

from PySide import QtCore, QtGui

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(258, 605)
        self.averageGroup = QtGui.QGroupBox(Form)
        self.averageGroup.setGeometry(QtCore.QRect(10, 200, 242, 182))
        self.averageGroup.setCheckable(True)
        self.averageGroup.setChecked(False)
        self.averageGroup.setObjectName("averageGroup")
        self.gridLayout_5 = QtGui.QGridLayout(self.averageGroup)
        self.gridLayout_5.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_5.setSpacing(0)
        self.gridLayout_5.setObjectName("gridLayout_5")
        self.avgParamList = QtGui.QListWidget(self.averageGroup)
        self.avgParamList.setObjectName("avgParamList")
        self.gridLayout_5.addWidget(self.avgParamList, 0, 0, 1, 1)
        self.decimateGroup = QtGui.QGroupBox(Form)
        self.decimateGroup.setGeometry(QtCore.QRect(0, 70, 242, 160))
        self.decimateGroup.setCheckable(True)
        self.decimateGroup.setObjectName("decimateGroup")
        self.gridLayout_4 = QtGui.QGridLayout(self.decimateGroup)
        self.gridLayout_4.setContentsMargins(0, 0, 0, 0)
        self.gridLayout_4.setSpacing(0)
        self.gridLayout_4.setObjectName("gridLayout_4")
        self.manualDecimateRadio = QtGui.QRadioButton(self.decimateGroup)
        self.manualDecimateRadio.setChecked(True)
        self.manualDecimateRadio.setObjectName("manualDecimateRadio")
        self.gridLayout_4.addWidget(self.manualDecimateRadio, 0, 0, 1, 1)
        self.downsampleSpin = QtGui.QSpinBox(self.decimateGroup)
        self.downsampleSpin.setMinimum(1)
        self.downsampleSpin.setMaximum(100000)
        self.downsampleSpin.setProperty("value", 1)
        self.downsampleSpin.setObjectName("downsampleSpin")
        self.gridLayout_4.addWidget(self.downsampleSpin, 0, 1, 1, 1)
        self.autoDecimateRadio = QtGui.QRadioButton(self.decimateGroup)
        self.autoDecimateRadio.setChecked(False)
        self.autoDecimateRadio.setObjectName("autoDecimateRadio")
        self.gridLayout_4.addWidget(self.autoDecimateRadio, 1, 0, 1, 1)
        self.maxTracesCheck = QtGui.QCheckBox(self.decimateGroup)
        self.maxTracesCheck.setObjectName("maxTracesCheck")
        self.gridLayout_4.addWidget(self.maxTracesCheck, 2, 0, 1, 1)
        self.maxTracesSpin = QtGui.QSpinBox(self.decimateGroup)
        self.maxTracesSpin.setObjectName("maxTracesSpin")
        self.gridLayout_4.addWidget(self.maxTracesSpin, 2, 1, 1, 1)
        self.forgetTracesCheck = QtGui.QCheckBox(self.decimateGroup)
        self.forgetTracesCheck.setObjectName("forgetTracesCheck")
        self.gridLayout_4.addWidget(self.forgetTracesCheck, 3, 0, 1, 2)
        self.transformGroup = QtGui.QFrame(Form)
        self.transformGroup.setGeometry(QtCore.QRect(0, 0, 154, 79))
        self.transformGroup.setObjectName("transformGroup")
        self.gridLayout = QtGui.QGridLayout(self.transformGroup)
        self.gridLayout.setObjectName("gridLayout")
        self.fftCheck = QtGui.QCheckBox(self.transformGroup)
        self.fftCheck.setObjectName("fftCheck")
        self.gridLayout.addWidget(self.fftCheck, 0, 0, 1, 1)
        self.logXCheck = QtGui.QCheckBox(self.transformGroup)
        self.logXCheck.setObjectName("logXCheck")
        self.gridLayout.addWidget(self.logXCheck, 1, 0, 1, 1)
        self.logYCheck = QtGui.QCheckBox(self.transformGroup)
        self.logYCheck.setObjectName("logYCheck")
        self.gridLayout.addWidget(self.logYCheck, 2, 0, 1, 1)
        self.pointsGroup = QtGui.QGroupBox(Form)
        self.pointsGroup.setGeometry(QtCore.QRect(10, 550, 234, 58))
        self.pointsGroup.setCheckable(True)
        self.pointsGroup.setObjectName("pointsGroup")
        self.verticalLayout_5 = QtGui.QVBoxLayout(self.pointsGroup)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.autoPointsCheck = QtGui.QCheckBox(self.pointsGroup)
        self.autoPointsCheck.setChecked(True)
        self.autoPointsCheck.setObjectName("autoPointsCheck")
        self.verticalLayout_5.addWidget(self.autoPointsCheck)
        self.gridGroup = QtGui.QFrame(Form)
        self.gridGroup.setGeometry(QtCore.QRect(10, 460, 221, 81))
        self.gridGroup.setObjectName("gridGroup")
        self.gridLayout_2 = QtGui.QGridLayout(self.gridGroup)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.xGridCheck = QtGui.QCheckBox(self.gridGroup)
        self.xGridCheck.setObjectName("xGridCheck")
        self.gridLayout_2.addWidget(self.xGridCheck, 0, 0, 1, 2)
        self.yGridCheck = QtGui.QCheckBox(self.gridGroup)
        self.yGridCheck.setObjectName("yGridCheck")
        self.gridLayout_2.addWidget(self.yGridCheck, 1, 0, 1, 2)
        self.gridAlphaSlider = QtGui.QSlider(self.gridGroup)
        self.gridAlphaSlider.setMaximum(255)
        self.gridAlphaSlider.setProperty("value", 128)
        self.gridAlphaSlider.setOrientation(QtCore.Qt.Horizontal)
        self.gridAlphaSlider.setObjectName("gridAlphaSlider")
        self.gridLayout_2.addWidget(self.gridAlphaSlider, 2, 1, 1, 1)
        self.label = QtGui.QLabel(self.gridGroup)
        self.label.setObjectName("label")
        self.gridLayout_2.addWidget(self.label, 2, 0, 1, 1)
        self.alphaGroup = QtGui.QGroupBox(Form)
        self.alphaGroup.setGeometry(QtCore.QRect(10, 390, 234, 60))
        self.alphaGroup.setCheckable(True)
        self.alphaGroup.setObjectName("alphaGroup")
        self.horizontalLayout = QtGui.QHBoxLayout(self.alphaGroup)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.autoAlphaCheck = QtGui.QCheckBox(self.alphaGroup)
        self.autoAlphaCheck.setChecked(False)
        self.autoAlphaCheck.setObjectName("autoAlphaCheck")
        self.horizontalLayout.addWidget(self.autoAlphaCheck)
        self.alphaSlider = QtGui.QSlider(self.alphaGroup)
        self.alphaSlider.setMaximum(1000)
        self.alphaSlider.setProperty("value", 1000)
        self.alphaSlider.setOrientation(QtCore.Qt.Horizontal)
        self.alphaSlider.setObjectName("alphaSlider")
        self.horizontalLayout.addWidget(self.alphaSlider)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtGui.QApplication.translate("Form", "Form", None, QtGui.QApplication.UnicodeUTF8))
        self.averageGroup.setToolTip(QtGui.QApplication.translate("Form", "Display averages of the curves displayed in this plot. The parameter list allows you to choose parameters to average over (if any are available).", None, QtGui.QApplication.UnicodeUTF8))
        self.averageGroup.setTitle(QtGui.QApplication.translate("Form", "Average", None, QtGui.QApplication.UnicodeUTF8))
        self.decimateGroup.setTitle(QtGui.QApplication.translate("Form", "Downsample", None, QtGui.QApplication.UnicodeUTF8))
        self.manualDecimateRadio.setText(QtGui.QApplication.translate("Form", "Manual", None, QtGui.QApplication.UnicodeUTF8))
        self.autoDecimateRadio.setText(QtGui.QApplication.translate("Form", "Auto", None, QtGui.QApplication.UnicodeUTF8))
        self.maxTracesCheck.setToolTip(QtGui.QApplication.translate("Form", "If multiple curves are displayed in this plot, check this box to limit the number of traces that are displayed.", None, QtGui.QApplication.UnicodeUTF8))
        self.maxTracesCheck.setText(QtGui.QApplication.translate("Form", "Max Traces:", None, QtGui.QApplication.UnicodeUTF8))
        self.maxTracesSpin.setToolTip(QtGui.QApplication.translate("Form", "If multiple curves are displayed in this plot, check \"Max Traces\" and set this value to limit the number of traces that are displayed.", None, QtGui.QApplication.UnicodeUTF8))
        self.forgetTracesCheck.setToolTip(QtGui.QApplication.translate("Form", "If MaxTraces is checked, remove curves from memory after they are hidden (saves memory, but traces can not be un-hidden).", None, QtGui.QApplication.UnicodeUTF8))
        self.forgetTracesCheck.setText(QtGui.QApplication.translate("Form", "Forget hidden traces", None, QtGui.QApplication.UnicodeUTF8))
        self.fftCheck.setText(QtGui.QApplication.translate("Form", "Power Spectrum (FFT)", None, QtGui.QApplication.UnicodeUTF8))
        self.logXCheck.setText(QtGui.QApplication.translate("Form", "Log X", None, QtGui.QApplication.UnicodeUTF8))
        self.logYCheck.setText(QtGui.QApplication.translate("Form", "Log Y", None, QtGui.QApplication.UnicodeUTF8))
        self.pointsGroup.setTitle(QtGui.QApplication.translate("Form", "Points", None, QtGui.QApplication.UnicodeUTF8))
        self.autoPointsCheck.setText(QtGui.QApplication.translate("Form", "Auto", None, QtGui.QApplication.UnicodeUTF8))
        self.xGridCheck.setText(QtGui.QApplication.translate("Form", "Show X Grid", None, QtGui.QApplication.UnicodeUTF8))
        self.yGridCheck.setText(QtGui.QApplication.translate("Form", "Show Y Grid", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("Form", "Opacity", None, QtGui.QApplication.UnicodeUTF8))
        self.alphaGroup.setTitle(QtGui.QApplication.translate("Form", "Alpha", None, QtGui.QApplication.UnicodeUTF8))
        self.autoAlphaCheck.setText(QtGui.QApplication.translate("Form", "Auto", None, QtGui.QApplication.UnicodeUTF8))

