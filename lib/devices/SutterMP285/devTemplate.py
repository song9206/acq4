# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'devTemplate.ui'
#
# Created: Wed Aug 31 11:25:48 2011
#      by: PyQt4 UI code generator 4.8.4
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(228, 221)
        self.gridLayout_4 = QtGui.QGridLayout(Form)
        self.gridLayout_4.setMargin(3)
        self.gridLayout_4.setSpacing(3)
        self.gridLayout_4.setObjectName(_fromUtf8("gridLayout_4"))
        self.groupBox_2 = QtGui.QGroupBox(Form)
        self.groupBox_2.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox_2.setObjectName(_fromUtf8("groupBox_2"))
        self.gridLayout_3 = QtGui.QGridLayout(self.groupBox_2)
        self.gridLayout_3.setMargin(5)
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.widget = QtGui.QWidget(self.groupBox_2)
        self.widget.setMinimumSize(QtCore.QSize(100, 0))
        self.widget.setObjectName(_fromUtf8("widget"))
        self.gridLayout = QtGui.QGridLayout(self.widget)
        self.gridLayout.setMargin(0)
        self.gridLayout.setMargin(0)
        self.gridLayout.setHorizontalSpacing(10)
        self.gridLayout.setVerticalSpacing(0)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.label_5 = QtGui.QLabel(self.widget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_5.sizePolicy().hasHeightForWidth())
        self.label_5.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_5.setFont(font)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout.addWidget(self.label_5, 0, 0, 1, 1)
        self.xPosLabel = QtGui.QLabel(self.widget)
        self.xPosLabel.setObjectName(_fromUtf8("xPosLabel"))
        self.gridLayout.addWidget(self.xPosLabel, 0, 1, 1, 1)
        self.label_7 = QtGui.QLabel(self.widget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_7.sizePolicy().hasHeightForWidth())
        self.label_7.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_7.setFont(font)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.gridLayout.addWidget(self.label_7, 1, 0, 1, 1)
        self.yPosLabel = QtGui.QLabel(self.widget)
        self.yPosLabel.setObjectName(_fromUtf8("yPosLabel"))
        self.gridLayout.addWidget(self.yPosLabel, 1, 1, 1, 1)
        self.label_8 = QtGui.QLabel(self.widget)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_8.sizePolicy().hasHeightForWidth())
        self.label_8.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_8.setFont(font)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout.addWidget(self.label_8, 2, 0, 1, 1)
        self.zPosLabel = QtGui.QLabel(self.widget)
        self.zPosLabel.setObjectName(_fromUtf8("zPosLabel"))
        self.gridLayout.addWidget(self.zPosLabel, 2, 1, 1, 1)
        self.updatePosBtn = QtGui.QPushButton(self.widget)
        self.updatePosBtn.setObjectName(_fromUtf8("updatePosBtn"))
        self.gridLayout.addWidget(self.updatePosBtn, 3, 0, 1, 2)
        self.gridLayout_3.addWidget(self.widget, 0, 0, 1, 1)
        self.joyBtn = JoystickButton(self.groupBox_2)
        self.joyBtn.setMinimumSize(QtCore.QSize(50, 50))
        self.joyBtn.setText(_fromUtf8(""))
        self.joyBtn.setObjectName(_fromUtf8("joyBtn"))
        self.gridLayout_3.addWidget(self.joyBtn, 0, 2, 1, 1)
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.fineStepRadio = QtGui.QRadioButton(self.groupBox_2)
        self.fineStepRadio.setChecked(True)
        self.fineStepRadio.setObjectName(_fromUtf8("fineStepRadio"))
        self.horizontalLayout_2.addWidget(self.fineStepRadio)
        self.coarseStepRadio = QtGui.QRadioButton(self.groupBox_2)
        self.coarseStepRadio.setObjectName(_fromUtf8("coarseStepRadio"))
        self.horizontalLayout_2.addWidget(self.coarseStepRadio)
        self.gridLayout_3.addLayout(self.horizontalLayout_2, 1, 0, 1, 3)
        spacerItem = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout_3.addItem(spacerItem, 0, 1, 1, 1)
        self.gridLayout_4.addWidget(self.groupBox_2, 0, 0, 1, 1)
        self.limitsGroup = QtGui.QGroupBox(Form)
        self.limitsGroup.setAlignment(QtCore.Qt.AlignCenter)
        self.limitsGroup.setCheckable(False)
        self.limitsGroup.setObjectName(_fromUtf8("limitsGroup"))
        self.gridLayout_2 = QtGui.QGridLayout(self.limitsGroup)
        self.gridLayout_2.setSpacing(1)
        self.gridLayout_2.setContentsMargins(3, 0, 3, 0)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.xMinBtn = QtGui.QPushButton(self.limitsGroup)
        self.xMinBtn.setMaximumSize(QtCore.QSize(70, 20))
        self.xMinBtn.setObjectName(_fromUtf8("xMinBtn"))
        self.gridLayout_2.addWidget(self.xMinBtn, 0, 1, 1, 1)
        self.xMinLabel = QtGui.QLabel(self.limitsGroup)
        self.xMinLabel.setObjectName(_fromUtf8("xMinLabel"))
        self.gridLayout_2.addWidget(self.xMinLabel, 0, 2, 1, 1)
        self.xMaxLabel = QtGui.QLabel(self.limitsGroup)
        self.xMaxLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.xMaxLabel.setObjectName(_fromUtf8("xMaxLabel"))
        self.gridLayout_2.addWidget(self.xMaxLabel, 0, 4, 1, 1)
        self.xMaxBtn = QtGui.QPushButton(self.limitsGroup)
        self.xMaxBtn.setMaximumSize(QtCore.QSize(70, 20))
        self.xMaxBtn.setObjectName(_fromUtf8("xMaxBtn"))
        self.gridLayout_2.addWidget(self.xMaxBtn, 0, 5, 1, 1)
        self.yMinBtn = QtGui.QPushButton(self.limitsGroup)
        self.yMinBtn.setMaximumSize(QtCore.QSize(70, 20))
        self.yMinBtn.setObjectName(_fromUtf8("yMinBtn"))
        self.gridLayout_2.addWidget(self.yMinBtn, 1, 1, 1, 1)
        self.yMinLabel = QtGui.QLabel(self.limitsGroup)
        self.yMinLabel.setObjectName(_fromUtf8("yMinLabel"))
        self.gridLayout_2.addWidget(self.yMinLabel, 1, 2, 1, 1)
        self.label_3 = QtGui.QLabel(self.limitsGroup)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_3.sizePolicy().hasHeightForWidth())
        self.label_3.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_3.setFont(font)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout_2.addWidget(self.label_3, 1, 3, 1, 1)
        self.yMaxLabel = QtGui.QLabel(self.limitsGroup)
        self.yMaxLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.yMaxLabel.setObjectName(_fromUtf8("yMaxLabel"))
        self.gridLayout_2.addWidget(self.yMaxLabel, 1, 4, 1, 1)
        self.yMaxBtn = QtGui.QPushButton(self.limitsGroup)
        self.yMaxBtn.setMaximumSize(QtCore.QSize(70, 20))
        self.yMaxBtn.setObjectName(_fromUtf8("yMaxBtn"))
        self.gridLayout_2.addWidget(self.yMaxBtn, 1, 5, 1, 1)
        self.zMinBtn = QtGui.QPushButton(self.limitsGroup)
        self.zMinBtn.setMaximumSize(QtCore.QSize(70, 20))
        self.zMinBtn.setObjectName(_fromUtf8("zMinBtn"))
        self.gridLayout_2.addWidget(self.zMinBtn, 2, 1, 1, 1)
        self.zMinLabel = QtGui.QLabel(self.limitsGroup)
        self.zMinLabel.setObjectName(_fromUtf8("zMinLabel"))
        self.gridLayout_2.addWidget(self.zMinLabel, 2, 2, 1, 1)
        self.label = QtGui.QLabel(self.limitsGroup)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout_2.addWidget(self.label, 2, 3, 1, 1)
        self.zMaxLabel = QtGui.QLabel(self.limitsGroup)
        self.zMaxLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.zMaxLabel.setObjectName(_fromUtf8("zMaxLabel"))
        self.gridLayout_2.addWidget(self.zMaxLabel, 2, 4, 1, 1)
        self.zMaxBtn = QtGui.QPushButton(self.limitsGroup)
        self.zMaxBtn.setMaximumSize(QtCore.QSize(70, 20))
        self.zMaxBtn.setObjectName(_fromUtf8("zMaxBtn"))
        self.gridLayout_2.addWidget(self.zMaxBtn, 2, 5, 1, 1)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_4 = QtGui.QLabel(self.limitsGroup)
        self.label_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.horizontalLayout_3.addWidget(self.label_4)
        self.maxSpeedSpin = SpinBox(self.limitsGroup)
        self.maxSpeedSpin.setObjectName(_fromUtf8("maxSpeedSpin"))
        self.horizontalLayout_3.addWidget(self.maxSpeedSpin)
        self.gridLayout_2.addLayout(self.horizontalLayout_3, 3, 1, 1, 5)
        self.xMinCheck = QtGui.QCheckBox(self.limitsGroup)
        self.xMinCheck.setText(_fromUtf8(""))
        self.xMinCheck.setObjectName(_fromUtf8("xMinCheck"))
        self.gridLayout_2.addWidget(self.xMinCheck, 0, 0, 1, 1)
        self.yMinCheck = QtGui.QCheckBox(self.limitsGroup)
        self.yMinCheck.setText(_fromUtf8(""))
        self.yMinCheck.setObjectName(_fromUtf8("yMinCheck"))
        self.gridLayout_2.addWidget(self.yMinCheck, 1, 0, 1, 1)
        self.zMinCheck = QtGui.QCheckBox(self.limitsGroup)
        self.zMinCheck.setText(_fromUtf8(""))
        self.zMinCheck.setObjectName(_fromUtf8("zMinCheck"))
        self.gridLayout_2.addWidget(self.zMinCheck, 2, 0, 1, 1)
        self.xMaxCheck = QtGui.QCheckBox(self.limitsGroup)
        self.xMaxCheck.setText(_fromUtf8(""))
        self.xMaxCheck.setObjectName(_fromUtf8("xMaxCheck"))
        self.gridLayout_2.addWidget(self.xMaxCheck, 0, 6, 1, 1)
        self.yMaxCheck = QtGui.QCheckBox(self.limitsGroup)
        self.yMaxCheck.setText(_fromUtf8(""))
        self.yMaxCheck.setObjectName(_fromUtf8("yMaxCheck"))
        self.gridLayout_2.addWidget(self.yMaxCheck, 1, 6, 1, 1)
        self.zMaxCheck = QtGui.QCheckBox(self.limitsGroup)
        self.zMaxCheck.setText(_fromUtf8(""))
        self.zMaxCheck.setObjectName(_fromUtf8("zMaxCheck"))
        self.gridLayout_2.addWidget(self.zMaxCheck, 2, 6, 1, 1)
        self.label_2 = QtGui.QLabel(self.limitsGroup)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.label_2.sizePolicy().hasHeightForWidth())
        self.label_2.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setWeight(75)
        font.setBold(True)
        self.label_2.setFont(font)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout_2.addWidget(self.label_2, 0, 3, 1, 1)
        self.gridLayout_4.addWidget(self.limitsGroup, 1, 0, 1, 1)
        spacerItem1 = QtGui.QSpacerItem(40, 20, QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Minimum)
        self.gridLayout_4.addItem(spacerItem1, 1, 1, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtGui.QApplication.translate("Form", "Form", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox_2.setTitle(QtGui.QApplication.translate("Form", "Position", None, QtGui.QApplication.UnicodeUTF8))
        self.label_5.setText(QtGui.QApplication.translate("Form", "X", None, QtGui.QApplication.UnicodeUTF8))
        self.xPosLabel.setText(QtGui.QApplication.translate("Form", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_7.setText(QtGui.QApplication.translate("Form", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.yPosLabel.setText(QtGui.QApplication.translate("Form", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.label_8.setText(QtGui.QApplication.translate("Form", "Z", None, QtGui.QApplication.UnicodeUTF8))
        self.zPosLabel.setText(QtGui.QApplication.translate("Form", "0", None, QtGui.QApplication.UnicodeUTF8))
        self.updatePosBtn.setText(QtGui.QApplication.translate("Form", "Update", None, QtGui.QApplication.UnicodeUTF8))
        self.fineStepRadio.setText(QtGui.QApplication.translate("Form", "Fine step", None, QtGui.QApplication.UnicodeUTF8))
        self.coarseStepRadio.setText(QtGui.QApplication.translate("Form", "Coarse step", None, QtGui.QApplication.UnicodeUTF8))
        self.limitsGroup.setTitle(QtGui.QApplication.translate("Form", "Limits", None, QtGui.QApplication.UnicodeUTF8))
        self.xMinBtn.setText(QtGui.QApplication.translate("Form", " ", None, QtGui.QApplication.UnicodeUTF8))
        self.xMinLabel.setText(QtGui.QApplication.translate("Form", "<--", None, QtGui.QApplication.UnicodeUTF8))
        self.xMaxLabel.setText(QtGui.QApplication.translate("Form", "-->", None, QtGui.QApplication.UnicodeUTF8))
        self.xMaxBtn.setText(QtGui.QApplication.translate("Form", " ", None, QtGui.QApplication.UnicodeUTF8))
        self.yMinBtn.setText(QtGui.QApplication.translate("Form", " ", None, QtGui.QApplication.UnicodeUTF8))
        self.yMinLabel.setText(QtGui.QApplication.translate("Form", "<--", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("Form", "Y", None, QtGui.QApplication.UnicodeUTF8))
        self.yMaxLabel.setText(QtGui.QApplication.translate("Form", "-->", None, QtGui.QApplication.UnicodeUTF8))
        self.yMaxBtn.setText(QtGui.QApplication.translate("Form", " ", None, QtGui.QApplication.UnicodeUTF8))
        self.zMinBtn.setText(QtGui.QApplication.translate("Form", " ", None, QtGui.QApplication.UnicodeUTF8))
        self.zMinLabel.setText(QtGui.QApplication.translate("Form", "<--", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("Form", "Z", None, QtGui.QApplication.UnicodeUTF8))
        self.zMaxLabel.setText(QtGui.QApplication.translate("Form", "-->", None, QtGui.QApplication.UnicodeUTF8))
        self.zMaxBtn.setText(QtGui.QApplication.translate("Form", " ", None, QtGui.QApplication.UnicodeUTF8))
        self.label_4.setText(QtGui.QApplication.translate("Form", "Max Speed", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("Form", "X", None, QtGui.QApplication.UnicodeUTF8))

from JoystickButton import JoystickButton
from SpinBox import SpinBox