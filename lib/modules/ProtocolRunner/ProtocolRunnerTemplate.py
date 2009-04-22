# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ProtocolRunnerTemplate.ui'
#
# Created: Wed Apr 22 14:02:34 2009
#      by: PyQt4 UI code generator 4.3.3
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(QtCore.QSize(QtCore.QRect(0,0,1118,593).size()).expandedTo(MainWindow.minimumSizeHint()))
        MainWindow.setDockNestingEnabled(True)

        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        MainWindow.setCentralWidget(self.centralwidget)

        self.LoaderDock = QtGui.QDockWidget(MainWindow)
        self.LoaderDock.setFeatures(QtGui.QDockWidget.DockWidgetFloatable|QtGui.QDockWidget.DockWidgetMovable|QtGui.QDockWidget.DockWidgetVerticalTitleBar|QtGui.QDockWidget.NoDockWidgetFeatures)
        self.LoaderDock.setObjectName("LoaderDock")

        self.dockWidgetContents = QtGui.QWidget(self.LoaderDock)
        self.dockWidgetContents.setObjectName("dockWidgetContents")

        self.gridlayout = QtGui.QGridLayout(self.dockWidgetContents)
        self.gridlayout.setObjectName("gridlayout")

        self.newProtocolBtn = QtGui.QPushButton(self.dockWidgetContents)
        self.newProtocolBtn.setObjectName("newProtocolBtn")
        self.gridlayout.addWidget(self.newProtocolBtn,1,1,1,1)

        self.loadProtocolBtn = QtGui.QPushButton(self.dockWidgetContents)
        self.loadProtocolBtn.setObjectName("loadProtocolBtn")
        self.gridlayout.addWidget(self.loadProtocolBtn,2,1,1,1)

        self.saveProtocolBtn = QtGui.QPushButton(self.dockWidgetContents)
        self.saveProtocolBtn.setEnabled(False)
        self.saveProtocolBtn.setObjectName("saveProtocolBtn")
        self.gridlayout.addWidget(self.saveProtocolBtn,3,1,1,1)

        self.saveAsProtocolBtn = QtGui.QPushButton(self.dockWidgetContents)
        self.saveAsProtocolBtn.setEnabled(True)
        self.saveAsProtocolBtn.setObjectName("saveAsProtocolBtn")
        self.gridlayout.addWidget(self.saveAsProtocolBtn,4,1,1,1)

        spacerItem = QtGui.QSpacerItem(20,77,QtGui.QSizePolicy.Minimum,QtGui.QSizePolicy.Expanding)
        self.gridlayout.addItem(spacerItem,6,1,1,1)

        self.deleteProtocolBtn = QtGui.QPushButton(self.dockWidgetContents)
        self.deleteProtocolBtn.setEnabled(False)
        self.deleteProtocolBtn.setObjectName("deleteProtocolBtn")
        self.gridlayout.addWidget(self.deleteProtocolBtn,5,1,1,1)

        self.protocolList = QtGui.QTreeView(self.dockWidgetContents)
        self.protocolList.setAcceptDrops(True)
        self.protocolList.setDragEnabled(True)
        self.protocolList.setDragDropMode(QtGui.QAbstractItemView.InternalMove)
        self.protocolList.setObjectName("protocolList")
        self.gridlayout.addWidget(self.protocolList,2,0,5,1)

        self.label_3 = QtGui.QLabel(self.dockWidgetContents)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName("label_3")
        self.gridlayout.addWidget(self.label_3,1,0,1,1)
        self.LoaderDock.setWidget(self.dockWidgetContents)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(4),self.LoaderDock)

        self.ProtocolDock = QtGui.QDockWidget(MainWindow)
        self.ProtocolDock.setEnabled(True)
        self.ProtocolDock.setFeatures(QtGui.QDockWidget.DockWidgetFloatable|QtGui.QDockWidget.DockWidgetMovable|QtGui.QDockWidget.DockWidgetVerticalTitleBar|QtGui.QDockWidget.NoDockWidgetFeatures)
        self.ProtocolDock.setObjectName("ProtocolDock")

        self.dockWidgetContents_5 = QtGui.QWidget(self.ProtocolDock)
        self.dockWidgetContents_5.setObjectName("dockWidgetContents_5")

        self.gridlayout1 = QtGui.QGridLayout(self.dockWidgetContents_5)
        self.gridlayout1.setObjectName("gridlayout1")

        self.deviceList = QtGui.QListWidget(self.dockWidgetContents_5)
        self.deviceList.setObjectName("deviceList")
        self.gridlayout1.addWidget(self.deviceList,3,0,3,2)

        self.label_8 = QtGui.QLabel(self.dockWidgetContents_5)
        self.label_8.setObjectName("label_8")
        self.gridlayout1.addWidget(self.label_8,2,2,1,1)

        self.protoDurationSpin = QtGui.QDoubleSpinBox(self.dockWidgetContents_5)
        self.protoDurationSpin.setObjectName("protoDurationSpin")
        self.gridlayout1.addWidget(self.protoDurationSpin,3,2,1,1)

        self.protoContinuousCheck = QtGui.QCheckBox(self.dockWidgetContents_5)
        self.protoContinuousCheck.setObjectName("protoContinuousCheck")
        self.gridlayout1.addWidget(self.protoContinuousCheck,4,2,1,1)

        spacerItem1 = QtGui.QSpacerItem(20,91,QtGui.QSizePolicy.Minimum,QtGui.QSizePolicy.Expanding)
        self.gridlayout1.addItem(spacerItem1,5,2,1,1)

        self.testSingleBtn = QtGui.QPushButton(self.dockWidgetContents_5)
        self.testSingleBtn.setEnabled(False)
        self.testSingleBtn.setObjectName("testSingleBtn")
        self.gridlayout1.addWidget(self.testSingleBtn,6,0,1,1)

        spacerItem2 = QtGui.QSpacerItem(13,20,QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Minimum)
        self.gridlayout1.addItem(spacerItem2,6,1,1,1)

        self.runProtocolBtn = QtGui.QPushButton(self.dockWidgetContents_5)
        self.runProtocolBtn.setEnabled(False)
        self.runProtocolBtn.setObjectName("runProtocolBtn")
        self.gridlayout1.addWidget(self.runProtocolBtn,6,2,1,1)

        self.label = QtGui.QLabel(self.dockWidgetContents_5)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName("label")
        self.gridlayout1.addWidget(self.label,2,0,1,2)
        self.ProtocolDock.setWidget(self.dockWidgetContents_5)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(4),self.ProtocolDock)

        self.SequenceDock = QtGui.QDockWidget(MainWindow)
        self.SequenceDock.setEnabled(False)
        self.SequenceDock.setFeatures(QtGui.QDockWidget.DockWidgetFloatable|QtGui.QDockWidget.DockWidgetMovable|QtGui.QDockWidget.DockWidgetVerticalTitleBar|QtGui.QDockWidget.NoDockWidgetFeatures)
        self.SequenceDock.setObjectName("SequenceDock")

        self.dockWidgetContents_7 = QtGui.QWidget(self.SequenceDock)
        self.dockWidgetContents_7.setObjectName("dockWidgetContents_7")

        self.gridlayout2 = QtGui.QGridLayout(self.dockWidgetContents_7)
        self.gridlayout2.setObjectName("gridlayout2")

        self.sequenceParamList = QtGui.QListWidget(self.dockWidgetContents_7)
        self.sequenceParamList.setObjectName("sequenceParamList")
        self.gridlayout2.addWidget(self.sequenceParamList,0,0,6,2)

        self.seqParamUpBtn = QtGui.QPushButton(self.dockWidgetContents_7)
        self.seqParamUpBtn.setObjectName("seqParamUpBtn")
        self.gridlayout2.addWidget(self.seqParamUpBtn,0,2,1,1)

        self.seqParamDnBtn = QtGui.QPushButton(self.dockWidgetContents_7)
        self.seqParamDnBtn.setObjectName("seqParamDnBtn")
        self.gridlayout2.addWidget(self.seqParamDnBtn,1,2,1,1)

        self.seqParamGroupBtn = QtGui.QPushButton(self.dockWidgetContents_7)
        self.seqParamGroupBtn.setObjectName("seqParamGroupBtn")
        self.gridlayout2.addWidget(self.seqParamGroupBtn,2,2,1,1)

        self.label_2 = QtGui.QLabel(self.dockWidgetContents_7)
        self.label_2.setObjectName("label_2")
        self.gridlayout2.addWidget(self.label_2,3,2,1,1)

        self.paramSpaceLabel = QtGui.QLabel(self.dockWidgetContents_7)
        self.paramSpaceLabel.setObjectName("paramSpaceLabel")
        self.gridlayout2.addWidget(self.paramSpaceLabel,4,2,1,1)

        spacerItem3 = QtGui.QSpacerItem(20,77,QtGui.QSizePolicy.Minimum,QtGui.QSizePolicy.Expanding)
        self.gridlayout2.addItem(spacerItem3,5,2,1,1)

        self.testSequenceBtn = QtGui.QPushButton(self.dockWidgetContents_7)
        self.testSequenceBtn.setObjectName("testSequenceBtn")
        self.gridlayout2.addWidget(self.testSequenceBtn,6,0,1,1)

        spacerItem4 = QtGui.QSpacerItem(146,20,QtGui.QSizePolicy.Expanding,QtGui.QSizePolicy.Minimum)
        self.gridlayout2.addItem(spacerItem4,6,1,1,1)

        self.runSequenceBtn = QtGui.QPushButton(self.dockWidgetContents_7)
        self.runSequenceBtn.setObjectName("runSequenceBtn")
        self.gridlayout2.addWidget(self.runSequenceBtn,6,2,1,1)
        self.SequenceDock.setWidget(self.dockWidgetContents_7)
        MainWindow.addDockWidget(QtCore.Qt.DockWidgetArea(4),self.SequenceDock)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtGui.QApplication.translate("MainWindow", "MainWindow", None, QtGui.QApplication.UnicodeUTF8))
        MainWindow.setStyleSheet(QtGui.QApplication.translate("MainWindow", "QDockWidget::title { background-color: #446; color: #DDD }\n"
        "QDockWidget > QWidget { background-color: #BBB }\n"
        "QMainWindow { background-color: #000 }\n"
        "QSplitter::handle {background-color: #666}\n"
        "QMainWindow::separator {background-color: #666}", None, QtGui.QApplication.UnicodeUTF8))
        self.LoaderDock.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Loader", None, QtGui.QApplication.UnicodeUTF8))
        self.newProtocolBtn.setText(QtGui.QApplication.translate("MainWindow", "New", None, QtGui.QApplication.UnicodeUTF8))
        self.loadProtocolBtn.setText(QtGui.QApplication.translate("MainWindow", "Load", None, QtGui.QApplication.UnicodeUTF8))
        self.saveProtocolBtn.setText(QtGui.QApplication.translate("MainWindow", "Save", None, QtGui.QApplication.UnicodeUTF8))
        self.saveAsProtocolBtn.setText(QtGui.QApplication.translate("MainWindow", "Save As..", None, QtGui.QApplication.UnicodeUTF8))
        self.deleteProtocolBtn.setText(QtGui.QApplication.translate("MainWindow", "Delete", None, QtGui.QApplication.UnicodeUTF8))
        self.label_3.setText(QtGui.QApplication.translate("MainWindow", "Protocols", None, QtGui.QApplication.UnicodeUTF8))
        self.ProtocolDock.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Protocol", None, QtGui.QApplication.UnicodeUTF8))
        self.label_8.setText(QtGui.QApplication.translate("MainWindow", "Duration", None, QtGui.QApplication.UnicodeUTF8))
        self.protoContinuousCheck.setText(QtGui.QApplication.translate("MainWindow", "Continuous", None, QtGui.QApplication.UnicodeUTF8))
        self.testSingleBtn.setText(QtGui.QApplication.translate("MainWindow", "Test", None, QtGui.QApplication.UnicodeUTF8))
        self.runProtocolBtn.setText(QtGui.QApplication.translate("MainWindow", "Run Protocol", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("MainWindow", "Devices", None, QtGui.QApplication.UnicodeUTF8))
        self.SequenceDock.setWindowTitle(QtGui.QApplication.translate("MainWindow", "Sequence", None, QtGui.QApplication.UnicodeUTF8))
        self.seqParamUpBtn.setText(QtGui.QApplication.translate("MainWindow", "Up", None, QtGui.QApplication.UnicodeUTF8))
        self.seqParamDnBtn.setText(QtGui.QApplication.translate("MainWindow", "Dn", None, QtGui.QApplication.UnicodeUTF8))
        self.seqParamGroupBtn.setText(QtGui.QApplication.translate("MainWindow", "Group", None, QtGui.QApplication.UnicodeUTF8))
        self.label_2.setText(QtGui.QApplication.translate("MainWindow", "Parameter Space: ", None, QtGui.QApplication.UnicodeUTF8))
        self.testSequenceBtn.setText(QtGui.QApplication.translate("MainWindow", "Test", None, QtGui.QApplication.UnicodeUTF8))
        self.runSequenceBtn.setText(QtGui.QApplication.translate("MainWindow", "Run Sequence", None, QtGui.QApplication.UnicodeUTF8))

