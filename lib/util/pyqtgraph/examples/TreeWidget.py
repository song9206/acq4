# -*- coding: utf-8 -*-
import initExample ## Add path to library (just for examples; you do not need this)

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np


app = QtGui.QApplication([])

w = pg.TreeWidget()
w.setColumnCount(2)
w.show()

i1  = QtGui.QTreeWidgetItem(["Item 1"])
i11  = QtGui.QTreeWidgetItem(["Item 1.1"])
i12  = QtGui.QTreeWidgetItem(["Item 1.2"])
i2  = QtGui.QTreeWidgetItem(["Item 2"])
i21  = QtGui.QTreeWidgetItem(["Item 2.1"])
i211  = QtGui.QTreeWidgetItem(["Item 2.1.1"])
i212  = QtGui.QTreeWidgetItem(["Item 2.1.2"])
i22  = QtGui.QTreeWidgetItem(["Item 2.2"])
i3  = QtGui.QTreeWidgetItem(["Item 3"])
i4  = QtGui.QTreeWidgetItem(["Item 4"])
i5  = QtGui.QTreeWidgetItem(["Item 5"])

w.addTopLevelItem(i1)
w.addTopLevelItem(i2)
w.addTopLevelItem(i3)
w.addTopLevelItem(i4)
w.addTopLevelItem(i5)
i1.addChild(i11)
i1.addChild(i12)
i2.addChild(i21)
i21.addChild(i211)
i21.addChild(i212)
i2.addChild(i22)

b1 = QtGui.QPushButton("B1")
w.setItemWidget(i1, 1, b1)

## Start Qt event loop unless running in interactive mode or using pyside.
import sys
if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
    QtGui.QApplication.instance().exec_()
