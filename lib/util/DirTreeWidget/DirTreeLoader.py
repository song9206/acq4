# -*- coding: utf-8 -*-
from DirTreeTemplate import Ui_Form
from PyQt4 import QtGui,QtCore
from debug import *

class DirTreeLoader(QtGui.QWidget):
    def __init__(self, baseDir, *args):
        QtGui.QWidget.__init__(self, *args)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.baseDir = baseDir
        self.currentFile = None
        
        #self.fileTree = DirTreeModel(baseDir)
        #self.ui.fileTree.setModel(self.fileTree)
        
        self.ui.fileTree.setBaseDirHandle(baseDir)
        
        self.deleteState = 0

        self.ui.deleteBtn.focusOutEvent = self.delBtnLostFocus

        QtCore.QObject.connect(self.ui.newBtn, QtCore.SIGNAL('clicked()'), self.newClicked)
        QtCore.QObject.connect(self.ui.newDirBtn, QtCore.SIGNAL('clicked()'), self.newDirClicked)
        QtCore.QObject.connect(self.ui.saveBtn, QtCore.SIGNAL('clicked()'), self.saveClicked)
        QtCore.QObject.connect(self.ui.loadBtn, QtCore.SIGNAL('clicked()'), self.loadClicked)
        QtCore.QObject.connect(self.ui.saveAsBtn, QtCore.SIGNAL('clicked()'), self.saveAsClicked)
        QtCore.QObject.connect(self.ui.deleteBtn, QtCore.SIGNAL('clicked()'), self.deleteClicked)


    def selectedFile(self):
        return self.ui.fileTree.selectedFile()

    def newClicked(self):
        if self.new():
            self.setCurrentFile(None)
            #self.ui.currentLabel.setText('[ new ]')
            #self.ui.saveBtn.setEnabled(False)
            #self.currentFile = None
    
    def new(self):
        raise Exception("Function must be reimplemented in subclass.")
    
    def saveClicked(self):
        self.save(self.currentFile)
        
    def save(self, fileHandle):
        raise Exception("Function must be reimplemented in subclass.")
    
    def loadClicked(self):
        fh = self.ui.fileTree.selectedFile()
        
        if self.load(fh):
            fn = fh.name(relativeTo=self.baseDir)
            self.setCurrentFile(fh)
            #self.ui.currentLabel.setText(fn)
            #self.ui.saveBtn.setEnabled(True)
            #self.currentFile = fh

    def load(self, handle):
        raise Exception("Function must be reimplemented in subclass.")
    
    def saveAsClicked(self):
        ## Decide on new file name
        fileName = self.suggestNewFilename(self.currentFile)
        baseDir = self.selectedDir()
        
        fh = baseDir.createFile(fileName, autoIncrement=True)
            
        ## write
        if not self.save(fh):
            fh.delete()
            return
        
        
        ## Start editing new file name
        self.ui.fileTree.flushSignals()
        self.ui.fileTree.editItem(fh)
        
        self.setCurrentFile(fh)
        #self.ui.currentLabel.setText(fh.name(relativeTo=self.baseDir))
        #self.ui.saveBtn.setEnabled(True)
        #self.currentFile = fh

    def suggestNewFilename(self, fh):
        """Suggest a file name to use when saveAs is clicked. saveAsClicked will 
        automatically add a numerical suffix if the suggested name exists already."""
        if fh is None:
            return "NewFile"
        else:
            return fh.shortName()

    def deleteClicked(self):
        ## Delete button must be clicked twice.
        if self.deleteState == 0:
            self.ui.deleteBtn.setText('Really?')
            self.deleteState = 1
        elif self.deleteState == 1:
            try:
                self.selectedFile().delete()
            except:
                printExc('Error while deleting protocol file:')
                return
            finally:
                self.resetDeleteState()
                
    def selectedFileName(self):
        """Return the file name of the selected item"""
        sel = list(self.ui.fileTree.selectedIndexes())
        if len(sel) == 1:
            index = sel[0]
        else:
            raise Exception("Can not load--%d items selected" % len(sel))
        return self.protocolList.getFileName(index)
    
    def resetDeleteState(self):
        self.deleteState = 0
        self.ui.deleteBtn.setText('Delete')

    def selectedDir(self):
        """Return the directory of the selected file"""
        fh = self.selectedFile()
        if fh is None:
            dh = self.baseDir
        else:
            if fh.isDir():
                dh = fh
            else:
                dh = fh.parent()
        return dh
        
    def newDirClicked(self):
        dh = self.selectedDir()
        
        ndh = dh.mkdir("NewDirectory", autoIncrement=True)
        self.ui.fileTree.flushSignals()   ## Item may take time to appear in the tree..
        self.ui.fileTree.editItem(ndh) 
        
    def delBtnLostFocus(self, ev):
        self.resetDeleteState()
        
    def setCurrentFile(self, handle):
        if self.currentFile is not None:
            QtCore.QObject.disconnect(self.currentFile, QtCore.SIGNAL('changed'), self.currentFileChanged)
            
        if handle is None:
            self.ui.currentLabel.setText("")
            self.ui.saveBtn.setEnabled(False)
        else:
            self.ui.currentLabel.setText(handle.name(relativeTo=self.baseDir))
            self.ui.saveBtn.setEnabled(True)
            QtCore.QObject.connect(handle, QtCore.SIGNAL('changed'), self.currentFileChanged)
            
        self.currentFile = handle
            
        
    def currentFileChanged(self, handle, change, *args):
        if change == 'deleted':
            self.ui.currentLabel.setText("[deleted]")
        else:
            self.ui.currentLabel.setText(self.currentFile.name(relativeTo=self.baseDir))