# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'port.ui'
#
# Created by: PyQt5 UI code generator 5.15.6
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(288, 337)
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.loadPoseButton = QtWidgets.QPushButton(self.centralwidget)
        self.loadPoseButton.setGeometry(QtCore.QRect(10, 20, 131, 34))
        self.loadPoseButton.setObjectName("loadPoseButton")
        self.recordRegPointButton = QtWidgets.QPushButton(self.centralwidget)
        self.recordRegPointButton.setGeometry(QtCore.QRect(10, 70, 131, 34))
        self.recordRegPointButton.setObjectName("recordRegPointButton")
        self.deleteRegPointButton = QtWidgets.QPushButton(self.centralwidget)
        self.deleteRegPointButton.setGeometry(QtCore.QRect(150, 70, 131, 34))
        self.deleteRegPointButton.setObjectName("deleteRegPointButton")
        self.calcTButton = QtWidgets.QPushButton(self.centralwidget)
        self.calcTButton.setGeometry(QtCore.QRect(150, 20, 131, 34))
        self.calcTButton.setObjectName("calcTButton")
        self.recordPointButton = QtWidgets.QPushButton(self.centralwidget)
        self.recordPointButton.setGeometry(QtCore.QRect(10, 120, 131, 34))
        self.recordPointButton.setObjectName("recordPointButton")
        self.deletePointButton = QtWidgets.QPushButton(self.centralwidget)
        self.deletePointButton.setGeometry(QtCore.QRect(150, 120, 131, 34))
        self.deletePointButton.setObjectName("deletePointButton")
        self.startShapeButton = QtWidgets.QPushButton(self.centralwidget)
        self.startShapeButton.setGeometry(QtCore.QRect(10, 170, 131, 34))
        self.startShapeButton.setObjectName("startShapeButton")
        self.stopShapeButton = QtWidgets.QPushButton(self.centralwidget)
        self.stopShapeButton.setGeometry(QtCore.QRect(150, 170, 131, 34))
        self.stopShapeButton.setObjectName("stopShapeButton")
        self.pushButton = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton.setGeometry(QtCore.QRect(10, 220, 131, 31))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_2.setGeometry(QtCore.QRect(150, 220, 131, 31))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.centralwidget)
        self.pushButton_3.setGeometry(QtCore.QRect(10, 270, 131, 31))
        self.pushButton_3.setObjectName("pushButton_3")
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "PORT"))
        self.loadPoseButton.setText(_translate("MainWindow", "Load Pose"))
        self.recordRegPointButton.setText(_translate("MainWindow", "Record Ref"))
        self.deleteRegPointButton.setText(_translate("MainWindow", "Delete Ref"))
        self.calcTButton.setText(_translate("MainWindow", "Calc T"))
        self.recordPointButton.setText(_translate("MainWindow", "Record Tar"))
        self.deletePointButton.setText(_translate("MainWindow", "Delete Tar"))
        self.startShapeButton.setText(_translate("MainWindow", "Start Shape"))
        self.stopShapeButton.setText(_translate("MainWindow", "Stop Shape"))
        self.pushButton.setText(_translate("MainWindow", "Start Surface"))
        self.pushButton_2.setText(_translate("MainWindow", "Stop Surface"))
        self.pushButton_3.setText(_translate("MainWindow", "Reset Demo"))