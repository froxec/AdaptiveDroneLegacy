# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'interfaceHGkeaj.ui'
##
## Created by: Qt User Interface Compiler version 6.5.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QFrame, QHBoxLayout, QLabel,
    QPushButton, QSizePolicy, QSpacerItem, QStackedWidget,
    QVBoxLayout, QWidget, QMainWindow)
import resources_rc

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(882, 500)
        MainWindow.setMinimumSize(QSize(640, 420))
        MainWindow.setAutoFillBackground(False)
        MainWindow.setStyleSheet(u"*{\n"
"	border: none;\n"
"	background-color:transparent;\n"
"	background: transparent;\n"
"	padding: 0;\n"
"	margin: 0;\n"
"	color: #000\n"
"\n"
"}\n"
"\n"
"#Main {\n"
"	background-color:#F0FCD4;\n"
"}\n"
"#leftMenuSubContainer{\n"
"	background-color:  #CCCAAB;\n"
"}\n"
"\n"
"QPushButton{\n"
"	text-align: left;\n"
"	padding: 2px;\n"
"}\n"
"\n"
"#centralMenuSubContainer {\n"
"	background-color: #CCCAAB;\n"
"}\n"
"\n"
"#frame_4 {\n"
"	background-color: #F0FCD4;\n"
"	border-radius: 10px;\n"
"}")
        self.horizontalLayout = QHBoxLayout(MainWindow)
        self.horizontalLayout.setSpacing(0)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.leftMenuContainer = QWidget(MainWindow)
        self.leftMenuContainer.setObjectName(u"leftMenuContainer")
        sizePolicy = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.leftMenuContainer.sizePolicy().hasHeightForWidth())
        self.leftMenuContainer.setSizePolicy(sizePolicy)
        self.leftMenuContainer.setMaximumSize(QSize(40, 16777215))
        self.verticalLayout = QVBoxLayout(self.leftMenuContainer)
        self.verticalLayout.setSpacing(0)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.leftMenuSubContainer = QWidget(self.leftMenuContainer)
        self.leftMenuSubContainer.setObjectName(u"leftMenuSubContainer")
        self.leftMenuSubContainer.setMaximumSize(QSize(40, 16777215))
        self.verticalLayout_2 = QVBoxLayout(self.leftMenuSubContainer)
        self.verticalLayout_2.setSpacing(0)
        self.verticalLayout_2.setObjectName(u"verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.frame_3 = QFrame(self.leftMenuSubContainer)
        self.frame_3.setObjectName(u"frame_3")
        sizePolicy1 = QSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        sizePolicy1.setHorizontalStretch(0)
        sizePolicy1.setVerticalStretch(0)
        sizePolicy1.setHeightForWidth(self.frame_3.sizePolicy().hasHeightForWidth())
        self.frame_3.setSizePolicy(sizePolicy1)
        self.frame_3.setFrameShape(QFrame.StyledPanel)
        self.frame_3.setFrameShadow(QFrame.Raised)
        self.verticalLayout_5 = QVBoxLayout(self.frame_3)
        self.verticalLayout_5.setSpacing(0)
        self.verticalLayout_5.setObjectName(u"verticalLayout_5")
        self.verticalLayout_5.setContentsMargins(8, 0, 0, 0)
        self.mainButton = QPushButton(self.frame_3)
        self.mainButton.setObjectName(u"mainButton")
        icon = QIcon()
        icon.addFile(u":/icons/feather/align-justify.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.mainButton.setIcon(icon)
        self.mainButton.setIconSize(QSize(24, 24))

        self.verticalLayout_5.addWidget(self.mainButton)


        self.verticalLayout_2.addWidget(self.frame_3)

        self.frame = QFrame(self.leftMenuSubContainer)
        self.frame.setObjectName(u"frame")
        self.frame.setFrameShape(QFrame.StyledPanel)
        self.frame.setFrameShadow(QFrame.Raised)
        self.verticalLayout_3 = QVBoxLayout(self.frame)
        self.verticalLayout_3.setSpacing(6)
        self.verticalLayout_3.setObjectName(u"verticalLayout_3")
        self.verticalLayout_3.setContentsMargins(8, 0, 0, 0)
        self.HomeButton = QPushButton(self.frame)
        self.HomeButton.setObjectName(u"HomeButton")
        icon1 = QIcon()
        icon1.addFile(u":/icons/feather/home.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.HomeButton.setIcon(icon1)
        self.HomeButton.setIconSize(QSize(24, 24))

        self.verticalLayout_3.addWidget(self.HomeButton)

        self.ConnectionButton = QPushButton(self.frame)
        self.ConnectionButton.setObjectName(u"ConnectionButton")
        icon2 = QIcon()
        icon2.addFile(u":/icons/feather/wifi.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.ConnectionButton.setIcon(icon2)
        self.ConnectionButton.setIconSize(QSize(24, 24))

        self.verticalLayout_3.addWidget(self.ConnectionButton)

        self.ControlsButton = QPushButton(self.frame)
        self.ControlsButton.setObjectName(u"ControlsButton")
        icon3 = QIcon()
        icon3.addFile(u":/icons/feather/sliders.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.ControlsButton.setIcon(icon3)
        self.ControlsButton.setIconSize(QSize(24, 24))

        self.verticalLayout_3.addWidget(self.ControlsButton)


        self.verticalLayout_2.addWidget(self.frame)

        self.verticalSpacer = QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding)

        self.verticalLayout_2.addItem(self.verticalSpacer)

        self.frame_2 = QFrame(self.leftMenuSubContainer)
        self.frame_2.setObjectName(u"frame_2")
        sizePolicy1.setHeightForWidth(self.frame_2.sizePolicy().hasHeightForWidth())
        self.frame_2.setSizePolicy(sizePolicy1)
        self.frame_2.setFrameShape(QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QFrame.Raised)
        self.verticalLayout_4 = QVBoxLayout(self.frame_2)
        self.verticalLayout_4.setSpacing(6)
        self.verticalLayout_4.setObjectName(u"verticalLayout_4")
        self.verticalLayout_4.setContentsMargins(8, 0, 0, 0)
        self.SettingsButton = QPushButton(self.frame_2)
        self.SettingsButton.setObjectName(u"SettingsButton")
        icon4 = QIcon()
        icon4.addFile(u":/icons/feather/settings.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.SettingsButton.setIcon(icon4)
        self.SettingsButton.setIconSize(QSize(24, 24))

        self.verticalLayout_4.addWidget(self.SettingsButton)

        self.AlarmsButton = QPushButton(self.frame_2)
        self.AlarmsButton.setObjectName(u"AlarmsButton")
        icon5 = QIcon()
        icon5.addFile(u":/icons/feather/bell.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.AlarmsButton.setIcon(icon5)
        self.AlarmsButton.setIconSize(QSize(24, 24))

        self.verticalLayout_4.addWidget(self.AlarmsButton)

        self.ExitButton = QPushButton(self.frame_2)
        self.ExitButton.setObjectName(u"ExitButton")
        icon6 = QIcon()
        icon6.addFile(u":/icons/feather/external-link.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.ExitButton.setIcon(icon6)
        self.ExitButton.setIconSize(QSize(24, 24))

        self.verticalLayout_4.addWidget(self.ExitButton)


        self.verticalLayout_2.addWidget(self.frame_2)


        self.verticalLayout.addWidget(self.leftMenuSubContainer)


        self.horizontalLayout.addWidget(self.leftMenuContainer)

        self.centralMenuContainer = QWidget(MainWindow)
        self.centralMenuContainer.setObjectName(u"centralMenuContainer")
        sizePolicy.setHeightForWidth(self.centralMenuContainer.sizePolicy().hasHeightForWidth())
        self.centralMenuContainer.setSizePolicy(sizePolicy)
        self.centralMenuContainer.setLayoutDirection(Qt.LeftToRight)
        self.centralMenuContainer.setAutoFillBackground(False)
        self.verticalLayout_6 = QVBoxLayout(self.centralMenuContainer)
        self.verticalLayout_6.setSpacing(0)
        self.verticalLayout_6.setObjectName(u"verticalLayout_6")
        self.verticalLayout_6.setContentsMargins(0, 0, 0, 0)
        self.centralMenuSubContainer = QWidget(self.centralMenuContainer)
        self.centralMenuSubContainer.setObjectName(u"centralMenuSubContainer")
        sizePolicy.setHeightForWidth(self.centralMenuSubContainer.sizePolicy().hasHeightForWidth())
        self.centralMenuSubContainer.setSizePolicy(sizePolicy)
        self.centralMenuSubContainer.setMinimumSize(QSize(200, 0))
        self.verticalLayout_7 = QVBoxLayout(self.centralMenuSubContainer)
        self.verticalLayout_7.setSpacing(0)
        self.verticalLayout_7.setObjectName(u"verticalLayout_7")
        self.verticalLayout_7.setContentsMargins(0, 0, 0, 0)
        self.frame_4 = QFrame(self.centralMenuSubContainer)
        self.frame_4.setObjectName(u"frame_4")
        self.frame_4.setFrameShape(QFrame.StyledPanel)
        self.frame_4.setFrameShadow(QFrame.Raised)
        self.verticalLayout_8 = QVBoxLayout(self.frame_4)
        self.verticalLayout_8.setSpacing(0)
        self.verticalLayout_8.setObjectName(u"verticalLayout_8")
        self.verticalLayout_8.setContentsMargins(0, 0, 0, 0)
        self.pushButton = QPushButton(self.frame_4)
        self.pushButton.setObjectName(u"pushButton")
        icon7 = QIcon()
        icon7.addFile(u":/icons/feather/x-circle.svg", QSize(), QIcon.Normal, QIcon.Off)
        self.pushButton.setIcon(icon7)
        self.pushButton.setIconSize(QSize(24, 24))

        self.verticalLayout_8.addWidget(self.pushButton)


        self.verticalLayout_7.addWidget(self.frame_4, 0, Qt.AlignTop)

        self.stackedWidget = QStackedWidget(self.centralMenuSubContainer)
        self.stackedWidget.setObjectName(u"stackedWidget")
        self.settings_page = QWidget()
        self.settings_page.setObjectName(u"settings_page")
        self.verticalLayout_9 = QVBoxLayout(self.settings_page)
        self.verticalLayout_9.setSpacing(0)
        self.verticalLayout_9.setObjectName(u"verticalLayout_9")
        self.verticalLayout_9.setContentsMargins(0, 0, 0, 0)
        self.label = QLabel(self.settings_page)
        self.label.setObjectName(u"label")
        sizePolicy.setHeightForWidth(self.label.sizePolicy().hasHeightForWidth())
        self.label.setSizePolicy(sizePolicy)
        font = QFont()
        font.setPointSize(13)
        self.label.setFont(font)
        self.label.setAlignment(Qt.AlignCenter)

        self.verticalLayout_9.addWidget(self.label)

        self.stackedWidget.addWidget(self.settings_page)
        self.connection_page = QWidget()
        self.connection_page.setObjectName(u"connection_page")
        self.verticalLayout_10 = QVBoxLayout(self.connection_page)
        self.verticalLayout_10.setSpacing(0)
        self.verticalLayout_10.setObjectName(u"verticalLayout_10")
        self.verticalLayout_10.setContentsMargins(0, 0, 0, 0)
        self.label_2 = QLabel(self.connection_page)
        self.label_2.setObjectName(u"label_2")
        self.label_2.setFont(font)
        self.label_2.setAlignment(Qt.AlignCenter)

        self.verticalLayout_10.addWidget(self.label_2)

        self.stackedWidget.addWidget(self.connection_page)
        self.controls_page = QWidget()
        self.controls_page.setObjectName(u"controls_page")
        self.verticalLayout_11 = QVBoxLayout(self.controls_page)
        self.verticalLayout_11.setObjectName(u"verticalLayout_11")
        self.label_3 = QLabel(self.controls_page)
        self.label_3.setObjectName(u"label_3")
        self.label_3.setFont(font)
        self.label_3.setAlignment(Qt.AlignCenter)

        self.verticalLayout_11.addWidget(self.label_3)

        self.stackedWidget.addWidget(self.controls_page)

        self.verticalLayout_7.addWidget(self.stackedWidget)


        self.verticalLayout_6.addWidget(self.centralMenuSubContainer)


        self.horizontalLayout.addWidget(self.centralMenuContainer)

        self.mainBodyContainer = QWidget(MainWindow)
        self.mainBodyContainer.setObjectName(u"mainBodyContainer")
        sizePolicy2 = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        sizePolicy2.setHorizontalStretch(0)
        sizePolicy2.setVerticalStretch(0)
        sizePolicy2.setHeightForWidth(self.mainBodyContainer.sizePolicy().hasHeightForWidth())
        self.mainBodyContainer.setSizePolicy(sizePolicy2)
        self.verticalLayout_12 = QVBoxLayout(self.mainBodyContainer)
        self.verticalLayout_12.setSpacing(0)
        self.verticalLayout_12.setObjectName(u"verticalLayout_12")
        self.verticalLayout_12.setContentsMargins(0, 0, 0, 0)
        self.statusContainer = QWidget(self.mainBodyContainer)
        self.statusContainer.setObjectName(u"statusContainer")
        self.horizontalLayout_3 = QHBoxLayout(self.statusContainer)
        self.horizontalLayout_3.setSpacing(0)
        self.horizontalLayout_3.setObjectName(u"horizontalLayout_3")
        self.horizontalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.frame_5 = QFrame(self.statusContainer)
        self.frame_5.setObjectName(u"frame_5")
        self.frame_5.setFrameShape(QFrame.StyledPanel)
        self.frame_5.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_5 = QHBoxLayout(self.frame_5)
        self.horizontalLayout_5.setSpacing(0)
        self.horizontalLayout_5.setObjectName(u"horizontalLayout_5")
        self.horizontalLayout_5.setContentsMargins(10, 0, 0, 0)
        self.label_4 = QLabel(self.frame_5)
        self.label_4.setObjectName(u"label_4")
        self.label_4.setPixmap(QPixmap(u":/icons/feather/battery.svg"))

        self.horizontalLayout_5.addWidget(self.label_4)


        self.horizontalLayout_3.addWidget(self.frame_5)

        self.frame_6 = QFrame(self.statusContainer)
        self.frame_6.setObjectName(u"frame_6")
        self.frame_6.setFrameShape(QFrame.StyledPanel)
        self.frame_6.setFrameShadow(QFrame.Raised)

        self.horizontalLayout_3.addWidget(self.frame_6)

        self.frame_7 = QFrame(self.statusContainer)
        self.frame_7.setObjectName(u"frame_7")
        self.frame_7.setFrameShape(QFrame.StyledPanel)
        self.frame_7.setFrameShadow(QFrame.Raised)
        self.horizontalLayout_4 = QHBoxLayout(self.frame_7)
        self.horizontalLayout_4.setSpacing(0)
        self.horizontalLayout_4.setObjectName(u"horizontalLayout_4")
        self.horizontalLayout_4.setContentsMargins(0, 0, 0, 0)

        self.horizontalLayout_3.addWidget(self.frame_7)


        self.verticalLayout_12.addWidget(self.statusContainer, 0, Qt.AlignTop)

        self.mapContainer = QWidget(self.mainBodyContainer)
        self.mapContainer.setObjectName(u"mapContainer")

        self.verticalLayout_12.addWidget(self.mapContainer)

        self.infoContainer = QWidget(self.mainBodyContainer)
        self.infoContainer.setObjectName(u"infoContainer")
        self.horizontalLayout_2 = QHBoxLayout(self.infoContainer)
        self.horizontalLayout_2.setSpacing(0)
        self.horizontalLayout_2.setObjectName(u"horizontalLayout_2")
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.gyroContainer = QWidget(self.infoContainer)
        self.gyroContainer.setObjectName(u"gyroContainer")
        self.numericalDataContainer = QWidget(self.gyroContainer)
        self.numericalDataContainer.setObjectName(u"numericalDataContainer")
        self.numericalDataContainer.setGeometry(QRect(-20, 10, 309, 149))

        self.horizontalLayout_2.addWidget(self.gyroContainer)


        self.verticalLayout_12.addWidget(self.infoContainer)


        self.horizontalLayout.addWidget(self.mainBodyContainer)


        self.retranslateUi(MainWindow)

        self.stackedWidget.setCurrentIndex(2)


        QMetaObject.connectSlotsByName(MainWindow)
    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"Form", None))
        self.mainButton.setText("")
#if QT_CONFIG(tooltip)
        self.HomeButton.setToolTip(QCoreApplication.translate("MainWindow", u"Home page", None))
#endif // QT_CONFIG(tooltip)
        self.HomeButton.setText(QCoreApplication.translate("MainWindow", u"Home", None))
#if QT_CONFIG(tooltip)
        self.ConnectionButton.setToolTip(QCoreApplication.translate("MainWindow", u"Connection settings", None))
#endif // QT_CONFIG(tooltip)
        self.ConnectionButton.setText(QCoreApplication.translate("MainWindow", u"Connection", None))
#if QT_CONFIG(tooltip)
        self.ControlsButton.setToolTip(QCoreApplication.translate("MainWindow", u"Advanced controls", None))
#endif // QT_CONFIG(tooltip)
        self.ControlsButton.setText(QCoreApplication.translate("MainWindow", u"Controls", None))
        self.SettingsButton.setText(QCoreApplication.translate("MainWindow", u"Settings", None))
        self.AlarmsButton.setText(QCoreApplication.translate("MainWindow", u"Alarms", None))
        self.ExitButton.setText(QCoreApplication.translate("MainWindow", u"Exit", None))
        self.pushButton.setText(QCoreApplication.translate("MainWindow", u"More menu", None))
        self.label.setText(QCoreApplication.translate("MainWindow", u"Settings", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", u"Connection", None))
        self.label_3.setText(QCoreApplication.translate("MainWindow", u"Controls", None))
        self.label_4.setText("")
    # retranslateUi

