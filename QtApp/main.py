import os
import sys

from ui_interface import *
from PySide6.QtWidgets import QMainWindow
from PySide6.QtUiTools import QUiLoader
from Custom_Widgets.Widgets import *
from PySide6.QtCore import Slot
from Custom_Widgets.ProgressIndicator import test
class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        QMainWindow.__init__(self)

        #load UI
        loader = QUiLoader()
        loader.registerCustomWidget(QCustomSlideMenu)
        loader.registerCustomWidget(QCustomStackedWidget)
        print(loader.availableWidgets())
        self.window = loader.load("interface.ui", self)
        #connect signals
        #self.window.mainButton.clicked.connect(self.test)

        #read style
        loadJsonStyle(self, self.window)

        # expand menu
        self.window.HomeButton.clicked.connect(self.window.centralMenuContainer.expandMenu)
        self.window.ConnectionButton.clicked.connect(self.window.centralMenuContainer.expandMenu)
        self.window.ControlsButton.clicked.connect(self.window.centralMenuContainer.expandMenu)

        #close menu
        self.window.closeMenuBtn.clicked.connect(self.window.centralMenuContainer.collapseMenu)
        #show
        self.show()

    @Slot()
    def test(self):
        print("lalala")


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = MainWindow()
    sys.exit(app.exec())