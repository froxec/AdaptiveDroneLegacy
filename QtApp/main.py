import os
import sys
import threading

from ui_interface import *
from PySide6.QtWidgets import QMainWindow
from PySide6.QtWebEngineWidgets import QWebEngineView
from PySide6.QtUiTools import QUiLoader
from Custom_Widgets.Widgets import *
from PySide6.QtCore import Slot, QTimer
from QuadcopterIntegration.Utilities.comm_definitions import commands
from QuadcopterIntegration.Utilities.gcs_comm_functions import readThread
import folium
from folium import plugins
from pglive.sources.data_connector import DataConnector
from pglive.sources.live_plot import LiveLinePlot
from pglive.sources.live_plot_widget import LivePlotWidget
from collections import deque
import io
import serial
from math import sin
from time import sleep
from threading import Thread
from datetime import datetime
import itertools
class MainWindow(QMainWindow):
    def __init__(self, serial_connection, parent=None):
        QMainWindow.__init__(self)

        # memorize serial connection
        self.serial_connection = serial_connection

        # telemetry
        self.read_telemetry = readThread(self.serial_connection)
        self.telemetry_timer = QTimer()
        self.telemetry_timer.setInterval(500)
        self.telemetry_timer.start()
        self.telemetry_timer.timeout.connect(self.readTelemetry)
        self.telemetry = None

        # readings update timer
        self.readings_timer = QTimer()
        self.readings_timer.setInterval(500)
        self.readings_timer.start()
        self.readings_timer.timeout.connect(self.update_readings)

        #load UI
        loader = QUiLoader()
        loader.registerCustomWidget(QCustomSlideMenu)
        loader.registerCustomWidget(QCustomStackedWidget)
        loader.registerCustomWidget(LivePlotWidget)
        print(loader.availableWidgets())
        self.window = loader.load("interface.ui", self)
        #connect signals
        #self.window.mainButton.clicked.connect(self.test)


        #read style
        loadJsonStyle(self, self.window)
        # folium map

        #self.nav_icon = folium.DivIcon()
        # self.coordinate = (54.258978, 18.664669)
        # self.heading = 0.0
        # data = self.create_byte_map(zoom=60)
        # self.webView = QWebEngineView()
        # self.webView.setHtml(data.getvalue().decode())
        # self.webView.setParent(self.window.map_frame)


        # RT plots embedding
        self.live_plot_widgets = [self.window.x_plot, self.window.y_plot, self.window.z_plot,
                             self.window.Vx_plot, self.window.Vy_plot, self.window.Vz_plot]
        self.telemetry_to_plot = [('position_local', 0), ('position_local', 1), ('position_local', 2),
                                  ('velocity', 0), ('velocity', 1), ('velocity', 2)]
        self.data_connectors = []
        self.telemetry_updated_event = threading.Event()
        for live_plot in self.live_plot_widgets:
            plot_curve = LiveLinePlot()
            live_plot.addItem(plot_curve)
            self.data_connectors.append(DataConnector(plot_curve, max_points=300, update_rate=50))

        # expand menu
        self.window.HomeButton.clicked.connect(self.window.centralMenuContainer.expandMenu)
        self.window.ConnectionButton.clicked.connect(self.window.centralMenuContainer.expandMenu)
        self.window.ControlsButton.clicked.connect(self.window.centralMenuContainer.expandMenu)

        #close menu
        self.window.closeMenuBtn.clicked.connect(self.window.centralMenuContainer.collapseMenu)

        #arm/disarm vehicle
        self.window.armBtn.clicked.connect(lambda: self.arm_disarm_vehicle("arm"))
        self.window.disarmBtn.clicked.connect(lambda: self.arm_disarm_vehicle("disarm"))

        #change mode
        self.window.stabilizeButton.clicked.connect(lambda: self.change_flight_mode("STABILIZE"))
        self.window.guidedButton.clicked.connect(lambda: self.change_flight_mode("GUIDED"))
        self.window.acroButton.clicked.connect(lambda: self.change_flight_mode("ACRO"))

        #show
        self.show()

    @Slot()
    def arm_disarm_vehicle(self, mode):
        if mode == "arm":
            print(commands["ARM"])
            self.serial_connection.write(commands["ARM"])
            print("Arming command sent..")
        elif mode == "disarm":
            self.serial_connection.write(commands["DISARM"])
            print("Disarming command sent..")

    @Slot()
    def readTelemetry(self):
        if self.read_telemetry.telemetry_set_event.is_set():
            if self.read_telemetry.telemetry is None:
                return
            else:
                self.telemetry = self.read_telemetry.telemetry
                self.telemetry_updated_event.set()

    @Slot()
    def update_readings(self):
        if self.telemetry is None:
            return
        if self.telemetry['attitude'] is not None:
            self.window.pitch.setText(str(self.telemetry['attitude'][0]))
            self.window.roll.setText(str(self.telemetry['attitude'][1]))
            self.window.yaw.setText(str(self.telemetry['attitude'][2]))
        else:
            self.window.pitch.setText("No data")
            self.window.roll.setText("No data")
            self.window.yaw.setText("No data")
        # if self.telemetry['position_global'] is not None or \
        #         self.telemetry['heading'] is not None:
        #     if self.telemetry['position_global'] is not None:
        #         (lat, lon) = self.telemetry['position_global']
        #         self.coordinate = (lat, lon)
        #     if self.telemetry['heading'] is not None:
        #         self.heading = self.telemetry['heading']
        #     data = self.create_byte_map(60)
        #     self.webView.setHtml(data.getvalue().decode())
        if self.telemetry['flight_mode'] is not None:
            self.window.flightModeStatus.setText(str(self.telemetry['flight_mode']))

    def change_flight_mode(self, mode):
        self.serial_connection.write(commands[mode])
    def create_byte_map(self, zoom):
        map = folium.Map(
            title='Quad Location',
            zoom_start=zoom,
            location=self.coordinate
        )
        folium.Marker(
            location=self.coordinate,
            icon=folium.plugins.BeautifyIcon(icon='location-arrow',
                                      border_color='transparent',
                                      background_color='transparent',
                                      border_width=1,
                                      text_color='#003EFF',
                                      inner_icon_style='margin:0px;font-size:4em;transform: rotate({0}deg);'.format(
                                          self.heading))
        ).add_to(map)
        data = io.BytesIO()
        map.save(data, close_file=False)
        return data

    def update_plots(self, *connectors):
        x = 0
        while True:
            if self.telemetry_updated_event.is_set():
                for i, connector in enumerate(connectors):
                    telem_index = self.telemetry_to_plot[i]
                    if len(telem_index) == 1:
                        data = self.telemetry[telem_index]
                    elif len(telem_index) == 2:
                        data = self.telemetry[telem_index[0]][telem_index[1]]
                    if data is not None:
                        connector.cb_append_data_point(data, x)
                self.telemetry_updated_event.clear()
            x += 1
            sleep(0.01)


if __name__ == "__main__":
    try:
        ser = serial.Serial('/dev/pts/4', baudrate=115200, timeout=0.05)
        print("Connected to serial port")
    except:
        ser = None
    app = QApplication(sys.argv)
    window = MainWindow(ser)
    Thread(target=window.update_plots, args=window.data_connectors).start()
    sys.exit(app.exec())