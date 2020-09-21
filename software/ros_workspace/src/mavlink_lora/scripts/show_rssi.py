#!/usr/bin/env python

'''
This script uses bokeh to show radio_status on live updating plots. Makes it easier to experiment with antennas
and see the results in real time.

This script is made by Frederik Mazur Andersen for his Master Thesis
and is taken from his repository: https://gitlab.com/Crowdedlight/dynamic-pathplanner-for-uavs

Revision
2019-03.15 FMA simple version from master thesis included to show rssi in plots
'''

# imports
import rospy
from mavlink_lora.msg import mavlink_lora_radio_status
from datetime import datetime
import signal
from bokeh.plotting import figure, curdoc
from bokeh.models.sources import ColumnDataSource
from bokeh.client import push_session
from bokeh.models import Button
from functools import partial
from bokeh.layouts import column, row
from bokeh.server.server import Server

# parameters
mavlink_lora_radio_status_topic = '/mavlink_radio_status'


class rssi_node:
    def __init__(self):

        # launch node
        rospy.init_node('mavlink_lora_radio_status')

        # open file to log data.
        self.time_offset = rospy.get_time()

        self.rate = rospy.Rate(10)
        self.doc = None

        # data holders
        self.data_x = []
        self.data_y = []

        # install ctrl-c handler
        signal.signal(signal.SIGINT, self.signal_handler)

        # setup liveplot
        self.fig = figure(plot_width=800,
                          plot_height=400,
                          x_axis_label='Seconds',
                          # x_axis_type='datetime',
                          x_axis_location='below',
                          # x_range=('2018-01-01', '2018-06-30'),
                          y_axis_label='dBm',
                          y_axis_type='linear',
                          y_axis_location='left',
                          # y_range=(0, 100),
                          title='Rssi',
                          title_location='above',
                          toolbar_location='right'
                          # tools='save'
                          )

        self.fig.x_range.follow = "end"
        self.fig.x_range.follow_interval = 20
        self.fig.x_range.range_padding = 0

        self.plot_ds = ColumnDataSource(data=dict(x=self.data_x, y=self.data_y))
        self.line_rssi = self.fig.line(x="x", y="y", color="firebrick", line_width=2, source=self.plot_ds)

        # subs
        self.radio_msg_sub = rospy.Subscriber(mavlink_lora_radio_status_topic, mavlink_lora_radio_status,
                                              self.on_mavlink_msg)

        # wait until everything is running
        rospy.sleep(1)

        # start bokeh server
        self.server = Server({'/': self.addLayout}, num_procs=1)
        self.server.start()

        self.server.io_loop.add_callback(self.server.show, "/")
        self.server.io_loop.start()

    # define ctrl-c handler
    def signal_handler(self, signal, frame):
        # self.stop_flag = True
        self.radio_msg_sub.unregister()
        print("graceful exit")
        exit()

    def plotUpdate(self):
        # do liveplot. Best practice to make new dict and update at once
        new_data = dict()
        new_data['x'] = self.data_x
        new_data['y'] = self.data_y
        self.plot_ds.data = new_data

    def on_mavlink_msg(self, msg):
        # save data in file before doing live plot
        now = rospy.get_time()
        time = now - self.time_offset  # time in sec since start

        # save data to array
        self.data_x.append(time)
        self.data_y.append(msg.rssi)

        # schedule liveplot
        try:
            self.doc.add_next_tick_callback(partial(self.plotUpdate))
        except:
            pass

    def addLayout(self, doc):
        # set layout
        doc.add_root(row(self.fig))
        self.doc = doc


if __name__ == '__main__':
    rssi = rssi_node()
