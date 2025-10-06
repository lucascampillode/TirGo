#!/usr/bin/env python

# Copyright (C) 2014, PAL Robotics S.L.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
# * Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
# * Neither the name of PAL Robotics S.L. nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal
from python_qt_binding.QtWidgets import QWidget, QFormLayout

from pal_control_msgs.msg import ActuatorCurrentLimit
from controller_manager_msgs.utils\
    import ControllerLister, ControllerManagerLister

from double_editor import DoubleEditor
from update_combo import update_combo

# TODO:
# - Allow to configure:
#   - Command publishing and state update frequency
#   - Controller manager and clc monitor frequency


class CurrentLimitController(Plugin):
    """
    Graphical frontend for a C{CurrentLimitController}.

    There are two modes for interacting with a controller:
        1. B{Monitor mode} Actuator displays are updated with the value reported
          by the controller. This is a read-only mode and does I{not} send
          control commands. Every time a new controller is selected, it starts
          in monitor mode for safety reasons.

        2. B{Control mode} Actuator displays update the control command that is
        sent to the controller. Commands are sent periodically evan if the
        displays are not being updated by the user.

    This plugin is able to detect and keep track of all active controller
    managers, as well as the CurrentLimitControllers that are I{running}
    in each controller manager.
    For a controller to be compatible with this plugin, it must comply with
    the following requisites:
        - The controller type contains the C{CurrentLimitController}
        substring, e.g., C{current_controllers/CurrentLimitController}
        - The controller exposes the C{command} topic in its
        ROS interface.

    A reference implementation of the C{CurrentLimitController} is
    available in the C{current_limit_controller} ROS package.
    """
    _cmd_pub_freq = 10.0  # Hz
    _widget_update_freq = 30.0  # Hz
    _ctrlrs_update_freq = 1  # Hz
    _min_traj_dur = 5.0 / _cmd_pub_freq  # Minimum trajectory duration

    currentLimitChanged = Signal([dict])

    def __init__(self, context):
        super(CurrentLimitController, self).__init__(context)
        self.setObjectName('CurrentLimitController')

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_current_limit_controller'),
                               'resource',
                               'current_limit_controller.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('CurrentLimitControllerUi')

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)

        # Initialize members
        self._clc_name = []  # Name of selected controller
        self._cm_ns = []  # Namespace of the selected controller manager
        self._actuator_curr = {}  # name->curr map for actuators of selected controller
        self._actuator_names = []  # Ordered list of selected controller actuators

        # Timer for sending commands to active controller
        self._update_cmd_timer = QTimer(self)
        self._update_cmd_timer.setInterval(1000.0 / self._cmd_pub_freq)
        self._update_cmd_timer.timeout.connect(self._update_cmd_cb)

        # Timer for updating the actuator widgets from the controller state
        self._update_act_curr_timer = QTimer(self)
        self._update_act_curr_timer.setInterval(1000.0 /
                                                self._widget_update_freq)
        self._update_act_curr_timer.timeout.connect(self._update_actuator_widgets)

        # Timer for controller manager updates
        self._list_cm = ControllerManagerLister()
        self._update_cm_list_timer = QTimer(self)
        self._update_cm_list_timer.setInterval(1000.0 /
                                               self._ctrlrs_update_freq)
        self._update_cm_list_timer.timeout.connect(self._update_cm_list)
        self._update_cm_list_timer.start()

        # Timer for running controller updates
        self._update_clc_list_timer = QTimer(self)
        self._update_clc_list_timer.setInterval(1000.0 /
                                                self._ctrlrs_update_freq)
        self._update_clc_list_timer.timeout.connect(self._update_clc_list)
        self._update_clc_list_timer.start()

        # Signal connections
        w = self._widget
        w.enable_button.toggled.connect(self._on_clc_enabled)
        w.clc_combo.currentIndexChanged[str].connect(self._on_clc_change)
        w.cm_combo.currentIndexChanged[str].connect(self._on_cm_change)

        self._cmd_pub = None    # Controller command publisher
        self._state_sub = None  # Controller state subscriber

        self._list_controllers = None

    def shutdown_plugin(self):
        self._update_cmd_timer.stop()
        self._update_act_curr_timer.stop()
        self._update_cm_list_timer.stop()
        self._update_clc_list_timer.stop()
        self._unregister_state_sub()
        self._unregister_cmd_pub()

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    # def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget
        # title bar
        # Usually used to open a modal configuration dialog

    def _update_cm_list(self):
        update_combo(self._widget.cm_combo, self._list_cm())

    def _update_clc_list(self):
        # Clear controller list if no controller information is available
        if not self._list_controllers:
            self._widget.clc_combo.clear()
            return

        # Update widget
        running_clc = self._running_clc_info()
        running_clc_names = [data.name for data in running_clc]
        update_combo(self._widget.clc_combo, sorted(running_clc_names))

    def _on_actuator_state_change(self, actual_curr):
        assert(len(actual_curr) == len(self._actuator_curr))
        for name in actual_curr.keys():
            self._actuator_curr[name]['state'] = actual_curr[name]

    def _on_cm_change(self, cm_ns):
        self._cm_ns = cm_ns
        if cm_ns:
            self._list_controllers = ControllerLister(cm_ns)
            self._widget.clc_combo.clear()
            self._update_clc_list()
        else:
            self._list_controllers = None

    def _on_clc_change(self, clc_name):
        self._unload_clc()
        self._clc_name = clc_name
        if self._clc_name:
            self._load_clc()

    def _on_clc_enabled(self, val):
        # Don't allow enabling if there are no controllers selected
        if not self._clc_name:
            self._widget.enable_button.setChecked(False)
            return

        # Enable/disable actuator displays
        for actuator_widget in self._actuator_widgets():
            actuator_widget.setEnabled(val)

        if val:
            # Widgets send desired current limit commands to controller
            self._update_act_curr_timer.stop()
            self._update_cmd_timer.start()
        else:
            # Controller updates widgets with actual current limit
            self._update_cmd_timer.stop()
            self._update_act_curr_timer.start()

    def _load_clc(self):
        # Initialize actuator data corresponding to selected controller
        running_clc = self._running_clc_info()
        self._actuator_names = next(_clc_joint_names(x) for x in running_clc
                                 if x.name == self._clc_name)
        for name in self._actuator_names:
            self._actuator_curr[name] = {}

        # Update actuator display
        try:
            layout = self._widget.actuator_group.layout()
            for name in self._actuator_names:
                actuator_widget = DoubleEditor(0.0, 1.0)
                layout.addRow(name, actuator_widget)
                # NOTE: Using partial instead of a lambda because lambdas
                # "will not evaluate/look up the argument values before it is
                # effectively called, breaking situations like using a loop
                # variable inside it"
                from functools import partial
                par = partial(self._update_single_cmd_cb, name=name)
                actuator_widget.valueChanged.connect(par)
        except:
            # TODO: Can we do better than swallow the exception?
            from sys import exc_info
            print 'Unexpected error:', exc_info()[0]

        # Enter monitor mode (sending commands disabled)
        self._on_clc_enabled(False)

        # Setup ROS interfaces
        clc_ns = _resolve_controller_ns(self._cm_ns, self._clc_name)
        state_topic = clc_ns + '/state'
        cmd_topic = clc_ns + '/command'
        self._state_sub = rospy.Subscriber(state_topic,
                                           ActuatorCurrentLimit,
                                           self._state_cb,
                                           queue_size=1)
        self._cmd_pub = rospy.Publisher(cmd_topic,
                                        ActuatorCurrentLimit,
                                        queue_size=1)

        # Start updating the actuator current limits
        self.currentLimitChanged.connect(self._on_actuator_state_change)

    def _unload_clc(self):
        # Stop updating the actuator current limits
        try:
            self.currentLimitChanged.disconnect(self._on_actuator_state_change)
        except:
            pass

        # Reset ROS interfaces
        self._unregister_state_sub()
        self._unregister_cmd_pub()

        # Clear actuator widgets
        # NOTE: Implementation is a workaround for:
        # https://bugreports.qt-project.org/browse/QTBUG-15990 :(
        layout = self._widget.actuator_group.layout()
        if layout is not None:
            while layout.count():
                layout.takeAt(0).widget().deleteLater()
            # Delete existing layout by reparenting to temporary
            QWidget().setLayout(layout)
        self._widget.actuator_group.setLayout(QFormLayout())

        # Reset actuator data
        self._actuator_names = []
        self._actuator_curr = {}

        # Enforce monitor mode (sending commands disabled)
        self._widget.enable_button.setChecked(False)

    def _running_clc_info(self):
        from controller_manager_msgs.utils\
            import filter_by_type, filter_by_state

        controller_list = self._list_controllers()
        clc_list = filter_by_type(controller_list,
                                  'CurrentLimitController',
                                  match_substring=True)
        running_clc_list = filter_by_state(clc_list, 'running')
        return running_clc_list

    def _unregister_cmd_pub(self):
        if self._cmd_pub is not None:
            self._cmd_pub.unregister()
            self._cmd_pub = None

    def _unregister_state_sub(self):
        if self._state_sub is not None:
            self._state_sub.unregister()
            self._state_sub = None

    def _state_cb(self, msg):
        actual_curr = {}
        for i in range(len(msg.actuator_names)):
            actuator_name = msg.actuator_names[i]
            actuator_curr = msg.current_limits[i]
            actual_curr[actuator_name] = actuator_curr
        self.currentLimitChanged.emit(actual_curr)

    def _update_single_cmd_cb(self, val, name):
        self._actuator_curr[name]['command'] = val

    def _update_cmd_cb(self):
        cmd = ActuatorCurrentLimit()
        cmd.actuator_names = self._actuator_names
        for name in cmd.actuator_names:
            curr_limit = self._actuator_curr[name]['command']
            cmd.current_limits.append(curr_limit)
        self._cmd_pub.publish(cmd)

    def _update_actuator_widgets(self):
        actuator_widgets = self._actuator_widgets()
        for id in range(len(actuator_widgets)):
            actuator_name = self._actuator_names[id]
            try:
                actuator_curr = self._actuator_curr[actuator_name]['state']
                actuator_widgets[id].setValue(actuator_curr)
            except (KeyError):
                pass  # Can happen when first connected to controller

    def _actuator_widgets(self):  # TODO: Cache instead of compute every time?
        widgets = []
        layout = self._widget.actuator_group.layout()
        for row_id in range(layout.rowCount()):
            widgets.append(layout.itemAt(row_id,
                                         QFormLayout.FieldRole).widget())
        return widgets

def _clc_joint_names(clc_info):
    # NOTE: We assume that there is at least one hardware interface that
    # claims resources (there should be), and the resource list is fetched
    # from the first available interface
    #return jtc_info.claimed_resources[0].resources
    names = [];
    for claimed_res in clc_info.claimed_resources:
        names.extend(claimed_res.resources)
    return names


def _resolve_controller_ns(cm_ns, controller_name):
    """
    Resolve a controller's namespace from that of the controller manager.
    Controllers are assumed to live one level above the controller
    manager, e.g.

        >>> _resolve_controller_ns('/path/to/controller_manager', 'foo')
        '/path/to/foo'

    In the particular case in which the controller manager is not
    namespaced, the controller is assumed to live in the root namespace

        >>> _resolve_controller_ns('/', 'foo')
        '/foo'
        >>> _resolve_controller_ns('', 'foo')
        '/foo'
    @param cm_ns Controller manager namespace (can be an empty string)
    @type cm_ns str
    @param controller_name Controller name (non-empty string)
    @type controller_name str
    @return Controller namespace
    @rtype str
    """
    assert(controller_name)
    ns = cm_ns.rsplit('/', 1)[0]
    if ns != '/':
        ns += '/'
    ns += controller_name
    return ns
