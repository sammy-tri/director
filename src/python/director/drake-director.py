# This script is executed in the main console namespace so
# that all the variables defined here become console variables.

from __future__ import division

import director

import os
import sys
import PythonQt
import json
from PythonQt import QtCore, QtGui
from time import time
import imp
import director.applogic as app
from director import drcargs
from director import vtkAll as vtk
from director import jointcontrol
from director import callbacks
from director import camerabookmarks
from director import cameracontrol
from director import cameracontrolpanel
from director import ik
from director import ikplanner
from director import objectmodel as om
from director import spreadsheet
from director import transformUtils
from director import tdx
from director import skybox
from director import drakevisualizer
from director.fieldcontainer import FieldContainer
from director import robotstate
from director import roboturdf
from director import robotsystem
from director import filterUtils
from director import lcmloggerwidget
from director import lcmgl
from director import lcmoctomap
from director import lcmcollections
from director import handcontrolpanel
from director.jointpropagator import JointPropagator
from director import planningutils

from director import robotplanlistener
from director import playbackpanel
from director import screengrabberpanel
from director import splinewidget
from director import teleoppanel
from director import motionplanningpanel
from director import vtkNumpy as vnp
from director import visualization as vis
from director import actionhandlers
from director.timercallback import TimerCallback
from director.pointpicker import PointPicker, ImagePointPicker
from director import lcmUtils
from director.utime import getUtime
from director.shallowCopy import shallowCopy

import bot_core as lcmbotcore

from collections import OrderedDict
import functools
import math

import numpy as np
from director.debugVis import DebugData
from director import ioUtils as io

drcargs.requireStrict()
drcargs.args()
app.startup(globals(), requireDrcBase=False)
om.init(app.getMainWindow().objectTree(), app.getMainWindow().propertiesPanel())
actionhandlers.init()

quit = app.quit
exit = quit
view = app.getDRCView()
camera = view.camera()
tree = app.getMainWindow().objectTree()
orbit = cameracontrol.OrbitController(view)

###############################################################################


robotSystem = robotsystem.create(view,
                                 useFootsteps=False,
                                 usePerception=False,
                                 useSegmentation=False,
                                 useAffordance=False,
                                 useAtlasDriver=False,
                                 useExotica=False,
                                 usePyDrake=True,
                                 useHands=False)
globals().update(dict(robotSystem))


useIk = True
useRobotState = True
useGrid = True
useSpreadsheet = True
useHands = False
usePlanning = True
useLCMGL = True
useOctomap = False
useLightColorScheme = True
useLoggingWidget = True
useDrakeVisualizer = True
useForceDisplay = True
useSkybox = False
useDataFiles = True
useGamepad = False
useCOPMonitor = False
useCourseModel = False
useLimitJointsSentToPlanner = False

poseCollection = PythonQt.dd.ddSignalMap()
costCollection = PythonQt.dd.ddSignalMap()

if 'fixedBaseArm' in drcargs.getDirectorConfig()['userConfig']:
    ikPlanner.fixedBaseArm = True

if 'disableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['disableComponents']:
        print "Disabling", component
        locals()[component] = False

if 'enableComponents' in drcargs.getDirectorConfig():
    for component in drcargs.getDirectorConfig()['enableComponents']:
        print "Enabling", component
        locals()[component] = True


if useSpreadsheet:
    spreadsheet.init(poseCollection, costCollection)


if useGrid:
    grid = vis.showGrid(view, color=[0,0,0], alpha=0.1)
    grid.setProperty('Surface Mode', 'Surface with edges')

app.setBackgroundColor([0.3, 0.3, 0.35], [0.95,0.95,1])

viewOptions = vis.ViewOptionsItem(view)
om.addToObjectModel(viewOptions, parentObj=om.findObjectByName('sensors'))

class ViewBackgroundLightHandler(object):

    def __init__(self, viewOptions, grid):
        self.viewOptions = viewOptions
        self.action = app.getToolsMenuActions()['ActionToggleBackgroundLight']
        self.action.connect('triggered()', self.toggle)

        self.properties = { viewOptions : {'Gradient background':True, 'Background color':[0.0, 0.0, 0.0], 'Background color 2':[0.3, 0.3, 0.3]},
                            grid : {'Surface Mode':'Wireframe', 'Alpha':0.05, 'Color':[1.0, 1.0, 1.0], 'Color By':0}
                          }

        self.cachedProperties = {}
        self.storeProperties()

    def storeProperties(self):

        def grab(obj, props):
            for key in props.keys():
                self.cachedProperties.setdefault(obj, dict())[key] = obj.getProperty(key)

        for obj, props in self.properties.iteritems():
            grab(obj, props)

    def applyProperties(self, properties):

        def send(obj, props):
            for key, value in props.iteritems():
                obj.setProperty(key, value)

        for obj, props in properties.iteritems():
            send(obj, props)

    def toggle(self):
        if self.action.checked:
            self.storeProperties()
            self.applyProperties(self.properties)
        else:
            self.applyProperties(self.cachedProperties)


viewBackgroundLightHandler = ViewBackgroundLightHandler(viewOptions, grid)
if not useLightColorScheme:
    viewBackgroundLightHandler.action.trigger()

if useHands:
    handcontrolpanel.init(lHandDriver, rHandDriver, robotStateModel, robotStateJointController, view)


if useLCMGL:
    lcmglManager = lcmgl.init(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - LCM GL', lcmglManager.isEnabled, lcmglManager.setEnabled)

if useOctomap:
    octomapManager = lcmoctomap.init(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - Octomap', octomapManager.isEnabled, octomapManager.setEnabled)

useCollections = False
if useCollections:
    collectionsManager = lcmcollections.init(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - Collections', collectionsManager.isEnabled, collectionsManager.setEnabled)

if useDrakeVisualizer:
    drakeVisualizer = drakevisualizer.DrakeVisualizer(view)
    app.MenuActionToggleHelper('Tools', 'Renderer - Drake', drakeVisualizer.isEnabled, drakeVisualizer.setEnabled)


if usePlanning:

    def showPose(pose):
        playbackRobotModel.setProperty('Visible', True)
        playbackJointController.setPose('show_pose', pose)

    def playPlan(plan):
        playPlans([plan])

    def playPlans(plans):
        planPlayback.stopAnimation()
        playbackRobotModel.setProperty('Visible', True)
        planPlayback.playPlans(plans, playbackJointController)

    def playManipPlan():
        playPlan(manipPlanner.lastManipPlan)

    def plotManipPlan():
        planPlayback.plotPlan(manipPlanner.lastManipPlan)

    def planStand():
        ikPlanner.computeStandPlan(robotStateJointController.q)

    def planNominal():
        ikPlanner.computeNominalPlan(robotStateJointController.q)

    handJoints = []
    if drcargs.args().directorConfigFile.find('atlas') != -1:
        handJoints = roboturdf.getRobotiqJoints() + ['neck_ay']
    else:
        for handModel in ikPlanner.handModels:
            handJoints += handModel.handModel.model.getJointNames()
        # filter base joints out
        handJoints = [ joint for joint in handJoints if joint.find('base')==-1 ]

    teleopJointPropagator = JointPropagator(robotStateModel, teleopRobotModel, handJoints)
    playbackJointPropagator = JointPropagator(robotStateModel, playbackRobotModel, handJoints)
    def doPropagation(model=None):
        if teleopRobotModel.getProperty('Visible'):
            teleopJointPropagator.doPropagation()
        if playbackRobotModel.getProperty('Visible'):
            playbackJointPropagator.doPropagation()
    robotStateModel.connectModelChanged(doPropagation)

    planningUtils = planningutils.PlanningUtils(robotStateModel, robotStateJointController)
    if useLimitJointsSentToPlanner:
        planningUtils.clampToJointLimits = True

    jointLimitChecker = teleoppanel.JointLimitChecker(robotStateModel, robotStateJointController)
    jointLimitChecker.setupMenuAction()
    jointLimitChecker.start()

    postureShortcuts = teleoppanel.PosturePlanShortcuts(robotStateJointController, ikPlanner, planningUtils)


    ikPlanner.addPostureGoalListener(robotStateJointController)

    playbackPanel = playbackpanel.init(
        planPlayback, playbackRobotModel, playbackJointController,
        robotStateModel, robotStateJointController, manipPlanner)
    manipPlanner.connectPlanReceived(playbackPanel.setPlan)

    teleopPanel = teleoppanel.init(
        robotStateModel, robotStateJointController, teleopRobotModel,
        teleopJointController, ikPlanner, manipPlanner, affordanceManager,
        playbackPanel.setPlan, playbackPanel.hidePlan, planningUtils,
        footstepsDriver)


    motionPlanningPanel = motionplanningpanel.init(planningUtils, robotStateModel, robotStateJointController, teleopRobotModel, teleopJointController,
                                                   ikPlanner, manipPlanner, affordanceManager, playbackPanel.setPlan, playbackPanel.hidePlan, None)

    if useGamepad:
        from director import gamepad
        gamePad = gamepad.Gamepad(teleopPanel, teleopJointController, ikPlanner, view)

    splinewidget.init(view, handFactory, robotStateModel)

    for obj in om.getObjects():
        obj.setProperty('Deletable', False)

if useCOPMonitor and not ikPlanner.fixedBaseArm:
    from director import copmonitor
    copMonitor = copmonitor.COPMonitor(robotSystem, view);


if useLoggingWidget:
    w = lcmloggerwidget.LCMLoggerWidget(statusBar=app.getMainWindow().statusBar())
    app.getMainWindow().statusBar().addPermanentWidget(w.button)



if useForceDisplay:

    class LCMForceDisplay(object):
        '''
        Displays foot force sensor signals in a status bar widget or label widget
        '''


        def onRobotState(self,msg):
            self.l_foot_force_z = msg.force_torque.l_foot_force_z
            self.r_foot_force_z = msg.force_torque.r_foot_force_z

        def __init__(self, channel, statusBar=None):

            self.sub = lcmUtils.addSubscriber(channel, lcmbotcore.robot_state_t, self.onRobotState)
            self.label = QtGui.QLabel('')
            statusBar.addPermanentWidget(self.label)

            self.timer = TimerCallback(targetFps=10)
            self.timer.callback = self.showRate
            self.timer.start()

            self.l_foot_force_z = 0
            self.r_foot_force_z = 0

        def __del__(self):
            lcmUtils.removeSubscriber(self.sub)

        def showRate(self):
            global leftInContact, rightInContact
            self.label.text = '%.2f | %.2f' % (self.l_foot_force_z,self.r_foot_force_z)

    rateComputer = LCMForceDisplay('EST_ROBOT_STATE', app.getMainWindow().statusBar())


if useSkybox:

    skyboxDataDir = os.path.expanduser('~/Downloads/skybox')
    imageMap = skybox.getSkyboxImages(skyboxDataDir)
    skyboxObjs = skybox.createSkybox(imageMap, view)
    skybox.connectSkyboxCamera(view)
    #skybox.createTextureGround(os.path.join(skyboxDataDir, 'Dirt_seamless.jpg'), view)
    #view.camera().SetViewAngle(60)


class RobotLinkHighligher(object):

    def __init__(self, robotModel):
        self.robotModel = robotModel
        self.previousColors = {}

    def highlightLink(self, linkName, color):

        currentColor = self.robotModel.model.getLinkColor(linkName)
        if not currentColor.isValid():
            return

        if linkName not in self.previousColors:
            self.previousColors[linkName] = currentColor

        alpha = self.robotModel.getProperty('Alpha')
        newColor = QtGui.QColor(color[0]*255, color[1]*255, color[2]*255, alpha*255)

        self.robotModel.model.setLinkColor(linkName, newColor)

    def dehighlightLink(self, linkName):

        color = self.previousColors.pop(linkName, None)
        if color is None:
            return

        color.setAlpha(self.robotModel.getProperty('Alpha')*255)
        self.robotModel.model.setLinkColor(linkName, color)

robotHighlighter = RobotLinkHighligher(robotStateModel)

if useDataFiles:

    for filename in drcargs.args().data_files:
        actionhandlers.onOpenFile(filename)

screengrabberpanel.init(view)
camerabookmarks.init(view)

cameraControlPanel = cameracontrolpanel.CameraControlPanel(
    view, robotStateJointController)
app.addWidgetToDock(cameraControlPanel.widget, action=None).hide()


def getLinkFrame(linkName, model=None):
    model = model or robotStateModel
    return model.getLinkFrame(linkName)


def showLinkFrame(linkName, model=None):
    frame = getLinkFrame(linkName, model)
    if not frame:
        raise Exception('Link not found: ' + linkName)
    return vis.updateFrame(frame, linkName, parent='link frames')


def sendEstRobotState(pose=None):
    if pose is None:
        pose = robotStateJointController.q
    msg = robotstate.drakePoseToRobotState(pose)
    lcmUtils.publish('EST_ROBOT_STATE', msg)

estRobotStatePublisher = TimerCallback(callback=sendEstRobotState)

app.setCameraTerrainModeEnabled(view, True)
app.resetCamera(viewDirection=[-1,0,0], view=view)

def drawCenterOfMass(model):
    com = list(model.model.getCenterOfMass())
    # TODO(sam.creasey) If we support footsteps again, fix this.
    #stanceFrame = footstepsDriver.getFeetMidPoint(model)
    #com[2] = stanceFrame.GetPosition()[2]
    d = DebugData()
    d.addSphere(com, radius=0.015)
    obj = vis.updatePolyData(d.getPolyData(), 'COM %s' % model.getProperty('Name'), color=[1,0,0], visible=False, parent=model)

def initCenterOfMassVisulization():
    for model in [robotStateModel, teleopRobotModel, playbackRobotModel]:
        model.connectModelChanged(drawCenterOfMass)
        drawCenterOfMass(model)

if not ikPlanner.fixedBaseArm:
    initCenterOfMassVisulization()


class RobotMoverWidget(object):
    def __init__(self, jointController):
        self.jointController = jointController
        pos, rpy = jointController.q[:3], jointController.q[3:6]
        t = transformUtils.frameFromPositionAndRPY(pos, np.degrees(rpy))
        self.frame = vis.showFrame(t, 'mover widget', scale=0.3)
        self.frame.setProperty('Edit', True)
        self.frame.connectFrameModified(self.onFrameModified)

    def onFrameModified(self, frame):
        pos, rpy = self.frame.transform.GetPosition(), transformUtils.rollPitchYawFromTransform(self.frame.transform)
        q = self.jointController.q.copy()
        q[:3] = pos
        q[3:6] = rpy
        self.jointController.setPose('moved_pose', q)


class RobotGridUpdater(object):

    def __init__(self, gridFrame, robotModel, jointController):
        self.gridFrame = gridFrame
        self.robotModel = robotModel
        self.jointController = jointController
        self.robotModel.connectModelChanged(self.updateGrid)

    def updateGrid(self, model):
        pos = self.jointController.q[:3]

        x = int(np.round(pos[0])) / 10
        y = int(np.round(pos[1])) / 10
        z = int(np.round(pos[2] - 0.85)) / 1

        t = vtk.vtkTransform()
        t.Translate((x*10,y*10,z))
        self.gridFrame.copyFrame(t)

#gridUpdater = RobotGridUpdater(grid.getChildFrame(), robotStateModel, robotStateJointController)


class IgnoreOldStateMessagesSelector(object):

    def __init__(self, jointController):
        self.jointController = jointController
        self.action = app.addMenuAction('Tools', 'Ignore Old State Messages')
        self.action.setCheckable(True)
        self.action.setChecked(self.jointController.ignoreOldStateMessages)
        self.action.connect('triggered()', self.toggle)

    def toggle(self):
        self.jointController.ignoreOldStateMessages = bool(self.action.checked)

IgnoreOldStateMessagesSelector(robotStateJointController)


if useCourseModel:
    from director import coursemodel
    courseModel = coursemodel.CourseModel()

if 'startup' in drcargs.args():
    for filename in drcargs.args().startup:
        execfile(filename)
