## These run an old rendering version - should not be needed, but are here incase it is required for some functionality etc
# import direct.directbase.DirectStart
# from direct.showbase.DirectObject import DirectObject

# Allows for an easy access set of data available to all scripts
import pathfuncs
import math
# Libraries for backend path conversion
import sys, os

from direct.showbase.ShowBaseGlobal import globalClock
from direct.task.TaskManagerGlobal import taskMgr
from panda3d.core import Filename, BitMask32, CollisionTraverser
from pathlib import Path

# import panda3d

# Basic imports required for the window, materials on objects, and HUD
from direct.showbase.ShowBase import ShowBase
from panda3d.core import PerspectiveLens
from panda3d.core import NodePath
from panda3d.core import AmbientLight, DirectionalLight
from panda3d.core import PointLight, Spotlight
from panda3d.core import AlphaTestAttrib, RenderAttrib, TransparencyAttrib
from panda3d.core import TextNode
from panda3d.core import Material
from panda3d.core import LVector3, Vec3
from panda3d.core import MouseWatcher, KeyboardButton  # MouseWatcher looks for both mouse AND keyboard events.
from direct.gui.OnscreenText import OnscreenText

# Libraries that will probably be used for manipulation of objects
import math
import sys
import colorsys
# Libraries for animated objects

from direct.actor.Actor import Actor
from direct.interval.IntervalGlobal import Sequence

# Physics engine and other libraries for cars/maps
from panda3d.core import Point3
from panda3d.bullet import (BulletDebugNode, BulletBoxShape, BulletRigidBodyNode, BulletGhostNode,
                            BulletVehicle, BulletWorld, ZUp,
                            BulletPlaneShape, BulletHelper, BulletTriangleMesh, BulletTriangleMeshShape)
from panda3d.core import TransformState

# Threading similar to standard pythonic threading
from panda3d.core import Thread

print(Thread.isThreadingSupported())

import gen.funcs


# Simple function to keep a value in a given range (by default 0 to 1)

# from panda3d.core import loadPrcFileData
#
# loadPrcFileData('', 'win-size 1024 768')
#
# w, h = 1900, 1080
#
# props = WindowProperties()
# props.setSize(w, h)
#
# base.win.requestProperties(props)

# os.system('xset r off')

class BulletCar:
    def __init__(self, world, traverser, keymonitor,
                 forward_button, left_button, right_button, brake_button, reverse_button,
                 max_steering=40.0, steer_speed=100.0, body_node_name="Car",
                 hitbox_dimensions=(1, 2.2, 0.55), hitbox_location=(0, 0, 0.75),
                 spawn_location=(0, 0, 0), mass=1520.0,
                 rel_body_path="/src/models/cars/Supra Body ReScale rotate Nglass.bam",
                 rel_lwheel_path="/src/models/cars/Supra Wheel L RS.bam",
                 rel_rwheel_path="/src/models/cars/Supra Wheel R RS.bam",
                 front_wheel_distance=(0.9, 1.35, 0.65),
                 rear_wheel_distance=(0.9, 1.25, 0.65)):

        pathfuncs.carObjects.append(self)
        self.keymonitor = keymonitor

        self.forward_button = forward_button
        self.left_button = left_button
        self.right_button = right_button
        self.brake_button = brake_button
        self.reverse_button = reverse_button

        # Steering info
        self.steering = 0.0  # degrees
        self.steeringClamp = max_steering  # degrees
        self.steeringIncrement = steer_speed  # degrees per second
        self.nosteerinput = False

        self.hitbox_shape = BulletBoxShape(Vec3(hitbox_dimensions[0], hitbox_dimensions[1], hitbox_dimensions[2]))
        self.ts = TransformState.makePos(Point3(hitbox_location[0], hitbox_location[1], hitbox_location[2]))

        self.chassisNP = render.attachNewNode(BulletRigidBodyNode(body_node_name))
        self.chassisNP.node().addShape(self.hitbox_shape, self.ts)
        self.chassisNP.setPos(spawn_location[0], spawn_location[1], spawn_location[2])
        self.chassisNP.node().setMass(mass)
        self.chassisNP.node().setDeactivationEnabled(False)
        self.chassisNP.node().setCcdMotionThreshold(1e-7)
        self.chassisNP.node().setCcdSweptSphereRadius(0.50)
        # traverser.addCollider(self.chassisNP, handler)
        world.attachRigidBody(self.chassisNP.node())

        # Chassis geometry
        loader.loadModel(pathfuncs.rel_path(None, path=rel_body_path)).reparentTo(
            self.chassisNP)

        # Vehicle
        self.vehicle = BulletVehicle(world, self.chassisNP.node())
        self.vehicle.setCoordinateSystem(ZUp)
        world.attachVehicle(self.vehicle)

        self.LFwheelNP = loader.loadModel(pathfuncs.rel_path(None, rel_lwheel_path))
        self.LFwheelNP.reparentTo(render)

        self.RFwheelNP = loader.loadModel(pathfuncs.rel_path(None, rel_rwheel_path))
        self.RFwheelNP.reparentTo(render)

        self.LBwheelNP = loader.loadModel(pathfuncs.rel_path(None, rel_lwheel_path))
        self.LBwheelNP.reparentTo(render)

        self.RBwheelNP = loader.loadModel(pathfuncs.rel_path(None, rel_rwheel_path))
        self.RBwheelNP.reparentTo(render)

        self.LFwheel = self.addWheel(Point3(front_wheel_distance[0], front_wheel_distance[1], front_wheel_distance[2]), True, self.LFwheelNP)
        self.RFwheel = self.addWheel(Point3(-(front_wheel_distance[0]), front_wheel_distance[1], front_wheel_distance[2]), True, self.RFwheelNP)
        self.LBwheel = self.addWheel(Point3(rear_wheel_distance[0], -(rear_wheel_distance[1]), rear_wheel_distance[2]), False, self.LBwheelNP)
        self.RBwheel = self.addWheel(Point3(-(rear_wheel_distance[0]), -(rear_wheel_distance[1]), rear_wheel_distance[2]), False, self.RBwheelNP)

        # self.headlightL = camera.attachNewNode(Spotlight(body_node_name + "left headlight"))
        # self.headlightL.node().setColor((.45, .45, .45, 1))
        # self.headlightL.node().setSpecularColor((0, 0, 0, 1))
        # self.headlightL.node().setLens(PerspectiveLens())  # lens controls FOV
        # self.headlightL.node().getLens().setFov(16, 16)  # width of the light's cone
        # self.headlightL.node().setAttenuation(LVector3(1, 0.0, 0.0))  # Light fade from distance
        # self.headlightL.node().setExponent(60.0)  # Edge softness
        # # self.headlightL.node().setPosition(Vec3(front_wheel_distance[0]-0.2, front_wheel_distance[1]+0.2, front_wheel_distance[2]))
        # render.setLight(self.headlightL)

        self.headlightL = loader.loadModel("src/models/other/Light placeholderb2b.bam")
        self.headlightL.setColor((1, 0, 0, 1))
        self.headlightL.setPos(front_wheel_distance[0]-0.2, front_wheel_distance[1]+0.7, front_wheel_distance[2])
        self.headlightL.setScale(.25)
        self.headlightL.reparentTo(self.chassisNP)
        self.leftLight = self.headlightL.attachNewNode(
            PointLight(body_node_name + "left headlight"))
        self.leftLight.node().setColor((.35, 0.35, 0.8, 1))
        self.leftLight.node().setAttenuation(LVector3(.05, 0.04, 0.0))

        self.headlightR = loader.loadModel("src/models/other/Light placeholderb2b.bam")
        self.headlightR.setColor((1, 0, 0, 1))
        self.headlightR.setPos(-front_wheel_distance[0]+0.2, front_wheel_distance[1]+0.7, front_wheel_distance[2])
        self.headlightR.setScale(.25)
        self.headlightR.reparentTo(self.chassisNP)
        self.rightLight = self.headlightR.attachNewNode(
            PointLight(body_node_name + "right headlight"))
        self.rightLight.node().setColor((.35, 0.35, 0.8, 1))
        self.rightLight.node().setAttenuation(LVector3(.05, 0.04, 0.0))

        render.setLight(self.leftLight)
        render.setLight(self.rightLight)

    def addWheel(self, pos, isfrontwheel, np):
        wheel = self.vehicle.createWheel()

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(pos)
        wheel.setFrontWheel(isfrontwheel)

        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(0.25)
        wheel.setMaxSuspensionTravelCm(10.0)

        wheel.setSuspensionStiffness(40.0)
        wheel.setWheelsDampingRelaxation(2.3)
        wheel.setWheelsDampingCompression(4.4)
        wheel.setFrictionSlip(8.0)
        wheel.setRollInfluence(0.1)

    def accelerate(self):
        self.vehicle.applyEngineForce(5000, 2)
        self.vehicle.applyEngineForce(5000, 3)

    def reverse(self):
        self.vehicle.applyEngineForce(-1000, 2)
        self.vehicle.applyEngineForce(-1000, 3)

    def noaccelerate(self):
        self.vehicle.applyEngineForce(0, 2)
        self.vehicle.applyEngineForce(0, 3)
        # self.vehicle.(0, 0)

    def brake(self):
        self.vehicle.setBrake(100, 0)
        self.vehicle.setBrake(100, 1)
        self.vehicle.setBrake(100, 2)
        self.vehicle.setBrake(100, 3)

    def nobrake(self):
        self.vehicle.setBrake(0, 0)
        self.vehicle.setBrake(0, 1)
        self.vehicle.setBrake(0, 2)
        self.vehicle.setBrake(0, 3)

    def turnleft(self):
        self.nosteerinput = False
        self.steering += globalClock.getDt() * self.steeringIncrement
        self.steering = min(self.steering, self.steeringClamp)
        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

    def nosteermethod(self, *args, **kwargs):
        self.nosteerinput = True

    def noturn(self):
        if self.steering < 0:
            self.steering += globalClock.getDt() * self.steeringIncrement
            self.steering = min(self.steering, self.steeringClamp)

        elif self.steering > 0:
            self.steering -= globalClock.getDt() * self.steeringIncrement
            self.steering = max(self.steering, -self.steeringClamp)

        if (self.steering >= -0.5 and self.steering < 0) or (self.steering <= 0.5 and self.steering > 0):
            self.steering = 0

        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

    def turnright(self):
        self.nosteerinput = False
        self.steering -= globalClock.getDt() * self.steeringIncrement
        self.steering = max(self.steering, -self.steeringClamp)
        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

    def move_task(self, *args, **kwargs):
        # speed = 0.0

        if self.keymonitor(self.forward_button):
            self.accelerate()

        # if is_down(self.reverse_button): # seems to disallow forward engine force when reverse engine force is used
        #     self.reverse()

        else:
            self.noaccelerate()

        if self.keymonitor(self.brake_button):
            self.brake()

        else:
            self.nobrake()

        if self.keymonitor(self.left_button):
            self.turnleft()

        if self.keymonitor(self.right_button):
            self.turnright()

        if not(self.keymonitor(self.left_button)) and not(self.keymonitor(self.right_button)):
            self.nosteerinput = True

        if self.nosteerinput:
            self.noturn()

    def reverse_task(self, *args, **kwargs):
        if self.keymonitor(self.reverse_button): # seems to disallow forward engine force when reverse engine force is used
            self.reverse()


class CheckpointBox:

    def __init__(self, world, position=(0, 10, 0), scale=(5, 2, 1), nodeName="Ghost", gateID=-1):
        pathfuncs.checkpointObjects.append(self)

        self.gateID = gateID
        self.passed = False

        self.world = world
        self.ghostShape = BulletBoxShape(Vec3(scale[0], scale[1], scale[2]))

        self.ghost = BulletGhostNode(nodeName)
        self.ghost.addShape(self.ghostShape)

        self.ghostNP = render.attachNewNode(self.ghost)
        self.ghostNP.setPos(position[0], position[1], position[2])
        self.ghostNP.setCollideMask(BitMask32(0x0f))

        self.world.attachGhost(self.ghost)

    def passTest(self):
        if not self.passed:
            for node in self.ghost.getOverlappingNodes():
                str_node = str(node)
                split_node = str_node.split(" ")
                if split_node[1][:3] == "Car":
                    self.passed = True

        # print(self.passed)

class StartLineBox:

    def __init__(self, world, position=(0, 5, 0), scale=(5, 2, 1), nodeName="StartLine", gateID=0):
        pathfuncs.startLine = self

        self.gateID = gateID
        self.passed = False

        self.world = world
        self.ghostShape = BulletBoxShape(Vec3(scale[0], scale[1], scale[2]))

        self.ghost = BulletGhostNode(nodeName)
        self.ghost.addShape(self.ghostShape)

        self.ghostNP = render.attachNewNode(self.ghost)
        self.ghostNP.setPos(position[0], position[1], position[2])
        self.ghostNP.setCollideMask(BitMask32(0x0f))

        self.world.attachGhost(self.ghost)

    def _NewLap(self):
        for i in pathfuncs.checkpointObjects:
            if not i.passed:
                return False

        for i in pathfuncs.checkpointObjects:
            i.passed = False


        pathfuncs.lapNum += 1

    def passTest(self):
        if not self.passed:
            for node in self.ghost.getOverlappingNodes():
                str_node = str(node)
                split_node = str_node.split(" ")
                if split_node[1][:3] == "Car":
                    self.passed = True

        # print(self.passed)


class MainWindow(ShowBase):

    def doExit(self):
        # os.system('xset r on')
        sys.exit(1)

    def rel_path(self, path="/src"):
        # Get the location of the 'py' file I'm running:
        dir = os.path.abspath(sys.path[0])

        # Convert that to panda's unix-style notation.
        dir = Filename.fromOsSpecific(dir).getFullpath()

        return dir + path

    # Macro-like function to reduce the amount of code needed to create the
    # onscreen instructions
    def makeStatusLabel(self, i):
        return OnscreenText(
            parent=base.a2dTopLeft, align=TextNode.ALeft,
            style=1, fg=(1, 1, 0, 1), shadow=(0, 0, 0, .4),
            pos=(0.06, -0.1 -(.06 * i)), scale=.05, mayChange=True)
    
    def convZUp(self, x, y, z):
        return (x, -z, y)

    def camera_task(self, *args, **kwargs):
        current_steer = self.steering
        # xcoord = None
        x = 8+gen.funcs.smootherstep(10*(8*(math.sin(current_steer*0.00025))), -1, 1)/10
        y = -8-gen.funcs.smootherstep(10*(8*(math.cos(current_steer*0.00025))), -1, 1)/10

        base.cam.setPos((8*(math.sin(current_steer*0.00025))), -(8*(math.cos(current_steer*0.00025))), 2.1)
        base.cam.lookAt(0, 0, 0.3)

    def addWheel(self, pos, isfrontwheel, np):
        wheel = self.vehicle.createWheel()

        wheel.setNode(np.node())
        wheel.setChassisConnectionPointCs(pos)
        wheel.setFrontWheel(isfrontwheel)

        wheel.setWheelDirectionCs(Vec3(0, 0, -1))
        wheel.setWheelAxleCs(Vec3(1, 0, 0))
        wheel.setWheelRadius(0.25)
        wheel.setMaxSuspensionTravelCm(10.0)

        wheel.setSuspensionStiffness(40.0)
        wheel.setWheelsDampingRelaxation(2.3)
        wheel.setWheelsDampingCompression(4.4)
        wheel.setFrictionSlip(8.0)
        wheel.setRollInfluence(0.1)

    def accelerate(self):
        self.vehicle.applyEngineForce(5000, 2)
        self.vehicle.applyEngineForce(5000, 3)

    def reverse(self):
        self.vehicle.applyEngineForce(-1000, 2)
        self.vehicle.applyEngineForce(-1000, 3)

    def noaccelerate(self):
        self.vehicle.applyEngineForce(0, 2)
        self.vehicle.applyEngineForce(0, 3)
        # self.vehicle.(0, 0)

    def brake(self):
        self.vehicle.setBrake(100, 0)
        self.vehicle.setBrake(100, 1)
        self.vehicle.setBrake(100, 2)
        self.vehicle.setBrake(100, 3)

    def nobrake(self):
        self.vehicle.setBrake(0, 0)
        self.vehicle.setBrake(0, 1)
        self.vehicle.setBrake(0, 2)
        self.vehicle.setBrake(0, 3)

    def turnleft(self):
        self.nosteerinput = False
        self.steering += globalClock.getDt() * self.steeringIncrement
        self.steering = min(self.steering, self.steeringClamp)
        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

    def nosteermethod(self, *args, **kwargs):
        self.nosteerinput = True

    def noturn(self):
        if self.steering < 0:
            self.steering += globalClock.getDt() * self.steeringIncrement
            self.steering = min(self.steering, self.steeringClamp)

        elif self.steering > 0:
            self.steering -= globalClock.getDt() * self.steeringIncrement
            self.steering = max(self.steering, -self.steeringClamp)

        if (self.steering >= -0.1 and self.steering < 0) or (self.steering <= 0.1 and self.steering > 0):
            self.steering = 0

        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

    def turnright(self):
        self.nosteerinput = False
        self.steering -= globalClock.getDt() * self.steeringIncrement
        self.steering = max(self.steering, -self.steeringClamp)
        self.vehicle.setSteeringValue(self.steering, 0)
        self.vehicle.setSteeringValue(self.steering, 1)

    def move_task(self, *args, **kwargs):
        # speed = 0.0

        if self.keymonitor(self.forward_button):
            self.accelerate()

        # if is_down(self.reverse_button): # seems to disallow forward engine force when reverse engine force is used
        #     self.reverse()

        else:
            self.noaccelerate()

        if self.keymonitor(self.brake_button):
            self.brake()

        else:
            self.nobrake()

        if self.keymonitor(self.left_button):
            self.turnleft()

        if self.keymonitor(self.right_button):
            self.turnright()

        if not(self.keymonitor(self.left_button)) and not(self.keymonitor(self.right_button)):
            self.nosteerinput = True

        if self.nosteerinput:
            self.noturn()

    def reverse_task(self, *args, **kwargs):
        if self.keymonitor(self.reverse_button): # seems to disallow forward engine force when reverse engine force is used
            self.reverse()



    def hud_task(self, *args, **kwargs):
        # kph_measure = globals.carObjects[0].vehicle.getCurrentSpeedKmHour()
        kph_measure = self.test_car.vehicle.getCurrentSpeedKmHour()
        self.speedometer_kph.setText("Speed (kph): " + str(round(kph_measure, 1)))
        self.speedometer_mph.setText("Speed (mph): " + str(round(mph_measure := (kph_measure * 0.6213712), 1)))

    # def updateStatusLabel(self, *args, **kwargs):

    def update(self, task):
        dt = globalClock.getDt()
        self.world.doPhysics(dt, 10, 0.008)
        # if self.nosteerinput:
        #     self.noturn()
        # self.move_task()
        # self.reverse_task()
        for i in pathfuncs.carObjects:
            i.move_task()
            i.reverse_task()
        # self.camera_task()
        self.hud_task()
        for i in pathfuncs.checkpointObjects:
            i.passTest()
            # for node in i.ghost.getOverlappingNodes():
            #     print(node)
            # print(type(node))
        return task.cont

    def __init__(self):
        # Initialize the ShowBase class from which we inherit, which will
        # create a window and set up everything we need for rendering into it.
        ShowBase.__init__(self)

        self.keymonitor = base.mouseWatcherNode.is_button_down

        self.traverser = CollisionTraverser('collision traverser')
        base.cTrav = self.traverser
        # self.traverser.addCollider(fromObject, handler)

        dir_path = Path(sys.path[0])
        self.use_kph = True
        # self.is_down = base.mouseWatcherNode.is_button_down

        self.forward_button = KeyboardButton.ascii_key('w')  # 'raw-' prefix is being used to mantain WASD positioning on other keyboard layouts
        self.left_button = KeyboardButton.ascii_key('a')
        self.brake_button = KeyboardButton.ascii_key('s')
        self.right_button = KeyboardButton.ascii_key('d')
        self.reverse_button = KeyboardButton.ascii_key('r')

        self.speedometer_kph = self.makeStatusLabel(0)
        self.speedometer_mph = self.makeStatusLabel(1)

        # Plane
        self.worldNP = render.attachNewNode('World')
        shape = BulletPlaneShape(Vec3(0, 0, 1), 0)

        # np = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
        # np.node().addShape(shape)
        # np.setPos(0, 0, -1)
        # meshgeom = (loader.loadModel(globals.rel_path(None, path="/src/models/tracks/Road ML TT1SNb2b.bam")))

        geomNodes = loader.loadModel((pathfuncs.rel_path(None, path="/src/models/tracks/TrackN.bam"))).findAllMatches('**/+GeomNode')
        geomNode = geomNodes.getPath(0).node()
        geom = geomNode.getGeom(0)

        mesh1 = BulletTriangleMesh()
        mesh1.addGeom(geom)

        mesh = BulletTriangleMeshShape(mesh1, False)

        # mesh.reparentTo(render)
        self.trackNode = BulletRigidBodyNode('Track')

        self.trackNP = self.worldNP.attachNewNode(self.trackNode)
        # self.trackNP.attachNewNode(geomNode)
        self.trackNP.setPos(Vec3(0, 0, -2))

        # mesh.reparentTo(self.trackNP)
        # self.trackNP.s
        # geometry
        loader.loadModel(pathfuncs.rel_path(None, path="/src/models/tracks/TrackN.bam")).reparentTo(
            self.trackNP)

        self.debugNode = BulletDebugNode('Debug')
        # self.debugNode.showWireframe(True)

        self.debugNP = self.worldNP.attachNewNode(self.debugNode)
        self.debugNP.show()
        self.world = BulletWorld()
        self.world.setDebugNode(self.debugNP.node())

        self.mapNP = self.worldNP.attachNewNode(self.trackNode)
        self.mapNP.node().addShape(mesh)

        self.world.attachRigidBody(self.mapNP.node())
        self.world.setGravity(Vec3(0, 0, -9.81))

        # self.trackNP.set
        # visNP
        # self.trackNP.node().setIntoCollideMask(BitMask32(0x0))

        # bodyNPs = BulletHelper.fromCollisionSolids(visNP, True)
        # print(bodyNPs)
        # print(type(bodyNPs))
        # self.ballNP = bodyNPs[0]


        # self.world.attachRigidBody(self.trackNP)

        # self.ghostShape = BulletBoxShape(Vec3(1, 1, 1))
        #
        # self.ghost = BulletGhostNode('Ghost')
        # self.ghost.addShape(self.ghostShape)
        #
        # self.ghostNP = render.attachNewNode(self.ghost)
        # self.ghostNP.setPos(0, 10, 0.5)
        # self.ghostNP.setCollideMask(BitMask32(0x0f))
        #
        # self.world.attachGhost(self.ghost)

        self.check1 = CheckpointBox(self.world, (0, 10, 0), (5, 2, 1), "Checkpoint1", 0)

        # np.setCollideMask(BitMask32.allOn())

        # Steering info
        self.steering = 0.0  # degrees
        self.steeringClamp = 40.0  # degrees
        self.steeringIncrement = 100.0  # degrees per second
        self.nosteerinput = False

        base.cam.setPos(0, -8, 2.1)
        base.cam.lookAt(0, 0, 0.3)

        # Directional light 02
        directionalLight = DirectionalLight('directionalLight')
        directionalLight.setColorTemperature(6250)
        # directionalLight.setColor((0.2, 0.2, 0.8, 1))
        directionalLightNP = render.attachNewNode(directionalLight)
        # This light is facing forwards, away from the camera.
        # directionalLightNP.setPos(0, -20, 0)
        directionalLightNP.setHpr(0, -20, 0)
        render.setLight(directionalLightNP)

        self.baseLight = render.attachNewNode(AmbientLight("ambientLight"))
        # Set the color of the ambient light
        self.baseLight.node().setColor((.1, .1, .1, 1))
        render.setLight(self.baseLight)

        self.lowPassFilter = AlphaTestAttrib.make(TransparencyAttrib.MDual, 0.5)

        self.test_car = BulletCar(world=self.world, traverser=self.trackNP, keymonitor=self.keymonitor,
                                  forward_button=self.forward_button, left_button=self.left_button,
                                  right_button=self.right_button, brake_button=self.brake_button,
                                  reverse_button=self.reverse_button)

        # Directional light 02
        directionalLight = DirectionalLight("HeadlightL")
        directionalLight.setColorTemperature(6250)
        # directionalLight.setColor((0.2, 0.2, 0.8, 1))
        directionalLightNP = render.attachNewNode(directionalLight)
        # This light is facing forwards, away from the camera.
        directionalLightNP.setHpr(0, -20, 0)
        render.setLight(directionalLightNP)

        base.cam.reparentTo(self.test_car.chassisNP)  # Use this line to make the camera follow the car

        ## TODO: coord system - xpositive = right, ypositive = back, zpositive = up

        map = base.win.get_keyboard_map()

        # Use this to print all key mappings
        print(map)

        # Find out which virtual key is associated with the ANSI US "w"
        w_button = map.get_mapped_button("w")

        # Use a 512x512 resolution shadow map
        directionalLight.setShadowCaster(True, 512, 512)
        # Enable the shader generator for the receiving nodes
        render.setShaderAuto()


        # Update


        taskMgr.add(self.update, 'update')
        # taskMgr.add(self.move_task, 'inputmanager')


if __name__ == '__main__':
    # Make an instance of our class and run the demo
    app = MainWindow()
    app.run()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
