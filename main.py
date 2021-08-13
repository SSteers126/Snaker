import random

from direct.showbase.ShowBaseGlobal import globalClock
from direct.task.TaskManagerGlobal import taskMgr
from panda3d.core import Filename, BitMask32, CollisionTraverser
from pathlib import Path

from direct.showbase.ShowBase import ShowBase
from panda3d.core import PerspectiveLens
from panda3d.core import NodePath
from panda3d.core import AmbientLight, DirectionalLight
from panda3d.core import PointLight, Spotlight
from panda3d.core import AlphaTestAttrib, RenderAttrib, TransparencyAttrib
from panda3d.core import TextNode
from panda3d.core import Material
from panda3d.core import LVector3, Vec3, LPoint3f
from panda3d.core import MouseWatcher, ButtonHandle, KeyboardButton, MouseButton  # MouseWatcher looks for both mouse AND keyboard events.
from direct.gui.OnscreenText import OnscreenText

# Physics engine and other libraries for cars/maps
from panda3d.core import Point3
from panda3d.bullet import (BulletDebugNode, BulletBoxShape, BulletRigidBodyNode, BulletGhostNode,
                            BulletVehicle, BulletWorld, ZUp,
                            BulletPlaneShape, BulletHelper, BulletTriangleMesh, BulletTriangleMeshShape)
from panda3d.core import TransformState, ConfigVariableManager

import sys, os

import globalfile
import pathfuncs

from uuid import uuid4
from time import perf_counter, perf_counter_ns
import functions

from panda3d.core import DynamicTextFont, Filename
from direct.gui.DirectGui import *

class Target:

    def __init__(self, world, worldnp, loader, position, movezone=(5, 0, 5), zonemult=0.3):
        targetGeoms = loader.loadModel((pathfuncs.rel_path(None, path="/models/targetblit2.bam"))).findAllMatches(
            '**/+GeomNode')

        targetGeomNode = targetGeoms.getPath(0).node()
        tarGeom = targetGeomNode.getGeom(0)

        targetMesh = BulletTriangleMesh()
        targetMesh.addGeom(tarGeom)

        targetShape = BulletTriangleMeshShape(targetMesh, False)

        self.moveZone = movezone
        self.zoneMult = zonemult
        self.curPos = position
        self.score = 0

        newid = False
        while not newid:
            newid = True
            self.id = "Target-" + str(uuid4())
            for id in globalfile.targets:
                if id == self.id:
                    newid = False

        # print(self.id)
        globalfile.targets[self.id] = self

        self.targetNode = BulletRigidBodyNode(self.id)
        self.targetNP = worldnp.attachNewNode(self.targetNode)
        self.targetNP.node().addShape(targetShape)
        self.targetNP.setPos(Vec3(position))

        loader.loadModel((pathfuncs.rel_path(None, path="/models/targetblit2.bam"))).reparentTo(
            self.targetNP)

        world.attachRigidBody(self.targetNode)

        self.position = position
        for target in globalfile.targets:
            if globalfile.targets[target].position == self.position:
                self.gridMove()

    def gridScore(self):
        time_to_hit = globalfile.hittime - globalfile.lasthittime
        smooth_time = functions.smootherstep(0, 2, 2 - functions.clamp(time_to_hit, 0, 2))
        hitscore = functions.clamp(10 * smooth_time, 0.1, 10)
        globalfile.score += hitscore

    def gridMove(self):
        newposmade = False
        while not newposmade:
            chosenmods = []
            for distance in self.moveZone:
                if distance > 0:
                    chosendistance = random.randint(0, distance)
                    posorneg = random.choice([1, -1])
                    chosenmods.append(chosendistance*posorneg*self.zoneMult)
                else:
                    chosenmods.append(0)
            curPos = self.targetNP.getPos()
            # print(type(curPos))
            newcoord = LPoint3f(self.position[0]+chosenmods[0], self.position[1]+chosenmods[1], self.position[2]+chosenmods[2])
            newposmade = True
            # print("Old coords: {0}".format(self.position))
            for target in globalfile.targets:
                if globalfile.targets[target].position == newcoord:
                    newposmade = False
                # print(hitscore)
                # print(self.score)
        self.targetNP.setPos(newcoord)
        self.position = newcoord
        # print("New coords: {0}".format(self.position))

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

    def update(self, task):
        dt = globalClock.getDt()
        self.world.doPhysics(dt, 10, 0.008)
        self.scoreNum.setText("{0}".format(round(globalfile.score, 3)))
        self.timer.setText(functions.runTimeToCounter(perf_counter()))
        if self.training:
            md = base.win.getPointer(0)
            x = md.getX()
            y = md.getY()
            if base.win.movePointer(0, base.win.getXSize() // 2, base.win.getYSize() // 2):
                camera.setH(camera.getH() - (x - base.win.getXSize() / 2) * self.mouse_sens)
                camera.setP(camera.getP() - (y - base.win.getYSize() / 2) * self.mouse_sens)
                    # item.setPos(Vec3(0, 5, 5))
                    # self.targetNP.setPos(Vec3(0, -5, 1))
                    # del item

            # print(render.getRelativePoint(camera, (0, 1000, 0)))
            self.directionalLightNP.setHpr(camera.getH() - (x - base.win.getXSize() / 2) * self.mouse_sens, camera.getP() - (y - base.win.getYSize() / 2) * self.mouse_sens, 0)
        # if self.keymonitor(self.left_click):
        #     print(camera.getHpr())

        if self.keymonitor(self.esc_button):
            sys.exit()
        # if self.nosteerinput:
        #     self.noturn()
        # self.move_task()
        # self.reverse_task()
        # for i in globals.carObjects:
        #     i.move_task()
        #     i.reverse_task()
        # self.camera_task()
        # self.hud_task()
        # for i in globals.checkpointObjects:
        #     i.passTest()
            # for node in i.ghost.getOverlappingNodes():
            #     print(node)
            # print(type(node))
        return task.cont

    def makeDynamicLabel(self, pos, scale=.1):
        usedfont = DynamicTextFont(font_filename=Filename(pathfuncs.rel_path(None, "/font/Aquire.otf")), face_index=0)
        print(usedfont)
        return OnscreenText(
            parent=base.a2dTopLeft, align=TextNode.ALeft,
            style=1, fg=(0.3, 0.7, 1, 1), shadow=(0, 0, 0, .4),
            pos=pos, scale=scale, mayChange=True, sort=0, font=usedfont)

    def makeCentredDynamicLabel(self, pos, scale=.1):
        usedfont = DynamicTextFont(font_filename=Filename(pathfuncs.rel_path(None, "/font/Aquire.otf")), face_index=0)
        print(usedfont)
        return OnscreenText(
            parent=base.a2dTopLeft, align=TextNode.ACenter,
            style=1, fg=(0.3, 0.7, 1, 1), shadow=(0, 0, 0, .4),
            pos=pos, scale=scale, mayChange=True, sort=0, font=usedfont)

    def makeLabel(self, text, pos, scale=.1):
        usedfont = DynamicTextFont(font_filename=Filename(pathfuncs.rel_path(None, "/font/Aquire.otf")), face_index=0)
        print(usedfont)
        return OnscreenText(
            parent=base.a2dTopLeft, align=TextNode.ALeft, text=text,
            style=1, fg=(0.3, 0.7, 1, 1), shadow=(0, 0, 0, .4),
            pos=pos, scale=scale, mayChange=False, sort=0, font=usedfont)

    def makeCentredLabel(self, text, pos, scale=.1):
        usedfont = DynamicTextFont(font_filename=Filename(pathfuncs.rel_path(None, "/font/Aquire.otf")), face_index=0)
        print(usedfont)
        return OnscreenText(
            parent=base.a2dTopLeft, align=TextNode.ACenter, text=text,
            style=1, fg=(0.3, 0.7, 1, 1), shadow=(0, 0, 0, .4),
            pos=pos, scale=scale, mayChange=False, sort=0, font=usedfont)

    def startTraining(self):
        md = base.win.getPointer(0)
        base.win.movePointer(0, base.win.getXSize() // 2, base.win.getYSize() // 2)
        for i in range(self.targets):
            Target(world=self.world, worldnp=self.worldNP, loader=loader, position=(-1, 5, 0))
        globalfile.hittime = perf_counter()
        self.training = True
        self.start.destroy()
        self.targetLabel.destroy()
        self.targetSlider.destroy()
        self.targetNumLabel.destroy()

    def targetNum(self):
        self.targets = int(self.targetSlider["value"])
        self.targetNumLabel.setText(str(self.targets))
        # print(self.targets)

    def __init__(self):
        # Initialize the ShowBase class from which we inherit, which will
        # create a window and set up everything we need for rendering into it.
        ShowBase.__init__(self)
        # cvMgr = ConfigVariableManager.getGlobalPtr()
        # cvMgr.listVariables()
        self.mouse_sens = 0.05
        self.targets = 3
        self.lastHit = 0
        self.training = False
        self.start = DirectButton(text=("Start training", "Starting...", "Ready?", "disabled"), scale=(0.1), pos=(0, 0, 0.2), command=self.startTraining,
                                  text_fg=(0.1, 0.5, 1, 1), text_scale=(.9, .9))

        self.targetSlider = DirectSlider(range=(1, 10), value=3, pageSize=3, command=self.targetNum, scale=0.50)

        self.targetLabel = self.makeLabel(text="Targets ||", pos=(0.75, -1.02), scale=.075)
        self.targetNumLabel = self.makeDynamicLabel(pos=(2.355, -1.02), scale=.075)
        self.targetNumLabel.setText("5")

        self.hitSound = base.loader.loadSfx((pathfuncs.rel_path(None, path="/sfx/CoD-Hitmarker.wav")))

        self.keymonitor = base.mouseWatcherNode.is_button_down

        self.hudLeftModel = loader.loadModel((pathfuncs.rel_path(None, path="/models/HUD-top.bam")))
        self.hudLeft = aspect2d.attachNewNode("hud-left")
        self.hudLeft.setScale(0.35)
        self.hudLeft.setPos(0, 0, 0.64)
        self.hudLeftModel.reparentTo(self.hudLeft)
        # self.hudLeftTex = loader.loadTexture((pathfuncs.rel_path(None, path="/textures/Top-panel.png")))
        # self.hudLeftModel.setTexture(self.hudLeftTex, 1)
        self.scoreText = self.makeLabel(text="Score ||", pos=(0.65, -0.18), scale=.075)
        self.scoreNum = self.makeDynamicLabel(pos=(1.0225, -0.18), scale=.075)
        self.timer = self.makeCentredDynamicLabel(pos=(1.775, -0.2), scale=.075)
        self.timeText = self.makeCentredLabel(text="Time", pos=(1.775, -0.125), scale=.045)

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
        self.esc_button = KeyboardButton.ascii_key('p')
        self.left_click = MouseButton.one()
        # base.cam.setPos(0, 0, 0)
        camera.lookAt(0, 5, 0)
        base.camLens.setFov(90)

        # self.speedometer_kph = self.makeDynamicLabel(0)
        # self.speedometer_mph = self.makeDynamicLabel(1)

        # Plane
        self.worldNP = render.attachNewNode('World')
        shape = BulletPlaneShape(Vec3(0, 0, 1), 0)

        # np = self.worldNP.attachNewNode(BulletRigidBodyNode('Ground'))
        # np.node().addShape(shape)
        # np.setPos(0, 0, -1)
        # meshgeom = (loader.loadModel(globals.rel_path(None, path="/src/models/tracks/Road ML TT1SNb2b.bam")))
        # targetGeoms = loader.loadModel((pathfuncs.rel_path(None, path="/models/targetblit2.bam"))).findAllMatches(
        #     '**/+GeomNode')
        #
        # targetGeomNode = targetGeoms.getPath(0).node()
        # tarGeom = targetGeomNode.getGeom(0)
        #
        # targetMesh = BulletTriangleMesh()
        # targetMesh.addGeom(tarGeom)
        #
        # targetShape = BulletTriangleMeshShape(targetMesh, False)
        #
        # self.playerNode = BulletRigidBodyNode('Player')

        # geomNodes = loader.loadModel((globals.rel_path(None, path="/src/models/tracks/TrackN.bam"))).findAllMatches(
        #     '**/+GeomNode')
        # geomNode = geomNodes.getPath(0).node()
        # geom = geomNode.getGeom(0)
        #
        # mesh1 = BulletTriangleMesh()
        # mesh1.addGeom(geom)
        #
        # mesh = BulletTriangleMeshShape(mesh1, False)
        #
        # # mesh.reparentTo(render)
        # self.trackNode = BulletRigidBodyNode('Track')
        #
        # self.trackNP = self.worldNP.attachNewNode(self.trackNode)
        # # self.trackNP.attachNewNode(geomNode)
        # self.trackNP.setPos(Vec3(0, 0, -2))

        # mesh.reparentTo(self.trackNP)
        # self.trackNP.s
        # geometry
        # loader.loadModel(globals.rel_path(None, path="/src/models/tracks/TrackN.bam")).reparentTo(
        #     self.trackNP)

        self.debugNode = BulletDebugNode('Debug')
        self.debugNode.showWireframe(True)

        self.debugNP = self.worldNP.attachNewNode(self.debugNode)
        self.debugNP.show()
        self.world = BulletWorld()
        self.world.setDebugNode(self.debugNP.node())

        # self.targetNode = BulletRigidBodyNode('Target1')
        # self.targetNP = self.worldNP.attachNewNode(self.targetNode)
        # self.targetNP.node().addShape(targetShape)
        # self.targetNP.setPos(Vec3(-5, 5, 0))

        # TODO: Rotate player object, and then use code from APRG to place the camera in front of where the oject is pointing

        # loader.loadModel((pathfuncs.rel_path(None, path="/models/targetblit2.bam"))).reparentTo(
        #     self.targetNP)
        #
        # self.targetNode2 = BulletRigidBodyNode('Target2')
        # self.targetNP2 = self.worldNP.attachNewNode(self.targetNode2)
        # self.targetNP2.node().addShape(targetShape)
        # self.targetNP2.setPos(Vec3(-3, 5, 0))
        # camera.setH(0)
        # camera.setP(camera.getP() - 20)
        # # camera.lookAt(self.targetNP2)
        #
        # loader.loadModel((pathfuncs.rel_path(None, path="/models/targetblit2.bam"))).reparentTo(
        #     self.targetNP2)
        #
        # self.world.attachRigidBody(self.targetNode)
        # self.world.attachRigidBody(self.targetNode2)

        # self.mapNP = self.worldNP.attachNewNode(self.trackNode)
        # self.mapNP.node().addShape(mesh)

        # self.world.attachRigidBody(self.mapNP.node())

        self.world.setGravity(Vec3(0, 0, 0))
        self.env = self.loader.loadModel("models/environment")
        self.env.setScale(0.25, 0.25, 0.25)
        self.env.setPos(-8, 42, -3)
        self.env.reparentTo(self.render)

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

        # self.check1 = CheckpointBox(self.world, (0, 10, 0), (5, 2, 1), "Checkpoint1", 0)

        # np.setCollideMask(BitMask32.allOn())

        # Steering info
        self.steering = 0.0  # degrees
        self.steeringClamp = 40.0  # degrees
        self.steeringIncrement = 100.0  # degrees per second
        self.nosteerinput = False

        # base.cam.setPos(0, 0, 0)
        # base.cam.lookAt(0, -5, 0)

        # Directional light 02
        self.directionalLight = DirectionalLight('directionalLight')
        self.directionalLight.setColorTemperature(6250)
        # print(self.directionalLight.getPoint())
        # directionalLight.setColor((0.2, 0.2, 0.8, 1))
        self.directionalLightNP = render.attachNewNode(self.directionalLight)
        # This light is facing forwards, away from the camera.
        # directionalLightNP.setPos(0, -20, 0)
        self.directionalLightNP.setHpr(0, -20, 0)
        render.setLight(self.directionalLightNP)

        self.baseLight = render.attachNewNode(AmbientLight("ambientLight"))
        # Set the color of the ambient light
        self.baseLight.node().setColor((.1, .1, .1, 1))
        render.setLight(self.baseLight)

        self.lowPassFilter = AlphaTestAttrib.make(TransparencyAttrib.MDual, 0.5)

        # self.test_car = BulletCar(world=self.world, traverser=self.trackNP, keymonitor=self.keymonitor,
        #                           forward_button=self.forward_button, left_button=self.left_button,
        #                           right_button=self.right_button, brake_button=self.brake_button,
        #                           reverse_button=self.reverse_button)

        # Directional light 02
        directionalLight = DirectionalLight("HeadlightL")
        directionalLight.setColorTemperature(12250)
        # directionalLight.setColor((0.2, 0.2, 0.8, 1))
        directionalLightNP = render.attachNewNode(directionalLight)
        # This light is facing forwards, away from the camera.
        directionalLightNP.setHpr(0, -20, 0)
        render.setLight(directionalLightNP)

        # base.cam.reparentTo(self.test_car.chassisNP)  # Use this line to make the camera follow the car

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

        base.disableMouse()

        # scene = self.loader.loadModel("models/environment")
        # scene.reparentTo(render)

        md = base.win.getPointer(0)
        base.win.movePointer(0, base.win.getXSize() // 2, base.win.getYSize() // 2)

        self.taskMgr.add(self.update, "update")

        # Update

        taskMgr.add(self.update, 'update')
        # taskMgr.add(self.move_task, 'inputmanager')

        self.accept('mouse1', self.onClick)

    def onClick(self):
        # print(perf_counter())
        globalfile.lasthittime = globalfile.hittime
        globalfile.hittime = perf_counter()
        if (item := self.world.rayTestClosest(camera.getPos(),
                                              render.getRelativePoint(camera, (0, 1000, 0))).getNode()) is not None:
            # TODO: make wrapper class for target, and then find a way to use it to move any given target
            # TODO: Idea is to use the name of the node in the bullet world to set a flag, and then use that to tell the target to move in an event
            # print(item.name)
            globalfile.targets[item.name].gridMove()
            globalfile.targets[item.name].gridScore()
            if self.hitSound.status() == self.hitSound.PLAYING:
                self.hitSound.stop()
            self.hitSound.play()

if __name__ == '__main__':
    # Make an instance of our class and run the demo
    app = MainWindow()
    app.run()

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
