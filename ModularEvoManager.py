import asyncio
import py3gazebo.pygazebo as pygazebo
from py3gazebo.pygazebo.msg.v9 import vector3d_pb2

class GazeboMessageSubscriber:

    def __init__(self, host, port, timeout=30):
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.connected = False
        self.modelInserted = False
        self.simulationActivated = False
        self.running = False
        self.timeout = timeout
        self.generationCounter = 1

    async def connect(self):
        for i in range(self.timeout):
            try:
                self.manager = await pygazebo.connect((self.host, self.port))
                self.connected = True
                print("Connected to GAZEBO")
                break
            except Exception as e:
                pass
            await asyncio.sleep(0.5)

    async def executeTest(self, experiment):
        if self.connected:
            self.publisherToInsertModels = await self.manager.advertise('/gazebo/default/receiveModels',
                                                                        'gazebo.msgs.Vector3d')
            self.publisherToActivateSimulation = await self.manager.advertise(
                '/gazebo/default/robotManagerController', 'gazebo.msgs.Vector3d')

            self.receiver = self.manager.subscribe('/gazebo/default/responseTopic', 'gazebo.msgs.Vector3d',
                                                   self.checkCallback)
            await self.receiver.wait_for_connection()
            self.running = True

            while self.running:
                '''
                    TRY TO INSERT A MODEL
                '''
                if not self.modelInserted:
                    await self.tryToInsertAModel()
                    print("Model Inserted")

                if not self.simulationActivated:
                    await self.tryToActivateSimulation(experiment)
                    print("Simulation ACTIVATED")


                await asyncio.sleep(1)


        else:
            raise Exception("Timeout connecting to Gazebo.")

    async def tryToInsertAModel(self):
        attempt = 1
        while not self.modelInserted:
            print("Attempt to insert a model")
            print(attempt)
            message = vector3d_pb2.Vector3d()
            ## 36 = INSERT MODEL, 37 = ACK
            message.x = message.y = message.z = 36
            try:
                await self.publisherToInsertModels.publish(message)
            except Exception as e:
                print(e)

            await asyncio.sleep(1)
            attempt += 1

    async def tryToActivateSimulation(self, kindOfSimulation):

        attempt = 1
        while not self.simulationActivated:
            print("Try To activate the simulation")
            print(attempt)
            message = vector3d_pb2.Vector3d()
            ## 45 = DIRECT ENCODING, 46 = ACK
            ## 125 = INDIRECT ENCODING, 126 = ACK
            ## 86 = DIRECT ENCODING MODULAR, 87 = ACK
            ## 101 = INDIRECT ENCODING MODULAR, 102 = ACK
            message.x = message.y = message.z = kindOfSimulation
            try:

                await self.publisherToActivateSimulation.publish(message)
            except Exception as e:
                print(e)
            await asyncio.sleep(1)
            attempt += 1

    def checkCallback(self, msg):
        message = vector3d_pb2.Vector3d()
        message.ParseFromString(msg)


        ## Model inserted ACK
        if message.x == 37:
            self.modelInserted = True


        ## Simulation activation ACK
        elif message.x == 46 or message.x == 126 or message.x == 87 or message.x == 102:
            self.simulationActivated = True


        ## Fitness reporting
        elif message.x == 58:
            self.generationCounter += 1
            print(message.z)

