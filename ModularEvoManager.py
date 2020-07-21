import asyncio
import py3gazebo.pygazebo as pygazebo
from py3gazebo.pygazebo.msg.v9 import vector3d_pb2
from py3gazebo.pygazebo.msg.v9 import gz_string_v_pb2
import numpy as np
import matplotlib.pyplot as plt
import networkx as nx
from networkx.drawing.nx_agraph import graphviz_layout

class GazeboMessageSubscriber:

    def __init__(self, host, port, timeout=30):
        self.ID = port
        self.host = host
        self.port = port
        self.loop = asyncio.get_event_loop()
        self.connected = False
        self.modelInserted = False
        self.simulationActivated = False
        self.running = False
        self.timeout = timeout
        self.generationCounter = 0
        self.lastFitness = 0
        self.filename = "./results/"+str(self.ID)+"_"

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

    async def executeTest(self, experiment, continueLearning):
        if self.connected:
            self.publisherToInsertModels = await self.manager.advertise('/gazebo/default/receiveModels',
                                                                        'gazebo.msgs.Vector3d')
            self.publisherToActivateSimulation = await self.manager.advertise('/gazebo/default/robotManagerController', 'gazebo.msgs.Vector3d')

            self.receiver = self.manager.subscribe('/gazebo/default/responseTopic', 'gazebo.msgs.Vector3d',
                                                   self.checkCallback)

            self.receiveNN = self.manager.subscribe('/gazebo/default/nnMatrixTopic', 'gazebo.msgs.GzString_V', self.matrixCallback)

            await self.receiver.wait_for_connection()
            self.filename += str(experiment)+".csv"
            if experiment == 45:
                with open(self.filename, "w") as file:
                    file.write("Experiment of DIRECT ENCODING \n")
            elif experiment == 125:
                with open(self.filename, "w") as file:
                    file.write("Experiment of INDIRECT ENCODING \n")
            elif experiment == 86:
                with open(self.filename, "w") as file:
                    file.write("Experiment of DIRECT ENCODING WITH MODULARITY \n" )
            elif experiment == 101:
                with open(self.filename, "w") as file:
                    file.write("Experiment of INDIRECT ENCODING WITH MODULARITY \n")

            with open(self.filename, "a") as file:
                file.write("Generation, Fitness, Modularity \n")

            self.running = True

            while self.running:
                '''
                    TRY TO INSERT A MODEL
                '''

                if not self.modelInserted:
                    await self.tryToInsertAModel()
                    print("Model Inserted")



                await self.receiveNN.wait_for_connection()
                if not self.simulationActivated:
                    await self.tryToActivateSimulation(experiment, continueLearning)
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
            message.x = self.ID
            message.y = message.z = 36
            try:
                await self.publisherToInsertModels.publish(message)
            except Exception as e:
                print(e)

            await asyncio.sleep(1)
            attempt += 1


    async def tryToActivateSimulation(self, kindOfSimulation, continueLearning):

        attempt = 1
        while not self.simulationActivated:
            print("Try To activate the simulation")
            print(attempt)
            message = vector3d_pb2.Vector3d()
            ## 45 = DIRECT ENCODING, 46 = ACK
            ## 125 = INDIRECT ENCODING, 126 = ACK
            ## 86 = DIRECT ENCODING MODULAR, 87 = ACK
            ## 101 = INDIRECT ENCODING MODULAR, 102 = ACK
            message.x = self.ID
            message.y = kindOfSimulation
            message.z = continueLearning
            try:
                await self.publisherToActivateSimulation.publish(message)
            except Exception as e:
                print(e)
            await asyncio.sleep(1)
            attempt += 1
            if attempt > 15:
                break

    def checkCallback(self, msg):
        message = vector3d_pb2.Vector3d()
        message.ParseFromString(msg)

        if message.x == self.ID:

            ## Model inserted ACK
            if message.y == 37:
                self.modelInserted = True
            elif message.y == 1:
                print("Closing procedure\n")
                self.f.close()
                exit(0)

            ## Simulation activation ACK
            elif message.y == 46 or message.y == 126 or message.y == 87 or message.y == 102:
                self.simulationActivated = True


            ## Fitness reporting
            elif message.y == 58:
                self.generationCounter += 1
                self.lastFitness = message.z
                #print(message.z)
            elif message.y == 59:
                with open(self.filename, "a") as file:
                    file.write(str(self.generationCounter) + ", " + str(self.lastFitness) + ", " + str(message.z) + "\n")

                print("Generation " +str(self.generationCounter) + " Fitness: " + str(self.lastFitness)+ " Modularity:" + str(message.z))
                self.f.flush()
                #print(message.z)

        
    def matrixCallback(self, msg):
        message = gz_string_v_pb2.GzString_V()
        message.ParseFromString(msg)
        #   print(message)
        matrix = []
        semiString = []
        if int(message.data[0]) == self.ID:
            for row in message.data[1:]:
                semiString = row.split(" ")
                matrix.append(semiString)

            ed = []
            i = 0
            for row in matrix:
                for values in row[2:]:
                    ed.append([i , values, -1])
                i += 1


            adjacentMatrix = np.zeros((len(matrix), len(matrix)))
            labels = []
            i = 0
            for row in matrix:
                labels.append(row[1])
                for link in row[2:]:
                    adjacentMatrix[i][int(link)] = 1
                i += 1


            l = {}
            for i, label in enumerate(labels):
                l[i] = label
            ##self.show_graph_with_labels(adjacentMatrix, l)



    def show_graph_with_labels(self, adjacency_matrix, mylabels):
        try:
            plt.close()
            G = nx.from_numpy_matrix(adjacency_matrix, create_using=nx.DiGraph)
            rows, cols = np.where(adjacency_matrix == 1.)
            edges = zip(rows.tolist(), cols.tolist())
            gr = nx.Graph()
            gr.add_edges_from(edges)
            pos = graphviz_layout(G, prog='dot', args="-Grankdir=LR")
            nx.draw(gr, node_size=500, labels=mylabels, with_labels=True, pos=pos, font_weight='bold')
            plt.show()
        except Exception as e:
            print(e)
            pass







    
